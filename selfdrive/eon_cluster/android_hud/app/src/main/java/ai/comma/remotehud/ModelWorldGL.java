package ai.comma.remotehud;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.opengl.EGL14;
import android.opengl.EGLConfig;
import android.opengl.EGLContext;
import android.opengl.EGLDisplay;
import android.opengl.EGLSurface;
import android.opengl.GLES20;
import android.util.Log;

import org.json.JSONArray;
import org.json.JSONObject;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;

/**
 * Fifth-stage modelV2-only road renderer.
 *
 * The renderer keeps model geometry authoritative and can place a lightweight
 * phone-local building/road context underneath it. It adds a separately gated TMAP route
 * trace, holds brief radar dropouts, and releases EGL on renderer shutdown.
 * Geometry is projected with the fork's established camera constants, then an
 * unlit OpenGL ES 2.0 shader draws road, observed boundaries, lane lines and
 * the final lateral path into an offscreen pbuffer.  Any EGL/GL failure returns
 * false so HudService can immediately draw the Canvas model-world fallback.
 */
final class ModelWorldGL {
    private static final String TAG = "ModelWorldGL";
    private static final int WIDTH = 952;
    static final int TOP = 217;
    static final int BOTTOM = 454;
    private static final int HEIGHT = BOTTOM - TOP;
    private static final float CX = 476f;
    private static final float FOCAL = 520f;
    private static final float CAM_H = 4.6f;
    private static final float CAM_BACK = 13.0f;
    private static final float HORIZON = 249f;
    private static final float NEAR_DEPTH = 11.4f;
    private static final float[] ROAD_EDGE_SAMPLE_XS = {12f, 25f, 45f};
    // Area clipping can add up to four viewport-edge intersections to the
    // converter's 80-point polygons.
    private static final int MAX_POINTS = 96;
    private static final int MAX_VERTEX_FLOATS = 4096;

    /**
     * BSD 경고 — 순정 계기판과 같은 레이더 파동 아크.
     *
     * 자차 뒤범퍼 모서리를 중심으로 한 동심 아크 3개와 경고 삼각형을
     * 화면 좌표에 직접 그린다. 노면 투영을 쓰지 않으므로 근거리 클리핑
     * (NEAR_DEPTH)에 잘리지 않는다.
     */
    private static final float EGO_SPRITE_W = 78f;   // HudService 가 얹는 자차 그림 폭
    private static final float EGO_BASELINE = 433f;  // 자차 접지선(패널 좌표)
    private static final float BSD_CORNER_DY = 32f;  // 접지선 위로 올린 아크 중심
    private static final float[] BSD_ARC_RADII = {42f, 64f, 86f};
    private static final float BSD_ARC_SQUASH = 0.62f;   // 원근으로 눌린 세로비
    private static final float BSD_ARC_A0 = 104f;        // 좌측 아크 시작각(도)
    private static final float BSD_ARC_A1 = 196f;        // 좌측 아크 끝각(도)
    private static final int BSD_ARC_SEGMENTS = 24;
    private static final int BSD_ARC_CHUNKS = 3;         // 끝단 알파를 낮출 분할 수
    private static final float BSD_ARC_CHUNK_FADE = 0.5f;
    private static final float BSD_CORE_ALPHA = 165f / 255f;
    // 굵기 가산 / 알파 배수 — 겹칠수록 번져 보인다.
    private static final float[][] BSD_ARC_LAYERS = {
            {10f, 0.06f}, {6f, 0.10f}, {3f, 0.16f}, {0f, 0.55f}};
    private static final float BSD_TRI_DX = 112f;    // 자차 옆면에서 삼각형까지
    private static final float BSD_TRI_DY = 14f;     // 접지선 위 삼각형 중심
    private static final float BSD_TRI_SIZE = 34f;
    private static final float BSD_TRI_STROKE = 4f;
    private static final float BSD_TRI_ALPHA = 200f / 255f;
    private static final int BSD_COLOR = Color.rgb(228, 62, 62);

    /** 가드레일. 도로경계 바깥쪽에 세운다 — 위치는 모델 edges 그대로다. */
    private static final float RAIL_INSET = 0.34f;   // 경계선 바깥으로 밀어낼 거리(m)
    private static final float RAIL_BOTTOM = 0.44f;  // 노면 위 레일 하단(m)
    private static final float RAIL_TOP = 0.80f;     // 노면 위 레일 상단(m)
    private static final float RAIL_MAX_X = 95f;     // 이보다 먼 구간은 그리지 않는다
    private static final float RAIL_POST_SPACING = 6f;
    private static final float RAIL_POST_TOP = 0.86f;

    /** 원경 헤이즈. 지평선 아래를 하늘색으로 덮어 깊이감을 준다. */
    private static final int HAZE_BANDS = 7;
    private static final float HAZE_DEPTH_PX = 46f;

    private EGLDisplay display = EGL14.EGL_NO_DISPLAY;
    private EGLContext context = EGL14.EGL_NO_CONTEXT;
    private EGLSurface surface = EGL14.EGL_NO_SURFACE;
    private int program;
    private int positionHandle;
    private int colorHandle;
    private boolean failed;

    private final FloatBuffer vertexBuffer = ByteBuffer
            .allocateDirect(MAX_VERTEX_FLOATS * 4)
            .order(ByteOrder.nativeOrder()).asFloatBuffer();
    private final float[] vertices = new float[MAX_VERTEX_FLOATS];
    private final IntBuffer readBuffer = ByteBuffer
            .allocateDirect(WIDTH * HEIGHT * 4)
            .order(ByteOrder.nativeOrder()).asIntBuffer();
    private final int[] pixels = new int[WIDTH * HEIGHT];
    private final float[] projected = new float[2];
    private final float[] lineScreenX = new float[MAX_POINTS];
    private final float[] lineScreenY = new float[MAX_POINTS];
    private final float[] mapBaseX = new float[MAX_POINTS];
    private final float[] mapBaseY = new float[MAX_POINTS];
    private final int[] visibleMapBuildings = new int[70];
    private final int[] visibleMapAreas = new int[50];
    private final int[] mapIndices = new int[MAX_POINTS];
    // 앞차를 자차와 같은 그림으로 그리기 위한 화면 좌표. GL 은 텍스처를 쓰지
    // 않으므로 위치만 계산해 두고 비트맵은 HudService 가 Canvas 로 얹는다.
    private final float[] leadSpriteX = new float[2];
    private final float[] leadSpriteY = new float[2];
    private final float[] leadSpriteW = new float[2];
    private final float[] leadSpriteAlpha = new float[2];
    private final boolean[] leadSpriteValid = new boolean[2];
    private final boolean[] leadSpriteBraking = new boolean[2];

    private final float[] worldQuad = new float[8];
    private final Bitmap frame = Bitmap.createBitmap(WIDTH, HEIGHT, Bitmap.Config.ARGB_8888);

    private long lastTimestamp = Long.MIN_VALUE;
    private int lastStyle;
    private long previousSceneTimestamp = Long.MIN_VALUE;
    private long nextRenderNanos;
    private float horizonShift;
    private float roadZGain = 1f;

    private static final class Line {
        final float[] x = new float[MAX_POINTS];
        final float[] y = new float[MAX_POINTS];
        final float[] z = new float[MAX_POINTS];
        int count;
        float confidence = 1f;
    }

    private final Line smoothedPath = new Line();
    private final Line[] smoothedLanes = {new Line(), new Line(), new Line(), new Line()};
    private final Line[] smoothedEdges = {new Line(), new Line()};
    private final Line smoothedRoute = new Line();
    private final Line mapLine = new Line();
    private final HudMapStore mapStore;
    private final float[] laneConfidence = new float[4];
    private final boolean[] laneVisible = new boolean[4];
    private final float[] leadDistance = new float[2];
    private final float[] leadLateral = new float[2];
    private final float[] leadAcceleration = new float[2];
    private final long[] leadLastSeenTimestamp = new long[2];
    private final boolean[] leadSeen = new boolean[2];

    ModelWorldGL(Context context) {
        mapStore = new HudMapStore(context.getApplicationContext());
    }

    boolean draw(Canvas canvas, Paint paint, JSONObject scene, boolean enabled,
                 int driveBg, int roadTop, int roadBottom, int pathColor,
                 boolean dark, float roadZPercent, float livePitch,
                 float pitchPercent, float calibPitch,
                 boolean leadSprite, boolean guardrail, int haze) {
        if (failed || scene == null) {
            return false;
        }
        try {
            if (!ensureGl()) {
                return false;
            }
            long timestamp = scene.optLong("t", 0L);
            // BSD 는 좌우 경고 상태가 바뀌면 같은 타임스탬프라도
            // 다시 그려야 한다. 안 그러면 경고가 켜져도 옛 프레임이 남는다.
            int style = driveBg ^ roadTop ^ roadBottom ^ pathColor ^ (dark ? 1 : 0)
                    ^ (guardrail ? 1 << 8 : 0) ^ (haze << 9)
                    ^ (scene.optBoolean("leftBsd", false) ? 1 << 6 : 0)
                    ^ (scene.optBoolean("rightBsd", false) ? 1 << 7 : 0);
            boolean styleChanged = style != lastStyle;
            if (timestamp != lastTimestamp || styleChanged) {
                long started = System.nanoTime();
                if (styleChanged || started >= nextRenderNanos) {
                    if (!render(scene, enabled, driveBg, roadTop, roadBottom, pathColor,
                            dark, roadZPercent, livePitch, pitchPercent, calibPitch,
                            leadSprite, guardrail, haze)) {
                        return false;
                    }
                    long cost = System.nanoTime() - started;
                    // If readback is unexpectedly expensive on a hot S9, reuse
                    // the last frame briefly instead of queuing more GL work.
                    nextRenderNanos = started + (cost > 65_000_000L
                            ? 180_000_000L : 0L);
                    lastTimestamp = timestamp;
                    lastStyle = style;
                }
            }
            paint.setShader(null);
            paint.setAlpha(255);
            paint.setFilterBitmap(true);
            canvas.drawBitmap(frame, 0f, TOP, paint);
            return true;
        } catch (Throwable error) {
            failed = true;
            Log.e(TAG, "OpenGL renderer disabled; falling back to Canvas model world", error);
            release();
            return false;
        }
    }

    private boolean ensureGl() {
        if (program != 0) {
            return EGL14.eglMakeCurrent(display, surface, surface, context);
        }
        display = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY);
        if (display == EGL14.EGL_NO_DISPLAY) {
            fail("No EGL display");
            return false;
        }
        int[] version = new int[2];
        if (!EGL14.eglInitialize(display, version, 0, version, 1)) {
            fail("EGL initialize failed");
            return false;
        }

        EGLConfig config = chooseConfig(true);
        if (config == null) {
            config = chooseConfig(false);
        }
        if (config == null) {
            fail("No pbuffer EGL config");
            return false;
        }

        int[] contextAttrs = {
                EGL14.EGL_CONTEXT_CLIENT_VERSION, 2,
                EGL14.EGL_NONE
        };
        context = EGL14.eglCreateContext(display, config, EGL14.EGL_NO_CONTEXT,
                contextAttrs, 0);
        int[] surfaceAttrs = {
                EGL14.EGL_WIDTH, WIDTH,
                EGL14.EGL_HEIGHT, HEIGHT,
                EGL14.EGL_NONE
        };
        surface = EGL14.eglCreatePbufferSurface(display, config, surfaceAttrs, 0);
        if (context == EGL14.EGL_NO_CONTEXT || surface == EGL14.EGL_NO_SURFACE
                || !EGL14.eglMakeCurrent(display, surface, surface, context)) {
            fail("EGL context or pbuffer creation failed");
            return false;
        }

        String vertexShader =
                "attribute vec2 aPosition;\n" +
                "void main() { gl_Position = vec4(aPosition, 0.0, 1.0); }";
        String fragmentShader =
                "precision mediump float;\n" +
                "uniform vec4 uColor;\n" +
                "void main() { gl_FragColor = uColor; }";
        int vs = compile(GLES20.GL_VERTEX_SHADER, vertexShader);
        int fs = compile(GLES20.GL_FRAGMENT_SHADER, fragmentShader);
        program = GLES20.glCreateProgram();
        GLES20.glAttachShader(program, vs);
        GLES20.glAttachShader(program, fs);
        GLES20.glLinkProgram(program);
        int[] linked = new int[1];
        GLES20.glGetProgramiv(program, GLES20.GL_LINK_STATUS, linked, 0);
        GLES20.glDeleteShader(vs);
        GLES20.glDeleteShader(fs);
        if (linked[0] == 0) {
            fail("GL link failed: " + GLES20.glGetProgramInfoLog(program));
            return false;
        }
        positionHandle = GLES20.glGetAttribLocation(program, "aPosition");
        colorHandle = GLES20.glGetUniformLocation(program, "uColor");
        GLES20.glUseProgram(program);
        GLES20.glEnableVertexAttribArray(positionHandle);
        GLES20.glDisable(GLES20.GL_DEPTH_TEST);
        GLES20.glDisable(GLES20.GL_CULL_FACE);
        GLES20.glEnable(GLES20.GL_BLEND);
        GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA);
        return true;
    }

    private EGLConfig chooseConfig(boolean multisample) {
        int[] attrs = multisample ? new int[] {
                EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT,
                EGL14.EGL_SURFACE_TYPE, EGL14.EGL_PBUFFER_BIT,
                EGL14.EGL_RED_SIZE, 8,
                EGL14.EGL_GREEN_SIZE, 8,
                EGL14.EGL_BLUE_SIZE, 8,
                EGL14.EGL_ALPHA_SIZE, 8,
                EGL14.EGL_SAMPLE_BUFFERS, 1,
                EGL14.EGL_SAMPLES, 4,
                EGL14.EGL_NONE
        } : new int[] {
                EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT,
                EGL14.EGL_SURFACE_TYPE, EGL14.EGL_PBUFFER_BIT,
                EGL14.EGL_RED_SIZE, 8,
                EGL14.EGL_GREEN_SIZE, 8,
                EGL14.EGL_BLUE_SIZE, 8,
                EGL14.EGL_ALPHA_SIZE, 8,
                EGL14.EGL_NONE
        };
        EGLConfig[] configs = new EGLConfig[1];
        int[] count = new int[1];
        return EGL14.eglChooseConfig(display, attrs, 0, configs, 0, 1, count, 0)
                && count[0] > 0 ? configs[0] : null;
    }

    private int compile(int type, String source) {
        int shader = GLES20.glCreateShader(type);
        GLES20.glShaderSource(shader, source);
        GLES20.glCompileShader(shader);
        int[] compiled = new int[1];
        GLES20.glGetShaderiv(shader, GLES20.GL_COMPILE_STATUS, compiled, 0);
        if (compiled[0] == 0) {
            String log = GLES20.glGetShaderInfoLog(shader);
            GLES20.glDeleteShader(shader);
            throw new IllegalStateException("GL compile failed: " + log);
        }
        return shader;
    }

    private boolean render(JSONObject scene, boolean enabled, int driveBg,
                           int roadTop, int roadBottom, int pathColor,
                           boolean dark, float roadZPercent, float livePitch,
                           float pitchPercent, float calibPitch,
                           boolean leadSprite, boolean guardrail, int haze) {
        if (!EGL14.eglMakeCurrent(display, surface, surface, context)) {
            return false;
        }
        long timestamp = scene.optLong("t", 0L);
        float geometryAlpha = smoothingAlpha(timestamp);
        Line rawPath = decode(scene.optJSONArray("path"), 1f);
        if (rawPath.count < 2) {
            return false;
        }
        Line path = smoothLine(smoothedPath, rawPath, geometryAlpha);

        roadZGain = clamp(roadZPercent * 0.01f, -3f, 3f);
        float dynamicPitch = clamp(livePitch * clamp(pitchPercent * 0.01f, 0f, 2f),
                -0.05f, 0.05f);
        float pitch = clamp(calibPitch + dynamicPitch, -0.15f, 0.15f);
        horizonShift = clamp(FOCAL * (float) Math.tan(pitch), -46f, 46f);

        GLES20.glViewport(0, 0, WIDTH, HEIGHT);
        GLES20.glClearColor(Color.red(driveBg) / 255f, Color.green(driveBg) / 255f,
                Color.blue(driveBg) / 255f, 1f);
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT);

        int sky = dark ? blend(driveBg, Color.BLACK, 0.35f)
                : blend(driveBg, Color.WHITE, 0.35f);
        int ground = dark ? blend(driveBg, Color.BLACK, 0.15f)
                : blend(driveBg, Color.BLACK, 0.10f);
        drawRect(0f, 0f, WIDTH, Math.max(0f, HORIZON + horizonShift - TOP), sky);
        drawRect(0f, Math.max(0f, HORIZON + horizonShift - TOP), WIDTH, HEIGHT, ground);

        // The local vector context is deliberately below the camera-observed
        // model road, lanes, route and cars. GPS error therefore cannot move
        // any driving-critical overlay.
        drawMapContext(scene, path, dark);

        Line rawLeftEdge = null;
        Line rawRightEdge = null;
        JSONArray edges = scene.optJSONArray("edges");
        if (edges != null) {
            for (int i = 0; i < edges.length(); i++) {
                JSONObject edgeObject = edges.optJSONObject(i);
                if (edgeObject == null) {
                    continue;
                }
                float confidence = (float) edgeObject.optDouble("c", 0d);
                Line edge = decode(edgeObject.optJSONArray("p"), confidence);
                if (edge.count < 2) {
                    continue;
                }
                if (edge.y[0] > 0f && rawLeftEdge == null) {
                    rawLeftEdge = edge;
                } else if (edge.y[0] <= 0f && rawRightEdge == null) {
                    rawRightEdge = edge;
                }
            }
        }
        Line leftEdge = smoothOptional(smoothedEdges[0], rawLeftEdge,
                geometryAlpha, 0.28f);
        Line rightEdge = smoothOptional(smoothedEdges[1], rawRightEdge,
                geometryAlpha, 0.28f);

        drawRoad(path, leftEdge, rightEdge, scene, blend(roadTop, roadBottom, 0.55f));
        if (leftEdge != null) {
            drawRoadEdge(leftEdge, path, dark);
        }
        if (rightEdge != null) {
            drawRoadEdge(rightEdge, path, dark);
        }
        if (guardrail) {
            // 도로경계 위에 세운다. 차선·경로보다 먼저 그려 뒤로 물러나게 한다.
            drawGuardrail(leftEdge, path, dark, 1f);
            drawGuardrail(rightEdge, path, dark, -1f);
        }

        // TMAP never changes road or lane geometry.  It is only a gated,
        // translucent intent trace and becomes visible where a turn/exit
        // diverges from the model path.
        Line route = navigationRoute(scene, path, geometryAlpha);
        if (route != null) {
            int routeColor = dark ? Color.rgb(54, 218, 178)
                    : Color.rgb(0, 153, 123);
            int routeShadow = dark ? Color.rgb(18, 30, 29)
                    : Color.rgb(65, 92, 86);
            drawWorldLine(route, path, routeShadow, 7.0f, 0.38f, 0.028f);
            drawWorldLine(route, path, routeColor, 3.4f, 0.66f, 0.065f);
        }

        JSONArray lanes = scene.optJSONArray("lanes");
        for (int i = 0; i < smoothedLanes.length; i++) {
            JSONObject laneObject = lanes == null ? null : lanes.optJSONObject(i);
            float rawConfidence = laneObject == null ? 0f
                    : (float) laneObject.optDouble("c", 0d);
            Line rawLane = laneObject == null ? null
                    : decode(laneObject.optJSONArray("p"), rawConfidence);
            boolean fresh = smoothedLanes[i].count < 2;
            if (rawLane != null && rawLane.count >= 2) {
                smoothLine(smoothedLanes[i], rawLane, geometryAlpha);
            }
            laneConfidence[i] = fresh ? rawConfidence
                    : laneConfidence[i] * 0.68f + rawConfidence * 0.32f;

            boolean allowed = laneAllowed(scene, i)
                    && laneInsideRoadEdges(smoothedLanes[i], leftEdge, rightEdge);
            if (!allowed) {
                laneVisible[i] = false;
            } else if (laneVisible[i]) {
                laneVisible[i] = laneConfidence[i] >= 0.28f;
            } else {
                laneVisible[i] = laneConfidence[i] >= 0.56f;
            }
            if (!laneVisible[i] || smoothedLanes[i].count < 2) {
                continue;
            }

            int laneColor = (i == 1 || i == 2)
                    ? (dark ? Color.rgb(246, 206, 92) : Color.rgb(238, 196, 70))
                    : (dark ? Color.rgb(220, 226, 232) : Color.rgb(249, 250, 250));
            drawLaneMarking(smoothedLanes[i], path, laneColor,
                    i == 1 || i == 2 ? 3.0f : 2.2f,
                    clamp(0.22f + laneConfidence[i] * 0.78f, 0f, 1f), dark);
        }

        if (enabled) {
            // Use the already-rendered lead state to taper the path before the
            // car ahead.  No extra radar/model work or EON telemetry is needed.
            float pathEnd = leadSeen[0]
                    ? Math.max(3f, leadDistance[0] - 2.6f)
                    : Float.POSITIVE_INFINITY;
            drawPathLayers(path, pathColor, pathEnd);
            drawDesiredDistance(scene, path, dark);
        }
        drawBsd(scene);
        drawLead(scene.optJSONObject("lead2"), path, 1, dark, true, timestamp, leadSprite);
        drawLead(scene.optJSONObject("lead"), path, 0, dark, false, timestamp, leadSprite);
        // 헤이즈는 맨 마지막. 지평선 근처만 덮으므로 근경에는 영향이 없다.
        drawHaze(sky, haze);
        // glReadPixels already waits for rendering completion.  Avoid a second
        // full GPU/CPU synchronization immediately before it.
        copyPixels();
        int glError = GLES20.glGetError();
        if (glError != GLES20.GL_NO_ERROR) {
            throw new IllegalStateException("GL render/readback error=0x"
                    + Integer.toHexString(glError));
        }
        return true;
    }

    private Line navigationRoute(JSONObject scene, Line modelPath, float alpha) {
        if (scene.optInt("hudNavRoute", 1) == 0) {
            smoothedRoute.count = 0;
            return null;
        }
        JSONObject navi = scene.optJSONObject("navi");
        JSONObject naviScene = navi == null ? null : navi.optJSONObject("scene");
        if (navi == null || !navi.optBoolean("active", false) || naviScene == null) {
            smoothedRoute.count = 0;
            return null;
        }
        Line raw = decode(naviScene.optJSONArray("curve"), 1f);
        if (raw.count < 2) {
            smoothedRoute.count = 0;
            return null;
        }

        // A stale GPS/heading frame can place the map trace on a neighboring
        // road.  Require agreement close to the car, but allow the far trace
        // to diverge so genuine turns and exits remain visible.
        float checkX = clamp(Math.max(12f, raw.x[0] + 6f), 12f, 32f);
        float nearMismatch = Math.abs(yAt(raw, checkX) - yAt(modelPath, checkX));
        if (!Float.isFinite(nearMismatch) || nearMismatch > 2.2f) {
            smoothedRoute.count = 0;
            return null;
        }
        return smoothLine(smoothedRoute, raw, clamp(alpha, 0.28f, 0.55f));
    }

    private float smoothingAlpha(long timestamp) {
        if (previousSceneTimestamp == Long.MIN_VALUE || timestamp <= previousSceneTimestamp
                || timestamp - previousSceneTimestamp > 600L) {
            previousSceneTimestamp = timestamp;
            return 1f;
        }
        float alpha = clamp((timestamp - previousSceneTimestamp) / 300f, 0.30f, 0.62f);
        previousSceneTimestamp = timestamp;
        return alpha;
    }

    private static Line smoothLine(Line target, Line sample, float alpha) {
        if (target.count != sample.count || target.count < 2) {
            target.count = sample.count;
            for (int i = 0; i < sample.count; i++) {
                target.x[i] = sample.x[i];
                target.y[i] = sample.y[i];
                target.z[i] = sample.z[i];
            }
        } else {
            for (int i = 0; i < sample.count; i++) {
                target.x[i] += (sample.x[i] - target.x[i]) * alpha;
                target.y[i] += (sample.y[i] - target.y[i]) * alpha;
                target.z[i] += (sample.z[i] - target.z[i]) * alpha;
            }
        }
        target.confidence = sample.confidence;
        return target;
    }

    private static Line smoothOptional(Line target, Line sample, float alpha,
                                       float minimumConfidence) {
        if (sample != null && sample.count >= 2) {
            boolean fresh = target.count < 2;
            float previousConfidence = target.confidence;
            smoothLine(target, sample, alpha);
            target.confidence = fresh ? sample.confidence
                    : previousConfidence * 0.68f + sample.confidence * 0.32f;
        } else {
            target.confidence *= 0.68f;
        }
        return target.count >= 2 && target.confidence >= minimumConfidence ? target : null;
    }

    private static boolean laneAllowed(JSONObject scene, int index) {
        if (index == 1 || index == 2) {
            return true;
        }
        JSONObject position = scene.optJSONObject("lanePosition");
        if (position == null || position.optDouble("confidence", 0d) < 0.40d) {
            return false;
        }
        int count = position.optInt("n", 0);
        int current = position.optInt("cur", 0);
        if (count < 1 || current < 1 || current > count) {
            return false;
        }
        return index == 0 ? current > 1 : index == 3 && current < count;
    }

    /**
     * Reject model lane markings that sit outside the camera-observed road
     * boundaries.  A single noisy point is tolerated; two valid near/mid
     * samples must be outside by more than 0.45 m before hiding the line.
     */
    private static boolean laneInsideRoadEdges(Line lane, Line leftEdge, Line rightEdge) {
        if (lane == null || lane.count < 2) {
            return false;
        }
        int valid = 0;
        int outside = 0;
        for (float x : ROAD_EDGE_SAMPLE_XS) {
            float laneY = yAt(lane, x);
            if (laneY >= 0f && leftEdge != null && leftEdge.count >= 2) {
                valid++;
                if (laneY > yAt(leftEdge, x) + 0.45f) {
                    outside++;
                }
            } else if (laneY < 0f && rightEdge != null && rightEdge.count >= 2) {
                valid++;
                if (laneY < yAt(rightEdge, x) - 0.45f) {
                    outside++;
                }
            }
        }
        return valid < 2 || outside < 2;
    }

    private void drawLead(JSONObject lead, Line roadHeight, int index,
                          boolean dark, boolean secondary, long timestamp,
                          boolean sprite) {
        if (index < 0 || index >= leadSeen.length) {
            return;
        }
        if (index < leadSpriteValid.length) {
            leadSpriteValid[index] = false;
        }

        boolean live = lead != null;
        if (live) {
            float rawDistance = clamp((float) lead.optDouble("d", 0d), 0f, 180f);
            float rawLateral = clamp((float) lead.optDouble("y", 0d), -12f, 12f);
            if (rawDistance < 2f) {
                leadSeen[index] = false;
                return;
            }
            if (!leadSeen[index] || Math.abs(rawDistance - leadDistance[index]) > 18f) {
                leadDistance[index] = rawDistance;
                leadLateral[index] = rawLateral;
                leadSeen[index] = true;
            } else {
                leadDistance[index] += (rawDistance - leadDistance[index]) * 0.46f;
                leadLateral[index] += (rawLateral - leadLateral[index]) * 0.42f;
            }
            leadAcceleration[index] = (float) lead.optDouble("a", 0d);
            leadLastSeenTimestamp[index] = timestamp;
        } else {
            long age = timestamp - leadLastSeenTimestamp[index];
            if (!leadSeen[index] || timestamp <= 0L || age < 0L || age > 350L) {
                leadSeen[index] = false;
                return;
            }
        }

        float holdAlpha = live ? 1f : clamp(
                1f - (timestamp - leadLastSeenTimestamp[index]) / 350f, 0f, 1f);
        float distance = leadDistance[index];
        float z = zAt(roadHeight, distance) * roadZGain + 0.12f;
        if (!project(distance, leadLateral[index], z, projected)) {
            return;
        }
        float sx = projected[0];
        float sy = projected[1] - TOP;
        if (sx < -60f || sx > WIDTH + 60f || sy < -30f || sy > HEIGHT + 40f) {
            return;
        }
        float scale = FOCAL / (distance + CAM_BACK);
        float width = clamp(1.88f * scale, secondary ? 8f : 10f, secondary ? 38f : 44f);
        float height = clamp(0.86f * scale, secondary ? 5f : 7f, secondary ? 18f : 25f);
        int shadow = dark ? Color.rgb(4, 7, 10) : Color.rgb(58, 63, 68);
        int body = secondary
                ? (dark ? Color.rgb(116, 128, 140) : Color.rgb(150, 158, 166))
                : (dark ? Color.rgb(220, 229, 237) : Color.rgb(246, 248, 250));
        drawScreenRect(sx - width * 0.58f, sy - height * 0.08f,
                sx + width * 0.58f, sy + height * 0.20f, shadow,
                (secondary ? 0.45f : 0.68f) * holdAlpha);
        if (sprite && index < leadSpriteValid.length) {
            // 그림자만 GL 로 깔고 차체는 Canvas 가 그린다. 폭은 GL 박스와 같은
            // 원근 계산을 쓰므로 거리에 따른 축소가 그대로 유지된다.
            leadSpriteX[index] = sx;
            leadSpriteY[index] = sy + TOP;
            // 1.34 배는 근거리에서 자차(78px)와 거의 같은 크기가 되어 과했다.
            leadSpriteW[index] = width * 1.05f;
            leadSpriteAlpha[index] = (secondary ? 0.72f : 1f) * holdAlpha;
            leadSpriteBraking[index] = leadAcceleration[index] < -0.45f;
            leadSpriteValid[index] = true;
            return;
        }
        drawScreenRect(sx - width * 0.50f, sy - height,
                sx + width * 0.50f, sy, body,
                (secondary ? 0.62f : 0.94f) * holdAlpha);
        drawScreenRect(sx - width * 0.28f, sy - height * 0.82f,
                sx + width * 0.28f, sy - height * 0.48f,
                dark ? Color.rgb(29, 40, 51) : Color.rgb(53, 66, 78),
                (secondary ? 0.55f : 0.90f) * holdAlpha);
        if (leadAcceleration[index] < -0.45f) {
            int brake = Color.rgb(255, 55, 62);
            drawScreenRect(sx - width * 0.40f, sy - height * 0.25f,
                    sx - width * 0.18f, sy - height * 0.05f,
                    brake, 0.96f * holdAlpha);
            drawScreenRect(sx + width * 0.18f, sy - height * 0.25f,
                    sx + width * 0.40f, sy - height * 0.05f,
                    brake, 0.96f * holdAlpha);
        }
    }

    private void drawScreenRect(float left, float top, float right, float bottom,
                                int color, float alpha) {
        int v = 0;
        v = addTriangle(v, left, top, left, bottom, right, top);
        v = addTriangle(v, right, top, left, bottom, right, bottom);
        drawVertices(GLES20.GL_TRIANGLES, v / 2, color, alpha);
    }

    private void drawRoad(Line path, Line left, Line right, JSONObject scene, int color) {
        int count;
        if (left != null && right != null) {
            count = Math.min(left.count, right.count);
            int v = 0;
            for (int i = 0; i < count && v + 4 <= vertices.length; i++) {
                float x = (left.x[i] + right.x[i]) * 0.5f;
                float z = zAt(path, x) * roadZGain;
                float ly = Math.max(left.y[i], right.y[i]);
                float ry = Math.min(left.y[i], right.y[i]);
                if (!project(x, ly, z, projected)) continue;
                vertices[v++] = ndcX(projected[0]);
                vertices[v++] = ndcY(projected[1] - TOP);
                if (!project(x, ry, z, projected)) {
                    v -= 2;
                    continue;
                }
                vertices[v++] = ndcX(projected[0]);
                vertices[v++] = ndcY(projected[1] - TOP);
            }
            drawVertices(GLES20.GL_TRIANGLE_STRIP, v / 2, color, 1f);
            return;
        }

        float laneWidth = clamp((float) scene.optDouble("laneWidth", 3.5d), 2.2f, 4.2f);
        JSONObject lanePosition = scene.optJSONObject("lanePosition");
        int laneCount = lanePosition == null ? 1 : Math.max(1, lanePosition.optInt("n", 1));
        float half = clamp(laneWidth * laneCount * 0.5f + 0.65f, 2.2f, 10f);
        int v = 0;
        for (int i = 0; i < path.count && v + 4 <= vertices.length; i++) {
            float z = path.z[i] * roadZGain;
            if (!project(path.x[i], path.y[i] + half, z, projected)) continue;
            vertices[v++] = ndcX(projected[0]);
            vertices[v++] = ndcY(projected[1] - TOP);
            if (!project(path.x[i], path.y[i] - half, z, projected)) {
                v -= 2;
                continue;
            }
            vertices[v++] = ndcX(projected[0]);
            vertices[v++] = ndcY(projected[1] - TOP);
        }
        drawVertices(GLES20.GL_TRIANGLE_STRIP, v / 2, color, 1f);
    }

    private void drawMapContext(JSONObject scene, Line roadHeight, boolean dark) {
        JSONArray pose = scene.optJSONArray("mapPose");
        if (pose == null || pose.length() < 3) {
            return;
        }
        double lat = pose.optDouble(0, Double.NaN);
        double lon = pose.optDouble(1, Double.NaN);
        double heading = pose.optDouble(2, Double.NaN);
        if (!Double.isFinite(lat) || !Double.isFinite(lon) || !Double.isFinite(heading)
                || lat < -85.0 || lat > 85.0 || lon < -180.0 || lon > 180.0) {
            return;
        }
        mapStore.update(lat, lon);
        HudMapStore.Snapshot snapshot = mapStore.snapshot();
        if (snapshot == HudMapStore.Snapshot.EMPTY) {
            return;
        }
        double headingRad = Math.toRadians(heading);
        double sinHeading = Math.sin(headingRad);
        double cosHeading = Math.cos(headingRad);
        double metersLat = 111320.0;
        double metersLon = metersLat * Math.max(0.1, Math.cos(Math.toRadians(lat)));

        // Draw green first so lakes and rivers remain visible inside parks.
        drawMapAreas(snapshot.greens, lat, lon, metersLat, metersLon,
                sinHeading, cosHeading, roadHeight,
                dark ? Color.rgb(45, 76, 60) : Color.rgb(128, 167, 133), 0.68f);
        drawMapAreas(snapshot.waters, lat, lon, metersLat, metersLon,
                sinHeading, cosHeading, roadHeight,
                dark ? Color.rgb(40, 73, 91) : Color.rgb(117, 166, 187), 0.76f);

        int roadColor = dark ? Color.rgb(64, 74, 84) : Color.rgb(126, 138, 148);
        for (HudMapStore.Road road : snapshot.roads) {
            if (!fillLocalLine(road.lat, road.lon, lat, lon, metersLat, metersLon,
                    sinHeading, cosHeading, 2)) {
                continue;
            }
            drawWorldRibbon(mapLine, roadHeight, roadColor,
                    clamp(road.width * 0.5f, 1.25f, 9f), dark ? 0.34f : 0.52f, 0.008f);
        }

        int walls = dark ? Color.rgb(80, 92, 105) : Color.rgb(112, 126, 140);
        int roofs = dark ? Color.rgb(104, 116, 130) : Color.rgb(150, 164, 176);
        int visible = 0;
        // The loader sorts near-to-far. Select the nearest visible blocks first;
        // selecting from the array tail would discard the buildings around the car.
        for (int i = 0; i < snapshot.buildings.length
                && visible < visibleMapBuildings.length; i++) {
            HudMapStore.Building building = snapshot.buildings[i];
            if (!fillLocalLine(building.lat, building.lon, lat, lon, metersLat, metersLon,
                    sinHeading, cosHeading, 3) || !mapFeatureVisible(mapLine)) {
                continue;
            }
            visibleMapBuildings[visible++] = i;
        }
        // Draw the selected near set far-to-near so nearer blocks cover distant
        // ones without allocating a depth buffer or per-frame collections.
        for (int selected = visible - 1; selected >= 0; selected--) {
            HudMapStore.Building building = snapshot.buildings[visibleMapBuildings[selected]];
            if (!fillLocalLine(building.lat, building.lon, lat, lon, metersLat, metersLon,
                    sinHeading, cosHeading, 3)) {
                continue;
            }
            drawMapBuilding(mapLine, roadHeight, building.height, walls, roofs);
        }
    }

    private void drawMapAreas(HudMapStore.Area[] areas,
                              double lat, double lon, double metersLat, double metersLon,
                              double sinHeading, double cosHeading, Line roadHeight,
                              int color, float alpha) {
        int visible = 0;
        // Areas are sorted near-to-far. Select the nearest visible set first.
        for (int i = 0; i < areas.length && visible < visibleMapAreas.length; i++) {
            HudMapStore.Area area = areas[i];
            if (!fillLocalLine(area.lat, area.lon, lat, lon, metersLat, metersLon,
                    sinHeading, cosHeading, 3) || !mapFeatureVisible(mapLine)) {
                continue;
            }
            visibleMapAreas[visible++] = i;
        }
        // Paint the selected near set far-to-near for stable overlap.
        for (int selected = visible - 1; selected >= 0; selected--) {
            HudMapStore.Area area = areas[visibleMapAreas[selected]];
            if (!fillLocalLine(area.lat, area.lon, lat, lon, metersLat, metersLon,
                    sinHeading, cosHeading, 3)) {
                continue;
            }
            Line clipped = clipMapArea(mapLine);
            if (clipped.count >= 3) drawMapArea(clipped, roadHeight, color, alpha);
        }
    }

    private final Line mapClipA = new Line();
    private final Line mapClipB = new Line();

    /** Clip large polygons to the forward HUD viewport before projection. */
    private Line clipMapArea(Line source) {
        clipMapBoundary(source, mapClipA, true, -1.5f, true);
        clipMapBoundary(mapClipA, mapClipB, true, 175f, false);
        clipMapBoundary(mapClipB, mapClipA, false, -72f, true);
        clipMapBoundary(mapClipA, mapClipB, false, 72f, false);
        return mapClipB;
    }

    private static void clipMapBoundary(Line source, Line output, boolean useX,
                                        float boundary, boolean keepGreater) {
        output.count = 0;
        if (source.count < 3) return;
        int previous = source.count - 1;
        boolean previousInside = mapPointInside(source, previous, useX, boundary, keepGreater);
        for (int current = 0; current < source.count; current++) {
            boolean currentInside = mapPointInside(source, current, useX, boundary, keepGreater);
            if (currentInside != previousInside) {
                float previousValue = useX ? source.x[previous] : source.y[previous];
                float currentValue = useX ? source.x[current] : source.y[current];
                float denominator = currentValue - previousValue;
                float ratio = Math.abs(denominator) < 1e-6f
                        ? 0f : (boundary - previousValue) / denominator;
                appendMapPoint(output,
                        source.x[previous] + ratio * (source.x[current] - source.x[previous]),
                        source.y[previous] + ratio * (source.y[current] - source.y[previous]));
            }
            if (currentInside) appendMapPoint(output, source.x[current], source.y[current]);
            previous = current;
            previousInside = currentInside;
        }
    }

    private static boolean mapPointInside(Line line, int index, boolean useX,
                                          float boundary, boolean keepGreater) {
        float value = useX ? line.x[index] : line.y[index];
        return keepGreater ? value >= boundary : value <= boundary;
    }

    private static void appendMapPoint(Line line, float x, float y) {
        if (line.count >= MAX_POINTS) return;
        int index = line.count++;
        line.x[index] = x;
        line.y[index] = y;
        line.z[index] = 0f;
    }

    private boolean fillLocalLine(double[] latitudes, double[] longitudes,
                                  double lat0, double lon0,
                                  double metersLat, double metersLon,
                                  double sinHeading, double cosHeading,
                                  int minimum) {
        mapLine.count = 0;
        int count = Math.min(Math.min(latitudes.length, longitudes.length), MAX_POINTS);
        for (int i = 0; i < count; i++) {
            double east = (longitudes[i] - lon0) * metersLon;
            double north = (latitudes[i] - lat0) * metersLat;
            float forward = (float) (east * sinHeading + north * cosHeading);
            float lateral = (float) (north * sinHeading - east * cosHeading);
            if (!Float.isFinite(forward) || !Float.isFinite(lateral)) {
                continue;
            }
            int n = mapLine.count++;
            mapLine.x[n] = forward;
            mapLine.y[n] = lateral;
            mapLine.z[n] = 0f;
        }
        return mapLine.count >= minimum;
    }

    private static boolean mapFeatureVisible(Line line) {
        float minX = Float.POSITIVE_INFINITY;
        float maxX = Float.NEGATIVE_INFINITY;
        float minY = Float.POSITIVE_INFINITY;
        float maxY = Float.NEGATIVE_INFINITY;
        for (int i = 0; i < line.count; i++) {
            minX = Math.min(minX, line.x[i]);
            maxX = Math.max(maxX, line.x[i]);
            minY = Math.min(minY, line.y[i]);
            maxY = Math.max(maxY, line.y[i]);
        }
        return maxX >= -14f && minX <= 175f && maxY >= -72f && minY <= 72f;
    }

    private void drawMapArea(Line area, Line roadHeight, int color, float alpha) {
        int count = area.count;
        if (count < 3) return;
        for (int i = 0; i < count; i++) {
            float z = zAt(roadHeight, Math.max(0f, area.x[i])) * roadZGain + 0.004f;
            if (!project(area.x[i], area.y[i], z, projected)) return;
            mapBaseX[i] = projected[0];
            mapBaseY[i] = projected[1] - TOP;
            mapIndices[i] = i;
        }

        float signedArea = 0f;
        for (int i = 0; i < count; i++) {
            int next = (i + 1) % count;
            signedArea += area.x[i] * area.y[next] - area.x[next] * area.y[i];
        }
        float orientation = signedArea >= 0f ? 1f : -1f;
        int remaining = count;
        int vertex = 0;
        int guard = count * count;
        while (remaining > 2 && guard-- > 0 && vertex + 6 <= vertices.length) {
            boolean clipped = false;
            for (int position = 0; position < remaining; position++) {
                int previous = mapIndices[(position + remaining - 1) % remaining];
                int current = mapIndices[position];
                int next = mapIndices[(position + 1) % remaining];
                float cross = triangleCross(area.x[previous], area.y[previous],
                        area.x[current], area.y[current], area.x[next], area.y[next]);
                if (cross * orientation <= 0.0001f) continue;
                boolean contains = false;
                for (int testPosition = 0; testPosition < remaining; testPosition++) {
                    int test = mapIndices[testPosition];
                    if (test == previous || test == current || test == next) continue;
                    if (pointInTriangle(area.x[test], area.y[test],
                            area.x[previous], area.y[previous], area.x[current], area.y[current],
                            area.x[next], area.y[next])) {
                        contains = true;
                        break;
                    }
                }
                if (contains) continue;
                vertex = addTriangle(vertex, mapBaseX[previous], mapBaseY[previous],
                        mapBaseX[current], mapBaseY[current], mapBaseX[next], mapBaseY[next]);
                for (int shift = position; shift + 1 < remaining; shift++) {
                    mapIndices[shift] = mapIndices[shift + 1];
                }
                remaining--;
                clipped = true;
                break;
            }
            if (!clipped) break;
        }
        if (vertex > 0) drawVertices(GLES20.GL_TRIANGLES, vertex / 2, color, alpha);
    }

    private static float triangleCross(float ax, float ay, float bx, float by,
                                       float cx, float cy) {
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    }

    private static boolean pointInTriangle(float px, float py,
                                           float ax, float ay, float bx, float by,
                                           float cx, float cy) {
        float ab = triangleCross(ax, ay, bx, by, px, py);
        float bc = triangleCross(bx, by, cx, cy, px, py);
        float ca = triangleCross(cx, cy, ax, ay, px, py);
        boolean negative = ab < 0f || bc < 0f || ca < 0f;
        boolean positive = ab > 0f || bc > 0f || ca > 0f;
        return !(negative && positive);
    }

    private void drawMapBuilding(Line footprint, Line roadHeight, float height,
                                 int wallColor, int roofColor) {
        int count = footprint.count;
        if (count < 3) return;
        for (int i = 0; i < count; i++) {
            float baseZ = zAt(roadHeight, Math.max(0f, footprint.x[i])) * roadZGain;
            if (!project(footprint.x[i], footprint.y[i], baseZ, projected)) return;
            mapBaseX[i] = projected[0];
            mapBaseY[i] = projected[1] - TOP;
            if (!project(footprint.x[i], footprint.y[i], baseZ + height, projected)) return;
            lineScreenX[i] = projected[0];
            lineScreenY[i] = projected[1] - TOP;
        }

        int v = 0;
        for (int i = 0; i < count && v + 12 <= vertices.length; i++) {
            int next = (i + 1) % count;
            v = addTriangle(v, mapBaseX[i], mapBaseY[i], mapBaseX[next], mapBaseY[next],
                    lineScreenX[i], lineScreenY[i]);
            v = addTriangle(v, lineScreenX[i], lineScreenY[i], mapBaseX[next], mapBaseY[next],
                    lineScreenX[next], lineScreenY[next]);
        }
        drawVertices(GLES20.GL_TRIANGLES, v / 2, wallColor, 0.84f);

        v = 0;
        for (int i = 1; i + 1 < count && v + 6 <= vertices.length; i++) {
            v = addTriangle(v, lineScreenX[0], lineScreenY[0],
                    lineScreenX[i], lineScreenY[i],
                    lineScreenX[i + 1], lineScreenY[i + 1]);
        }
        drawVertices(GLES20.GL_TRIANGLES, v / 2, roofColor, 0.90f);
    }

    private void drawPathLayers(Line path, int color, float maxX) {
        drawPathLayer(path, blend(color, Color.BLACK, 0.58f), 0.70f,
                0.015f, 0.52f, 0.18f, maxX);
        drawPathLayer(path, color, 0.52f, 0.040f, 0.88f, 0.16f, maxX);
        drawPathLayer(path, blend(color, Color.WHITE, 0.52f), 0.12f,
                0.070f, 0.94f, 0.04f, maxX);
    }

    private void drawPathLayer(Line path, int color, float baseHalfWidth,
                               float zOffset, float alpha, float farGrowth,
                               float maxX) {
        int v = 0;
        for (int i = 0; i < path.count && v + 4 <= vertices.length; i++) {
            if (path.x[i] > maxX) {
                break;
            }
            float half = baseHalfWidth + Math.min(farGrowth, path.x[i] * 0.002f);
            float z = path.z[i] * roadZGain + zOffset;
            if (!project(path.x[i], path.y[i] + half, z, projected)) continue;
            vertices[v++] = ndcX(projected[0]);
            vertices[v++] = ndcY(projected[1] - TOP);
            if (!project(path.x[i], path.y[i] - half, z, projected)) {
                v -= 2;
                continue;
            }
            vertices[v++] = ndcX(projected[0]);
            vertices[v++] = ndcY(projected[1] - TOP);
        }
        drawVertices(GLES20.GL_TRIANGLE_STRIP, v / 2, color, alpha);
    }

    private void drawDesiredDistance(JSONObject scene, Line path, boolean dark) {
        float distance = (float) scene.optDouble("desiredDistance", -1d);
        if (!Float.isFinite(distance) || distance < 2f || distance > 150f) {
            return;
        }
        float laneWidth = clamp((float) scene.optDouble("laneWidth", 3.5d), 2.2f, 4.2f);
        float center = yAt(path, distance);
        float halfWidth = laneWidth * 0.48f;
        float halfDepth = 0.18f;
        float z = zAt(path, distance) * roadZGain + 0.090f;
        if (!projectQuadPoint(0, distance - halfDepth, center + halfWidth, z)
                || !projectQuadPoint(2, distance - halfDepth, center - halfWidth, z)
                || !projectQuadPoint(4, distance + halfDepth, center - halfWidth, z)
                || !projectQuadPoint(6, distance + halfDepth, center + halfWidth, z)) {
            return;
        }
        int v = 0;
        v = addTriangle(v, worldQuad[0], worldQuad[1], worldQuad[2], worldQuad[3],
                worldQuad[6], worldQuad[7]);
        v = addTriangle(v, worldQuad[6], worldQuad[7], worldQuad[2], worldQuad[3],
                worldQuad[4], worldQuad[5]);
        drawVertices(GLES20.GL_TRIANGLES, v / 2,
                dark ? Color.rgb(245, 80, 218) : Color.rgb(202, 24, 173), 0.94f);
    }

    private boolean projectQuadPoint(int offset, float x, float y, float z) {
        if (!project(x, y, z, projected)) {
            return false;
        }
        worldQuad[offset] = projected[0];
        worldQuad[offset + 1] = projected[1] - TOP;
        return true;
    }

    /**
     * BSD 경고. 좌/우 유무만 오므로 자차 뒤범퍼 모서리에서 퍼지는 파동
     * 아크와 경고 삼각형으로 그린다. 순정 계기판과 같은 표현이라 옆차의
     * 앞뒤 위치를 몰라도 어색하지 않다.
     */
    private void drawBsd(JSONObject scene) {
        if (scene == null) {
            return;
        }
        if (scene.optBoolean("leftBsd", false)) {
            bsdWarning(1);
        }
        if (scene.optBoolean("rightBsd", false)) {
            bsdWarning(-1);
        }
    }

    /** side 1 = 좌측, -1 = 우측(세로축 대칭). */
    private void bsdWarning(int side) {
        float cx = CX - side * EGO_SPRITE_W * 0.5f;
        float cy = EGO_BASELINE - TOP - BSD_CORNER_DY;
        // 우측은 좌측 각도를 세로축 기준으로 뒤집는다: theta -> 180 - theta.
        float a0 = side > 0 ? BSD_ARC_A0 : 180f - BSD_ARC_A0;
        float a1 = side > 0 ? BSD_ARC_A1 : 180f - BSD_ARC_A1;

        for (float[] layer : BSD_ARC_LAYERS) {
            float extraWidth = layer[0];
            float alphaWeight = layer[1];
            for (int chunk = 0; chunk < BSD_ARC_CHUNKS; chunk++) {
                float u0 = (float) chunk / BSD_ARC_CHUNKS;
                float u1 = (float) (chunk + 1) / BSD_ARC_CHUNKS;
                boolean middle = chunk == BSD_ARC_CHUNKS / 2;
                float chunkAlpha = middle ? 1f : BSD_ARC_CHUNK_FADE;
                int v = 0;
                for (int i = 0; i < BSD_ARC_RADII.length; i++) {
                    v = bsdArcChunk(v, cx, cy, BSD_ARC_RADII[i], a0, a1, u0, u1,
                            (7f - i) + extraWidth);
                }
                drawVertices(GLES20.GL_TRIANGLES, v / 2, BSD_COLOR,
                        BSD_CORE_ALPHA * alphaWeight * chunkAlpha);
            }
        }
        bsdTriangle(side, cx);
    }

    /**
     * 아크 한 조각을 사각 띠로 쌓는다. 굵기는 양 끝에서 0 으로 좁혀
     * 끝이 뾰족하게 사라지도록 한다.
     */
    private int bsdArcChunk(int v, float cx, float cy, float radius,
                            float a0, float a1, float u0, float u1, float width) {
        int steps = Math.max(2, Math.round(BSD_ARC_SEGMENTS * (u1 - u0)));
        float prevX = 0f;
        float prevY = 0f;
        float prevHalf = 0f;
        for (int i = 0; i <= steps; i++) {
            float u = u0 + (u1 - u0) * i / steps;
            double theta = Math.toRadians(a0 + (a1 - a0) * u);
            float x = cx + radius * (float) Math.cos(theta);
            float y = cy + radius * (float) Math.sin(theta) * BSD_ARC_SQUASH;
            // 끝단 테이퍼. sin 곡선이라 중앙이 가장 굵다.
            float half = width * 0.5f * (float) Math.pow(Math.sin(Math.PI * u), 0.6);
            if (i > 0 && v + 12 <= vertices.length) {
                float dx = x - prevX;
                float dy = y - prevY;
                float length = (float) Math.sqrt(dx * dx + dy * dy);
                if (length >= 0.05f) {
                    float nx = -dy / length;
                    float ny = dx / length;
                    v = addTriangle(v,
                            prevX + nx * prevHalf, prevY + ny * prevHalf,
                            prevX - nx * prevHalf, prevY - ny * prevHalf,
                            x + nx * half, y + ny * half);
                    v = addTriangle(v,
                            x + nx * half, y + ny * half,
                            prevX - nx * prevHalf, prevY - ny * prevHalf,
                            x - nx * half, y - ny * half);
                }
            }
            prevX = x;
            prevY = y;
            prevHalf = half;
        }
        return v;
    }

    /** 아크 바깥쪽 경고 삼각형. 테두리 + 느낌표를 한 배치로 그린다. */
    private void bsdTriangle(int side, float arcCx) {
        float cx = arcCx - side * BSD_TRI_DX;
        float cy = EGO_BASELINE - TOP - BSD_TRI_DY;
        float half = BSD_TRI_SIZE * 0.5f;
        float h = BSD_TRI_SIZE * 0.88f;
        float topY = cy - h * 0.5f;
        float bottomY = cy + h * 0.5f;
        int v = 0;
        v = bsdStroke(v, cx, topY, cx - half, bottomY);
        v = bsdStroke(v, cx - half, bottomY, cx + half, bottomY);
        v = bsdStroke(v, cx + half, bottomY, cx, topY);
        v = bsdStroke(v, cx, cy - h * 0.16f, cx, cy + h * 0.12f);
        float dot = BSD_TRI_STROKE * 0.6f;
        float dotY = cy + h * 0.26f;
        if (v + 12 <= vertices.length) {
            v = addTriangle(v, cx - dot, dotY - dot, cx + dot, dotY - dot,
                    cx - dot, dotY + dot);
            v = addTriangle(v, cx + dot, dotY - dot, cx + dot, dotY + dot,
                    cx - dot, dotY + dot);
        }
        drawVertices(GLES20.GL_TRIANGLES, v / 2, BSD_COLOR, BSD_TRI_ALPHA);
    }

    private int bsdStroke(int v, float ax, float ay, float bx, float by) {
        float dx = bx - ax;
        float dy = by - ay;
        float length = (float) Math.sqrt(dx * dx + dy * dy);
        if (length < 0.05f || v + 12 > vertices.length) {
            return v;
        }
        float nx = -dy / length * BSD_TRI_STROKE * 0.5f;
        float ny = dx / length * BSD_TRI_STROKE * 0.5f;
        v = addTriangle(v, ax + nx, ay + ny, ax - nx, ay - ny, bx + nx, by + ny);
        v = addTriangle(v, bx + nx, by + ny, ax - nx, ay - ny, bx - nx, by - ny);
        return v;
    }

    /**
     * 앞차 그림을 얹을 자리. out = {중심 x, 접지 y, 폭} (캔버스 좌표).
     * 이번 프레임에 앞차를 그리지 않았으면 false.
     */
    boolean leadSprite(int index, float[] out) {
        if (index < 0 || index >= leadSpriteValid.length || !leadSpriteValid[index]) {
            return false;
        }
        out[0] = leadSpriteX[index];
        out[1] = leadSpriteY[index];
        out[2] = leadSpriteW[index];
        return true;
    }

    float leadSpriteAlpha(int index) {
        return index >= 0 && index < leadSpriteAlpha.length ? leadSpriteAlpha[index] : 0f;
    }

    boolean leadSpriteBraking(int index) {
        return index >= 0 && index < leadSpriteBraking.length && leadSpriteBraking[index];
    }

    /**
     * 가드레일. 모델 도로경계(edges) 위치를 그대로 쓰므로 없는 곳에는 서지
     * 않는다. 레일 띠 한 줄과 일정 간격 기둥으로만 구성한다.
     */
    private void drawGuardrail(Line edge, Line roadHeight, boolean dark, float side) {
        if (edge == null || edge.count < 2) {
            return;
        }
        int railColor = dark ? Color.rgb(122, 134, 146) : Color.rgb(178, 186, 193);
        int postColor = dark ? Color.rgb(74, 84, 94) : Color.rgb(146, 154, 161);

        int v = 0;
        for (int i = 0; i < edge.count && v + 4 <= vertices.length; i++) {
            float x = edge.x[i];
            if (x < 0f) {
                continue;
            }
            if (x > RAIL_MAX_X) {
                break;
            }
            float y = edge.y[i] + side * RAIL_INSET;
            float base = zAt(roadHeight, x) * roadZGain;
            if (!project(x, y, base + RAIL_TOP, projected)) {
                continue;
            }
            vertices[v++] = ndcX(projected[0]);
            vertices[v++] = ndcY(projected[1] - TOP);
            if (!project(x, y, base + RAIL_BOTTOM, projected)) {
                v -= 2;
                continue;
            }
            vertices[v++] = ndcX(projected[0]);
            vertices[v++] = ndcY(projected[1] - TOP);
        }
        drawVertices(GLES20.GL_TRIANGLE_STRIP, v / 2, railColor, 0.90f);

        // 기둥. 화면 폭은 거리에 따라 줄이되 1px 아래로는 내리지 않는다.
        v = 0;
        float first = edge.x[0] < 0f ? 0f : edge.x[0];
        for (float x = first + RAIL_POST_SPACING; x <= RAIL_MAX_X; x += RAIL_POST_SPACING) {
            if (v + 12 > vertices.length) {
                break;
            }
            float y = yAt(edge, x) + side * RAIL_INSET;
            float base = zAt(roadHeight, x) * roadZGain;
            if (!project(x, y, base + RAIL_POST_TOP, projected)) {
                continue;
            }
            float topX = projected[0];
            float topY = projected[1] - TOP;
            if (!project(x, y, base, projected)) {
                continue;
            }
            float half = Math.max(0.6f, Math.min(3.4f,
                    FOCAL / (x + CAM_BACK) * 0.055f));
            float bottomX = projected[0];
            float bottomY = projected[1] - TOP;
            v = addTriangle(v, topX - half, topY, topX + half, topY,
                    bottomX - half, bottomY);
            v = addTriangle(v, topX + half, topY, bottomX - half, bottomY,
                    bottomX + half, bottomY);
        }
        drawVertices(GLES20.GL_TRIANGLES, v / 2, postColor, 0.82f);
    }

    /**
     * 원경 헤이즈. 지평선 바로 아래를 하늘색으로 옅게 덮어 멀리 있는 노면과
     * 차선이 배경으로 녹아들게 한다. 밴드 알파를 계단식으로 줄여 그린다.
     */
    private void drawHaze(int sky, int strengthPercent) {
        float strength = clamp(strengthPercent * 0.01f, 0f, 1f);
        if (strength <= 0.01f) {
            return;
        }
        float horizon = HORIZON + horizonShift - TOP;
        float band = HAZE_DEPTH_PX / HAZE_BANDS;
        for (int i = 0; i < HAZE_BANDS; i++) {
            float top = horizon + i * band;
            if (top >= HEIGHT) {
                break;
            }
            float alpha = strength * (1f - i / (float) HAZE_BANDS);
            drawRect(0f, Math.max(0f, top), WIDTH,
                    Math.min(HEIGHT, top + band), sky, alpha * 0.9f);
        }
    }

    private void drawRoadEdge(Line edge, Line roadHeight, boolean dark) {
        int shadow = dark ? Color.rgb(18, 24, 31) : Color.rgb(92, 99, 105);
        int body = dark ? Color.rgb(126, 139, 153) : Color.rgb(152, 161, 168);
        int crest = dark ? Color.rgb(205, 216, 226) : Color.rgb(225, 230, 234);
        drawWorldLine(edge, roadHeight, shadow, 6.0f, 0.62f, 0.024f);
        drawWorldLine(edge, roadHeight, body, 3.5f, 0.94f, 0.055f);
        drawWorldLine(edge, roadHeight, crest, 1.2f, 0.92f, 0.086f);
    }

    private void drawLaneMarking(Line lane, Line roadHeight, int color,
                                 float widthPx, float alpha, boolean dark) {
        int border = dark ? Color.rgb(28, 34, 40) : Color.rgb(105, 110, 114);
        drawWorldLine(lane, roadHeight, border, widthPx + 3.0f,
                alpha * 0.66f, 0.030f);
        drawWorldLine(lane, roadHeight, color, widthPx, alpha, 0.052f);
    }

    private void drawWorldLine(Line line, Line roadHeight, int color,
                               float widthPx, float alpha) {
        drawWorldLine(line, roadHeight, color, widthPx, alpha, 0.045f);
    }

    private void drawWorldLine(Line line, Line roadHeight, int color,
                               float widthPx, float alpha, float zOffset) {
        if (line == null || line.count < 2) {
            return;
        }
        int count = 0;
        for (int i = 0; i < line.count && count < MAX_POINTS; i++) {
            float z = zAt(roadHeight, line.x[i]) * roadZGain + zOffset;
            if (project(line.x[i], line.y[i], z, projected)) {
                lineScreenX[count] = projected[0];
                lineScreenY[count] = projected[1] - TOP;
                count++;
            }
        }
        int v = 0;
        for (int i = 0; i + 1 < count && v + 12 <= vertices.length; i++) {
            float dx = lineScreenX[i + 1] - lineScreenX[i];
            float dy = lineScreenY[i + 1] - lineScreenY[i];
            float length = (float) Math.sqrt(dx * dx + dy * dy);
            if (length < 0.01f) continue;
            float nx = -dy / length * widthPx * 0.5f;
            float ny = dx / length * widthPx * 0.5f;
            v = addTriangle(v, lineScreenX[i] + nx, lineScreenY[i] + ny,
                    lineScreenX[i] - nx, lineScreenY[i] - ny,
                    lineScreenX[i + 1] + nx, lineScreenY[i + 1] + ny);
            v = addTriangle(v, lineScreenX[i + 1] + nx, lineScreenY[i + 1] + ny,
                    lineScreenX[i] - nx, lineScreenY[i] - ny,
                    lineScreenX[i + 1] - nx, lineScreenY[i + 1] - ny);
        }
        drawVertices(GLES20.GL_TRIANGLES, v / 2, color, alpha);
    }

    /**
     * Draws a ribbon with a width measured in world meters. Map roads depend on
     * this helper even though the navigation route uses pixel-width lines again.
     */
    private void drawWorldRibbon(Line line, Line roadHeight, int color,
                                 float halfWidth, float alpha, float zOffset) {
        if (line == null || line.count < 2) {
            return;
        }
        int v = 0;
        for (int i = 0; i < line.count && v + 4 <= vertices.length; i++) {
            int a = Math.max(0, i - 1);
            int b = Math.min(line.count - 1, i + 1);
            float tx = line.x[b] - line.x[a];
            float ty = line.y[b] - line.y[a];
            float length = (float) Math.sqrt(tx * tx + ty * ty);
            if (length < 1e-4f) {
                continue;
            }
            float nx = -ty / length * halfWidth;
            float ny = tx / length * halfWidth;
            float z = zAt(roadHeight, line.x[i]) * roadZGain + zOffset;
            if (!project(line.x[i] + nx, line.y[i] + ny, z, projected)) {
                continue;
            }
            vertices[v++] = ndcX(projected[0]);
            vertices[v++] = ndcY(projected[1] - TOP);
            if (!project(line.x[i] - nx, line.y[i] - ny, z, projected)) {
                v -= 2;
                continue;
            }
            vertices[v++] = ndcX(projected[0]);
            vertices[v++] = ndcY(projected[1] - TOP);
        }
        drawVertices(GLES20.GL_TRIANGLE_STRIP, v / 2, color, alpha);
    }

    private int addTriangle(int index, float ax, float ay, float bx, float by,
                            float cx, float cy) {
        vertices[index++] = ndcX(ax);
        vertices[index++] = ndcY(ay);
        vertices[index++] = ndcX(bx);
        vertices[index++] = ndcY(by);
        vertices[index++] = ndcX(cx);
        vertices[index++] = ndcY(cy);
        return index;
    }

    private void drawRect(float left, float top, float right, float bottom, int color) {
        drawRect(left, top, right, bottom, color, 1f);
    }

    private void drawRect(float left, float top, float right, float bottom,
                          int color, float alpha) {
        int v = 0;
        v = addTriangle(v, left, top, left, bottom, right, top);
        v = addTriangle(v, right, top, left, bottom, right, bottom);
        drawVertices(GLES20.GL_TRIANGLES, v / 2, color, alpha);
    }

    private void drawVertices(int mode, int count, int color, float alpha) {
        if (count < 3) {
            return;
        }
        vertexBuffer.clear();
        vertexBuffer.put(vertices, 0, count * 2);
        vertexBuffer.position(0);
        GLES20.glUseProgram(program);
        GLES20.glVertexAttribPointer(positionHandle, 2, GLES20.GL_FLOAT,
                false, 0, vertexBuffer);
        GLES20.glUniform4f(colorHandle, Color.red(color) / 255f,
                Color.green(color) / 255f, Color.blue(color) / 255f,
                clamp(alpha, 0f, 1f));
        GLES20.glDrawArrays(mode, 0, count);
    }

    private boolean project(float x, float y, float z, float[] out) {
        float depth = x + CAM_BACK;
        if (!Float.isFinite(depth) || depth < NEAR_DEPTH) {
            return false;
        }
        float scale = FOCAL / depth;
        out[0] = CX - y * scale;
        out[1] = HORIZON + horizonShift + (CAM_H - z) * scale;
        return Float.isFinite(out[0]) && Float.isFinite(out[1]);
    }

    private static Line decode(JSONArray points, float confidence) {
        Line line = new Line();
        line.confidence = confidence;
        if (points == null) {
            return line;
        }
        int limit = Math.min(points.length(), MAX_POINTS);
        for (int i = 0; i < limit; i++) {
            JSONArray point = points.optJSONArray(i);
            if (point == null || point.length() < 2) {
                continue;
            }
            double x = point.optDouble(0, Double.NaN);
            double y = point.optDouble(1, Double.NaN);
            double z = point.length() >= 3 ? point.optDouble(2, 0d) : 0d;
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)) {
                continue;
            }
            int n = line.count++;
            line.x[n] = (float) x;
            line.y[n] = (float) y;
            line.z[n] = (float) z;
        }
        return line;
    }

    private static float yAt(Line line, float x) {
        if (line == null || line.count == 0) {
            return 0f;
        }
        if (x <= line.x[0]) {
            return line.y[0];
        }
        for (int i = 1; i < line.count; i++) {
            if (x <= line.x[i]) {
                float span = line.x[i] - line.x[i - 1];
                float t = span > 0.001f ? (x - line.x[i - 1]) / span : 0f;
                return line.y[i - 1] + (line.y[i] - line.y[i - 1]) * t;
            }
        }
        return line.y[line.count - 1];
    }

    private static float zAt(Line path, float x) {
        if (path == null || path.count == 0) {
            return 0f;
        }
        if (x <= path.x[0]) {
            return path.z[0];
        }
        for (int i = 1; i < path.count; i++) {
            if (x <= path.x[i]) {
                float span = path.x[i] - path.x[i - 1];
                float t = span > 0.001f ? (x - path.x[i - 1]) / span : 0f;
                return path.z[i - 1] + (path.z[i] - path.z[i - 1]) * t;
            }
        }
        return path.z[path.count - 1];
    }

    private void copyPixels() {
        readBuffer.position(0);
        GLES20.glReadPixels(0, 0, WIDTH, HEIGHT, GLES20.GL_RGBA,
                GLES20.GL_UNSIGNED_BYTE, readBuffer);
        for (int y = 0; y < HEIGHT; y++) {
            int source = (HEIGHT - 1 - y) * WIDTH;
            int target = y * WIDTH;
            for (int x = 0; x < WIDTH; x++) {
                int abgr = readBuffer.get(source + x);
                pixels[target + x] = (abgr & 0xff00ff00)
                        | ((abgr & 0x000000ff) << 16)
                        | ((abgr & 0x00ff0000) >>> 16);
            }
        }
        frame.setPixels(pixels, 0, WIDTH, 0, 0, WIDTH, HEIGHT);
    }

    private static float ndcX(float pixel) {
        return pixel * 2f / WIDTH - 1f;
    }

    private static float ndcY(float pixel) {
        return 1f - pixel * 2f / HEIGHT;
    }

    private static float clamp(float value, float minimum, float maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    static int blend(int a, int b, float t) {
        t = clamp(t, 0f, 1f);
        return Color.rgb(
                (int) (Color.red(a) + (Color.red(b) - Color.red(a)) * t),
                (int) (Color.green(a) + (Color.green(b) - Color.green(a)) * t),
                (int) (Color.blue(a) + (Color.blue(b) - Color.blue(a)) * t));
    }

    private void fail(String message) {
        failed = true;
        Log.e(TAG, message + " EGL error=0x" + Integer.toHexString(EGL14.eglGetError()));
        release();
    }

    void release() {
        mapStore.close();
        if (display != EGL14.EGL_NO_DISPLAY) {
            EGL14.eglMakeCurrent(display, EGL14.EGL_NO_SURFACE,
                    EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT);
            if (surface != EGL14.EGL_NO_SURFACE) {
                EGL14.eglDestroySurface(display, surface);
            }
            if (context != EGL14.EGL_NO_CONTEXT) {
                EGL14.eglDestroyContext(display, context);
            }
            EGL14.eglTerminate(display);
        }
        display = EGL14.EGL_NO_DISPLAY;
        surface = EGL14.EGL_NO_SURFACE;
        context = EGL14.EGL_NO_CONTEXT;
        program = 0;
    }
}
