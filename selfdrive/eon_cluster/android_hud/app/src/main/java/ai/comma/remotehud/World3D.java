package ai.comma.remotehud;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.LinearGradient;
import android.graphics.Matrix;
import android.graphics.Typeface;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.graphics.Shader;

import org.json.JSONArray;
import org.json.JSONObject;

/**
 * v0.20 — 실제 원근투영(핀홀 카메라) 3D 주행씬.
 *
 *   depth = X + CAM_BACK                (X: 차 기준 전방 m)
 *   u     = CX     - FOCAL * Y / depth  (Y: 좌측이 +)
 *   v     = HORIZON + FOCAL * (CAM_H - Z) / depth
 *
 * v0.19 대비 바뀐 것
 *  - 카메라를 뒤/위로 물림 (9.5→13.0m, 3.4→4.6m). 자차가 화면을 덜 먹고
 *    앞이 더 보인다.
 *  - 경로 보간을 Catmull-Rom 으로. 17점 폴리라인의 각진 곡선이 사라진다.
 *  - **정차 중 좌우 흔들림 수정.** 모델 유효구간 밖 외삽 기울기 상한을
 *    0.12→0.06 으로 낮추고, 20m에 걸쳐 0으로 감쇠시킨다. 정차하면
 *    modelV2 position 이 0 근처에만 몰려서 먼 쪽이 ±8m씩 흔들렸다.
 *  - 노면 폭을 고정 ±4.45m 가 아니라 인식된 roadEdges 로 만든다.
 *  - 도로 경계를 연석 높이로 세운다.
 *  - 다크 / 라이트 팔레트 (hudTheme 연동).
 *  - BSD 를 차량 그림 대신 옆차선 경고 띠로. openpilot 은 옆차 유무만 알고
 *    앞뒤 위치는 모르므로, 특정 지점에 차를 그리면 없는 정보를 주장하게 된다.
 */
final class World3D {

    /** 주행 패널 화면 영역 */
    static final float LEFT = 0f;
    static final float RIGHT = 952f;
    static final float TOP = 217f;
    static final float BOTTOM = 454f;
    static final float CX = 476f;

    /** 카메라 */
    static final float FOCAL = 520f;
    static final float CAM_H = 4.6f;
    static final float CAM_BACK = 13.0f;
    static final float HORIZON = 249f;
    static final float NEAR_DEPTH = 11.4f;
    static final float FAR_DEPTH = 190f;

    /** 도로 */
    static final float ROAD_HALF = 4.45f;
    static final float SHOULDER = 0.65f;
    static final float CURB_HEIGHT = 0.13f;
    static final float LANE_PAINT_W = 0.16f;
    static final float EDGE_PAINT_W = 0.20f;
    static final float DASH_PERIOD = 8.0f;
    static final float DASH_ON = 3.0f;

    /** 차량 표현 방식 */
    static final int CAR_SPRITE = 1;   // 사진 스프라이트 (기본)
    static final int CAR_BOX = 2;      // 단색 3D 박스 + 음영
    // 3D 박스 차량 크기. 실제 치수(1.86 / 1.46 / 4.6)에서 30% 줄인 값 —
    // 화면에서 차가 과하게 커 보인다는 피드백 반영. 사진 스프라이트는 별개다.
    private static final float CAR_W = 1.30f;
    private static final float CAR_H = 1.02f;
    private static final float CAR_LEN = 3.22f;

    /** BSD 표시 방식 */
    static final int BSD_BAR = 1;      // 막대만
    static final int BSD_SOFT = 2;     // 옅은 면 + 막대 (기본)
    static final int BSD_SOLID = 3;    // 진한 면 + 막대
    private static final float BSD_NEAR = -6f;
    private static final float BSD_FAR = 16f;
    private static final float BSD_INNER = 2.0f;
    private static final float BSD_OUTER = 5.2f;

    /** 건물 */
    private static final float BLOCK = 24f;
    private static final int BLOCK_COUNT = 13;
    private static final float BUILD_NEAR = 14f;
    private static final float BUILD_FAR = 170f;
    private static final float HAZE_START = 95f;

    /** 과속방지턱 노면 표시 */
    private static final float BUMP_VISIBLE_M = 60f;   // 이보다 멀면 우측 아이콘만
    private static final float BUMP_DEPTH = 1.12f;     // 진행방향 길이 (m) — 1.6에서 30% 축소
    private static final float BUMP_H = 0.29f;         // 과장한 높이 (m) — 0.42에서 30% 축소

    /** 노면 표시 축소 배율. 1.0 이 차로 폭·기본 길이. */
    private static final float SIGN_SCALE = 0.70f;

    private static final int SLICES = 30;
    private static final int MAX_PTS = 80;

    // ── 프레임당 재할당을 피하기 위한 버퍼 ────────────────────────────────
    private final float[] pathX = new float[MAX_PTS];
    private final float[] pathY = new float[MAX_PTS];
    /**
     * 노면 높낮이 (m, 위가 +). modelV2.position.z 를 그대로 받는다.
     * project() 가 모든 점에 이 값을 더하므로 노면·차선·연석·앞차·BSD 가
     * 한꺼번에 오르막/내리막을 탄다. 별도 호출부 수정이 필요 없다.
     */
    private final float[] pathZ = new float[MAX_PTS];
    /** 노면 높낮이 배율. 0 이면 평지, 음수면 위아래 반전. */
    private float roadZGain = 1f;
    /** 모델이 추정한 자기 차로 폭(m). 0 이면 예전 상수(3.5m)를 쓴다. */
    private float laneWidthM = 0f;
    /** 신호/E2E 정지선까지 거리(m). 음수면 안 그린다. */
    private float stopDist = -1f;
    private int pathCount;
    /** true when packet.path is the final lateral MPC path, not modelV2. */
    private boolean finalPath;

    private final float[] edgeLX = new float[MAX_PTS];
    private final float[] edgeLY = new float[MAX_PTS];
    private int edgeLCount;
    private final float[] edgeRX = new float[MAX_PTS];
    private final float[] edgeRY = new float[MAX_PTS];
    private int edgeRCount;

    // 자차 차선 (modelV2 laneLines index 1=좌, 2=우). 그리기용으로만 쓰던 것을
    // 노면 표시(제한속도/방지턱)의 폭을 맞추기 위해 샘플 배열로도 보관한다.
    private final float[] laneLX = new float[MAX_PTS];
    private final float[] laneLY = new float[MAX_PTS];
    private int laneLCount;
    private final float[] laneRX = new float[MAX_PTS];
    private final float[] laneRY = new float[MAX_PTS];
    private int laneRCount;

    // 노면 제한속도 타일. 값이 바뀔 때만 다시 만든다(텍스트 그리기가 렌더 비용의
    // 대부분이라 매 프레임 그리면 안 된다).
    private Bitmap limitTile;
    private int limitTileValue = -1;
    private boolean limitTileDark;
    private final Matrix roadMatrix = new Matrix();
    private final float[] polySrc = new float[8];
    private final float[] polyDst = new float[8];

    private final float[] pa = new float[2];
    private final float[] pb = new float[2];
    private final float[] pc = new float[2];
    private final float[] pd = new float[2];

    private final float[] lx = new float[SLICES];
    private final float[] ly = new float[SLICES];
    private final float[] rx = new float[SLICES];
    private final float[] ry = new float[SLICES];

    private final Path poly = new Path();
    private final Path poly2 = new Path();
    private final RectF rect = new RectF();

    private LinearGradient roadShader;
    private int roadShaderTop;
    private int roadShaderBottom;
    private LinearGradient hazeShader;
    private int hazeShaderColor;
    private LinearGradient ribbonShader;
    private LinearGradient ribbonStrokeShader;
    private int ribbonShaderColor;

    private boolean dark;
    private int carStyle = CAR_SPRITE;

    /**
     * lateral_planner 가 최종 경로에 더하는 OffsetTotal (m). 경로 리본에만
     * 적용한다 — 차선·도로경계는 모델 원본이고 오프셋이 안 들어가 있다.
     */
    private float pathOffset = 0f;

    /**
     * liveCalibration pitch (rad). 카메라가 실제로 얼마나 아래를 보는지에
     * 맞춰 수평선을 옮긴다. HORIZON 은 상수지만 실제 EON 카메라는 캘리브에
     * 따라 다르므로, 이 값만큼 보정하면 이온 화면과 세로 구도가 가까워진다.
     */
    private float horizonShift = 0f;

    // ── 티맵 차로/경로 (packet.navi.scene) ────────────────────────────
    //  lane_current 원본: count/current_lane/turn_info[]/available[]/distance_m/road_category
    //  curve: 티맵 route.polyline 을 자차 기준 로컬좌표(전방 x, 좌 +y)로 변환한 것.
    private static final float TMAP_LANE_W = 3.5f;
    private static final float CURVE_BLEND_START = 60f;
    private static final float CURVE_BLEND_SPAN = 60f;
    private static final int TMAP_MAX_LANES = 8;
    private int tmapLaneCount = 0;
    private int tmapLaneCur = 0;
    private int tmapRoadCat = -1;
    private final int[] tmapTurns = new int[TMAP_MAX_LANES];
    private final int[] tmapAvail = new int[TMAP_MAX_LANES];
    private final float[] curveX = new float[32];
    private final float[] curveY = new float[32];
    private int curveCount = 0;

    /** 티맵 씬 데이터 주입. null 이면 전부 해제(안내 종료). */
    void setNavi(JSONObject scene) {
        tmapLaneCount = 0;
        tmapLaneCur = 0;
        tmapRoadCat = -1;
        curveCount = 0;
        if (scene == null) {
            return;
        }
        tmapRoadCat = scene.optInt("cat", -1);
        JSONObject lane = scene.optJSONObject("lane");
        if (lane != null) {
            int n = lane.optInt("n", 0);
            int cur = lane.optInt("cur", 0);
            if (n >= 2 && n <= TMAP_MAX_LANES && cur >= 1 && cur <= n) {
                tmapLaneCount = n;
                tmapLaneCur = cur;
                JSONArray turns = lane.optJSONArray("turns");
                JSONArray avail = lane.optJSONArray("avail");
                for (int i = 0; i < n; i++) {
                    tmapTurns[i] = turns == null ? 0 : turns.optInt(i, 0);
                    tmapAvail[i] = avail == null ? 1 : avail.optInt(i, 1);
                }
            }
        }
        JSONArray curve = scene.optJSONArray("curve");
        if (curve != null) {
            int m = Math.min(curve.length(), curveX.length);
            int k = 0;
            float lastX = -999f;
            for (int i = 0; i < m; i++) {
                JSONArray pt = curve.optJSONArray(i);
                if (pt == null) {
                    continue;
                }
                float x = (float) pt.optDouble(0, Double.NaN);
                float y = (float) pt.optDouble(1, Double.NaN);
                if (Float.isNaN(x) || Float.isNaN(y) || x <= lastX) {
                    continue;
                }
                curveX[k] = x;
                curveY[k] = y;
                lastX = x;
                k++;
            }
            curveCount = k >= 2 ? k : 0;
        }
    }

    boolean tmapHighway() {
        // roadcate 0(고속도로)/1(도시고속화도로). 실측 후 보정 여지 있음.
        return tmapRoadCat == 0 || tmapRoadCat == 1;
    }

    // ── OSM 실제 지형 (OsmWorld.snapshot) ─────────────────────────────
    private OsmWorld.Snapshot osm;

    void setOsm(OsmWorld.Snapshot snapshot) {
        osm = snapshot;
    }

    private final float[] boxX = new float[8];
    private final float[] boxY = new float[8];

    // ── 투영 ──────────────────────────────────────────────────────────────

    boolean project(float x, float y, float z, float[] out) {
        float depth = x + CAM_BACK;
        if (depth < NEAR_DEPTH) {
            return false;
        }
        float inv = FOCAL / depth;
        out[0] = CX - y * inv;
        // z 는 "노면 위 높이"로 쓴다(연석 0.13, 방지턱, 차량 박스 등).
        // 노면 자체의 오르내림은 roadZ(x) 가 담당한다.
        out[1] = HORIZON + horizonShift + (CAM_H - (z + roadZ(x))) * inv;
        return true;
    }

    static float pxPerMeter(float depth) {
        return FOCAL / Math.max(NEAR_DEPTH, depth);
    }

    // ── 씬 디코딩 ─────────────────────────────────────────────────────────

    void setScene(JSONObject s) {
        pathCount = 0;
        finalPath = false;
        edgeLCount = 0;
        edgeRCount = 0;
        laneLCount = 0;
        laneRCount = 0;
        if (s == null) {
            return;
        }
        finalPath = s.optBoolean("pathFinal", false);
        pathCount = decode(s.optJSONArray("path"), pathX, pathY, pathZ);

        JSONArray lanes = s.optJSONArray("lanes");
        if (lanes != null && lanes.length() >= 3) {
            JSONObject left = lanes.optJSONObject(1);
            JSONObject right = lanes.optJSONObject(2);
            if (left != null) {
                laneLCount = decode(left.optJSONArray("p"), laneLX, laneLY);
            }
            if (right != null) {
                laneRCount = decode(right.optJSONArray("p"), laneRX, laneRY);
            }
        }

        JSONArray edges = s.optJSONArray("edges");
        if (edges == null) {
            return;
        }
        for (int i = 0; i < edges.length(); i++) {
            JSONObject e = edges.optJSONObject(i);
            if (e == null || e.optDouble("c", 0d) < 0.18d) {
                continue;
            }
            JSONArray pts = e.optJSONArray("p");
            if (pts == null || pts.length() < 2) {
                continue;
            }
            JSONArray first = pts.optJSONArray(0);
            if (first == null) {
                continue;
            }
            boolean leftSide = first.optDouble(1, 0d) > 0d;
            if (leftSide && edgeLCount == 0) {
                edgeLCount = decode(pts, edgeLX, edgeLY);
            } else if (!leftSide && edgeRCount == 0) {
                edgeRCount = decode(pts, edgeRX, edgeRY);
            }
        }
    }

    private static int decode(JSONArray a, float[] xs, float[] ys) {
        return decode(a, xs, ys, null);
    }

    private static int decode(JSONArray a, float[] xs, float[] ys, float[] zs) {
        if (a == null) {
            return 0;
        }
        int n = 0;
        int limit = Math.min(a.length(), xs.length);
        for (int i = 0; i < limit; i++) {
            JSONArray q = a.optJSONArray(i);
            if (q == null) {
                continue;
            }
            double dx = q.optDouble(0, Double.NaN);
            double dy = q.optDouble(1, Double.NaN);
            if (Double.isNaN(dx) || Double.isNaN(dy)) {
                continue;
            }
            float x = (float) dx;
            if (n > 0 && x <= xs[n - 1]) {
                continue;
            }
            xs[n] = x;
            ys[n] = (float) dy;
            if (zs != null) {
                // 3번째 원소가 없는 옛 EON 패킷과도 호환되게 기본 0.
                double dz = q.optDouble(2, 0d);
                zs[n] = Double.isNaN(dz) ? 0f : (float) dz;
            }
            n++;
        }
        return n;
    }

    /**
     * Catmull-Rom 보간. 모델 유효구간 밖에서는 기울기를 20m에 걸쳐 0으로
     * 감쇠시킨다 — 정차 중 먼 쪽이 좌우로 쓸리는 것을 막는 핵심.
     */
    /**
     * 전방 x(m) 지점의 노면 높이(m). 데이터가 없거나 배율이 0 이면 0 을 돌려
     * 예전처럼 완전 평지로 그린다. 모델이 튀어도 화면이 뒤집히지 않게 ±3m 로 자른다.
     */
    private float roadZ(float x) {
        if (pathCount < 2 || roadZGain == 0f) {
            return 0f;
        }
        float z = sample(pathX, pathZ, pathCount, x) * roadZGain;
        if (Float.isNaN(z) || Float.isInfinite(z)) {
            return 0f;
        }
        return Math.max(-3f, Math.min(3f, z));
    }

    private static float sample(float[] xs, float[] ys, int n, float x) {
        if (n == 0) {
            return 0f;
        }
        if (n == 1 || x <= xs[0]) {
            return ys[0];
        }
        int last = n - 1;
        if (x >= xs[last]) {
            float dx = xs[last] - xs[last - 1];
            float slope = dx > 0.01f ? (ys[last] - ys[last - 1]) / dx : 0f;
            slope = Math.max(-0.06f, Math.min(0.06f, slope));
            float over = x - xs[last];
            float decay = Math.max(0f, 1f - over / 20f);
            return ys[last] + slope * over * decay;
        }
        int i = 0;
        int hi = last;
        while (hi - i > 1) {
            int mid = (i + hi) >>> 1;
            if (xs[mid] <= x) {
                i = mid;
            } else {
                hi = mid;
            }
        }
        float y0 = ys[Math.max(0, i - 1)];
        float y1 = ys[i];
        float y2 = ys[i + 1];
        float y3 = ys[Math.min(last, i + 2)];
        float span = xs[i + 1] - xs[i];
        float t = span > 0.001f ? (x - xs[i]) / span : 0f;
        float t2 = t * t;
        float t3 = t2 * t;
        return 0.5f * ((2f * y1)
                + (-y0 + y2) * t
                + (2f * y0 - 5f * y1 + 4f * y2 - y3) * t2
                + (-y0 + 3f * y1 - 3f * y2 + y3) * t3);
    }

    float centerAt(float x) {
        float y = sample(pathX, pathY, pathCount, x);
        // Preserve the final MPC path over its complete valid horizon. TMAP is
        // used only beyond that horizon; otherwise ATC curvature would be
        // blended a second time from the old fixed 60 m point.
        float curveBlendStart = finalPath && pathCount >= 2
                ? Math.max(CURVE_BLEND_START, pathX[pathCount - 1])
                : CURVE_BLEND_START;
        if (curveCount >= 2 && x > curveBlendStart) {
            // 근거리는 모델, 원거리는 티맵 폴리라인. GPS 오프셋(1~5m)이 그대로
            // 보이지 않도록 블렌드 시작점에서 두 곡선의 차이를 바이어스로 빼서
            // 이어붙인다 → 형상(커브)만 가져오고 절대 오프셋은 버린다.
            float w = Math.min(1f, (x - curveBlendStart) / CURVE_BLEND_SPAN);
            float bias = sample(curveX, curveY, curveCount, curveBlendStart)
                    - sample(pathX, pathY, pathCount, curveBlendStart);
            float curveYv = sample(curveX, curveY, curveCount, x) - bias;
            y = y * (1f - w) + curveYv * w;
        }
        return y;
    }

    /** 인식된 도로경계가 있으면 그것으로, 없으면 고정 폭으로. */
    /** 전방 x(m) 에서 자차 차선의 좌/우 경계 y(m). 차선이 없으면 ±1.75m 폴백.
     *  y 는 좌측이 + 이므로 왼쪽 경계가 큰 값이다. */
    private float laneEdgeAt(float x, boolean leftSide) {
        // modelV2 laneLines 의 인덱스 순서(1=좌, 2=우)를 믿지 않는다. 순서가 뒤바뀐
        // 패킷이 오면 노면 사각형의 좌우가 맞바뀌어 글자가 거울처럼 뒤집힌다.
        // y 는 좌측이 + 이므로 큰 쪽이 항상 왼쪽 경계다.
        boolean haveL = laneLCount >= 2;
        boolean haveR = laneRCount >= 2;
        if (!haveL && !haveR) {
            return leftSide ? 1.75f : -1.75f;
        }
        float a = haveL ? sample(laneLX, laneLY, laneLCount, x) : -1.75f;
        float b = haveR ? sample(laneRX, laneRY, laneRCount, x) : 1.75f;
        float hi = Math.max(a, b);
        float lo = Math.min(a, b);
        // 한쪽만 잡혔거나 두 선이 붙어 버린 경우 최소 차로폭을 확보한다.
        if (hi - lo < 2.0f) {
            float mid = (hi + lo) * 0.5f;
            hi = mid + 1.55f;
            lo = mid - 1.55f;
        }
        return leftSide ? hi : lo;
    }


    // ── 노면 표시 (제한속도 / 과속방지턱) ──────────────────────────────────

    private float laneMid(float x) {
        return (laneEdgeAt(x, true) + laneEdgeAt(x, false)) * 0.5f;
    }

    private float laneHalf(float x) {
        return (laneEdgeAt(x, true) - laneEdgeAt(x, false)) * 0.5f;
    }

    /** 지면 사각형(전방 x0~x1, 좌우 경계는 자차 차선)에 비트맵을 원근 매핑한다. */
    private boolean groundQuad(float x0, float x1, float widthScale, float[] out) {
        float l1 = laneMid(x1) + laneHalf(x1) * widthScale;
        float r1 = laneMid(x1) - laneHalf(x1) * widthScale;
        float l0 = laneMid(x0) + laneHalf(x0) * widthScale;
        float r0 = laneMid(x0) - laneHalf(x0) * widthScale;
        if (!project(x1, l1, 0f, pa)) return false;
        if (!project(x1, r1, 0f, pb)) return false;
        if (!project(x0, r0, 0f, pc)) return false;
        if (!project(x0, l0, 0f, pd)) return false;
        out[0] = pa[0]; out[1] = pa[1];   // 먼쪽 좌
        out[2] = pb[0]; out[3] = pb[1];   // 먼쪽 우
        out[4] = pc[0]; out[5] = pc[1];   // 가까운쪽 우
        out[6] = pd[0]; out[7] = pd[1];   // 가까운쪽 좌
        return true;
    }

    private Bitmap limitTile(int limit, int color) {
        if (limitTile != null && limitTileValue == limit && limitTileDark == dark) {
            return limitTile;
        }
        if (limitTile != null) {
            limitTile.recycle();
        }
        int n = 420;
        Bitmap bmp = Bitmap.createBitmap(n, n, Bitmap.Config.ARGB_8888);
        Canvas tc = new Canvas(bmp);
        Paint tp = new Paint(Paint.ANTI_ALIAS_FLAG);
        tp.setColor(color);
        tp.setStyle(Paint.Style.STROKE);
        tp.setStrokeWidth(30f);
        tc.drawCircle(n * 0.5f, n * 0.5f, n * 0.5f - 22f, tp);
        tp.setStyle(Paint.Style.FILL);
        tp.setTypeface(Typeface.create("sans", Typeface.BOLD));
        tp.setTextAlign(Paint.Align.CENTER);
        String txt = Integer.toString(limit);
        tp.setTextSize(txt.length() < 3 ? 200f : 158f);
        Paint.FontMetrics fm = tp.getFontMetrics();
        tc.drawText(txt, n * 0.5f, n * 0.5f - (fm.ascent + fm.descent) * 0.5f, tp);
        limitTile = bmp;
        limitTileValue = limit;
        limitTileDark = dark;
        return bmp;
    }

    /** 제한속도를 자차 차선 노면에 눕혀 그린다(흰 원 테두리 + 숫자).
     *  원이 화면에서 정원으로 보이려면 진행방향 길이를 폭보다 늘려야 한다. */
    private void drawLimitPaint(Canvas c, Paint p, int limit, float leadDist, float bumpDist) {
        if (limit <= 0 || limit > 200) {
            return;
        }
        float x0 = 7f;
        float x1 = x0 + 20f * SIGN_SCALE;   // 길이도 30% 축소 (20m -> 14m)
        if (leadDist > 0f) {
            x1 = Math.min(x1, leadDist - 6f);
        }
        // 방지턱이 이 구간 안에 있으면 그 앞에서 끊는다. 둘이 겹치면 둘 다 못 읽는다.
        if (bumpDist > 0f && bumpDist <= BUMP_VISIBLE_M) {
            x1 = Math.min(x1, bumpDist - BUMP_DEPTH * 0.5f - 2.5f);
        }
        if (x1 - x0 < 10f * SIGN_SCALE) {
            return;
        }
        if (!groundQuad(x0, x1, SIGN_SCALE, polyDst)) {
            return;
        }
        Bitmap tile = limitTile(limit, dark ? Color.rgb(238, 242, 245) : Color.rgb(70, 76, 84));
        int n = tile.getWidth();
        polySrc[0] = 0f;  polySrc[1] = 0f;
        polySrc[2] = n;   polySrc[3] = 0f;
        polySrc[4] = n;   polySrc[5] = n;
        polySrc[6] = 0f;  polySrc[7] = n;
        roadMatrix.reset();
        if (!roadMatrix.setPolyToPoly(polySrc, 0, polyDst, 0, 4)) {
            return;
        }
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColorFilter(null);
        p.setAlpha(dark ? 225 : 205);
        p.setFilterBitmap(true);
        c.drawBitmap(tile, roadMatrix, p);
        p.setAlpha(255);
    }

    /** 과속방지턱. 앞면(밝음) + 윗면(어두움) 두 면을 가진 낮은 입체로 그린다.
     *  실제 방지턱 높이 0.1m 는 원근에 눌려 안 보이므로 과장한다. */
    private void drawSpeedBump(Canvas c, Paint p, float dist) {
        if (dist <= 0f || dist > BUMP_VISIBLE_M) {
            return;
        }
        float x0 = dist - BUMP_DEPTH * 0.5f;
        float x1 = dist + BUMP_DEPTH * 0.5f;
        if (x1 + CAM_BACK < NEAR_DEPTH + 0.5f) {
            return;
        }
        x0 = Math.max(x0, NEAR_DEPTH - CAM_BACK + 0.2f);
        float yl0 = laneMid(x0) + laneHalf(x0) * SIGN_SCALE;
        float yr0 = laneMid(x0) - laneHalf(x0) * SIGN_SCALE;
        float yl1 = laneMid(x1) + laneHalf(x1) * SIGN_SCALE;
        float yr1 = laneMid(x1) - laneHalf(x1) * SIGN_SCALE;

        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setAlpha(245);
        // 윗면 (어둡게)
        bumpFace(c, p, x0, yl0, yr0, x1, yl1, yr1, BUMP_H, BUMP_H, 0.62f);
        // 앞면 (밝게) — 가까운 쪽 수직면
        bumpFace(c, p, x0, yl0, yr0, x0, yl0, yr0, 0f, BUMP_H, 1.0f);
        p.setAlpha(255);
    }

    /** 방지턱 한 면을 노랑/검정 줄무늬로 분할해 그린다. */
    private void bumpFace(Canvas c, Paint p, float xa, float yla, float yra,
                          float xb, float ylb, float yrb, float za, float zb, float shade) {
        int n = 8;
        for (int i = 0; i < n; i++) {
            float t0 = i / (float) n;
            float t1 = (i + 1) / (float) n;
            if (!project(xa, yla + (yra - yla) * t0, za, pa)) continue;
            if (!project(xa, yla + (yra - yla) * t1, za, pb)) continue;
            if (!project(xb, ylb + (yrb - ylb) * t1, zb, pc)) continue;
            if (!project(xb, ylb + (yrb - ylb) * t0, zb, pd)) continue;
            int base = (i % 2 == 0) ? Color.rgb(240, 192, 32) : Color.rgb(40, 42, 46);
            p.setColor(shade >= 0.99f ? base : blend(base, Color.BLACK, 1f - shade));
            poly2.reset();
            poly2.moveTo(pa[0], pa[1]);
            poly2.lineTo(pb[0], pb[1]);
            poly2.lineTo(pc[0], pc[1]);
            poly2.lineTo(pd[0], pd[1]);
            poly2.close();
            c.drawPath(poly2, p);
        }
    }

    private float roadEdgeAt(float x, boolean leftSide) {
        if (leftSide && edgeLCount >= 2) {
            return sample(edgeLX, edgeLY, edgeLCount, x);
        }
        if (!leftSide && edgeRCount >= 2) {
            return sample(edgeRX, edgeRY, edgeRCount, x);
        }
        // 모델이 준 실제 차로 폭이 있으면 그걸 쓴다. 좁은 골목과 넓은 간선이
        // 전부 같은 폭으로 보이던 문제를 없앤다. 값이 튀면 화면이 출렁이므로
        // 2.2~4.2m 로 자른다.
        float laneW = laneWidthM > 0.1f
                ? Math.max(2.2f, Math.min(4.2f, laneWidthM)) : TMAP_LANE_W;
        float half = (laneWidthM > 0.1f ? laneW * 1.27f : ROAD_HALF) + SHOULDER;
        if (tmapLaneCount >= 2) {
            int lanesOnSide = leftSide ? Math.max(0, tmapLaneCur - 1)
                    : Math.max(0, tmapLaneCount - tmapLaneCur);
            half = laneW * 0.5f + laneW * lanesOnSide + SHOULDER;
        }
        return centerAt(x) + (leftSide ? half : -half);
    }

    // ── 씬 전체 ───────────────────────────────────────────────────────────

    void draw(Canvas c, Paint p, JSONObject s, boolean enabled,
              Bitmap egoCar, Bitmap otherCar, float odoM,
              int bgColor, int roadTop, int roadBottom, int pathColor,
              int radarInfo, boolean buildings, boolean darkTheme, int bsdStyle,
              int carStyleMode, float offsetTotal, float calibPitch,
              boolean limitPaint, boolean bumpPaint,
              float roadZPercent, float livePitch, float pitchPercent,
              float laneWidthMeters, float stopDistM) {
        this.dark = darkTheme;
        this.carStyle = carStyleMode == CAR_BOX ? CAR_BOX : CAR_SPRITE;
        this.pathOffset = Math.max(-1f, Math.min(1f, offsetTotal));
        this.roadZGain = Math.max(-3f, Math.min(3f, roadZPercent * 0.01f));
        this.laneWidthM = laneWidthMeters;
        this.stopDist = stopDistM;
        // 정적 캘리브 pitch + 주행 중 실시간 pitch(게인 적용). 실시간 항은
        // 따로 ±0.05rad 로 자른다 — 게인을 크게 줘도 화면이 뒤집히지 않게.
        float dynPitch = Math.max(-0.05f, Math.min(0.05f,
                livePitch * Math.max(0f, Math.min(2f, pitchPercent * 0.01f))));
        float pitch = Math.max(-0.15f, Math.min(0.15f, calibPitch + dynPitch));
        // pitch 가 아래를 볼수록(양수) 수평선이 화면 아래로 내려간다.
        this.horizonShift = Math.max(-46f, Math.min(46f,
                FOCAL * (float) Math.tan(pitch)));
        setScene(s);

        int save = c.save();
        c.clipRect(LEFT, TOP, RIGHT, BOTTOM);

        int sky = dark ? blend(bgColor, Color.BLACK, 0.35f) : blend(bgColor, Color.WHITE, 0.35f);
        int ground = dark ? blend(bgColor, Color.BLACK, 0.15f) : blend(bgColor, Color.BLACK, 0.10f);

        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(sky);
        c.drawRect(LEFT, TOP, RIGHT, HORIZON, p);
        p.setColor(ground);
        c.drawRect(LEFT, HORIZON, RIGHT, BOTTOM, p);

        boolean highway = tmapHighway();
        drawRoad(c, p, roadTop, roadBottom);
        drawCurb(c, p, true, roadTop);
        drawCurb(c, p, false, roadTop);
        if (osm != null) {
            drawOsmRoads(c, p, roadTop);
        }
        drawMarkings(c, p, s);
        drawTmapLanes(c, p);
        if (highway) {
            drawGuardrail(c, p, true, roadTop);
            drawGuardrail(c, p, false, roadTop);
        }

        if (limitPaint && s != null) {
            JSONObject leadForPaint = s.optJSONObject("lead");
            float leadD = leadForPaint == null ? -1f : (float) leadForPaint.optDouble("d", -1d);
            drawLimitPaint(c, p, s.optInt("limit", 0), leadD,
                    (float) s.optDouble("bumpDist", -1d));
        }
        if (enabled) {
            drawPathRibbon(c, p, pathColor);
        }
        if (buildings && !highway) {
            if (osm != null && osm.buildingCount > 0) {
                drawOsmBuildings(c, p, sky);
            } else {
                drawBuildings(c, p, odoM, sky);
            }
        }
        if (bumpPaint && s != null) {
            drawSpeedBump(c, p, (float) s.optDouble("bumpDist", -1d));
        }

        drawBsd(c, p, s, bsdStyle);
        drawStopLine(c, p);
        drawVehicles(c, p, s, egoCar, otherCar, radarInfo);
        drawHaze(c, p, sky);

        p.setShader(null);
        c.restoreToCount(save);
    }

    // ── 노면 ──────────────────────────────────────────────────────────────

    private void drawRoad(Canvas c, Paint p, int topColor, int bottomColor) {
        float invNear = 1f / NEAR_DEPTH;
        float invFar = 1f / FAR_DEPTH;
        for (int i = 0; i < SLICES; i++) {
            float t = i / (SLICES - 1f);
            float depth = 1f / (invNear + (invFar - invNear) * t);
            float x = depth - CAM_BACK;
            float inv = FOCAL / depth;
            float vy = HORIZON + horizonShift + CAM_H * inv;
            lx[i] = CX - roadEdgeAt(x, true) * inv;
            ly[i] = vy;
            rx[i] = CX - roadEdgeAt(x, false) * inv;
            ry[i] = vy;
        }

        if (roadShader == null || roadShaderTop != topColor || roadShaderBottom != bottomColor) {
            roadShader = new LinearGradient(0f, HORIZON, 0f, BOTTOM, topColor, bottomColor,
                    Shader.TileMode.CLAMP);
            roadShaderTop = topColor;
            roadShaderBottom = bottomColor;
        }

        poly.rewind();
        poly.moveTo(lx[0], ly[0]);
        for (int i = 1; i < SLICES; i++) {
            poly.lineTo(lx[i], ly[i]);
        }
        for (int i = SLICES - 1; i >= 0; i--) {
            poly.lineTo(rx[i], ry[i]);
        }
        poly.close();
        p.setStyle(Paint.Style.FILL);
        p.setShader(roadShader);
        c.drawPath(poly, p);
        p.setShader(null);
    }

    /** 도로 경계를 연석 높이로 세워 평면감을 없앤다. */
    private void drawCurb(Canvas c, Paint p, boolean leftSide, int roadColor) {
        int face = dark ? blend(roadColor, Color.BLACK, 0.35f) : blend(roadColor, Color.BLACK, 0.22f);
        int lip = dark ? blend(roadColor, Color.WHITE, 0.22f) : blend(roadColor, Color.WHITE, 0.45f);
        float invNear = 1f / NEAR_DEPTH;
        float invFar = 1f / (FAR_DEPTH * 0.55f);
        boolean have = false;
        float prevBaseX = 0f, prevBaseY = 0f, prevTopX = 0f, prevTopY = 0f;

        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        for (int i = 0; i < SLICES; i++) {
            float t = i / (SLICES - 1f);
            float depth = 1f / (invNear + (invFar - invNear) * t);
            float x = depth - CAM_BACK;
            float y = roadEdgeAt(x, leftSide);
            if (!project(x, y, 0f, pa) || !project(x, y, CURB_HEIGHT, pb)) {
                have = false;
                continue;
            }
            if (have) {
                poly2.rewind();
                poly2.moveTo(prevBaseX, prevBaseY);
                poly2.lineTo(pa[0], pa[1]);
                poly2.lineTo(pb[0], pb[1]);
                poly2.lineTo(prevTopX, prevTopY);
                poly2.close();
                p.setColor(face);
                c.drawPath(poly2, p);
                p.setColor(lip);
                c.drawRect(Math.min(prevTopX, pb[0]), Math.min(prevTopY, pb[1]) - 1.4f,
                        Math.max(prevTopX, pb[0]), Math.min(prevTopY, pb[1]), p);
            }
            prevBaseX = pa[0];
            prevBaseY = pa[1];
            prevTopX = pb[0];
            prevTopY = pb[1];
            have = true;
        }
    }

    // ── 차선 / 도로 경계 ──────────────────────────────────────────────────

    private void drawMarkings(Canvas c, Paint p, JSONObject s) {
        if (s == null) {
            return;
        }
        int edgeColor = dark ? Color.rgb(120, 132, 146) : Color.rgb(148, 157, 163);
        JSONArray edges = s.optJSONArray("edges");
        if (edges != null) {
            for (int i = 0; i < edges.length(); i++) {
                JSONObject e = edges.optJSONObject(i);
                if (e == null || e.optDouble("c", 0d) < 0.18d) {
                    continue;
                }
                polyline(c, p, e.optJSONArray("p"), edgeColor, EDGE_PAINT_W, false, 255);
            }
        }
        JSONArray lanes = s.optJSONArray("lanes");
        if (lanes == null) {
            return;
        }
        int plain = dark ? Color.rgb(214, 222, 230) : Color.rgb(248, 250, 250);
        int ego = dark ? Color.rgb(246, 206, 92) : Color.rgb(238, 196, 70);
        for (int i = 0; i < lanes.length(); i++) {
            JSONObject lane = lanes.optJSONObject(i);
            if (lane == null) {
                continue;
            }
            double conf = lane.optDouble("c", 1d);
            int alpha = (int) Math.max(70d, Math.min(255d, 90d + conf * 170d));
            boolean isEgo = (lanes.length() >= 3) && (i == 1 || i == 2);
            polyline(c, p, lane.optJSONArray("p"), isEgo ? ego : plain, LANE_PAINT_W, true, alpha);
        }
    }

    /** 티맵 차로 수 기반 추가 구분선 + 차로별 방향 화살표.
     *  모델은 자기 차로 양옆 선까지만 주므로, 그 밖의 차로는 티맵 count 로
     *  자차 경로(centerAt)에 평행 오프셋해 그린다. 좌 +y. */
    private void drawTmapLanes(Canvas c, Paint p) {
        if (tmapLaneCount < 2) {
            return;
        }
        int divider = dark ? Color.rgb(196, 205, 214) : Color.rgb(240, 243, 244);
        int leftLanes = tmapLaneCur - 1;
        int rightLanes = tmapLaneCount - tmapLaneCur;
        // k=1 은 자기 차로 경계(모델이 이미 그림)라 k=2 부터.
        for (int k = 2; k <= leftLanes; k++) {
            drawParallelDash(c, p, 1.75f + TMAP_LANE_W * (k - 1), divider);
        }
        for (int k = 2; k <= rightLanes; k++) {
            drawParallelDash(c, p, -(1.75f + TMAP_LANE_W * (k - 1)), divider);
        }
        drawLaneArrows(c, p);
    }

    private void drawParallelDash(Canvas c, Paint p, float offset, int color) {
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(color);
        p.setAlpha(150);
        for (float x = 4f; x < 120f; x += DASH_PERIOD) {
            float x2 = x + DASH_ON;
            float y1 = centerAt(x) + offset;
            float y2 = centerAt(x2) + offset;
            if (!project(x, y1, 0f, pa) || !project(x2, y2, 0f, pb)) {
                continue;
            }
            float w1 = LANE_PAINT_W * 0.5f * pxPerMeter(x + CAM_BACK);
            float w2 = LANE_PAINT_W * 0.5f * pxPerMeter(x2 + CAM_BACK);
            poly2.rewind();
            poly2.moveTo(pa[0] - w1, pa[1]);
            poly2.lineTo(pa[0] + w1, pa[1]);
            poly2.lineTo(pb[0] + w2, pb[1]);
            poly2.lineTo(pb[0] - w2, pb[1]);
            poly2.close();
            c.drawPath(poly2, p);
        }
        p.setAlpha(255);
    }

    /** 티맵 nLaneTurnInfo 코드 → 화살표 종류. 0 직진 / 1 좌 / 2 우 / 3 유턴.
     *  ※ 코드표는 잠정 매핑 — 실주행 payload 로 검증 후 보정할 것. */
    private static int arrowKindFor(int code) {
        switch (code) {
            case 2: case 5: case 7: case 12: return 1;
            case 3: case 6: case 8: case 13: return 2;
            case 4: case 14: return 3;
            default: return 0;
        }
    }

    /** 노면 화살표. 진행 가능(available) 차로는 밝게, 아니면 어둡게. */
    private void drawLaneArrows(Canvas c, Paint p) {
        float ax = 21f;
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        for (int i = 1; i <= tmapLaneCount; i++) {
            float laneY = centerAt(ax) + TMAP_LANE_W * (tmapLaneCur - i);
            boolean ok = tmapAvail[i - 1] != 0;
            int color;
            if (i == tmapLaneCur) {
                color = dark ? Color.rgb(246, 206, 92) : Color.rgb(222, 168, 32);
            } else if (ok) {
                color = dark ? Color.rgb(232, 238, 244) : Color.rgb(252, 253, 253);
            } else {
                color = dark ? Color.rgb(96, 106, 118) : Color.rgb(168, 176, 182);
            }
            p.setColor(color);
            p.setAlpha(ok ? 235 : 130);
            arrowGlyph(c, p, ax, laneY, arrowKindFor(tmapTurns[i - 1]));
        }
        p.setAlpha(255);
    }

    /** 바닥에 눕힌 화살표(도색 느낌). kind: 0 직진 / 1 좌 / 2 우 / 3 유턴. */
    private void arrowGlyph(Canvas c, Paint p, float x, float y, int kind) {
        float hw = 0.38f;
        boolean okBase = worldQuad(c, p, x - 3.0f, y - hw, x - 3.0f, y + hw,
                x + 0.8f, y + hw, x + 0.8f, y - hw);
        if (!okBase) {
            return;
        }
        if (kind == 1 || kind == 2) {
            float dir = kind == 1 ? 1f : -1f;
            // 옆으로 꺾인 짧은 몸통 + 촉
            worldQuad(c, p, x + 0.2f, y - hw * dir, x + 0.2f, y + 1.5f * dir,
                    x + 0.9f, y + 1.5f * dir, x + 0.9f, y - hw * dir);
            worldTri(c, p, x + 0.55f, y + 2.6f * dir,
                    x - 0.35f, y + 1.3f * dir, x + 1.45f, y + 1.3f * dir);
        } else if (kind == 3) {
            // 유턴 : 몸통 위 반원 느낌의 굽은 촉
            worldQuad(c, p, x + 0.6f, y - hw, x + 1.6f, y - hw + 1.0f,
                    x + 1.6f, y + 1.2f, x + 0.6f, y + 1.2f);
            worldTri(c, p, x - 0.3f, y + 1.0f, x + 0.9f, y + 0.4f, x + 0.9f, y + 1.9f);
        } else {
            worldTri(c, p, x + 3.4f, y, x + 0.5f, y - 1.25f, x + 0.5f, y + 1.25f);
        }
    }

    private boolean worldQuad(Canvas c, Paint p, float x1, float y1, float x2, float y2,
                              float x3, float y3, float x4, float y4) {
        if (!project(x1, y1, 0f, pa) || !project(x2, y2, 0f, pb)
                || !project(x3, y3, 0f, pc) || !project(x4, y4, 0f, pd)) {
            return false;
        }
        poly2.rewind();
        poly2.moveTo(pa[0], pa[1]);
        poly2.lineTo(pb[0], pb[1]);
        poly2.lineTo(pc[0], pc[1]);
        poly2.lineTo(pd[0], pd[1]);
        poly2.close();
        c.drawPath(poly2, p);
        return true;
    }

    private void worldTri(Canvas c, Paint p, float x1, float y1, float x2, float y2,
                          float x3, float y3) {
        if (!project(x1, y1, 0f, pa) || !project(x2, y2, 0f, pb)
                || !project(x3, y3, 0f, pc)) {
            return;
        }
        poly2.rewind();
        poly2.moveTo(pa[0], pa[1]);
        poly2.lineTo(pb[0], pb[1]);
        poly2.lineTo(pc[0], pc[1]);
        poly2.close();
        c.drawPath(poly2, p);
    }

    /** 고속도로 가드레일 : 도로경계 바깥 0.3m, 레일 높이 0.75m + 기둥 12m 간격. */
    private void drawGuardrail(Canvas c, Paint p, boolean leftSide, int roadColor) {
        int rail = dark ? Color.rgb(150, 160, 172) : Color.rgb(186, 193, 199);
        int post = dark ? Color.rgb(104, 114, 126) : Color.rgb(158, 166, 173);
        float side = leftSide ? 1f : -1f;
        p.setShader(null);
        p.setStyle(Paint.Style.STROKE);
        boolean have = false;
        float px0 = 0f, py0 = 0f;
        p.setColor(rail);
        for (float x = 6f; x < 150f; x += 6f) {
            float y = roadEdgeAt(x, leftSide) + side * 0.3f;
            if (!project(x, y, 0.75f, pa)) {
                have = false;
                continue;
            }
            p.setStrokeWidth(Math.max(1.5f, 0.09f * pxPerMeter(x + CAM_BACK)));
            if (have) {
                c.drawLine(px0, py0, pa[0], pa[1], p);
            }
            px0 = pa[0];
            py0 = pa[1];
            have = true;
        }
        p.setColor(post);
        for (float x = 8f; x < 110f; x += 12f) {
            float y = roadEdgeAt(x, leftSide) + side * 0.3f;
            if (!project(x, y, 0f, pa) || !project(x, y, 0.75f, pb)) {
                continue;
            }
            p.setStrokeWidth(Math.max(1.2f, 0.07f * pxPerMeter(x + CAM_BACK)));
            c.drawLine(pa[0], pa[1], pb[0], pb[1], p);
        }
        p.setStyle(Paint.Style.FILL);
    }

    /** OSM 옆길: 어두운 리본. 자차가 달리는 도로 자체(중심선과 6m 이내)는 건너뛴다
     *  — 그 도로는 이미 drawRoad 가 그렸다. */
    private void drawOsmRoads(Canvas c, Paint p, int roadColor) {
        int shade = dark ? blend(roadColor, Color.BLACK, 0.22f)
                : blend(roadColor, Color.BLACK, 0.14f);
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(shade);
        for (int r = 0; r < osm.roadCount; r++) {
            float[] xs = osm.roadX[r];
            float[] ys = osm.roadY[r];
            float halfW = osm.roadW[r] * 0.5f;
            for (int i = 0; i < xs.length - 1; i++) {
                float x1 = xs[i];
                float y1 = ys[i];
                float x2 = xs[i + 1];
                float y2 = ys[i + 1];
                if (Math.max(x1, x2) < 5f || Math.min(x1, x2) > 180f) {
                    continue;
                }
                boolean nearEgo1 = x1 > -5f && Math.abs(y1 - centerAt(Math.max(0f, x1))) < 6f;
                boolean nearEgo2 = x2 > -5f && Math.abs(y2 - centerAt(Math.max(0f, x2))) < 6f;
                if (nearEgo1 && nearEgo2) {
                    continue;
                }
                float dx = x2 - x1;
                float dy = y2 - y1;
                float len = (float) Math.sqrt(dx * dx + dy * dy);
                if (len < 0.5f) {
                    continue;
                }
                float nx = -dy / len * halfW;
                float ny = dx / len * halfW;
                worldQuad(c, p, x1 + nx, y1 + ny, x2 + nx, y2 + ny,
                        x2 - nx, y2 - ny, x1 - nx, y1 - ny);
            }
        }
    }

    /** OSM 건물: 실제 외곽선을 벽면 단위로 세운다. 카메라를 향한 변만 그리고,
     *  변의 방향에 따라 명암을 줘 입체감을 낸다. 먼 건물부터. */
    private void drawOsmBuildings(Canvas c, Paint p, int fogColor) {
        Integer[] order = new Integer[osm.buildingCount];
        float[] dist = new float[osm.buildingCount];
        for (int i = 0; i < osm.buildingCount; i++) {
            order[i] = i;
            float[] xs = osm.ringX[i];
            float sum = 0f;
            for (float x : xs) {
                sum += x;
            }
            dist[i] = sum / xs.length;
        }
        java.util.Arrays.sort(order, new java.util.Comparator<Integer>() {
            @Override
            public int compare(Integer a, Integer b) {
                return Float.compare(dist[b], dist[a]);
            }
        });

        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        for (int oi = 0; oi < order.length; oi++) {
            int b = order[oi];
            float[] xs = osm.ringX[b];
            float[] ys = osm.ringY[b];
            float height = osm.ringH[b];
            float d = Math.max(0f, dist[b]);
            float fog = d <= HAZE_START ? 0f
                    : Math.min(0.85f, (d - HAZE_START) / (BUILD_FAR - HAZE_START) * 0.95f);

            // 링 방향(부호 면적)으로 바깥 법선을 정한다.
            float area = 0f;
            int n = xs.length;
            for (int i = 0; i < n; i++) {
                int j = (i + 1) % n;
                area += xs[i] * ys[j] - xs[j] * ys[i];
            }
            float flip = area >= 0f ? 1f : -1f;

            for (int i = 0; i < n; i++) {
                int j = (i + 1) % n;
                float ex = xs[j] - xs[i];
                float ey = ys[j] - ys[i];
                float len = (float) Math.sqrt(ex * ex + ey * ey);
                if (len < 0.4f) {
                    continue;
                }
                float nx = (ey / len) * flip;
                float ny = (-ex / len) * flip;
                float mx = (xs[i] + xs[j]) * 0.5f;
                float my = (ys[i] + ys[j]) * 0.5f;
                // 카메라(로컬 -CAM_BACK, 0)를 향한 면만.
                if (nx * (-CAM_BACK - mx) + ny * (0f - my) <= 0f) {
                    continue;
                }
                // 고정 조명(좌전방)으로 명암.
                float light = Math.max(0f, nx * -0.55f + ny * 0.45f);
                int tone = dark ? (int) (56f + light * 34f) : (int) (168f + light * 40f);
                int wall = blend(Color.rgb(tone, Math.min(255, tone + 4),
                        Math.min(255, tone + 10)), fogColor, fog);
                p.setColor(wall);
                if (!project(xs[i], ys[i], 0f, pa) || !project(xs[j], ys[j], 0f, pb)
                        || !project(xs[j], ys[j], height, pc)
                        || !project(xs[i], ys[i], height, pd)) {
                    continue;
                }
                poly2.rewind();
                poly2.moveTo(pa[0], pa[1]);
                poly2.lineTo(pb[0], pb[1]);
                poly2.lineTo(pc[0], pc[1]);
                poly2.lineTo(pd[0], pd[1]);
                poly2.close();
                c.drawPath(poly2, p);
            }
        }
    }

    private void polyline(Canvas c, Paint p, JSONArray pts, int color, float widthM,
                          boolean dashed, int alpha) {
        if (pts == null || pts.length() < 2) {
            return;
        }
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(color);
        p.setAlpha(alpha);
        for (int i = 0; i < pts.length() - 1; i++) {
            JSONArray a = pts.optJSONArray(i);
            JSONArray b = pts.optJSONArray(i + 1);
            if (a == null || b == null) {
                continue;
            }
            float x1 = (float) a.optDouble(0, 0d);
            float y1 = (float) a.optDouble(1, 0d);
            float x2 = (float) b.optDouble(0, 0d);
            float y2 = (float) b.optDouble(1, 0d);
            if (x2 > FAR_DEPTH) {
                continue;
            }
            if (dashed) {
                float mid = (x1 + x2) * 0.5f;
                float phase = mid - (float) Math.floor(mid / DASH_PERIOD) * DASH_PERIOD;
                if (phase > DASH_ON) {
                    continue;
                }
            }
            if (!project(x1, y1, 0f, pa) || !project(x2, y2, 0f, pb)) {
                continue;
            }
            quad(c, p, pa, pb,
                    Math.max(1.1f, widthM * pxPerMeter(x1 + CAM_BACK)),
                    Math.max(1.0f, widthM * pxPerMeter(x2 + CAM_BACK)));
        }
        p.setAlpha(255);
    }

    private void quad(Canvas c, Paint p, float[] a, float[] b, float wa, float wb) {
        float dx = b[0] - a[0];
        float dy = b[1] - a[1];
        float len = (float) Math.hypot(dx, dy);
        if (len < 0.4f) {
            return;
        }
        float nx = -dy / len;
        float ny = dx / len;
        float ha = wa * 0.5f;
        float hb = wb * 0.5f;
        poly2.rewind();
        poly2.moveTo(a[0] + nx * ha, a[1] + ny * ha);
        poly2.lineTo(b[0] + nx * hb, b[1] + ny * hb);
        poly2.lineTo(b[0] - nx * hb, b[1] - ny * hb);
        poly2.lineTo(a[0] - nx * ha, a[1] - ny * ha);
        poly2.close();
        c.drawPath(poly2, p);
    }

    // ── 주행 경로 리본 ────────────────────────────────────────────────────

    private void drawPathRibbon(Canvas c, Paint p, int color) {
        if (pathCount < 2) {
            return;
        }
        final float half = 0.9f;
        final int steps = 44;
        // 2026-08-18: 화면에 보이는 도로 끝까지 리본을 늘린다 (모델이 실제로
        // 넘겨준 마지막 경로 포인트보다는 더 늘릴 수 없음).
        float reach = Math.min(FAR_DEPTH * 0.95f, pathX[pathCount - 1] + 10f);
        poly.rewind();
        boolean started = false;
        int used = 0;
        for (int i = 0; i < steps; i++) {
            float x = reach * i / (steps - 1f);
            if (!project(x, centerAt(x) + pathOffset + half, 0f, pa)) {
                continue;
            }
            if (started) {
                poly.lineTo(pa[0], pa[1]);
            } else {
                poly.moveTo(pa[0], pa[1]);
                started = true;
            }
            used++;
        }
        if (used < 2) {
            return;
        }
        for (int i = steps - 1; i >= 0; i--) {
            float x = reach * i / (steps - 1f);
            if (project(x, centerAt(x) + pathOffset - half, 0f, pa)) {
                poly.lineTo(pa[0], pa[1]);
            }
        }
        poly.close();

        // 2026-08-18: 색 더 진하게(20% 어둡게) + 그라데이션 강화(원거리 더
        // 옅게, 근거리 더 진하게). 테두리도 단색 대신 같은 그라데이션을 써서
        // 채우기와 어울리게 한다.
        int darkR = Math.round(Color.red(color) * 0.8f);
        int darkG = Math.round(Color.green(color) * 0.8f);
        int darkB = Math.round(Color.blue(color) * 0.8f);
        if (ribbonShader == null || ribbonShaderColor != color) {
            ribbonShader = new LinearGradient(0f, HORIZON, 0f, BOTTOM,
                    Color.argb(20, darkR, darkG, darkB),
                    Color.argb(225, darkR, darkG, darkB),
                    Shader.TileMode.CLAMP);
            ribbonShaderColor = color;
        }
        p.setStyle(Paint.Style.FILL);
        p.setShader(ribbonShader);
        c.drawPath(poly, p);
        p.setShader(null);

        if (ribbonStrokeShader == null || ribbonShaderColor != color) {
            ribbonStrokeShader = new LinearGradient(0f, HORIZON, 0f, BOTTOM,
                    Color.argb(60, darkR, darkG, darkB),
                    Color.argb(255, darkR, darkG, darkB),
                    Shader.TileMode.CLAMP);
        }
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(2.8f);
        p.setStrokeJoin(Paint.Join.ROUND);
        p.setShader(ribbonStrokeShader);
        c.drawPath(poly, p);
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
    }

    // ── BSD 경고 띠 ───────────────────────────────────────────────────────

    private void drawBsd(Canvas c, Paint p, JSONObject s, int style) {
        if (s == null) {
            return;
        }
        if (s.optBoolean("leftBsd", false)) {
            bsdBand(c, p, 1, style);
        }
        if (s.optBoolean("rightBsd", false)) {
            bsdBand(c, p, -1, style);
        }
    }

    /**
     * 옆차선 전체를 띠로 칠한다. 특정 지점에 차를 그리지 않는 이유는
     * openpilot 이 옆차의 앞뒤 위치를 알려주지 않기 때문이다.
     */
    private void bsdBand(Canvas c, Paint p, int side, int style) {
        final int steps = 16;
        int fillAlpha = style == BSD_SOLID ? 96 : style == BSD_SOFT ? 48 : 0;

        if (fillAlpha > 0) {
            poly.rewind();
            boolean started = false;
            for (int i = 0; i < steps; i++) {
                float x = BSD_NEAR + (BSD_FAR - BSD_NEAR) * i / (steps - 1f);
                float base = centerAt(Math.max(0f, x));
                if (!project(x, base + side * BSD_INNER, 0f, pa)) {
                    continue;
                }
                if (started) {
                    poly.lineTo(pa[0], pa[1]);
                } else {
                    poly.moveTo(pa[0], pa[1]);
                    started = true;
                }
            }
            if (!started) {
                return;
            }
            for (int i = steps - 1; i >= 0; i--) {
                float x = BSD_NEAR + (BSD_FAR - BSD_NEAR) * i / (steps - 1f);
                float base = centerAt(Math.max(0f, x));
                if (project(x, base + side * BSD_OUTER, 0f, pa)) {
                    poly.lineTo(pa[0], pa[1]);
                }
            }
            poly.close();
            p.setShader(null);
            p.setStyle(Paint.Style.FILL);
            p.setColor(Color.argb(fillAlpha, 255, 168, 40));
            c.drawPath(poly, p);
        }

        // 안쪽 경계 막대는 어느 방식에서나 그린다.
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.argb(238, 255, 190, 70));
        boolean have = false;
        for (int i = 0; i < steps; i++) {
            float x = BSD_NEAR + (BSD_FAR - BSD_NEAR) * i / (steps - 1f);
            float base = centerAt(Math.max(0f, x));
            if (!project(x, base + side * BSD_INNER, 0f, pb)) {
                have = false;
                continue;
            }
            if (have) {
                quad(c, p, pa, pb, 7f, 6f);
            }
            pa[0] = pb[0];
            pa[1] = pb[1];
            have = true;
        }
    }

    // ── 건물 ──────────────────────────────────────────────────────────────

    private void drawBuildings(Canvas c, Paint p, float odoM, int fogColor) {
        int base = (int) Math.floor(odoM / BLOCK);
        for (int i = BLOCK_COUNT - 1; i >= 0; i--) {
            int k = base + i;
            float d = k * BLOCK - odoM;
            if (d < BUILD_NEAR || d > BUILD_FAR) {
                continue;
            }
            for (int side = -1; side <= 1; side += 2) {
                int salt = side > 0 ? 0x51ED : 0x2F19;
                if (rnd(k, salt) < 0.26f) {
                    continue;
                }
                float gap = 1.6f + rnd(k, salt + 1) * 3.2f;
                float width = 7f + rnd(k, salt + 2) * 9f;
                float depth = 8f + rnd(k, salt + 3) * 10f;
                float height = 4f + rnd(k, salt + 4) * 7f;
                int tone = dark ? 78 - (int) (rnd(k, salt + 5) * 22f)
                        : 196 - (int) (rnd(k, salt + 5) * 34f);
                building(c, p, d, d + depth, side, gap, width, height, tone, fogColor);
            }
        }
    }

    private void building(Canvas c, Paint p, float d0, float d1, int side,
                          float gap, float width, float height, int tone, int fogColor) {
        float inner = roadEdgeAt(d0, side > 0) + side * gap;
        float outer = inner + side * width;

        if (!project(d0, inner, 0f, pa) || !project(d0, outer, 0f, pb)
                || !project(d0, outer, height, pc) || !project(d0, inner, height, pd)) {
            return;
        }
        float minX = Math.min(Math.min(pa[0], pb[0]), Math.min(pc[0], pd[0]));
        float maxX = Math.max(Math.max(pa[0], pb[0]), Math.max(pc[0], pd[0]));
        if (maxX < LEFT - 40f || minX > RIGHT + 40f) {
            return;
        }

        float fog = d0 <= HAZE_START ? 0f
                : Math.min(0.82f, (d0 - HAZE_START) / (BUILD_FAR - HAZE_START) * 0.95f);
        int front = blend(Color.rgb(tone, Math.min(255, tone + 4), Math.min(255, tone + 11)),
                fogColor, fog);
        int flank = blend(Color.rgb(Math.max(24, tone - 34), Math.max(28, tone - 29),
                Math.max(34, tone - 21)), fogColor, fog);

        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        poly2.rewind();
        poly2.moveTo(pa[0], pa[1]);
        poly2.lineTo(pd[0], pd[1]);
        if (project(d1, inner, height, pc) && project(d1, inner, 0f, pd)) {
            poly2.lineTo(pc[0], pc[1]);
            poly2.lineTo(pd[0], pd[1]);
            poly2.close();
            p.setColor(flank);
            c.drawPath(poly2, p);
        }

        if (!project(d0, outer, height, pc) || !project(d0, inner, height, pd)) {
            return;
        }
        poly2.rewind();
        poly2.moveTo(pa[0], pa[1]);
        poly2.lineTo(pb[0], pb[1]);
        poly2.lineTo(pc[0], pc[1]);
        poly2.lineTo(pd[0], pd[1]);
        poly2.close();
        p.setColor(front);
        c.drawPath(poly2, p);

        float faceW = Math.abs(pb[0] - pa[0]);
        float faceH = Math.abs(pa[1] - pd[1]);
        if (faceW < 20f || faceH < 26f || fog > 0.55f) {
            return;
        }
        int cols = Math.max(2, Math.min(4, (int) (faceW / 14f)));
        int rows = Math.max(2, Math.min(5, (int) (faceH / 16f)));
        p.setColor(blend(dark ? Color.rgb(150, 176, 110) : Color.rgb(122, 152, 176), fogColor, fog));
        for (int r = 0; r < rows; r++) {
            for (int q = 0; q < cols; q++) {
                float xa = lerp(pa[0], pb[0], (q + 0.30f) / cols);
                float xb = lerp(pa[0], pb[0], (q + 0.70f) / cols);
                float ya = lerp(pa[1], pd[1], (r + 0.28f) / rows);
                float yb = lerp(pa[1], pd[1], (r + 0.66f) / rows);
                rect.set(Math.min(xa, xb), Math.min(ya, yb), Math.max(xa, xb), Math.max(ya, yb));
                c.drawRect(rect, p);
            }
        }
    }

    // ── 차량 ──────────────────────────────────────────────────────────────

    /**
     * 신호/E2E 정지 지점에 노면 정지선을 그린다. 거리는 EON 이 longitudinalPlan
     * 속도궤적을 적분해 보낸 값이라, 앞차 추종 정차에는 오지 않는다.
     */
    private void drawStopLine(Canvas c, Paint p) {
        if (stopDist < 1f || stopDist > 90f) {
            return;
        }
        float cy = centerAt(stopDist) + pathOffset;
        float half = (laneWidthM > 0.1f
                ? Math.max(2.2f, Math.min(4.2f, laneWidthM)) : TMAP_LANE_W) * 0.5f;
        // 0.4m 두께의 실선. 가까워질수록 진하게.
        float alpha = Math.max(90f, Math.min(235f, 235f - (stopDist - 8f) * 2.2f));
        if (!project(stopDist - 0.2f, cy + half, 0f, pa)
                || !project(stopDist - 0.2f, cy - half, 0f, pb)
                || !project(stopDist + 0.2f, cy - half, 0f, pc)
                || !project(stopDist + 0.2f, cy + half, 0f, pd)) {
            return;
        }
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(dark ? Color.rgb(236, 240, 245) : Color.rgb(250, 250, 250));
        p.setAlpha((int) alpha);
        poly2.reset();
        poly2.moveTo(pa[0], pa[1]);
        poly2.lineTo(pb[0], pb[1]);
        poly2.lineTo(pc[0], pc[1]);
        poly2.lineTo(pd[0], pd[1]);
        poly2.close();
        c.drawPath(poly2, p);
        p.setAlpha(255);
    }

    private void drawVehicles(Canvas c, Paint p, JSONObject s, Bitmap egoCar, Bitmap otherCar,
                              int radarInfo) {
        if (s == null) {
            return;
        }
        JSONObject lead = s.optJSONObject("lead");
        JSONObject lead2 = s.optJSONObject("lead2");
        float d1 = lead == null ? -1f : (float) lead.optDouble("d", -1d);
        float d2 = lead2 == null ? -1f : (float) lead2.optDouble("d", -1d);

        if (d2 > d1) {
            car(c, p, otherCar, lead2, 205);
            car(c, p, otherCar, lead, 245);
        } else {
            car(c, p, otherCar, lead, 245);
            car(c, p, otherCar, lead2, 205);
        }
        if (carStyle == CAR_BOX) {
            boxCar(c, p, 0f, 0f, dark ? Color.rgb(74, 130, 214) : Color.rgb(86, 132, 200), true, 255);
        } else {
            billboard(c, p, egoCar, 0f, 0f, 1.95f, egoRoll(), 255);
        }

        if (radarInfo == 3 || radarInfo == 4) {
            markLead(c, p, lead);
        }
    }

    private void car(Canvas c, Paint p, Bitmap bmp, JSONObject lead, int alpha) {
        if (lead == null) {
            return;
        }
        float d = (float) lead.optDouble("d", -1d);
        float y = (float) lead.optDouble("y", 0d);
        if (d <= 0f || d > FAR_DEPTH) {
            return;
        }
        // 앞차 감속중이면 후미등. aLeadK 는 칼만필터 가속도라 vRel 과 달리
        // "앞차가 실제로 브레이크를 밟았는지"를 나타낸다.
        float aLead = (float) lead.optDouble("a", 0d);
        boolean braking = aLead < -0.6f && d < 80f;
        if (carStyle == CAR_BOX) {
            boxCar(c, p, d, y, dark ? Color.rgb(126, 134, 146) : Color.rgb(112, 122, 136),
                    false, alpha);
            if (braking) {
                brakeLights(c, p, d, y, aLead);
            }
            return;
        }
        if (braking) {
            brakeLights(c, p, d, y, aLead);
        }
        billboard(c, p, bmp, d, y, 1.9f, 0f, alpha);
    }

    /**
     * 앞차 후미등. 차 뒷면 좌우에 붉은 사각형 두 개를 얹는다.
     * 감속이 셀수록 진해진다(-0.6 ~ -3.0 m/s^2 구간에서 알파 120~255).
     */
    private void brakeLights(Canvas c, Paint p, float x, float y, float aLead) {
        float rear = x - CAR_LEN / 2f;
        if (rear < NEAR_DEPTH - CAM_BACK) {
            return;
        }
        float t = Math.max(0f, Math.min(1f, (-aLead - 0.6f) / 2.4f));
        int alpha = (int) (120f + 135f * t);
        float inner = 0.42f;
        float outer = 0.86f;
        float zLo = 0.72f;
        float zHi = 0.98f;
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        for (int side = -1; side <= 1; side += 2) {
            float ya = y + side * inner;
            float yb = y + side * outer;
            if (!project(rear, ya, zLo, pa) || !project(rear, yb, zLo, pb)
                    || !project(rear, yb, zHi, pc) || !project(rear, ya, zHi, pd)) {
                continue;
            }
            // 바깥쪽 옅은 번짐 → 안쪽 진한 코어 순으로 두 번 그린다.
            p.setColor(Color.rgb(255, 96, 88));
            p.setAlpha(Math.max(40, alpha / 3));
            quad(c, p, pa, pb, pc, pd, 1.35f);
            p.setColor(Color.rgb(232, 40, 44));
            p.setAlpha(alpha);
            quad(c, p, pa, pb, pc, pd, 1.0f);
        }
        p.setAlpha(255);
    }

    /** 네 점을 중심 기준으로 scale 배 키워 채운다. */
    private void quad(Canvas c, Paint p, float[] a, float[] b, float[] d, float[] e, float scale) {
        float cx = (a[0] + b[0] + d[0] + e[0]) * 0.25f;
        float cy = (a[1] + b[1] + d[1] + e[1]) * 0.25f;
        poly2.reset();
        poly2.moveTo(cx + (a[0] - cx) * scale, cy + (a[1] - cy) * scale);
        poly2.lineTo(cx + (b[0] - cx) * scale, cy + (b[1] - cy) * scale);
        poly2.lineTo(cx + (d[0] - cx) * scale, cy + (d[1] - cy) * scale);
        poly2.lineTo(cx + (e[0] - cx) * scale, cy + (e[1] - cy) * scale);
        poly2.close();
        c.drawPath(poly2, p);
    }

    /**
     * 단색 3D 박스 차량. 뒷면 / 윗면 / 보이는 옆면 세 면에 명암을 준다.
     * 카메라가 차 지붕보다 위(4.6m)에 있으므로 윗면이 보이는 게 맞다.
     */
    private void boxCar(Canvas c, Paint p, float x, float y, int body, boolean ego, int alpha) {
        float rear = Math.max(x - CAR_LEN / 2f, NEAR_DEPTH - CAM_BACK + 0.2f);
        float front = Math.max(x + CAR_LEN / 2f, rear + 0.8f);
        float yl = y + CAR_W / 2f;
        float yr = y - CAR_W / 2f;

        // 0 rl 1 rr 2 rrTop 3 rlTop 4 fl 5 fr 6 frTop 7 flTop
        float[][] src = {
                {rear, yl, 0f}, {rear, yr, 0f}, {rear, yr, CAR_H}, {rear, yl, CAR_H},
                {front, yl, 0f}, {front, yr, 0f}, {front, yr, CAR_H}, {front, yl, CAR_H},
        };
        for (int i = 0; i < 8; i++) {
            if (!project(src[i][0], src[i][1], src[i][2], pa)) {
                return;
            }
            boxX[i] = pa[0];
            boxY[i] = pa[1];
        }

        float depth = x + CAM_BACK;
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);

        // 접지 그림자
        if (project(x, y, 0f, pa)) {
            float rx0 = CAR_W * 0.62f * FOCAL / depth;
            float ry0 = Math.max(1.2f, rx0 * CAM_H / depth);
            p.setColor(Color.argb(Math.min(120, alpha / 2), 6, 10, 14));
            rect.set(pa[0] - rx0, pa[1] - ry0, pa[0] + rx0, pa[1] + ry0);
            c.drawOval(rect, p);
        }

        p.setAlpha(alpha);
        // 보이는 옆면. 카메라가 Y=0 에 있으므로 차가 왼쪽(y>0)이면 왼쪽 면,
        // 오른쪽이면 오른쪽 면이 보인다.
        p.setColor(blend(body, Color.BLACK, 0.45f));
        if (y > 0f) {
            face(c, p, 0, 4, 7, 3);   // rl - fl - flTop - rlTop
        } else {
            face(c, p, 1, 5, 6, 2);   // rr - fr - frTop - rrTop
        }

        // 윗면
        p.setColor(blend(body, Color.WHITE, 0.18f));
        face(c, p, 3, 2, 6, 7);

        // 뒷면
        p.setColor(body);
        face(c, p, 0, 1, 2, 3);

        // 후미등 (자차는 생략)
        if (!ego) {
            p.setColor(Color.rgb(232, 62, 58));
            float lampR = Math.max(1.2f, CAR_W * 0.13f * FOCAL / depth);
            for (float f : new float[]{0.24f, 0.76f}) {
                float lxp = lerp(boxX[0], boxX[1], f);
                float lyp = lerp(boxY[0], boxY[3], 0.62f);
                rect.set(lxp - lampR, lyp - lampR * 0.55f, lxp + lampR, lyp + lampR * 0.55f);
                c.drawOval(rect, p);
            }
        }
        p.setAlpha(255);
    }

    private void face(Canvas c, Paint p, int a, int b, int d0, int e) {
        poly2.rewind();
        poly2.moveTo(boxX[a], boxY[a]);
        poly2.lineTo(boxX[b], boxY[b]);
        poly2.lineTo(boxX[d0], boxY[d0]);
        poly2.lineTo(boxX[e], boxY[e]);
        poly2.close();
        c.drawPath(poly2, p);
    }

    private void markLead(Canvas c, Paint p, JSONObject lead) {
        if (lead == null) {
            return;
        }
        float d = (float) lead.optDouble("d", -1d);
        if (d <= 0f || d > 90f) {
            return;
        }
        float y = (float) lead.optDouble("y", 0d);
        if (!project(d, y + 1.2f, 0f, pa) || !project(d, y - 1.2f, 0f, pb)) {
            return;
        }
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.argb(150, 236, 92, 72));
        quad(c, p, pa, pb, 2.4f, 2.4f);
    }

    private float egoRoll() {
        if (pathCount < 2) {
            return 0f;
        }
        float roll = (float) Math.toDegrees(Math.atan2(centerAt(2f) - centerAt(14f), 12f)) * 0.45f;
        return Math.max(-16f, Math.min(16f, roll));
    }

    private void billboard(Canvas c, Paint p, Bitmap b, float x, float y,
                           float widthM, float rollDeg, int alpha) {
        if (b == null || b.isRecycled() || !project(x, y, 0f, pa)) {
            return;
        }
        float depth = x + CAM_BACK;
        float w = widthM * FOCAL / depth;
        if (w < 6f) {
            return;
        }
        float h = b.getHeight() * w / b.getWidth();

        int save = c.save();
        c.translate(pa[0], pa[1]);
        if (rollDeg != 0f) {
            c.rotate(rollDeg);
        }
        float rx0 = w * 0.52f;
        float ry0 = Math.max(1.5f, rx0 * CAM_H / depth);
        p.setShader(null);
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.argb(Math.min(110, alpha / 2), 8, 12, 16));
        rect.set(-rx0, -ry0, rx0, ry0);
        c.drawOval(rect, p);

        p.setAlpha(alpha);
        p.setFilterBitmap(true);
        rect.set(-w / 2f, -h, w / 2f, 0f);
        c.drawBitmap(b, null, rect, p);
        p.setAlpha(255);
        c.restoreToCount(save);
    }

    // ── 상단 헤이즈 ───────────────────────────────────────────────────────

    private void drawHaze(Canvas c, Paint p, int skyColor) {
        if (hazeShader == null || hazeShaderColor != skyColor) {
            hazeShader = new LinearGradient(0f, TOP, 0f, TOP + 46f,
                    Color.argb(255, Color.red(skyColor), Color.green(skyColor), Color.blue(skyColor)),
                    Color.argb(0, Color.red(skyColor), Color.green(skyColor), Color.blue(skyColor)),
                    Shader.TileMode.CLAMP);
            hazeShaderColor = skyColor;
        }
        p.setStyle(Paint.Style.FILL);
        p.setShader(hazeShader);
        c.drawRect(LEFT, TOP, RIGHT, TOP + 46f, p);
        p.setShader(null);
    }

    // ── 유틸 ──────────────────────────────────────────────────────────────

    private static float lerp(float a, float b, float t) {
        return a + (b - a) * t;
    }

    static int blend(int a, int b, float t) {
        if (t <= 0f) {
            return a;
        }
        if (t >= 1f) {
            return b;
        }
        return Color.rgb(
                (int) (Color.red(a) + (Color.red(b) - Color.red(a)) * t),
                (int) (Color.green(a) + (Color.green(b) - Color.green(a)) * t),
                (int) (Color.blue(a) + (Color.blue(b) - Color.blue(a)) * t));
    }

    private static float rnd(int k, int salt) {
        int h = (k * 0x9E3779B1) ^ (salt * 0x85EBCA6B);
        h ^= h >>> 15;
        h *= 0x2545F491;
        h ^= h >>> 13;
        return ((h >>> 8) & 0xFFFF) / 65535f;
    }
}
