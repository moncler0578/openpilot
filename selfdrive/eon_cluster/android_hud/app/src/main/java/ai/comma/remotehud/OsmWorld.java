package ai.comma.remotehud;

import android.util.Log;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLEncoder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.LinkedBlockingQueue;

/** OSM(Overpass) 기반 실제 주변 지형: 건물 외곽선 + 옆길.
 *
 *  - 위치원은 티맵 vehicle 스트림(패킷 navi.scene.pos = [lat, lon, heading]).
 *  - 0.005°(~500m) 격자 타일로 현재 위치 주변 3x3 을 유지한다.
 *  - 다운로드는 데몬 스레드 1개, 실패 타일은 90초 쿨다운.
 *  - 응답 원문은 디스크 캐시(cacheDir/osm)에 저장하고 7일 뒤 백그라운드 갱신한다.
 *    갱신 중·실패 시 기존 캐시를 계속 표시하며 파일 500개 넘으면 오래된 것 삭제.
 *  - snapshot() 이 매 프레임 자차 로컬좌표(전방 x, 좌 +y)로 변환해 넘긴다.
 */
final class OsmWorld {
    private static final String TAG = "OsmWorld";
    private static final double TILE = 0.005;
    private static final long FAIL_COOLDOWN_MS = 90_000L;
    /** 오래된 캐시 갱신 실패 시 Overpass 재시도 간격. */
    private static final long REFRESH_FAIL_COOLDOWN_MS = 90L * 60L * 1000L;
    /** OSM 원문 캐시는 7일 동안 유효하다. */
    private static final long CACHE_TTL_MS = 7L * 24L * 60L * 60L * 1000L;
    private static final int CACHE_MAX_FILES = 500;
    private static final long CACHE_MAX_BYTES = 96L * 1024L * 1024L;
    private static final int MAX_RESPONSE_BYTES = 2 * 1024 * 1024;
    /** 환경 객체 쿼리가 없는 이전 캐시를 한 번 분리한다. */
    private static final String CACHE_VERSION = "v3_";
    static final int BARRIER_NOISE_WALL = 1;
    static final int BARRIER_GUARD_RAIL = 2;
    private static final String[] ENDPOINTS = {
            "https://overpass-api.de/api/interpreter",
            "https://overpass.kumi.systems/api/interpreter",
    };

    /** World3D 로 넘어가는 프레임 스냅샷 (자차 로컬좌표). */
    static final class Snapshot {
        float[][] ringX;
        float[][] ringY;
        float[] ringH;
        int buildingCount;
        float[][] roadX;
        float[][] roadY;
        float[] roadW;
        int roadCount;
        float[][] barrierX;
        float[][] barrierY;
        float[] barrierH;
        int[] barrierKind;
        int barrierCount;
        float[] treeX;
        float[] treeY;
        float[] treeH;
        int treeCount;
        float[] lampX;
        float[] lampY;
        float[] lampH;
        int lampCount;
    }

    private static final class Poly {
        long id;
        double[] pts;      // lat,lon 짝
        float h;           // 건물 높이(m) 또는 도로 폭(m)
        int kind;
    }

    private static final class PointFeature {
        long id;
        double lat;
        double lon;
        float h;
    }

    private static final class Tile {
        final List<Poly> buildings = new ArrayList<>();
        final List<Poly> roads = new ArrayList<>();
        final List<Poly> barriers = new ArrayList<>();
        final List<Poly> treeRows = new ArrayList<>();
        final List<PointFeature> trees = new ArrayList<>();
        final List<PointFeature> lamps = new ArrayList<>();
    }

    private final File cacheDir;
    private final Object lock = new Object();
    private final HashMap<String, Tile> tiles = new HashMap<>();
    private final HashSet<String> pending = new HashSet<>();
    private final HashMap<String, Long> failedAt = new HashMap<>();
    /** 메모리에 올라온 각 타일의 다음 자동 갱신 시각. */
    private final HashMap<String, Long> refreshDueAt = new HashMap<>();
    private final LinkedBlockingQueue<String> queue = new LinkedBlockingQueue<>();
    private final HashSet<String> activeKeys = new HashSet<>();
    private Thread worker;
    private String inFlight;
    private int endpointIndex = 0;
    private int activeKy = Integer.MIN_VALUE;
    private int activeKx = Integer.MIN_VALUE;

    // ── 진단용 계측 (출력모드 3 에 한 줄로 표시) ─────────────────────────
    /** 마지막 실패 사유. net/json/size, 빈 문자열은 정상. */
    private static volatile String lastError = "";
    /** 마지막 스냅샷의 건물 수. 폴백(가짜 건물)과 구분하는 기준. */
    private volatile int lastBuildings = 0;
    private volatile int lastEnvironment = 0;

    /**
     * 출력모드 3 에 찍을 한 줄. 예)
     *   "3/48+35" 타일/건물/환경객체(방음벽·수목·가로등) → OSM 정상
     *   "0/0"    데이터 없음               → 폴백(격자 건물)
     *   "ERR net"  요청 실패 / "ERR json"  응답 형식 문제
     */
    String status() {
        int loaded;
        synchronized (lock) {
            loaded = tiles.size();
        }
        String err = lastError;
        if (loaded == 0 && !err.isEmpty()) {
            return "ERR " + err;
        }
        return loaded + "/" + lastBuildings + "+" + lastEnvironment;
    }

    OsmWorld(File dir) {
        cacheDir = dir;
        if (!cacheDir.exists()) {
            cacheDir.mkdirs();
        }
    }

    /** 현재 위치 주변 3x3 타일이 로드/다운로드되도록 한다. 메인 스레드에서 호출해도 가볍다. */
    void ensure(double lat, double lon) {
        int ky = (int) Math.floor(lat / TILE);
        int kx = (int) Math.floor(lon / TILE);
        long now = System.currentTimeMillis();
        synchronized (lock) {
            if (ky != activeKy || kx != activeKx) {
                HashSet<String> wanted = new HashSet<>();
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        wanted.add((ky + dy) + "_" + (kx + dx));
                    }
                }
                activeKeys.clear();
                activeKeys.addAll(wanted);
                // 실제 메모리도 문서대로 현재 3x3 만 유지한다. 디스크 캐시는 남아
                // 돌아오는 길에 네트워크 없이 즉시 다시 읽을 수 있다.
                tiles.keySet().retainAll(wanted);
                failedAt.keySet().retainAll(wanted);
                refreshDueAt.keySet().retainAll(wanted);
                for (String queued : new ArrayList<>(queue)) {
                    if (!wanted.contains(queued) && queue.remove(queued)) {
                        pending.remove(queued);
                    }
                }
                activeKy = ky;
                activeKx = kx;
            }
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    String key = (ky + dy) + "_" + (kx + dx);
                    boolean loaded = tiles.containsKey(key);
                    Long due = refreshDueAt.get(key);
                    boolean stale = loaded && (due == null || now >= due);
                    if ((loaded && !stale) || pending.contains(key) || key.equals(inFlight)) {
                        continue;
                    }
                    Long failed = failedAt.get(key);
                    long cooldown = loaded ? REFRESH_FAIL_COOLDOWN_MS : FAIL_COOLDOWN_MS;
                    if (failed != null && now - failed < cooldown) {
                        continue;
                    }
                    pending.add(key);
                    queue.offer(key);
                }
            }
            if (worker == null || !worker.isAlive()) {
                worker = new Thread(new Runnable() {
                    @Override
                    public void run() {
                        workerLoop();
                    }
                }, "osm-fetch");
                worker.setDaemon(true);
                worker.start();
            }
        }
    }

    /** 자차 기준 로컬좌표 스냅샷. 렌더 스레드에서 매 프레임 호출. */
    Snapshot snapshot(double lat0, double lon0, double headingDeg) {
        double h = Math.toRadians(headingDeg);
        double sinH = Math.sin(h);
        double cosH = Math.cos(h);
        double mLat = 111320.0;
        double mLon = 111320.0 * Math.cos(Math.toRadians(lat0));
        int ky = (int) Math.floor(lat0 / TILE);
        int kx = (int) Math.floor(lon0 / TILE);

        List<float[]> bx = new ArrayList<>();
        List<float[]> by = new ArrayList<>();
        List<Float> bh = new ArrayList<>();
        List<float[]> rx = new ArrayList<>();
        List<float[]> ry = new ArrayList<>();
        List<Float> rw = new ArrayList<>();
        List<float[]> wx = new ArrayList<>();
        List<float[]> wy = new ArrayList<>();
        List<Float> wh = new ArrayList<>();
        List<Integer> wk = new ArrayList<>();
        List<Float> tx = new ArrayList<>();
        List<Float> ty = new ArrayList<>();
        List<Float> th = new ArrayList<>();
        List<Float> lx = new ArrayList<>();
        List<Float> ly = new ArrayList<>();
        List<Float> lh = new ArrayList<>();
        HashSet<Long> buildingIds = new HashSet<>();
        HashSet<Long> roadIds = new HashSet<>();
        HashSet<Long> barrierIds = new HashSet<>();
        HashSet<Long> treeNodeIds = new HashSet<>();
        HashSet<Long> treeRowIds = new HashSet<>();
        HashSet<Long> lampIds = new HashSet<>();

        synchronized (lock) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    Tile tile = tiles.get((ky + dy) + "_" + (kx + dx));
                    if (tile == null) {
                        continue;
                    }
                    for (Poly b : tile.buildings) {
                        if (bx.size() >= 48) {
                            break;
                        }
                        if (!buildingIds.add(b.id)) {
                            continue;
                        }
                        float[][] xy = toLocal(b.pts, lat0, lon0, sinH, cosH, mLat, mLon,
                                -25f, 210f, 75f);
                        if (xy != null) {
                            bx.add(xy[0]);
                            by.add(xy[1]);
                            bh.add(b.h);
                        }
                    }
                    for (Poly r : tile.roads) {
                        if (rx.size() >= 60) {
                            break;
                        }
                        if (!roadIds.add(r.id)) {
                            continue;
                        }
                        float[][] xy = toLocal(r.pts, lat0, lon0, sinH, cosH, mLat, mLon,
                                -10f, 200f, 90f);
                        if (xy != null) {
                            rx.add(xy[0]);
                            ry.add(xy[1]);
                            rw.add(r.h);
                        }
                    }
                    for (Poly wall : tile.barriers) {
                        if (wx.size() >= 60) {
                            break;
                        }
                        if (!barrierIds.add(wall.id)) {
                            continue;
                        }
                        float[][] xy = toLocal(wall.pts, lat0, lon0, sinH, cosH, mLat, mLon,
                                -10f, 210f, 80f);
                        if (xy != null) {
                            wx.add(xy[0]);
                            wy.add(xy[1]);
                            wh.add(wall.h);
                            wk.add(wall.kind);
                        }
                    }
                    for (PointFeature tree : tile.trees) {
                        if (tx.size() >= 80) {
                            break;
                        }
                        if (!treeNodeIds.add(tree.id)) {
                            continue;
                        }
                        float[] xy = pointToLocal(tree.lat, tree.lon, lat0, lon0,
                                sinH, cosH, mLat, mLon, -5f, 160f, 65f);
                        if (xy != null) {
                            tx.add(xy[0]);
                            ty.add(xy[1]);
                            th.add(tree.h);
                        }
                    }
                    for (Poly row : tile.treeRows) {
                        if (tx.size() >= 80 || !treeRowIds.add(row.id)) {
                            continue;
                        }
                        float[][] xy = toLocal(row.pts, lat0, lon0, sinH, cosH, mLat, mLon,
                                -5f, 160f, 65f);
                        if (xy != null) {
                            appendTreeRow(xy[0], xy[1], row.h, tx, ty, th, 80);
                        }
                    }
                    for (PointFeature lamp : tile.lamps) {
                        if (lx.size() >= 40) {
                            break;
                        }
                        if (!lampIds.add(lamp.id)) {
                            continue;
                        }
                        float[] xy = pointToLocal(lamp.lat, lamp.lon, lat0, lon0,
                                sinH, cosH, mLat, mLon, 0f, 80f, 35f);
                        if (xy != null) {
                            lx.add(xy[0]);
                            ly.add(xy[1]);
                            lh.add(lamp.h);
                        }
                    }
                }
            }
        }
        if (bx.isEmpty() && rx.isEmpty() && wx.isEmpty() && tx.isEmpty() && lx.isEmpty()) {
            lastBuildings = 0;
            lastEnvironment = 0;
            return null;
        }
        Snapshot s = new Snapshot();
        s.buildingCount = bx.size();
        lastBuildings = s.buildingCount;
        lastError = "";
        s.ringX = bx.toArray(new float[0][]);
        s.ringY = by.toArray(new float[0][]);
        s.ringH = new float[bh.size()];
        for (int i = 0; i < bh.size(); i++) {
            s.ringH[i] = bh.get(i);
        }
        s.roadCount = rx.size();
        s.roadX = rx.toArray(new float[0][]);
        s.roadY = ry.toArray(new float[0][]);
        s.roadW = new float[rw.size()];
        for (int i = 0; i < rw.size(); i++) {
            s.roadW[i] = rw.get(i);
        }
        s.barrierCount = wx.size();
        s.barrierX = wx.toArray(new float[0][]);
        s.barrierY = wy.toArray(new float[0][]);
        s.barrierH = new float[wh.size()];
        s.barrierKind = new int[wk.size()];
        for (int i = 0; i < wh.size(); i++) {
            s.barrierH[i] = wh.get(i);
            s.barrierKind[i] = wk.get(i);
        }
        s.treeCount = tx.size();
        s.treeX = toFloatArray(tx);
        s.treeY = toFloatArray(ty);
        s.treeH = toFloatArray(th);
        s.lampCount = lx.size();
        s.lampX = toFloatArray(lx);
        s.lampY = toFloatArray(ly);
        s.lampH = toFloatArray(lh);
        lastEnvironment = s.barrierCount + s.treeCount + s.lampCount;
        return s;
    }

    /** lat/lon 짝 배열 → 로컬 (x[], y[]). 범위 밖이면 null (컬링). */
    private static float[][] toLocal(double[] pts, double lat0, double lon0,
                                     double sinH, double cosH, double mLat, double mLon,
                                     float minX, float maxX, float maxAbsY) {
        int n = pts.length / 2;
        float[] xs = new float[n];
        float[] ys = new float[n];
        float lo = Float.MAX_VALUE;
        float hi = -Float.MAX_VALUE;
        float yMin = Float.MAX_VALUE;
        for (int i = 0; i < n; i++) {
            double e = (pts[i * 2 + 1] - lon0) * mLon;
            double nn = (pts[i * 2] - lat0) * mLat;
            float x = (float) (e * sinH + nn * cosH);
            float y = (float) (-e * cosH + nn * sinH);
            xs[i] = x;
            ys[i] = y;
            lo = Math.min(lo, x);
            hi = Math.max(hi, x);
            yMin = Math.min(yMin, Math.abs(y));
        }
        if (hi < minX || lo > maxX || yMin > maxAbsY) {
            return null;
        }
        return new float[][]{xs, ys};
    }

    private static float[] pointToLocal(double lat, double lon, double lat0, double lon0,
                                        double sinH, double cosH, double mLat, double mLon,
                                        float minX, float maxX, float maxAbsY) {
        double e = (lon - lon0) * mLon;
        double nn = (lat - lat0) * mLat;
        float x = (float) (e * sinH + nn * cosH);
        float y = (float) (-e * cosH + nn * sinH);
        if (x < minX || x > maxX || Math.abs(y) > maxAbsY) {
            return null;
        }
        return new float[]{x, y};
    }

    /** tree_row 는 선 전체를 보내되 HUD 에서는 약 12m 간격의 나무로 단순화한다. */
    private static void appendTreeRow(float[] xs, float[] ys, float height,
                                      List<Float> outX, List<Float> outY, List<Float> outH,
                                      int maximum) {
        for (int i = 0; i < xs.length - 1 && outX.size() < maximum; i++) {
            float dx = xs[i + 1] - xs[i];
            float dy = ys[i + 1] - ys[i];
            float length = (float) Math.sqrt(dx * dx + dy * dy);
            int steps = Math.max(1, (int) Math.floor(length / 12f));
            for (int step = 0; step < steps && outX.size() < maximum; step++) {
                float t = step / (float) steps;
                float x = xs[i] + dx * t;
                float y = ys[i] + dy * t;
                if (x < -5f || x > 160f || Math.abs(y) > 65f) {
                    continue;
                }
                outX.add(x);
                outY.add(y);
                outH.add(height);
            }
        }
    }

    private static float[] toFloatArray(List<Float> values) {
        float[] out = new float[values.size()];
        for (int i = 0; i < values.size(); i++) {
            out[i] = values.get(i);
        }
        return out;
    }

    // ── 다운로드/캐시 ────────────────────────────────────────────────────

    private void workerLoop() {
        while (true) {
            String key;
            try {
                key = queue.take();
            } catch (InterruptedException e) {
                return;
            }
            synchronized (lock) {
                inFlight = key;
            }

            long now = System.currentTimeMillis();
            Tile tile = null;
            long refreshDue = 0L;
            boolean refreshAttempted = false;
            boolean refreshSucceeded = false;
            File cached = new File(cacheDir, CACHE_VERSION + key + ".json");

            if (cached.isFile()) {
                tile = parseTile(readFile(cached));
                if (tile != null) {
                    // lastModified는 마지막 다운로드 시각이다. 읽을 때 갱신하지 않는다.
                    refreshDue = cached.lastModified() + CACHE_TTL_MS;
                    // stale-while-revalidate: 네트워크를 기다리지 않고 기존 타일부터 표시한다.
                    synchronized (lock) {
                        if (activeKeys.contains(key)) {
                            tiles.put(key, tile);
                            refreshDueAt.put(key, refreshDue);
                        }
                    }
                }
            }

            boolean stale = tile != null && now >= refreshDue;
            if (tile == null || stale) {
                refreshAttempted = true;
                String body = fetch(key);
                if (body != null) {
                    Tile fresh = parseTile(body);
                    if (fresh != null) {
                        tile = fresh;
                        writeFile(cached, body);
                        cached.setLastModified(now);
                        refreshDue = now + CACHE_TTL_MS;
                        refreshSucceeded = true;
                        trimCache();
                    }
                }
            }

            synchronized (lock) {
                inFlight = null;
                pending.remove(key);
                if (tile != null && activeKeys.contains(key)) {
                    tiles.put(key, tile);
                    refreshDueAt.put(key, refreshDue > 0L ? refreshDue : now + CACHE_TTL_MS);
                    if (!refreshAttempted || refreshSucceeded) {
                        failedAt.remove(key);
                    } else {
                        // 갱신 실패: 기존 캐시는 유지하고 90분 후 다시 시도한다.
                        failedAt.put(key, now);
                    }
                } else if (tile == null && activeKeys.contains(key)) {
                    // 표시할 캐시도 없는 최초 실패는 기존 90초 쿨다운을 사용한다.
                    failedAt.put(key, now);
                }
            }
        }
    }

    private String fetch(String key) {
        String[] parts = key.split("_");
        int ky = Integer.parseInt(parts[0]);
        int kx = Integer.parseInt(parts[1]);
        double s = ky * TILE;
        double w = kx * TILE;
        String bbox = String.format(Locale.US, "%.5f,%.5f,%.5f,%.5f", s, w, s + TILE, w + TILE);
        // 레이어별 out 제한을 둔다. 나무/가로등이 많은 도심에서도 핵심인 건물과
        // 도로가 뒤 레이어에 밀려 누락되지 않고 LTE 응답 크기도 예측 가능하다.
        String query = "[out:json][timeout:12];"
                + "way[\"building\"](" + bbox + ");out geom 160;"
                + "way[\"highway\"~\"^(motorway|trunk|primary|secondary|tertiary|unclassified"
                + "|residential|service|motorway_link|trunk_link|primary_link|secondary_link)$\"]"
                + "(" + bbox + ");out geom 100;"
                + "way[\"barrier\"=\"wall\"][\"wall\"=\"noise_barrier\"](" + bbox + ");out geom 40;"
                + "way[\"barrier\"=\"noise_barrier\"](" + bbox + ");out geom 20;"
                + "way[\"barrier\"=\"guard_rail\"](" + bbox + ");out geom 40;"
                + "way[\"natural\"=\"tree_row\"](" + bbox + ");out geom 30;"
                + "node[\"natural\"=\"tree\"](" + bbox + ");out 80;"
                + "node[\"highway\"=\"street_lamp\"](" + bbox + ");out 40;";
        for (int attempt = 0; attempt < ENDPOINTS.length; attempt++) {
            String endpoint = ENDPOINTS[(endpointIndex + attempt) % ENDPOINTS.length];
            HttpURLConnection conn = null;
            try {
                conn = (HttpURLConnection) new URL(endpoint).openConnection();
                conn.setConnectTimeout(6000);
                conn.setReadTimeout(12000);
                conn.setDoOutput(true);
                conn.setRequestProperty("Content-Type", "application/x-www-form-urlencoded");
                conn.setRequestProperty("User-Agent", "EONRemoteHUD/0.28");
                byte[] payload = ("data=" + URLEncoder.encode(query, "UTF-8"))
                        .getBytes(StandardCharsets.UTF_8);
                OutputStream out = conn.getOutputStream();
                out.write(payload);
                out.close();
                if (conn.getResponseCode() != 200) {
                    lastError = "net";
                    continue;
                }
                InputStream in = conn.getInputStream();
                ByteArrayOutputStream buf = new ByteArrayOutputStream();
                byte[] chunk = new byte[8192];
                int read;
                boolean oversized = false;
                while ((read = in.read(chunk)) > 0) {
                    buf.write(chunk, 0, read);
                    if (buf.size() > MAX_RESPONSE_BYTES) {
                        oversized = true;
                        break;
                    }
                }
                in.close();
                if (oversized) {
                    lastError = "size";
                    Log.w(TAG, "fetch " + key + " exceeded response limit");
                    continue;
                }
                endpointIndex = (endpointIndex + attempt) % ENDPOINTS.length;
                return buf.toString("UTF-8");
            } catch (Exception e) {
                lastError = "net";
                Log.w(TAG, "fetch " + key + " via " + endpoint + " failed: " + e.getMessage());
            } finally {
                if (conn != null) {
                    conn.disconnect();
                }
            }
        }
        return null;
    }

    private static Tile parseTile(String body) {
        if (body == null) {
            return null;
        }
        try {
            JSONArray elements = new JSONObject(body).optJSONArray("elements");
            Tile tile = new Tile();
            if (elements == null) {
                return tile;
            }
            for (int i = 0; i < elements.length(); i++) {
                JSONObject el = elements.optJSONObject(i);
                if (el == null) {
                    continue;
                }
                JSONObject tags = el.optJSONObject("tags");
                if (tags == null) {
                    continue;
                }
                String type = el.optString("type");
                if ("node".equals(type)) {
                    PointFeature point = new PointFeature();
                    point.id = el.optLong("id", 0L);
                    point.lat = el.optDouble("lat", Double.NaN);
                    point.lon = el.optDouble("lon", Double.NaN);
                    if (Double.isNaN(point.lat) || Double.isNaN(point.lon)) {
                        continue;
                    }
                    if ("tree".equals(tags.optString("natural")) && tile.trees.size() < 80) {
                        point.h = taggedHeight(tags, 8f, 3f, 35f);
                        tile.trees.add(point);
                    } else if ("street_lamp".equals(tags.optString("highway"))
                            && tile.lamps.size() < 40) {
                        point.h = taggedHeight(tags, 7f, 3f, 18f);
                        tile.lamps.add(point);
                    }
                    continue;
                }
                if (!"way".equals(type)) {
                    continue;
                }
                JSONArray geom = el.optJSONArray("geometry");
                if (geom == null || geom.length() < 2) {
                    continue;
                }
                Poly poly = new Poly();
                poly.id = el.optLong("id", 0L);
                poly.pts = new double[geom.length() * 2];
                for (int g = 0; g < geom.length(); g++) {
                    JSONObject pt = geom.optJSONObject(g);
                    poly.pts[g * 2] = pt == null ? 0d : pt.optDouble("lat", 0d);
                    poly.pts[g * 2 + 1] = pt == null ? 0d : pt.optDouble("lon", 0d);
                }
                if (tags.has("building")) {
                    poly.h = buildingHeight(tags);
                    if (geom.length() >= 4 && tile.buildings.size() < 160) {
                        tile.buildings.add(poly);
                    }
                } else if (tags.has("highway")) {
                    poly.h = roadWidth(tags.optString("highway", ""));
                    if (tile.roads.size() < 120) {
                        tile.roads.add(poly);
                    }
                } else if ("guard_rail".equals(tags.optString("barrier"))) {
                    poly.kind = BARRIER_GUARD_RAIL;
                    poly.h = taggedHeight(tags, 0.75f, 0.4f, 2.0f);
                    if (tile.barriers.size() < 80) {
                        tile.barriers.add(poly);
                    }
                } else if (("wall".equals(tags.optString("barrier"))
                        && "noise_barrier".equals(tags.optString("wall")))
                        || "noise_barrier".equals(tags.optString("barrier"))) {
                    poly.kind = BARRIER_NOISE_WALL;
                    poly.h = taggedHeight(tags, 3.0f, 1.0f, 12f);
                    if (tile.barriers.size() < 80) {
                        tile.barriers.add(poly);
                    }
                } else if ("tree_row".equals(tags.optString("natural"))) {
                    poly.h = taggedHeight(tags, 8f, 3f, 35f);
                    if (tile.treeRows.size() < 40) {
                        tile.treeRows.add(poly);
                    }
                }
            }
            return tile;
        } catch (Exception e) {
            lastError = "json";
            Log.w(TAG, "parse failed: " + e.getMessage());
            return null;
        }
    }

    /** 명시 height 를 우선하고, 없을 때만 levels×3.2m. 고층도 300m 까지 보존한다. */
    private static float buildingHeight(JSONObject tags) {
        String explicit = tags.optString("height", "");
        Float metres = parseMetres(explicit);
        if (metres != null) {
            return clampF(metres, 3f, 300f);
        }
        try {
            String levels = tags.optString("building:levels", "");
            if (!levels.isEmpty()) {
                float roof = Math.max(0f, taggedHeightValue(tags.optString("roof:height", ""), 0f));
                return clampF(Float.parseFloat(levels.trim()) * 3.2f + roof, 3f, 300f);
            }
        } catch (NumberFormatException ignored) {
        }
        return 8f;
    }

    private static float taggedHeight(JSONObject tags, float fallback, float lo, float hi) {
        Float parsed = parseMetres(tags.optString("height", ""));
        return parsed == null ? fallback : clampF(parsed, lo, hi);
    }

    private static float taggedHeightValue(String raw, float fallback) {
        Float parsed = parseMetres(raw);
        return parsed == null ? fallback : parsed;
    }

    /** OSM 의 보통 미터 표기와 간단한 ft 표기를 처리한다. 복합 11'4\" 는 폴백. */
    private static Float parseMetres(String raw) {
        if (raw == null) {
            return null;
        }
        String value = raw.trim().toLowerCase(Locale.US);
        if (value.isEmpty() || value.contains(";") || value.contains("'")) {
            return null;
        }
        float factor = 1f;
        if (value.endsWith("feet")) {
            value = value.substring(0, value.length() - 4).trim();
            factor = 0.3048f;
        } else if (value.endsWith("ft")) {
            value = value.substring(0, value.length() - 2).trim();
            factor = 0.3048f;
        } else if (value.endsWith("m")) {
            value = value.substring(0, value.length() - 1).trim();
        }
        try {
            return Float.parseFloat(value) * factor;
        } catch (NumberFormatException ignored) {
            return null;
        }
    }

    private static float roadWidth(String cls) {
        if (cls.startsWith("motorway") || cls.startsWith("trunk")) {
            return 9f;
        }
        if (cls.startsWith("primary")) {
            return 8f;
        }
        if (cls.startsWith("secondary")) {
            return 7f;
        }
        if (cls.equals("tertiary") || cls.equals("unclassified")) {
            return 6f;
        }
        if (cls.equals("service")) {
            return 3.5f;
        }
        return 5f;
    }

    private static float clampF(float v, float lo, float hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static String readFile(File f) {
        try (FileInputStream in = new FileInputStream(f)) {
            ByteArrayOutputStream buf = new ByteArrayOutputStream();
            byte[] chunk = new byte[8192];
            int read;
            while ((read = in.read(chunk)) > 0) {
                buf.write(chunk, 0, read);
            }
            return buf.toString("UTF-8");
        } catch (Exception e) {
            return null;
        }
    }

    private static void writeFile(File f, String body) {
        try (FileOutputStream out = new FileOutputStream(f)) {
            out.write(body.getBytes(StandardCharsets.UTF_8));
        } catch (Exception e) {
            Log.w(TAG, "cache write failed: " + e.getMessage());
        }
    }

    private void trimCache() {
        File[] files = cacheDir.listFiles();
        if (files == null) {
            return;
        }
        long totalBytes = 0L;
        for (File file : files) {
            totalBytes += Math.max(0L, file.length());
        }
        if (files.length <= CACHE_MAX_FILES && totalBytes <= CACHE_MAX_BYTES) {
            return;
        }
        Arrays.sort(files, new Comparator<File>() {
            @Override
            public int compare(File a, File b) {
                return Long.compare(a.lastModified(), b.lastModified());
            }
        });
        int remaining = files.length;
        for (File file : files) {
            if (remaining <= CACHE_MAX_FILES && totalBytes <= CACHE_MAX_BYTES) {
                break;
            }
            long bytes = Math.max(0L, file.length());
            if (file.delete()) {
                remaining--;
                totalBytes -= bytes;
            }
        }
    }
}
