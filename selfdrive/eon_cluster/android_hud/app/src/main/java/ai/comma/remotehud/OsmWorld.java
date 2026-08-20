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
 *  - 응답 원문은 디스크 캐시(cacheDir/osm)에 저장 → 같은 길은 재다운로드 없음,
 *    인터넷이 끊겨도 캐시 구간은 계속 보인다. 파일 500개 넘으면 오래된 것 삭제.
 *  - snapshot() 이 매 프레임 자차 로컬좌표(전방 x, 좌 +y)로 변환해 넘긴다.
 */
final class OsmWorld {
    private static final String TAG = "OsmWorld";
    private static final double TILE = 0.005;
    private static final long FAIL_COOLDOWN_MS = 90_000L;
    private static final int CACHE_MAX_FILES = 500;
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
    }

    private static final class Poly {
        double[] pts;      // lat,lon 짝
        float h;           // 건물 높이(m) 또는 도로 폭(m)
    }

    private static final class Tile {
        final List<Poly> buildings = new ArrayList<>();
        final List<Poly> roads = new ArrayList<>();
    }

    private final File cacheDir;
    private final Object lock = new Object();
    private final HashMap<String, Tile> tiles = new HashMap<>();
    private final HashSet<String> pending = new HashSet<>();
    private final HashMap<String, Long> failedAt = new HashMap<>();
    private final LinkedBlockingQueue<String> queue = new LinkedBlockingQueue<>();
    private Thread worker;
    private int endpointIndex = 0;

    // ── 진단용 계측 (출력모드 3 에 한 줄로 표시) ─────────────────────────
    /** 마지막 실패 사유. "net"=요청 실패, "json"=파싱 실패, ""=없음. */
    private static volatile String lastError = "";
    /** 마지막 스냅샷의 건물 수. 폴백(가짜 건물)과 구분하는 기준. */
    private volatile int lastBuildings = 0;

    /**
     * 출력모드 3 에 찍을 한 줄. 예)
     *   "3/48"   타일 3개 로드, 건물 48개  → OSM 정상
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
        return loaded + "/" + lastBuildings;
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
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    String key = (ky + dy) + "_" + (kx + dx);
                    if (tiles.containsKey(key) || pending.contains(key)) {
                        continue;
                    }
                    Long failed = failedAt.get(key);
                    if (failed != null && now - failed < FAIL_COOLDOWN_MS) {
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
                        float[][] xy = toLocal(r.pts, lat0, lon0, sinH, cosH, mLat, mLon,
                                -10f, 200f, 90f);
                        if (xy != null) {
                            rx.add(xy[0]);
                            ry.add(xy[1]);
                            rw.add(r.h);
                        }
                    }
                }
            }
        }
        if (bx.isEmpty() && rx.isEmpty()) {
            return null;
        }
        Snapshot s = new Snapshot();
        s.buildingCount = bx.size();
        lastBuildings = s.buildingCount;
        if (s.buildingCount > 0) {
            lastError = "";
        }
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

    // ── 다운로드/캐시 ────────────────────────────────────────────────────

    private void workerLoop() {
        while (true) {
            String key;
            try {
                key = queue.take();
            } catch (InterruptedException e) {
                return;
            }
            Tile tile = null;
            File cached = new File(cacheDir, key + ".json");
            if (cached.isFile()) {
                tile = parseTile(readFile(cached));
            }
            if (tile == null) {
                String body = fetch(key);
                if (body != null) {
                    tile = parseTile(body);
                    if (tile != null) {
                        writeFile(cached, body);
                        trimCache();
                    }
                }
            }
            synchronized (lock) {
                pending.remove(key);
                if (tile != null) {
                    tiles.put(key, tile);
                    failedAt.remove(key);
                } else {
                    failedAt.put(key, System.currentTimeMillis());
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
        String query = "[out:json][timeout:10];("
                + "way[\"building\"](" + bbox + ");"
                + "way[\"highway\"~\"^(motorway|trunk|primary|secondary|tertiary|unclassified"
                + "|residential|service|motorway_link|trunk_link|primary_link|secondary_link)$\"]"
                + "(" + bbox + ");"
                + ");out geom 220;";
        for (int attempt = 0; attempt < ENDPOINTS.length; attempt++) {
            String endpoint = ENDPOINTS[(endpointIndex + attempt) % ENDPOINTS.length];
            HttpURLConnection conn = null;
            try {
                conn = (HttpURLConnection) new URL(endpoint).openConnection();
                conn.setConnectTimeout(6000);
                conn.setReadTimeout(12000);
                conn.setDoOutput(true);
                conn.setRequestProperty("Content-Type", "application/x-www-form-urlencoded");
                byte[] payload = ("data=" + URLEncoder.encode(query, "UTF-8"))
                        .getBytes(StandardCharsets.UTF_8);
                OutputStream out = conn.getOutputStream();
                out.write(payload);
                out.close();
                if (conn.getResponseCode() != 200) {
                    continue;
                }
                InputStream in = conn.getInputStream();
                ByteArrayOutputStream buf = new ByteArrayOutputStream();
                byte[] chunk = new byte[8192];
                int read;
                while ((read = in.read(chunk)) > 0) {
                    buf.write(chunk, 0, read);
                    if (buf.size() > 2 * 1024 * 1024) {
                        break;
                    }
                }
                in.close();
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
                if (el == null || !"way".equals(el.optString("type"))) {
                    continue;
                }
                JSONArray geom = el.optJSONArray("geometry");
                if (geom == null || geom.length() < 2) {
                    continue;
                }
                JSONObject tags = el.optJSONObject("tags");
                Poly poly = new Poly();
                poly.pts = new double[geom.length() * 2];
                for (int g = 0; g < geom.length(); g++) {
                    JSONObject pt = geom.optJSONObject(g);
                    poly.pts[g * 2] = pt == null ? 0d : pt.optDouble("lat", 0d);
                    poly.pts[g * 2 + 1] = pt == null ? 0d : pt.optDouble("lon", 0d);
                }
                if (tags != null && tags.has("building")) {
                    poly.h = buildingHeight(tags);
                    if (geom.length() >= 4 && tile.buildings.size() < 160) {
                        tile.buildings.add(poly);
                    }
                } else if (tags != null && tags.has("highway")) {
                    poly.h = roadWidth(tags.optString("highway", ""));
                    if (tile.roads.size() < 120) {
                        tile.roads.add(poly);
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

    /** OSM 은 높이가 거의 없다 — levels×3.2m, height 태그, 기본 8m. 배치는 실제, 높이는 근사. */
    private static float buildingHeight(JSONObject tags) {
        try {
            String levels = tags.optString("building:levels", "");
            if (!levels.isEmpty()) {
                return clampF(Float.parseFloat(levels.trim()) * 3.2f, 4f, 40f);
            }
        } catch (NumberFormatException ignored) {
        }
        try {
            String height = tags.optString("height", "").replace("m", "").trim();
            if (!height.isEmpty()) {
                return clampF(Float.parseFloat(height), 4f, 40f);
            }
        } catch (NumberFormatException ignored) {
        }
        return 8f;
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
        if (files == null || files.length <= CACHE_MAX_FILES) {
            return;
        }
        Arrays.sort(files, new Comparator<File>() {
            @Override
            public int compare(File a, File b) {
                return Long.compare(a.lastModified(), b.lastModified());
            }
        });
        for (int i = 0; i < files.length - CACHE_MAX_FILES + 50; i++) {
            files[i].delete();
        }
    }
}
