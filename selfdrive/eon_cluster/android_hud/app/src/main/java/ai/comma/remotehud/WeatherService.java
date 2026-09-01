package ai.comma.remotehud;

import android.content.Context;
import android.content.SharedPreferences;

import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Locale;

/**
 * 하늘상태 조회. Open-Meteo 의 current 엔드포인트만 쓴다.
 *
 * 키가 필요 없는 API 라 공개 저장소에 그대로 둘 수 있다. 위치는 티맵이
 * 주는 navi.scene.pos 를 쓰고, 티맵이 꺼져 있으면 마지막 좌표로 계속
 * 시도한다.
 *
 * 스레드는 요청이 있을 때만 하나 뜨고 끝나면 사라진다. 실패하면 120 초
 * 쉬었다가 다시 시도한다.
 */
final class WeatherService {

    static final int ICON_NONE = 0;
    static final int ICON_CLEAR = 1;
    static final int ICON_FEW = 2;
    static final int ICON_OVERCAST = 3;
    static final int ICON_FOG = 4;
    static final int ICON_RAIN = 5;
    static final int ICON_SNOW = 6;
    static final int ICON_THUNDER = 7;

    private static final String PREFS = "hud_weather";
    private static final long REFRESH_MS = 15L * 60L * 1000L;
    private static final long COOLDOWN_MS = 120L * 1000L;
    /** 이보다 오래된 값은 표시하지 않는다. */
    private static final long MAX_AGE_MS = 3L * 60L * 60L * 1000L;
    /** 이만큼 움직이면 갱신 주기를 기다리지 않는다. */
    private static final double MOVE_M = 10000d;

    private final SharedPreferences prefs;

    private volatile int code = -1;
    private volatile boolean day = true;
    private volatile int cloudPct = -1;
    private volatile long fetchedAt = 0L;
    private volatile double fetchedLat = Double.NaN;
    private volatile double fetchedLon = Double.NaN;
    private volatile long cooldownUntil = 0L;
    private volatile boolean busy = false;
    private volatile String lastError = "";

    WeatherService(Context context) {
        prefs = context.getApplicationContext()
                .getSharedPreferences(PREFS, Context.MODE_PRIVATE);
        code = prefs.getInt("code", -1);
        day = prefs.getBoolean("day", true);
        cloudPct = prefs.getInt("cloud", -1);
        fetchedAt = prefs.getLong("at", 0L);
        fetchedLat = Double.longBitsToDouble(
                prefs.getLong("lat", Double.doubleToLongBits(Double.NaN)));
        fetchedLon = Double.longBitsToDouble(
                prefs.getLong("lon", Double.doubleToLongBits(Double.NaN)));
    }

    /** 매 프레임 호출해도 된다. 실제 요청은 조건이 맞을 때만 나간다. */
    void onPosition(double lat, double lon) {
        if (!isFinite(lat) || !isFinite(lon) || (lat == 0d && lon == 0d)) {
            return;
        }
        long now = System.currentTimeMillis();
        if (busy || now < cooldownUntil) {
            return;
        }
        boolean aged = now - fetchedAt >= REFRESH_MS;
        boolean moved = !isFinite(fetchedLat) || !isFinite(fetchedLon)
                || distanceM(lat, lon, fetchedLat, fetchedLon) >= MOVE_M;
        if (!aged && !moved) {
            return;
        }
        busy = true;
        final double reqLat = lat;
        final double reqLon = lon;
        Thread worker = new Thread(new Runnable() {
            @Override
            public void run() {
                fetch(reqLat, reqLon);
            }
        }, "hud-weather");
        worker.setDaemon(true);
        worker.start();
    }

    /** 표시할 아이콘. 값이 없거나 너무 오래됐으면 ICON_NONE. */
    int icon() {
        if (code < 0 || System.currentTimeMillis() - fetchedAt > MAX_AGE_MS) {
            return ICON_NONE;
        }
        return iconFor(code);
    }

    boolean isDay() {
        return day;
    }

    /** 구름량 0~100. 아직 못 받았으면 -1 (아이콘 분류로 대신 정한다). */
    int cloudPercent() {
        if (code < 0 || System.currentTimeMillis() - fetchedAt > MAX_AGE_MS) {
            return -1;
        }
        return cloudPct;
    }

    /** 출력모드 3 상태표용. */
    String status() {
        if (!lastError.isEmpty()) {
            return "ERR " + lastError;
        }
        if (code < 0) {
            return "-";
        }
        long ageMin = (System.currentTimeMillis() - fetchedAt) / 60000L;
        return String.format(Locale.US, "%d/%dm", code, ageMin);
    }

    private void fetch(double lat, double lon) {
        HttpURLConnection conn = null;
        try {
            String url = String.format(Locale.US,
                    "https://api.open-meteo.com/v1/forecast"
                            + "?latitude=%.4f&longitude=%.4f"
                            + "&current=weather_code,is_day,cloud_cover", lat, lon);
            conn = (HttpURLConnection) new URL(url).openConnection();
            conn.setConnectTimeout(4000);
            conn.setReadTimeout(6000);
            conn.setRequestProperty("Accept", "application/json");
            int status = conn.getResponseCode();
            if (status != 200) {
                fail("http" + status);
                return;
            }
            String body = readAll(conn.getInputStream());
            JSONObject current = new JSONObject(body).optJSONObject("current");
            if (current == null) {
                fail("json");
                return;
            }
            int wmo = current.optInt("weather_code", -1);
            if (wmo < 0) {
                fail("json");
                return;
            }
            code = wmo;
            day = current.optInt("is_day", 1) != 0;
            int cover = current.optInt("cloud_cover", -1);
            cloudPct = cover < 0 ? -1 : Math.max(0, Math.min(100, cover));
            fetchedAt = System.currentTimeMillis();
            fetchedLat = lat;
            fetchedLon = lon;
            lastError = "";
            prefs.edit()
                    .putInt("code", code)
                    .putBoolean("day", day)
                    .putInt("cloud", cloudPct)
                    .putLong("at", fetchedAt)
                    .putLong("lat", Double.doubleToLongBits(lat))
                    .putLong("lon", Double.doubleToLongBits(lon))
                    .apply();
        } catch (Throwable t) {
            fail("net");
        } finally {
            if (conn != null) {
                conn.disconnect();
            }
            busy = false;
        }
    }

    private void fail(String reason) {
        lastError = reason;
        cooldownUntil = System.currentTimeMillis() + COOLDOWN_MS;
    }

    private static String readAll(InputStream in) throws Exception {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        byte[] buffer = new byte[2048];
        int read;
        int total = 0;
        while ((read = in.read(buffer)) > 0 && total < 65536) {
            out.write(buffer, 0, read);
            total += read;
        }
        in.close();
        return out.toString("UTF-8");
    }

    /** WMO 코드 → 아이콘 7종. */
    private static int iconFor(int wmo) {
        if (wmo == 0) {
            return ICON_CLEAR;
        }
        if (wmo == 1 || wmo == 2) {
            return ICON_FEW;
        }
        if (wmo == 3) {
            return ICON_OVERCAST;
        }
        if (wmo == 45 || wmo == 48) {
            return ICON_FOG;
        }
        if (wmo >= 95) {
            return ICON_THUNDER;
        }
        if ((wmo >= 71 && wmo <= 77) || wmo == 85 || wmo == 86) {
            return ICON_SNOW;
        }
        if ((wmo >= 51 && wmo <= 67) || (wmo >= 80 && wmo <= 82)) {
            return ICON_RAIN;
        }
        return ICON_OVERCAST;
    }

    private static double distanceM(double lat1, double lon1,
                                    double lat2, double lon2) {
        double dLat = Math.toRadians(lat2 - lat1) * 6371000d;
        double dLon = Math.toRadians(lon2 - lon1) * 6371000d
                * Math.cos(Math.toRadians((lat1 + lat2) * 0.5));
        return Math.sqrt(dLat * dLat + dLon * dLon);
    }

    private static boolean isFinite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }
}
