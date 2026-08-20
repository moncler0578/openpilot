package ai.comma.remotehud;

import android.content.Context;
import android.content.SharedPreferences;

public final class AppPrefs {
    private static final String AUTO_START = "auto_start";
    private static final String FILE = "remote_hud_settings";
    private static final String GUIDE_SHOWN = "guide_shown_v06";

    private AppPrefs() {
    }

    public static boolean isAutoStart(Context context) {
        return prefs(context).getBoolean(AUTO_START, true);
    }

    public static void setAutoStart(Context context, boolean z) {
        prefs(context).edit().putBoolean(AUTO_START, z).apply();
    }

    public static boolean wasGuideShown(Context context) {
        return prefs(context).getBoolean(GUIDE_SHOWN, false);
    }

    public static void markGuideShown(Context context) {
        prefs(context).edit().putBoolean(GUIDE_SHOWN, true).apply();
    }

    private static SharedPreferences prefs(Context context) {
        return context.getSharedPreferences(FILE, 0);
    }
}
