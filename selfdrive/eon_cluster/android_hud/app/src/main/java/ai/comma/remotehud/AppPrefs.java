package ai.comma.remotehud;

import android.content.Context;
import android.content.SharedPreferences;

public final class AppPrefs {
    public static final int DISPLAY_PROFILE_AUTO = 0;
    public static final int DISPLAY_PROFILE_GENESIS_8 = 1;
    public static final int DISPLAY_PROFILE_GENESIS_9_2 = 2;

    private static final String AUTO_START = "auto_start";
    private static final String DISPLAY_PROFILE = "display_profile";
    private static final String FILE = "remote_hud_settings";
    private static final String GUIDE_SHOWN = "guide_shown_v37";
    private static final String ORIENTATION = "hud_orientation";
    private static final String MIRROR = "hud_mirror";

    private AppPrefs() {
    }

    public static boolean isAutoStart(Context context) {
        return prefs(context).getBoolean(AUTO_START, true);
    }

    public static void setAutoStart(Context context, boolean z) {
        prefs(context).edit().putBoolean(AUTO_START, z).apply();
    }

    public static int getDisplayProfile(Context context) {
        int profile = prefs(context).getInt(DISPLAY_PROFILE, DISPLAY_PROFILE_AUTO);
        if (profile < DISPLAY_PROFILE_AUTO || profile > DISPLAY_PROFILE_GENESIS_9_2) {
            return DISPLAY_PROFILE_AUTO;
        }
        return profile;
    }

    public static void setDisplayProfile(Context context, int profile) {
        int safeProfile = profile;
        if (safeProfile < DISPLAY_PROFILE_AUTO || safeProfile > DISPLAY_PROFILE_GENESIS_9_2) {
            safeProfile = DISPLAY_PROFILE_AUTO;
        }
        prefs(context).edit().putInt(DISPLAY_PROFILE, safeProfile).apply();
    }

    public static int getOrientation(Context context) {
        return prefs(context).getInt(ORIENTATION, 0) == 2 ? 2 : 0;
    }

    public static void setOrientation(Context context, int orientation) {
        prefs(context).edit().putInt(ORIENTATION, orientation == 2 ? 2 : 0).apply();
    }

    public static boolean isMirror(Context context) {
        return prefs(context).getBoolean(MIRROR, false);
    }

    public static void setMirror(Context context, boolean mirror) {
        prefs(context).edit().putBoolean(MIRROR, mirror).apply();
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
