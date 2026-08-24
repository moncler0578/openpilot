package ai.comma.remotehud;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;

/**
 * nMirror 즐겨찾기에 등록하는 1회성 HUD/TMAP 전환 아이콘.
 * 자체 화면을 만들지 않고 기존 HUD task만 열거나 닫은 뒤 즉시 종료한다.
 */
public final class HudFavoriteActivity extends Activity {

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        startHudService();
        toggleHudScreen();
        finish();
    }

    private void startHudService() {
        try {
            startForegroundService(new Intent(this, HudService.class));
        } catch (RuntimeException ignored) {
        }
    }

    private void toggleHudScreen() {
        Intent intent = new Intent(this, HudFullscreenActivity.class)
                .setAction(HudFullscreenActivity.shouldCloseForFavoriteLaunch()
                        ? HudFullscreenActivity.ACTION_SHOW_TMAP
                        : HudFullscreenActivity.ACTION_SHOW_HUD)
                .addFlags(Intent.FLAG_ACTIVITY_NEW_TASK
                        | Intent.FLAG_ACTIVITY_CLEAR_TOP
                        | Intent.FLAG_ACTIVITY_SINGLE_TOP);
        try {
            startActivity(intent);
        } catch (RuntimeException ignored) {
        }
    }
}
