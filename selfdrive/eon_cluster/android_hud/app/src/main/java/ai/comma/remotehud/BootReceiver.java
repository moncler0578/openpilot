package ai.comma.remotehud;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;

/**
 * v0.13
 * - BOOT_COMPLETED 외에 QUICKBOOT_POWERON, MY_PACKAGE_REPLACED 도 처리
 * - 부팅 경로로 시작할 때는 EXTRA_FROM_BOOT 를 실어 보내, HudService 가
 *   워커 스레드 시작을 지연시키도록 한다 (TMAP 초기화에 자리를 내주기 위함)
 */
public final class BootReceiver extends BroadcastReceiver {

    @Override
    public void onReceive(Context context, Intent intent) {
        if (intent == null || intent.getAction() == null) {
            return;
        }
        String action = intent.getAction();
        boolean bootAction =
                "android.intent.action.BOOT_COMPLETED".equals(action)
                        || "android.intent.action.QUICKBOOT_POWERON".equals(action)
                        || "android.intent.action.MY_PACKAGE_REPLACED".equals(action);

        if (!bootAction || !AppPrefs.isAutoStart(context)) {
            return;
        }

        Intent service = new Intent(context, HudService.class);
        // 패키지 교체 직후에는 이미 시스템이 안정된 상태이므로 지연을 걸지 않는다.
        service.putExtra(HudService.EXTRA_FROM_BOOT,
                !"android.intent.action.MY_PACKAGE_REPLACED".equals(action));
        try {
            context.startForegroundService(service);
        } catch (Exception ignored) {
            // 부팅 직후 드물게 FGS 시작이 거부될 수 있다. 이 경우 USB 연결 시점에
            // MainActivity 가 다시 시작하므로 여기서는 조용히 넘어간다.
        }
    }
}
