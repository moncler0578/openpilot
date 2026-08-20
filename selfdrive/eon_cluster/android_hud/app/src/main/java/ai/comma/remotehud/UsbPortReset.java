package ai.comma.remotehud;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

/**
 * v0.17 — 루트로 USB 포트를 재바인딩한다. 케이블을 뽑았다 꽂는 것과 같은 효과.
 *
 * 안드로이드의 UsbDeviceConnection 에는 libusb 의 dev.reset() 에 해당하는 API 가
 * 없다. carrot-wip 은 프레임 오류가 나면 clear_halt → dev.reset() → 재연결
 * 순으로 복구하는데, 마지막 단계를 앱에서 하려면 sysfs 를 직접 건드려야 한다.
 *
 * 루트가 없으면 조용히 false 를 돌려준다. 그 경우 동작은 v0.16 과 같다.
 */
final class UsbPortReset {

    private UsbPortReset() {
    }

    /**
     * @param deviceName UsbDevice.getDeviceName() 값
     *                   (예: "/dev/bus/usb/001/004"). null 이면 VID/PID 로 찾는다.
     * @return 재바인딩 명령을 실제로 실행했으면 true
     */
    static boolean resetPort(String deviceName) {
        String port = findPort();
        if (port == null) {
            return false;
        }
        String script =
                "echo " + port + " > /sys/bus/usb/drivers/usb/unbind 2>/dev/null; " +
                "sleep 2; " +
                "echo " + port + " > /sys/bus/usb/drivers/usb/bind 2>/dev/null; " +
                "echo done";
        return runAsRoot(script) != null;
    }

    /** /sys/bus/usb/devices 를 훑어 1CBE:0092 가 붙은 포트 이름을 찾는다. */
    private static String findPort() {
        String out = runAsRoot(
                "for d in /sys/bus/usb/devices/*; do " +
                "  [ -f \"$d/idVendor\" ] || continue; " +
                "  v=$(cat \"$d/idVendor\"); p=$(cat \"$d/idProduct\"); " +
                "  if [ \"$v\" = \"1cbe\" ] && [ \"$p\" = \"0092\" ]; then basename \"$d\"; fi; " +
                "done");
        if (out == null) {
            return null;
        }
        for (String line : out.split("\n")) {
            String trimmed = line.trim();
            // "1-1" 형태만 받는다. "1-1:1.0" 같은 인터페이스 노드는 제외.
            if (!trimmed.isEmpty() && !trimmed.contains(":") && trimmed.contains("-")) {
                return trimmed;
            }
        }
        return null;
    }

    private static String runAsRoot(String script) {
        Process process = null;
        try {
            process = Runtime.getRuntime().exec("su");
            OutputStreamWriter writer = new OutputStreamWriter(process.getOutputStream());
            writer.write(script);
            writer.write("\nexit\n");
            writer.flush();

            StringBuilder sb = new StringBuilder();
            BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
            String line;
            while ((line = reader.readLine()) != null) {
                sb.append(line).append('\n');
            }
            process.waitFor();
            return sb.toString();
        } catch (Exception e) {
            return null;
        } finally {
            if (process != null) {
                try {
                    process.destroy();
                } catch (Exception ignored) {
                }
            }
        }
    }
}
