package ai.comma.remotehud;

import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.os.SystemClock;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Calendar;

import javax.crypto.Cipher;
import javax.crypto.spec.IvParameterSpec;
import javax.crypto.spec.SecretKeySpec;

/**
 * TURZX 1CBE:0092 USB 패널 드라이버.
 *
 * v0.19 에서 고친 것
 * ------------------
 * 1) 매 프레임 clearHalt() 호출 제거.  ★ 주행 중 2~5분 뒤 화면이 굳던 증상의
 *    유력한 원인.
 *
 *    v0.17/0.18 의 sendJpeg() 은 프레임마다 controlTransfer(0x02, 0x01, 0, ep)
     *    로 CLEAR_FEATURE(ENDPOINT_HALT) 를 보냈다. 기존 파이썬 구현을
     *    따라한 것인데, 두 경우의 의미가 다르다.
 *
 *      - libusb 의 clear_halt() 는 커널 USBDEVFS_CLEAR_HALT 를 거쳐
 *        usb_clear_halt() 를 부르고, 이때 호스트 컨트롤러 쪽 data toggle 도
 *        같이 DATA0 으로 초기화된다.
 *      - 안드로이드 UsbDeviceConnection 에는 그 ioctl 이 없다. 우리가 보낸 건
 *        그냥 raw 제어전송이라 장치 쪽 toggle 만 DATA0 이 되고, 호스트 쪽
 *        toggle 은 그대로 남는다.
 *
 *    즉 정상 상태에서 이 호출을 반복하면 양쪽 toggle 이 어긋나고, 어긋난
 *    뒤에는 장치가 패킷을 무시하기 시작한다. 앱에서는 bulkTransfer 가
 *    여전히 성공한 것처럼 보이는데(호스트 버퍼까지만 들어감) 패널은 마지막
 *    프레임을 그대로 붙들고 있게 된다 — 백라이트는 켜져 있고, 물리적으로
 *    다시 꽂아 재열거해야만 풀리는, 보고된 증상 그대로다.
 *
 *    그래서 clearHalt() 는 실제로 전송이 실패한 뒤(recoverAfterError) 에만
 *    부른다. 정상 경로에서는 절대 부르지 않는다.
 *
 * 2) 패널 무응답 감시.  drainInput() 이 읽어들인 패킷 수를 세어 두고, 한
 *    번이라도 응답을 본 적 있는 패널이 오래 조용해지면 isUnresponsive() 가
 *    참이 된다. 원래 아무 말도 하지 않는 패널에서는 감시가 켜지지 않으므로
 *    괜한 리셋을 하지 않는다.
 *
 * 3) 프레임 중간에 쓰기가 실패하면 남은 길이만큼 0 을 채워 프레임을 끝낸다.
 *    패널은 헤더에 적힌 바이트 수를 다 받을 때까지 기다리므로, 중간에 끊고
 *    나가면 그대로 프레임 대기 상태로 고착된다. 채워서 끊어 주면 다음
 *    재연결 때 초기화 시퀀스가 먹힌다.
 */
public final class TurzxDisplay {

    static final String ACTION_PERMISSION = "ai.comma.remotehud.USB_PERMISSION";
    static final int VID = 0x1CBE;   // 7358
    static final int PID = 0x0092;   // 146

    private static final int CHUNK_BYTES = 16384;
    private static final long COMMAND_GAP_MS = 200L;

    private final Context context;
    private UsbManager manager;
    private UsbDevice device;
    private UsbDeviceConnection connection;
    private UsbInterface intf;
    private UsbEndpoint out;
    private UsbEndpoint in;
    private int permissionRequestedDeviceId = -1;

    private final byte[] drainBuffer = new byte[512];
    private boolean sawInbound;
    private long lastInboundElapsed;

    public TurzxDisplay(Context context) {
        this.context = context;
    }

    boolean isOpen() {
        return connection != null && out != null;
    }

    public String describeStatus() {
        manager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
        UsbDevice target = findTargetDevice();
        if (target == null) {
            return "미연결 · 1CBE:0092";
        }
        if (!manager.hasPermission(target)) {
            return "연결됨 · USB 권한 승인 대기";
        }
        return isOpen() ? "연결됨 · USB 권한 허용" : "연결됨 · 여는 중";
    }

    public synchronized String deviceNameOrNull() {
        return device == null ? null : device.getDeviceName();
    }

    /**
     * 패널이 마지막으로 응답한 뒤 경과 시간(ms). 한 번도 응답을 본 적이
     * 없으면 -1 (그 패널은 원래 조용한 것이므로 감시 대상이 아니다).
     */
    public synchronized long silenceMs() {
        if (!isOpen() || !sawInbound) {
            return -1L;
        }
        return SystemClock.elapsedRealtime() - lastInboundElapsed;
    }

    /**
     * 패널이 응답을 끊었는지. 스트리밍 중 IN 엔드포인트에서 한 번이라도
     * 응답을 본 적이 있어야만 참이 될 수 있다.
     */
    public synchronized boolean isUnresponsive(long silenceMs) {
        if (!isOpen() || !sawInbound) {
            return false;
        }
        return SystemClock.elapsedRealtime() - lastInboundElapsed > silenceMs;
    }

    public synchronized boolean openOrRequestPermission() throws Exception {
        if (isOpen()) {
            return true;
        }
        manager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
        device = findTargetDevice();
        if (device == null) {
            permissionRequestedDeviceId = -1;
            return false;
        }
        if (!manager.hasPermission(device)) {
            if (permissionRequestedDeviceId != device.getDeviceId()) {
                permissionRequestedDeviceId = device.getDeviceId();
                manager.requestPermission(device, PendingIntent.getBroadcast(context,
                        device.getDeviceId(),
                        new Intent(ACTION_PERMISSION).setPackage(context.getPackageName()),
                        PendingIntent.FLAG_UPDATE_CURRENT | PendingIntent.FLAG_IMMUTABLE));
            }
            return false;
        }
        permissionRequestedDeviceId = -1;

        intf = null;
        out = null;
        in = null;
        for (int i = 0; i < device.getInterfaceCount(); i++) {
            UsbInterface candidate = device.getInterface(i);
            UsbEndpoint bulkOut = null;
            UsbEndpoint bulkIn = null;
            for (int e = 0; e < candidate.getEndpointCount(); e++) {
                UsbEndpoint endpoint = candidate.getEndpoint(e);
                if (endpoint.getType() != android.hardware.usb.UsbConstants.USB_ENDPOINT_XFER_BULK) {
                    continue;
                }
                if (endpoint.getDirection() == android.hardware.usb.UsbConstants.USB_DIR_OUT) {
                    bulkOut = endpoint;
                } else if (endpoint.getDirection() == android.hardware.usb.UsbConstants.USB_DIR_IN) {
                    bulkIn = endpoint;
                }
            }
            if (bulkOut != null && bulkIn != null) {
                intf = candidate;
                out = bulkOut;
                in = bulkIn;
                break;
            }
        }
        if (intf == null) {
            throw new Exception("TURZX bulk endpoint missing");
        }

        connection = manager.openDevice(device);
        if (connection == null || !connection.claimInterface(intf, true)) {
            close();
            return false;
        }

        // 여는 시점에는 이전 세션의 halt 가 남아 있을 수 있으므로 여기서는 정리한다.
        clearHalt();
        drainInput(5, 2);
        sawInbound = false;
        lastInboundElapsed = SystemClock.elapsedRealtime();
        try {
            initialize();
        } catch (Exception first) {
            clearHalt();
            drainInput(5, 2);
            Thread.sleep(300L);
            initialize();
        }
        return true;
    }

    private void initialize() throws Exception {
        exchange(command(10, -1, 0), true);
        Thread.sleep(COMMAND_GAP_MS);
        exchange(command(13, 8, 0), false);
        Thread.sleep(30L);
        exchange(command(15, 8, 8), false);
        Thread.sleep(50L);
        drainInput(5, 2);
        exchange(command(14, 8, 66), false);
    }

    private UsbDevice findTargetDevice() {
        if (manager == null) {
            manager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
        }
        for (UsbDevice candidate : manager.getDeviceList().values()) {
            if (isTarget(candidate)) {
                return candidate;
            }
        }
        return null;
    }

    public static boolean isTarget(UsbDevice device) {
        return device != null && device.getVendorId() == VID && device.getProductId() == PID;
    }

    public synchronized void reset() {
        permissionRequestedDeviceId = -1;
        close();
        device = null;
    }

    /** 전송 실패 뒤에만 부른다. 정상 경로에서 부르면 data toggle 이 어긋난다. */
    public synchronized void recoverAfterError() {
        clearHalt();
        drainInput(6, 3);
    }

    public synchronized void clearHalt() {
        if (connection == null) {
            return;
        }
        UsbEndpoint[] endpoints = {out, in};
        for (UsbEndpoint endpoint : endpoints) {
            if (endpoint == null) {
                continue;
            }
            try {
                connection.controlTransfer(0x02, 0x01, 0, endpoint.getAddress(), null, 0, 200);
            } catch (Exception ignored) {
            }
        }
    }

    public synchronized void setBrightness(int value) throws Exception {
        if (!isOpen()) {
            throw new Exception("display closed");
        }
        exchange(command(14, 8, Math.max(1, Math.min(100, value))), false);
    }

    public synchronized void sendJpeg(byte[] jpeg) throws Exception {
        if (!isOpen()) {
            throw new Exception("display closed");
        }
        int length = jpeg.length;
        byte[] header = rawCommand(101);
        header[8] = (byte) (length >>> 24);
        header[9] = (byte) (length >>> 16);
        header[10] = (byte) (length >>> 8);
        header[11] = (byte) length;
        byte[] encrypted = encrypt(header);

        byte[] payload = new byte[encrypted.length + length];
        System.arraycopy(encrypted, 0, payload, 0, encrypted.length);
        System.arraycopy(jpeg, 0, payload, encrypted.length, length);

        // 패널이 보낸 응답만 비운다. halt 해제는 하지 않는다 (클래스 주석 1번 참고).
        drainInput(2, 1);
        writeChunked(payload);
    }

    private void writeChunked(byte[] payload) throws Exception {
        int offset = 0;
        byte[] chunk = null;
        while (offset < payload.length) {
            int size = Math.min(CHUNK_BYTES, payload.length - offset);
            if (chunk == null || chunk.length != size) {
                chunk = new byte[size];
            }
            System.arraycopy(payload, offset, chunk, 0, size);
            int written = connection.bulkTransfer(out, chunk, size, 3000);
            if (written != size) {
                int done = offset + Math.max(written, 0);
                padRemainder(payload.length - done);
                throw new Exception("USB 프레임 쓰기 실패 " + done + "/" + payload.length);
            }
            offset += size;
        }
    }

    /**
     * 프레임 중간에 끊겼을 때 남은 바이트를 0 으로 채워 패널의 수신 카운터를
     * 끝내 준다. 실패해도 무시 — 어차피 바로 close 된다.
     */
    private void padRemainder(int remaining) {
        if (remaining <= 0 || connection == null || out == null) {
            return;
        }
        byte[] zeros = new byte[Math.min(CHUNK_BYTES, remaining)];
        int left = remaining;
        while (left > 0) {
            int size = Math.min(zeros.length, left);
            int written = connection.bulkTransfer(out, zeros, size, 400);
            if (written <= 0) {
                return;
            }
            left -= written;
        }
    }

    private void exchange(byte[] payload, boolean expectResponse) throws Exception {
        if (connection.bulkTransfer(out, payload, payload.length, 2000) != payload.length) {
            throw new Exception("TURZX command failed");
        }
        if (expectResponse) {
            int read = connection.bulkTransfer(in, drainBuffer, drainBuffer.length, 2000);
            if (read <= 0) {
                throw new Exception("TURZX sync response missing");
            }
            noteInbound();
        }
    }

    /** 읽어 버린 패킷 수를 돌려준다. 패널 생존 감시에 쓴다. */
    private int drainInput(int maxPackets, int timeoutMs) {
        if (connection == null || in == null) {
            return 0;
        }
        int count = 0;
        for (int i = 0; i < maxPackets; i++) {
            if (connection.bulkTransfer(in, drainBuffer, drainBuffer.length, timeoutMs) <= 0) {
                break;
            }
            count++;
        }
        if (count > 0) {
            noteInbound();
        }
        return count;
    }

    private void noteInbound() {
        sawInbound = true;
        lastInboundElapsed = SystemClock.elapsedRealtime();
    }

    private static byte[] rawCommand(int code) {
        byte[] buffer = new byte[504];
        buffer[0] = (byte) code;
        buffer[2] = 26;
        buffer[3] = 109;
        Calendar calendar = Calendar.getInstance();
        int millisOfDay = ((((calendar.get(Calendar.HOUR_OF_DAY) * 60)
                + calendar.get(Calendar.MINUTE)) * 60)
                + calendar.get(Calendar.SECOND)) * 1000 + calendar.get(Calendar.MILLISECOND);
        ByteBuffer.wrap(buffer, 4, 4).order(ByteOrder.LITTLE_ENDIAN).putInt(millisOfDay);
        return buffer;
    }

    private static byte[] command(int code, int index, int value) throws Exception {
        byte[] buffer = rawCommand(code);
        if (index >= 0) {
            buffer[index] = (byte) value;
        }
        return encrypt(buffer);
    }

    private static byte[] encrypt(byte[] plain) throws Exception {
        byte[] key = "slv3tuzx".getBytes("US-ASCII");
        Cipher cipher = Cipher.getInstance("DES/CBC/NoPadding");
        cipher.init(Cipher.ENCRYPT_MODE, new SecretKeySpec(key, "DES"), new IvParameterSpec(key));
        byte[] encrypted = cipher.doFinal(plain);
        byte[] framed = new byte[512];
        System.arraycopy(encrypted, 0, framed, 0, encrypted.length);
        framed[510] = (byte) 0xA1;
        framed[511] = 26;
        return framed;
    }

    public synchronized void close() {
        if (connection != null && intf != null) {
            try {
                connection.bulkTransfer(out, command(14, 8, 0), 512, 200);
            } catch (Exception ignored) {
            }
            try {
                connection.releaseInterface(intf);
            } catch (Exception ignored) {
            }
            connection.close();
        }
        connection = null;
        intf = null;
        out = null;
        in = null;
        sawInbound = false;
    }
}
