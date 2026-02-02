#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

int main (int argc, char *argv[]) {
    GMainLoop *loop;
    GstRTSPServer *server;
    GstRTSPMountPoints *mounts;
    GstRTSPMediaFactory *factory;

    gst_init (&argc, &argv);
    loop = g_main_loop_new (NULL, FALSE);

    // 1. RTSP 서버 인스턴스 생성
    server = gst_rtsp_server_new ();
    gst_rtsp_server_set_address(server, "0.0.0.0");
    gst_rtsp_server_set_service(server, "8554"); // 포트 번호 8554

    // 2. 마운트 포인트 설정 (접속 경로)
    mounts = gst_rtsp_server_get_mount_points (server);
    factory = gst_rtsp_media_factory_new ();

    // 3. 핵심: 파이프라인 설계
    // v4l2src: 카메라 캡처
    // videoconvert: 포맷 변환
    // x264enc: H.264 소프트웨어 인코딩 (tune=zerolatency로 지연 시간 최소화)
    // rtph264pay: RTP 패킷화
    gst_rtsp_media_factory_set_launch (factory,
        "( v4l2src device=/dev/video0 ! videoconvert ! "
        "video/x-raw,width=640,height=480,framerate=30/1 ! "
        "x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! "
        "rtph264pay name=pay0 pt=96 )");

    // 공유 모드 설정 (여러 명 접속 가능)
    gst_rtsp_media_factory_set_shared (factory, TRUE);

    // /test 경로에 팩토리 연결 (rtsp://IP:8554/test)
    gst_rtsp_mount_points_add_factory (mounts, "/test", factory);
    g_object_unref (mounts);

    // 4. 서버 시작
    gst_rtsp_server_attach (server, NULL);
    g_print ("RTSP 서버 시작: rtsp://127.0.0.1:8554/test\n");

    g_main_loop_run (loop);

    return 0;
}