"""
1. 데이터 획득 계층
2. 전처리 계층
3.1. 인코딩 계층(멀티 프로세스)
3.2. inference 계층(멀티 프로세스)
4. 전송 계층

"""
#include <gst/gst.h>

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    /* 파이프라인 설명:
       - libcamerasrc: 카메라 입력
       - videoscale: 640x512로 리사이즈 (인코딩과 AI 공통 입력 사이즈)
       - tee name=t: 분기점 생성
       - t. ! queue ! x264enc... : 스트리밍 경로 (스레드 A)
       - t. ! queue ! fakesink: AI 경로 (스레드 B, 현재는 데이터만 버림)
    */
    const char *pipe_str = 
        "libcamerasrc ! "
        "video/x-raw, width=1640, height=1232, framerate=30/1 ! "
        "videoconvert ! "
        "videoscale ! "
        "video/x-raw, width=640, height=512 ! "
        "tee name=t "
        "t. ! queue ! x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! rtph264pay ! udpsink host=RECEIVER_IP port=5000 "
        "t. ! queue ! fakesink"; // 나중에 fakesink 대신 appsink를 넣어 AI 모델에 전달

    GstElement *pipeline = gst_parse_launch(pipe_str, NULL);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    g_print("Streaming started. (AI Branch is ready for expansion)\n");

    GstBus *bus = gst_element_get_bus(pipeline);
    GstMessage *msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE, GST_MESSAGE_ERROR | GST_MESSAGE_EOS);

    if (msg != NULL) gst_message_unref(msg);
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}