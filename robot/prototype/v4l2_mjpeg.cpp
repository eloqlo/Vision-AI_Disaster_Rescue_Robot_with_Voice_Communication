#include <iostream>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <jpeglib.h>

using namespace std;

struct Buffer {
    void* start;
    size_t length;
};

class V4L2Camera {
    int fd;
    vector<Buffer> buffers;
    int width, height;

public:
    V4L2Camera(const char* dev, int w, int h) : width(w), height(h) {
        fd = open(dev, O_RDWR);
        if (fd < 0) { perror("Device open failed"); exit(1); }

        v4l2_format fmt = {0};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24; // RGB 데이터 요청
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        ioctl(fd, VIDIOC_S_FMT, &fmt);

        // 메모리 매핑 설정 (Mmap)
        v4l2_requestbuffers req = {0};
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        ioctl(fd, VIDIOC_REQBUFS, &req);

        for (int i = 0; i < req.count; ++i) {
            v4l2_buffer buf = {0};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            ioctl(fd, VIDIOC_QUERYBUF, &buf);
            
            buffers.push_back({
                mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset),
                buf.length
            });
            ioctl(fd, VIDIOC_QBUF, &buf);
        }
    }

    void start_capture() {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd, VIDIOC_STREAMON, &type);
    }

    // JPEG 압축 함수
    void compress_to_jpeg(unsigned char* rgb_data, const char* filename) {
        struct jpeg_compress_struct cinfo;
        struct jpeg_error_mgr jerr;
        FILE* outfile;
        JSAMPROW row_pointer[1];

        cinfo.err = jpeg_std_error(&jerr);
        jpeg_create_compress(&cinfo);

        if ((outfile = fopen(filename, "wb")) == NULL) return;
        jpeg_stdio_dest(&cinfo, outfile);

        cinfo.image_width = width;
        cinfo.image_height = height;
        cinfo.input_components = 3;
        cinfo.in_color_space = JCS_RGB;

        jpeg_set_defaults(&cinfo);
        jpeg_set_quality(&cinfo, 85, TRUE); // 압축 품질 설정
        jpeg_start_compress(&cinfo, TRUE);

        while (cinfo.next_scanline < cinfo.image_height) {
            row_pointer[0] = &rgb_data[cinfo.next_scanline * width * 3];
            jpeg_write_scanlines(&cinfo, row_pointer, 1);
        }

        jpeg_finish_compress(&cinfo);
        fclose(outfile);
        jpeg_destroy_compress(&cinfo);
    }

    void capture_frame() {
        v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        ioctl(fd, VIDIOC_DQBUF, &buf); // 큐에서 버퍼 꺼내기

        cout << "Frame captured. Compressing..." << endl;
        compress_to_jpeg((unsigned char*)buffers[buf.index].start, "output.jpg");

        ioctl(fd, VIDIOC_QBUF, &buf); // 큐에 버퍼 다시 넣기
    }

    ~V4L2Camera() {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd, VIDIOC_STREAMOFF, &type);
        for (auto& b : buffers) munmap(b.start, b.length);
        close(fd);
    }
};

int main() {
    // /dev/video0는 상황에 따라 다를 수 있습니다 (v4l2-ctl --list-devices로 확인)
    V4L2Camera cam("/dev/video0", 640, 480);
    cam.start_capture();
    cam.capture_frame(); // 한 프레임을 캡처하여 JPEG로 저장
    return 0;
}