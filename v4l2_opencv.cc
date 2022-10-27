#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h> 
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <poll.h>
#include <linux/videodev2.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#define WIDTH 1600
#define HEIGHT 1200

struct buffer {
    void* start;
    size_t length;
};

static int xioctl(int fd, int request, void *arg){
    int r;
    do r = ioctl(fd,request,arg);
    while (-1 == r && EINTR == errno);
    return r;
}

/*global variables*/
/*buffer pointer struct*/
struct buffer *buffers;
/*file descriptor*/
int fd;
/*the number of buffers*/
int n_buffers;

void open_device(){
    fd = open("/dev/video0", O_RDWR);
    if (fd == -1){
        perror("device open");
    }
}

void cap_device(){
    struct v4l2_capability cap;
    if (-1 == xioctl(fd,VIDIOC_QUERYCAP,&cap)){
        perror("query capability");
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		perror("no video capture");
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)){
		perror("does not support stream");
    }

}

void set_device(){
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = WIDTH;
    fmt.fmt.pix.height = HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    /* set format */
    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)){
        perror("Setting Pixel Format");
    }

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    /* get format */
    if (-1 == xioctl(fd, VIDIOC_G_FMT,&fmt)){
            perror("get format");
    }
    if (fmt.fmt.pix.width != WIDTH || fmt.fmt.pix.height != HEIGHT || fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG){
        printf("The desired format is not supported\n");
    }
}

void request_buffer(){
    struct v4l2_requestbuffers req = {0};
    req.count = 3;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (-1 == xioctl(fd,VIDIOC_REQBUFS,&req)){
        perror("Requesting Buffer");
    }
    /* 確保できた枚数の確認 */
    n_buffers = req.count;
    if (req.count < 3) {
        fprintf(stderr, "Insufficient buffer memory on camera\n");
    }
}

void map_buffer(){
    buffers = (struct buffer*)calloc(n_buffers, sizeof(*buffers));
    if (buffers == NULL){
        printf("okasii");
    }
    for (int n_buffer = 0; n_buffer < n_buffers; n_buffer++){
        struct v4l2_buffer buff = {0};
        buff.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buff.memory = V4L2_MEMORY_MMAP;
        buff.index = n_buffer;
        if (-1 == xioctl(fd,VIDIOC_QUERYBUF,&buff)){
            perror("Querying Buffer");
        }
        buffers[n_buffer].length = buff.length;
        buffers[n_buffer].start = mmap (NULL, buff.length, PROT_READ | PROT_WRITE , MAP_SHARED, fd, buff.m.offset);
    }
}

void enqueue_buffer(int index){
    struct v4l2_buffer buf;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;
    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        perror("VIDIOC_QBUF");
}

void enqueue_buffers(){
    for (int n_buffer = 0; n_buffer < n_buffers; n_buffer++){
        enqueue_buffer(n_buffer);
    }
}

void stream_start(){
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type)){
        perror("VIDIOC_STREAMON");
    }
}

int make_image(){
    struct pollfd fds[1];
/* ファイルディスクリプタと読み込み可能であるというイベントを予め入れておく*/ 
    fds[0].fd = fd;
    fds[0].events = POLLIN;
    int p = poll(fds,1,5000);
    if (-1 == p){
        perror("Waiting for Frame");
        return -1;
    }
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
        perror("Retrieving Frame");
        return -1;
    }
    int size = buffers[buf.index].length;
    cv::Mat src(1, size , CV_8UC1 ,((uint8_t *)buffers[buf.index].start));
    cv::InputArray hoge(src);
    cv::Mat decodedImage = cv::imdecode(hoge,cv::IMREAD_ANYCOLOR);
    cv::imshow("test",decodedImage);
    return buf.index;
}

void stream_stop(){
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type)){
		perror("stream stop");
    }
}

void munmap_buffer(){
    for (int i = 0; i < n_buffers; ++i){
        if (-1 == munmap(buffers[i].start, buffers[i].length)){
				perror("munmap");
        }
    }
    free(buffers);
}

void close_device(){
    close(fd);
}

int main(){
    open_device();
    cap_device();
    set_device();
    request_buffer();
    map_buffer();
    enqueue_buffers();
    stream_start();
    while(true){
   	 int enqueue_index = make_image();
   	 if (enqueue_index == -1){
            return EXIT_FAILURE;
    	 }
    	enqueue_buffer(enqueue_index);
	if (cv::waitKey(10)!=-1){
		break;
	}
    }
    stream_stop();
    munmap_buffer();
    close_device();
    return EXIT_SUCCESS;
}
