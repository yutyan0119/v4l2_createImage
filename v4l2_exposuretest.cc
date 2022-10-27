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

#define WIDTH 1280
#define HEIGHT 720

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
/*for auto exposure*/
float cur_ev[3] = {1000.0,1000.0,1000.0};
int frame = 0;
float target_grey_fraction = 0.3;
int now_gain = 1;
float min_ev = 5 * 1.0;
float max_ev = 500 * 500.0;

float set_exposure_target(cv::Mat b, int x_start, int x_end, int x_skip, int y_start, int y_end, int y_skip) {
    int lum_med;
    uint32_t lum_binning[256] = {0};

    unsigned int lum_total = 0;
    for (int y = y_start; y < y_end; y += y_skip) {
        for (int x = x_start; x < x_end; x += x_skip) {
        uint8_t lum = b.data[y * WIDTH + x];
        lum_binning[lum]++;
        lum_total += 1;
        }
    }

    // Find mean lumimance value
    unsigned int lum_cur = 0;
    for (lum_med = 255; lum_med >= 0; lum_med--) {
    lum_cur += lum_binning[lum_med];
        if (lum_cur >= lum_total / 2) {
            break;
        }
    }

    return lum_med / 256.0;
}

void camera_autoexposure(float grey_frac) {
  const float dt = 0.05;

  const float ts_grey = 10.0;
  const float ts_ev = 0.05;

  const float k_grey = (dt / ts_grey) / (1.0 + dt / ts_grey);
  const float k_ev = (dt / ts_ev) / (1.0 + dt / ts_ev);

  const float cur_ev_ = cur_ev[frame % 3];

  float new_target_grey = std::max(0.1, std::min(0.9 - 0.4 * log2(1 + cur_ev_) / log2(6000.0), 0.9));
  float target_grey = (1 - k_grey) * target_grey_fraction + k_grey * new_target_grey;

  float desired_ev = std::max(min_ev , std::min(cur_ev_ * target_grey / grey_frac,  max_ev));
  float k = (1.0 - k_ev) / 3.0;
  desired_ev = (k * cur_ev[0]) + (k * cur_ev[1]) + (k * cur_ev[2]) + (k_ev * desired_ev);

  float best_ev_score = 1e6;
  int new_g = 0;
  int new_t = 0;
  printf("mean grey = %f, target_gery = %f, desired_ev = %f\n", grey_frac, target_grey, desired_ev);

  for (int gain = std::max(0, now_gain - 5); gain <= std::min(30, now_gain + 5); gain++) {
    int temp = std::min((int)desired_ev/(gain+10), 500);
    int t = std::max(1, temp);

    float score = std::abs(desired_ev - (t * (gain + 10))) * 10;
    float score2 = std::abs(desired_ev - ((t+1) * (gain+10)))*10;
    if (score > score2){
      score = score2;
      t++;
    }

    float m;
    if (gain > 20 ) {
      m = 20.0;
    } else if (gain > 10 ) {
      m = 20.0;
    } else if (gain > 0 ) {
      m = 20.0;
    } else {
      m = 1.0;
    }
    // 0 is desired gain
    score += gain * m;
    score += std::abs(gain - now_gain) * (score + 1.0) / 10.0;

    if (score < best_ev_score) {
      new_t = t;
      new_g = gain;
      best_ev_score = score;
    }
  }
  cur_ev[frame % 3] = (new_g + 10) * new_t;
  target_grey_fraction = target_grey;
  now_gain = new_g;
  printf("new_gain = %d, new_exposure_time = %d ms\n", new_g, new_t / 10);
  struct v4l2_control ctrl;
  ctrl.id = V4L2_CID_EXPOSURE_AUTO;
  ctrl.value = 1;
  ioctl(fd, VIDIOC_S_CTRL, &ctrl);
  ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  ctrl.value = new_t;
  ioctl(fd, VIDIOC_S_CTRL, &ctrl);
  ctrl.id = V4L2_CID_GAIN;
  ctrl.value = new_g;
  ioctl(fd, VIDIOC_S_CTRL, &ctrl);
}


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
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_EXPOSURE_AUTO;
    ctrl.value = 1;
    ioctl(fd, VIDIOC_S_CTRL, &ctrl);
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl.value = 100;
    ioctl(fd, VIDIOC_S_CTRL, &ctrl);
    ctrl.id = V4L2_CID_GAIN;
    ctrl.value = 1;
    ioctl(fd, VIDIOC_S_CTRL, &ctrl);
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
        printf("%dx%d\n", fmt.fmt.pix.width, fmt.fmt.pix.height);
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
    cv::Mat grey;
    cv::cvtColor(decodedImage, grey, cv::COLOR_RGB2GRAY);
    camera_autoexposure(set_exposure_target(grey, 0, 1280, 2, 100, 600, 2));
    /*next enqueue index return*/
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
        frame++;
    }
    stream_stop();
    munmap_buffer();
    close_device();
    return EXIT_SUCCESS;
}
