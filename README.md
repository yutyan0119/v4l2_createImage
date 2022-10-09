# v4l2_createImage
This program has been tested in the followin environments.
If you want to hadle image on opencv, you can use `v4l2_opencv.cc`.

```
Ubuntu 20.04 Lazer Stealth 13
```

## Requirements
Camera Device
### Optional
opencv (Version 4.0)

## Compile
```bash
gcc -o sample ./v4l2sample.c
```

or

```bash
g++ -o opencv v4l2_opencv.cc -I /usr/include/ -I /usr/local/include/opencv4/ -lopencv_core -lopencv_highgui -lopencv_videoio -lopencv_imgproc -lopencv_imgcodecs
```

## Execute
```bash
./sample

./opencv
```

If you want to test this program on your camera, you may change device file.
In `open_device` function, you can assign device file you want to do so.