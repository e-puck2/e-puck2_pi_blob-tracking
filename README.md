# e-puck2_pi_blob-tracking
Simple blob tracking example based on OpenCV 3.

## How to build
g++ $(pkg-config --libs --cflags opencv) -ljpeg -o omni_detect_blob omni_detect_blob.cpp

## How to run
sudo modprobe bcm2835-v4l2

./omni_detect_blob [H min] [H max] [S min] [S max] [V min] [V max] [center x] [center y] [radius1] [radius2] [debug]
H range = 0..180
S range = 0..255
V range = 0..255
center x,y = center of the circle representing the usable image
radius1 = radius of the circle covering the robot
radius2 = radius of the circle covering the actual image
debug = 0 is disabled, 1 is enabled
