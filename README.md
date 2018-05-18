## OpenCV C++ Stereo Camera Calibration

This repository contains some sources to calibrate the intrinsics of individual cameras and also the extrinsics of a stereo pair.

### Dependencies

- OpenCV
- popt

### Compilation

Compile all the files using the following commands.

```bash
mkdir build && cd build
cmake ..
make
```

### Calibration and Undistortion/Rectification

./calibrate_stereo -w [board_width] -h [board_height] -s [square_size] -n [num_imgs] -u [left_cam_calib] -v [right_cam_calib] -L [left_img_dir] -R [right_img_dir] -l [left_img_prefix] -r [right_img_prefix] -o [output_calib_file] -d [left_rectified_img_dir] -f [right_rectified_img_dir] -j [only_do_undistortion/rectification]

./calibrator_pkr -w 14 -h 7 -n 119 -s 0.164 -L cam2/ -R cam3/ -l '' -r '' -o stereo2_3.yml -e png -d rectified_cam2/ -f rectified_cam3/ -j 1

./calibrator_pkr -w 9 -h 6 -n 27 -s 0.02423 -L ../calib_imgs/1/ -R ../calib_imgs/1/ -l left -r right -o ../calib_imgs/1cam_stereo.yml -e jpg -d ../calib_imgs/1rectified/ -f ../calib_imgs/1rectified/ -j 0

./calibrator_pkr -w 9 -h 6 -n 27 -s 0.02423 -L 1/ -R 1/ -l left -r right -o 1cam_stereo.yml -e jpg -d 1rectified/ -f 1rectified/ -j 0
# stereo_calibrator_pkr
