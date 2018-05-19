# stereo_calibrator_pkr

Program to do efficient stereo image calibration and rectification. It outputs rectified images as well as camera intrinsic and extrinsic matrices.

### Calibration and Undistortion/Rectification

./calibrate_stereo -w [board_width] -h [board_height] -s [square_size] -p [first_img_number] -n [last_img_num] -u [left_cam_calib] -v [right_cam_calib] -L [left_img_dir] -R [right_img_dir] -l [left_img_prefix] -r [right_img_prefix] -o [output_calib_file] -d [left_rectified_img_dir] -f [right_rectified_img_dir] -j [stereo_calib_already_done_Only_do_undistortion/rectification] -x [only_do_single_camera_calib] -m [camera_calib_already_done] -u [left_cam_calib] -v [right_cam_calib]

./calibrator_pkr -w 14 -h 7 -p 1 -n 119 -s 0.164 -L cam2/ -R cam3/ -l '' -r '' -o stereo2_3.yml -e png -d rectified_cam2/ -f rectified_cam3/ -j 1

./calibrator_pkr -w 9 -h 6  -p 1 -n 27 -s 0.02423 -L ../calib_imgs/1/ -R ../calib_imgs/1/ -l left -r right -o ../calib_imgs/1cam_stereo.yml -e jpg -d ../calib_imgs/1rectified/ -f ../calib_imgs/1rectified/ -j 0

./calibrator_pkr -w 9 -h 6  -p 1 -n 27 -s 0.02423 -L 1/ -R 1/ -l left -r right -o 1cam_stereo.yml -e jpg -d 1rectified/ -f 1rectified/ -j 0

./calibrator_pkr -w 14 -h 7 -p 291 -n 300 -s 0.164 -L cam1/ -R cam2/ -l '' -r '' -o stereo_calib.yml -e png -d cam1r/ -f cam2r/ -j 0


./calibrator_pkr -w 14 -h 7 -p 1 -n 300 -s 0.164 -L cam1/ -R cam2/ -l '' -r '' -o stereo_calib.yml -e png -d cam1r/ -f cam2r/ -j 0 -x 0 -m 0 -u '' -v ''


/home/pkr/pkr-work/focus_at_25m_(15m & 35m in focus)/single_cam_calibration_2/cam1 (copy)

./calibrator_pkr -w 14 -h 7 -p 1 -n 170 -s 0.164 -L "/home/pkr/pkr-work/stereo-calibration/calib_imgs/" -R "/home/pkr/pkr-work/stereo-calibration/calib_imgs/" -l left -r right -o "/home/pkr/pkr-work/stereo-calibration/calib_imgs/may18stereo.yml" -e png -d "/home/pkr/pkr-work/stereo-calibration/calib_imgs/1rectified/" -f "/home/pkr/pkr-work/stereo-calibration/calib_imgs/1rectified/" -j 0 -x 1 -m 0 -u '' -v ''
