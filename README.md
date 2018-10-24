# stereo_calibrator_pkr

Program to do efficient stereo image calibration and rectification. It outputs rectified images as well as camera intrinsic and extrinsic matrices.

![alt text](https://github.com/pk17r/stereo_calibrator_pkr/blob/master/StereoCalibProg.png)



### Usage

./calibrator_pkr -w [board_width] -h [board_height] -s [square_size] -p [first_img_number] -n [last_img_num] -u [left_cam_calib] -v [right_cam_calib] -L [left_img_dir] -R [right_img_dir] -l [left_img_prefix] -r [right_img_prefix] -o [output_calib_file] -d [left_rectified_img_dir] -f [right_rectified_img_dir] -j [stereo_calib_already_done_Only_do_undistortion/rectification] -x [only_do_single_camera_calib] -m [camera_calib_already_done] -u [left_cam_calib] -v [right_cam_calib] -t [Blurriness_Threshold] -a [ask_for_accept] -q [no_of_img_to_rectify] -k [minCheckerboxDistInPixel]


single camera

./calibrator_pkr -w 10 -h 7 -p 49 -n 250 -s 0.0525 -L cam3/ -R '' -l '' -r '' -o cam3.yml -e png -d r/ -f '' -j 0 -x 1 -m 0 -u '' -v '' -t 0.00015 -a 0 -q 10 -k 80

./calibrator_pkr -w 7 -h 6 -p 82 -n 179 -s 0.0925 -L cam1/ -R '' -l '' -r '' -o cam1.yml -e png -d r/ -f '' -j 0 -x 1 -m 0 -u '' -v '' -t 0.000025 -a 0 -q 10 -k 50

./calibrator_pkr -w 8 -h 6 -p 26 -n 158 -s 0.0247 -L cam3/ -R '' -l '' -r '' -o cam3.yml -e png -d r/ -f '' -j 0 -x 1 -m 0 -u '' -v '' -t 0.00005 -a 0 -q 10 -k 70

./calibrator_pkr -w 10 -h 7 -p 103 -n 246 -s 0.0525 -L cam1/ -R '' -l '' -r '' -o cam1.yml -e png -d cam1r/ -f '' -j 0 -x 1 -m 0 -u '' -v '' -t 0.00015 -a 0 -q 10 -k 50


stereo

./calibrator_pkr -w 10 -h 7 -p 55 -n 239 -s 0.0525 -L stereo13b/cam1/ -R stereo13b/cam3/ -l '' -r '' -o stereo13b/cam13calib.yml -e png -d stereo13b/cam1r/ -f stereo13b/cam3r/ -j 0 -x 0 -m 0 -u '' -v '' -t 0.00015 -a 0 -q 10 -k 70

./calibrator_pkr -w 15 -h 10 -p 46 -n 122 -s 0.0525 -L cam1/ -R cam3/ -l '' -r '' -o cam13calib.yml -e png -d cam1r/ -f cam3r/ -j 0 -x 0 -m 1 -u cam1.yml -v cam3.yml -t 0.00003 -a 0 -q 10 -k 50

./calibrator_pkr -w 8 -h 6 -p 62 -n 166 -s 0.0247 -L cam1/ -R cam2/ -l '' -r '' -o cam12calib.yml -e png -d cam1r/ -f cam2r/ -j 0 -x 0 -m 1 -u cam1.yml -v cam2.yml -t 0.00003 -a 0 -q 10 -k 50

./calibrator_pkr -w 10 -h 7 -p 42 -n 115 -s 0.0525 -L cam1/ -R cam3/ -l '' -r '' -o cam13calib.yml -e png -d cam1r/ -f cam3r/ -j 0 -x 0 -m 1 -u '' -v '' -t 0.0002 -a 0 -q 20 -k 30


only rectification

./calibrator_pkr -w 15 -h 10 -p 31 -n 32 -s 0.0525 -L cam1/ -R cam3/ -l '' -r '' -o cam13calib.yml -e png -d cam1r/ -f cam3r/ -j 1 -x 0 -m 1 -u cam2.yml -v cam3.yml -t 0.00004 -a 0 -q 5 -k 0




