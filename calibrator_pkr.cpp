#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "popt_pp.h"

using namespace std;
using namespace cv;

vector< vector< Point3f > > object_points;
vector< vector< Point2f > > imagePoints1, imagePoints2;
vector< Point2f > corners1, corners2;
vector< vector< Point2f > > left_img_points, right_img_points;

Mat img1, img2, gray1, gray2;
Mat gray1sml, gray2sml;

Mat K1, K2;
Mat D1, D2;
cv::Mat R1, R2, P1, P2;

void load_image_points(int board_width, int board_height, int num_imgs, float square_size,
                      char* leftimg_dir, char* rightimg_dir, char* leftimg_filename, char* rightimg_filename, char* extension) {

  Size board_size = Size(board_width, board_height);
  int board_n = board_width * board_height;
  
  for (int i = 1; i <= num_imgs; i++) {
    char left_img[100], right_img[100];
    sprintf(left_img, "%s%s%d.%s", leftimg_dir, leftimg_filename, i, extension);
    sprintf(right_img, "%s%s%d.%s", rightimg_dir, rightimg_filename, i, extension);
    img1 = imread(left_img, CV_LOAD_IMAGE_COLOR);
    img2 = imread(right_img, CV_LOAD_IMAGE_COLOR);
    if(img1.rows == 0 && img1.cols == 0 || img2.rows == 0 && img2.cols == 0) {
	  cout << i << ". could not load image! left_img: " << left_img << " " << img1.rows << "x" << img1.cols << " right_img: " << right_img << " " << img2.rows << "x" << img2.cols << endl;
	  continue;
	}
    cvtColor(img1, gray1, CV_BGR2GRAY);
    cvtColor(img2, gray2, CV_BGR2GRAY);

    bool found1 = false, found2 = false;

    found1 = cv::findChessboardCorners(img1, board_size, corners1,
  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    found2 = cv::findChessboardCorners(img2, board_size, corners2,
  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    if (found1 && found2) {
      cout << i << ". Found corners!  " << left_img << "  " << right_img << endl;
      
      cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
  cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      cv::drawChessboardCorners(gray1, board_size, corners1, found1);
      
      cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
  cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      cv::drawChessboardCorners(gray2, board_size, corners2, found2);
      
      resize(gray1, gray1sml, Size(gray1.cols * 480 /gray1.rows, 480), 0, 0, INTER_LINEAR);
      resize(gray2, gray2sml, Size(gray2.cols * 480 /gray2.rows, 480), 0, 0, INTER_LINEAR);
      imshow("Left_Img", gray1sml);
      imshow("Right_Img", gray2sml);
      waitKey(1);
      
      vector< Point3f > obj;
      for (int i = 0; i < board_height; i++)
        for (int j = 0; j < board_width; j++)
          obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));
      
      imagePoints1.push_back(corners1);
      imagePoints2.push_back(corners2);
      object_points.push_back(obj);
    }
    else {
	  cout << i << ". Cound not find corners!  " << (found1 ? "" : left_img) << "  " << (found2 ? "" : right_img) << endl;
	}
  }
  for (int i = 0; i < imagePoints1.size(); i++) {
    vector< Point2f > v1, v2;
    for (int j = 0; j < imagePoints1[i].size(); j++) {
      v1.push_back(Point2f((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
      v2.push_back(Point2f((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
    }
    left_img_points.push_back(v1);
    right_img_points.push_back(v2);
  }
  cout << "Number of Images in which Corners were found: " << imagePoints1.size() << endl;
}

void rectify_images(int num_imgs, char* leftimg_dir, char* rightimg_dir, char* leftimg_filename, char* rightimg_filename, 
                      char* extension, char* leftrectified_dir, char* rightrectified_dir) {
  cv::Mat lmapx, lmapy, rmapx, rmapy;
  cv::Mat imgU1, imgU2;

  for (int i = 1; i <= num_imgs; i++) {
    char left_img[100], right_img[100];
    sprintf(left_img, "%s%s%d.%s", leftimg_dir, leftimg_filename, i, extension);
    sprintf(right_img, "%s%s%d.%s", rightimg_dir, rightimg_filename, i, extension);
    img1 = imread(left_img, CV_LOAD_IMAGE_COLOR);
    img2 = imread(right_img, CV_LOAD_IMAGE_COLOR);
    if(img1.rows == 0 && img1.cols == 0 || img2.rows == 0 && img2.cols == 0) {
	  cout << i << ".could not load image! left_img: " << left_img << " " << img1.rows << "x" << img1.cols << " right_img: " << right_img << " " << img2.rows << "x" << img2.cols << endl;
	  continue; 
	}
    
    cvtColor(img1, gray1, CV_BGR2GRAY);
    cvtColor(img2, gray2, CV_BGR2GRAY);
    
    cv::initUndistortRectifyMap(K1, D1, R1, P1, img1.size(), CV_32F, lmapx, lmapy);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, img2.size(), CV_32F, rmapx, rmapy);
    cv::remap(img1, imgU1, lmapx, lmapy, cv::INTER_LINEAR);
    cv::remap(img2, imgU2, rmapx, rmapy, cv::INTER_LINEAR);
    
    resize(imgU1, gray1sml, Size(imgU1.cols * 480 /imgU1.rows, 480), 0, 0, INTER_LINEAR);
    resize(imgU2, gray2sml, Size(imgU2.cols * 480 /imgU2.rows, 480), 0, 0, INTER_LINEAR);
    imshow("Left_Img", gray1sml);
    imshow("Right_Img", gray2sml);
    waitKey(1);
    
    char leftout_filename[100], rightout_filename[100];
    sprintf(leftout_filename, "%s%s%d.%s", leftrectified_dir, leftimg_filename, i, extension);
    sprintf(rightout_filename, "%s%s%d.%s", rightrectified_dir, rightimg_filename, i, extension);
    imwrite(leftout_filename, imgU1);
    imwrite(rightout_filename, imgU2);
	cout << i << ".Images rectified:  " << left_img << "  and  " << right_img << endl;
  }
}

double computeReprojectionErrors(const vector< vector< Point3f > >& objectPoints,
                                 const vector< vector< Point2f > >& imagePoints,
                                 const vector< Mat >& rvecs, const vector< Mat >& tvecs,
                                 const Mat& cameraMatrix , const Mat& distCoeffs) {
  vector< Point2f > imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  vector< float > perViewErrors;
  perViewErrors.resize(objectPoints.size());

  for (i = 0; i < (int)objectPoints.size(); ++i) {
    projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                  distCoeffs, imagePoints2);
    err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float) std::sqrt(err*err/n);
    totalErr += err*err;
    totalPoints += n;
  }
  return std::sqrt(totalErr/totalPoints);
}

int main(int argc, char const *argv[])
{
  int board_width, board_height;
  float square_size;
  char* leftcalib_file;
  char* rightcalib_file;
  char* leftimg_dir;
  char* rightimg_dir;
  char* leftimg_filename;
  char* rightimg_filename;
  char* extension;
  char* out_file;
  int num_imgs;
  char* leftrectified_dir;
  char* rightrectified_dir;
  int only_rectify = 0;

  static struct poptOption options[] = {
    { "board_width",'w',POPT_ARG_INT,&board_width,0,"Checkerboard width","NUM" },
    { "board_height",'h',POPT_ARG_INT,&board_height,0,"Checkerboard height","NUM" },
    { "square_size",'s',POPT_ARG_FLOAT,&square_size,0,"Size of checkerboard square","NUM" },
    { "num_imgs",'n',POPT_ARG_INT,&num_imgs,0,"Number of checkerboard images","NUM" },
    { "leftimg_dir",'L',POPT_ARG_STRING,&leftimg_dir,0,"Directory containing left images","STR" },
    { "rightimg_dir",'R',POPT_ARG_STRING,&rightimg_dir,0,"Directory containing right images","STR" },
    { "leftimg_filename",'l',POPT_ARG_STRING,&leftimg_filename,0,"Left image prefix","STR" },
    { "rightimg_filename",'r',POPT_ARG_STRING,&rightimg_filename,0,"Right image prefix","STR" },
    { "extension",'e',POPT_ARG_STRING,&extension,0,"Image extension","STR" },
    { "out_file",'o',POPT_ARG_STRING,&out_file,0,"Output calibration filename (YML)","STR" },
    { "leftrectified_dir",'d',POPT_ARG_STRING,&leftrectified_dir,0,"Directory to save left rectified images","STR" },
    { "rightrectified_dir",'f',POPT_ARG_STRING,&rightrectified_dir,0,"Directory to save right rectified images","STR" },
    { "only_rectify",'j',POPT_ARG_INT,&only_rectify,0,"Only rectify images and not calibrate","INT" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}
  
  namedWindow("Left_Img", CV_WINDOW_AUTOSIZE);
  namedWindow("Right_Img", CV_WINDOW_AUTOSIZE);
  
  if(only_rectify == 0) {
    
    printf("Load Image Points\n");
    
    load_image_points(board_width, board_height, num_imgs, square_size,
                     leftimg_dir, rightimg_dir, leftimg_filename, rightimg_filename, extension);
    
    printf("Starting Calibration\n");
    vector< Mat > rvecs1, tvecs1, rvecs2, tvecs2;
    int flag = 0;
    flag |= CV_CALIB_FIX_K4;
    flag |= CV_CALIB_FIX_K5;
    
    calibrateCamera(object_points, left_img_points, img1.size(), K1, D1, rvecs1, tvecs1, flag);
    cout << "Left Camera Calibration error: " << computeReprojectionErrors(object_points, left_img_points, rvecs1, tvecs1, K1, D1) << endl;
    
    calibrateCamera(object_points, right_img_points, img2.size(), K2, D2, rvecs2, tvecs2, flag);
    cout << "Right Camera Calibration error: " << computeReprojectionErrors(object_points, right_img_points, rvecs2, tvecs2, K2, D2) << endl;
    
    printf("Done Intrinsics Calibration\n");
    
    printf("Starting Extrinsics Calibration\n");
    Mat R, F, E;
    Vec3d T;
    flag = 0;
    flag |= CV_CALIB_FIX_INTRINSIC;
    
    stereoCalibrate(object_points, left_img_points, right_img_points, K1, D1, K2, D2, img1.size(), R, T, E, F);
    
    cv::FileStorage fs(out_file, cv::FileStorage::WRITE);
    fs << "board_width" << board_width;
    fs << "board_height" << board_height;
    fs << "square_size" << square_size;
    fs << "K1" << K1;
    fs << "K2" << K2;
    fs << "D1" << D1;
    fs << "D2" << D2;
    fs << "R" << R;
    fs << "T" << T;
    fs << "E" << E;
    fs << "F" << F;
    
    printf("Done Calibration\n");
    
    printf("Starting Rectification\n");
    
    cv::Mat Q;
    stereoRectify(K1, D1, K2, D2, img1.size(), R, T, R1, R2, P1, P2, Q);
    
    fs << "R1" << R1;
    fs << "R2" << R2;
    fs << "P1" << P1;
    fs << "P2" << P2;
    fs << "Q" << Q;
    fs.release();
      
    cout << "K1: " << K1 << endl;
    cout << "K2: " << K2 << endl;
    cout << "D1: " << D1 << endl;
    cout << "D2: " << D2 << endl;
    cout << "R1: " << R1 << endl;
    cout << "R2: " << R2 << endl;
    cout << "P1: " << P1 << endl;
    cout << "P2: " << P2 << endl;
    
    printf("Done Rectification\n");
    
  }
  else
  {
    cv::FileStorage fs(out_file, cv::FileStorage::READ);
    fs["K1"] >> K1;
    fs["K2"] >> K2;
    fs["D1"] >> D1;
    fs["D2"] >> D2;
    
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs.release();
    
    cout << "K1: " << K1 << endl;
    cout << "K2: " << K2 << endl;
    cout << "D1: " << D1 << endl;
    cout << "D2: " << D2 << endl;
    cout << "R1: " << R1 << endl;
    cout << "R2: " << R2 << endl;
    cout << "P1: " << P1 << endl;
    cout << "P2: " << P2 << endl;
    
    printf("Read rectification parameters\n");
  }
  
  printf("Starting rectifying images\n");
    
  rectify_images(num_imgs, leftimg_dir, rightimg_dir, leftimg_filename, rightimg_filename, 
                      extension, leftrectified_dir, rightrectified_dir);
  
  printf("Saved rectified images\n");
  
  return 0;
}