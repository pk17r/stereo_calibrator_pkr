#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "popt_pp.h"
#include <string>

using namespace std;
using namespace cv;

vector< vector< Point3f > > object_points;
vector< vector< Point2f > > imagePoints1, imagePoints2;
vector< Point2f > corners1, corners2;
vector< vector< Point2f > > left_img_points, right_img_points;

Mat img1, img2, gray1, gray2;
Mat gray1sml, gray2sml;
Mat img1covered, img2covered, img1covered_last, img2covered_last, img1coveredsml, img2coveredsml;

Mat K1, K2;
Mat D1, D2;
cv::Mat R1, R2, P1, P2;
float BlurThreshold = 0.000043;

Point2f ptA, ptB, ptC, ptD;
Point2i pt1, pt2;
Mat cropped;
//bool shift = false;

float calcBlurriness( const Mat &src )
{
  Mat Gx, Gy;
  Sobel( src, Gx, CV_32F, 1, 0 );
  Sobel( src, Gy, CV_32F, 0, 1 );
  double normGx = norm( Gx );
  double normGy = norm( Gy );
  double sumSq = normGx * normGx + normGy * normGy;
  return static_cast<float>( 1. / ( sumSq / src.size().area() + 1e-6 ));
}

float extractCheckerBox(int board_width, int board_height, vector< Point2f > corners, 
                        Mat gray, vector< Point> * contour) {
  ptA = corners[0];
  ptB = corners[board_width-1];
  ptC = corners[corners.size()-board_width];
  ptD = corners[corners.size()-1];
  
  //polylines
  (*contour).push_back(ptA);
  (*contour).push_back(ptB);
  (*contour).push_back(ptD);
  (*contour).push_back(ptC);
  
  //char strA[20], strB[20], strC[20], strD[20];
  //sprintf(strA, "A(%0.1f,%0.1f)", ptA.x, ptA.y);
  //sprintf(strB, "B(%0.1f,%0.1f)", ptB.x, ptB.y);
  //sprintf(strC, "C(%0.1f,%0.1f)", ptC.x, ptC.y);
  //sprintf(strD, "D(%0.1f,%0.1f)", ptD.x, ptD.y);
  //putText(img2, strA, ptA, FONT_HERSHEY_PLAIN, 3, Scalar(0,0,255), 2, 8, false);
  //putText(img2, strB, ptB, FONT_HERSHEY_PLAIN, 3, Scalar(0,0,255), 2, 8, false);
  //putText(img2, strC, ptC, FONT_HERSHEY_PLAIN, 3, Scalar(0,0,255), 2, 8, false);
  //putText(img2, strD, ptD, FONT_HERSHEY_PLAIN, 3, Scalar(0,0,255), 2, 8, false);
  //line(img2, ptA, ptB, Scalar(255,0,0), 2, 8, 0);
  //line(img2, ptB, ptD, Scalar(255,0,0), 2, 8, 0);
  //line(img2, ptD, ptC, Scalar(255,0,0), 2, 8, 0);
  //line(img2, ptC, ptA, Scalar(255,0,0), 2, 8, 0);
  //line(img2, ptA, ptD, Scalar(255,0,0), 2, 8, 0);
  //line(img2, ptB, ptC, Scalar(255,0,0), 2, 8, 0);
  
  //bigger rectangle which takes in all corners
  pt1 = Point2i((int)min(ptB.x,ptD.x),(int)min(ptD.y,ptC.y));
  pt2 = Point2i((int)max(ptC.x,ptA.x),(int)max(ptA.y,ptB.y));
  
  //rectangle(img2, pt1, pt2, Scalar(0,255,0), 2, 8, 0);
  
  //if(shift) {
  //  string imname = "cam2blurCheck/" + to_string(i) + ".png";
  //  imwrite(imname,cropped);
  //}
  //shift = !shift;
  
  cropped = Mat(gray, Rect(pt1, pt2));
  return calcBlurriness(cropped);
}

void load_image_points(int board_width, int board_height, int first_img, int num_imgs, 
                      float square_size, char* leftimg_dir, char* rightimg_dir, 
                      char* leftimg_filename, char* rightimg_filename, char* extension,
                      bool single_camera) {

  Size board_size = Size(board_width, board_height);
  int board_n = board_width * board_height;
  float img1Blur,img2Blur;
  int n_corners_found = 0;
  
  for (int i = first_img; i <= num_imgs; i++) {
    char left_img[100], right_img[100];
    
    sprintf(left_img, "%s%s%d.%s", leftimg_dir, leftimg_filename, i, extension);
    if(!single_camera) sprintf(right_img, "%s%s%d.%s", rightimg_dir, rightimg_filename, i, extension);

    img1 = imread(left_img, CV_LOAD_IMAGE_COLOR);
    if(!single_camera) img2 = imread(right_img, CV_LOAD_IMAGE_COLOR);
    else img2 = img1;

    if(img1.rows == 0 && img1.cols == 0 || img2.rows == 0 && img2.cols == 0) {
	  cout << i << ". could not load image! left_img: " << left_img << " " << 
	    img1.rows << "x" << img1.cols << " right_img: " << right_img << " " << 
	    img2.rows << "x" << img2.cols << endl;
	  continue;
	}
	
    cvtColor(img1, gray1, CV_BGR2GRAY);
    if(!single_camera) cvtColor(img2, gray2, CV_BGR2GRAY);
    
    //populate the img1covered, img2covered Mats
	if(i == first_img)
	{
	  img1covered = Mat(gray1.size(),CV_8U,Scalar(1));
	  img2covered = img1covered.clone();
	  //imshow("Left_Img", img1covered);
      //waitKey(5000);
	}


    bool found1 = false, found2 = false;

    found1 = cv::findChessboardCorners(img1, board_size, corners1,
                          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    if(!single_camera) found2 = cv::findChessboardCorners(img2, board_size, corners2,
                          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    else found2 = found1;

    if (found1 && found2) {
      if(!single_camera) cout << i << ". Found corners!  " << left_img << "  " << right_img;
      else cout << i << ". Found corners!  " << left_img;
      
      n_corners_found++;
      
      //keeping a copy of gray images, to not have Chessboard draw on it
      img1 = gray1.clone();
      if(!single_camera) img2 = gray2.clone();
      
      cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
                       cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      cv::drawChessboardCorners(gray1, board_size, corners1, found1);
      
      if(!single_camera) cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
                       cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      if(!single_camera) cv::drawChessboardCorners(gray2, board_size, corners2, found2);
      
      resize(gray1, gray1sml, Size(gray1.cols * 480 /gray1.rows, 480), 0, 0, INTER_LINEAR);
      if(!single_camera) resize(gray2, gray2sml, Size(gray2.cols * 480 /gray2.rows, 480), 0, 0, INTER_LINEAR);
      imshow("Left_Img", gray1sml);
      if(!single_camera) imshow("Right_Img", gray2sml);
      waitKey(1);
      
      //calculate blurriness, reject if blur is higher than a threshold
      vector< Point> contour1, contour2;
      img1Blur = extractCheckerBox(board_width, board_height, corners1, img1, &contour1);
      if(!single_camera) img2Blur = extractCheckerBox(board_width, board_height, corners2, img2, &contour2);
      if(!single_camera) printf("  Blurriness  %0.8f  %0.8f",img1Blur, img2Blur);
      else printf("  Blurriness  %0.8f",img1Blur);
      
      if(img1Blur > BlurThreshold || img2Blur > BlurThreshold) {
        printf("  Rejected\n");
        continue;
      }
      else
      {
		printf("\n");
		// draw the polygon 
        const Point *pts1 = (const cv::Point*) Mat(contour1).data;
        int npts1 = Mat(contour1).rows;
        polylines(img1covered, &pts1, &npts1, 1, true, Scalar(255));
        if(img1covered.cols > 480) resize(img1covered, img1coveredsml, Size(img1covered.cols * 480 /img1covered.rows, 480), 0, 0, INTER_LINEAR);
        imshow("Left_Img_Calibration", (img1covered.cols > 480 ? img1coveredsml : img1covered));
        
        if(!single_camera) {
		  const Point *pts2 = (const cv::Point*) Mat(contour2).data;
          int npts2 = Mat(contour2).rows;
          polylines(img2covered, &pts2, &npts2, 1, true, Scalar(255));
		  if(img2covered.cols > 480) resize(img2covered, img2coveredsml, Size(img2covered.cols * 480 /img2covered.rows, 480), 0, 0, INTER_LINEAR);
		  
          imshow("Right_Img_Calibration", (img2covered.cols > 480 ? img2coveredsml : img2covered));
		}
        waitKey(1);
	  }
      
      vector< Point3f > obj;
      for (int i = 0; i < board_height; i++)
        for (int j = 0; j < board_width; j++)
          obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));
      
      imagePoints1.push_back(corners1);
      if(!single_camera) imagePoints2.push_back(corners2);
      object_points.push_back(obj);
      
    }
    else {
      resize(img1, gray1sml, Size(gray1.cols * 480 /gray1.rows, 480), 0, 0, INTER_LINEAR);
      if(!single_camera) resize(img2, gray2sml, Size(gray2.cols * 480 /gray2.rows, 480), 0, 0, INTER_LINEAR);
      imshow("Left_Img", gray1sml);
      if(!single_camera) imshow("Right_Img", gray2sml);
      waitKey(1);
      if(!single_camera) cout << i << ". Could not find corners!  " << (found1 ? "" : left_img) << "  " << (found2 ? "" : right_img) << endl;
      else cout << i << ". Could not find corners!  " << (found1 ? "" : left_img) << endl;
	}
  }
  
  for (int i = 0; i < imagePoints1.size(); i++) {
    vector< Point2f > v1, v2;
    for (int j = 0; j < imagePoints1[i].size(); j++) {
      v1.push_back(Point2f((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
      if(!single_camera) v2.push_back(Point2f((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
    }
    left_img_points.push_back(v1);
    if(!single_camera) right_img_points.push_back(v2);
  }
  printf("Number of Images in which Corners were found: %i\n", n_corners_found);
  printf("Number of Images below the Blurriness Threshold: %lu\n", imagePoints1.size());
  imwrite("leftCamCoverage.png",img1covered);
  if(!single_camera) imwrite("rightCamCoverage.png",img2covered);
}

void rectify_images(int first_img, int num_imgs, char* leftimg_dir, char* rightimg_dir, char* leftimg_filename, char* rightimg_filename, 
                      char* extension, char* leftrectified_dir, char* rightrectified_dir, bool single_camera) {
  cv::Mat lmapx, lmapy, rmapx, rmapy;
  cv::Mat imgU1, imgU2;

  for (int i = first_img; i <= num_imgs; i++) {
    char left_img[100], right_img[100];
    sprintf(left_img, "%s%s%d.%s", leftimg_dir, leftimg_filename, i, extension);
    if(!single_camera) sprintf(right_img, "%s%s%d.%s", rightimg_dir, rightimg_filename, i, extension);
    
    img1 = imread(left_img, CV_LOAD_IMAGE_COLOR);
    if(!single_camera) img2 = imread(right_img, CV_LOAD_IMAGE_COLOR);
    else img2 = img1;
    
    if(img1.rows == 0 && img1.cols == 0 || img2.rows == 0 && img2.cols == 0) {
	  if(!single_camera) cout << i << ".could not load image! left_img: " << left_img << " " << img1.rows << "x" << img1.cols << " right_img: " << right_img << " " << img2.rows << "x" << img2.cols << endl;
	  else cout << i << ".could not load image! left_img: " << left_img << " " << img1.rows << "x" << img1.cols << endl;
	  continue; 
	}
    
    cvtColor(img1, gray1, CV_BGR2GRAY);
    if(!single_camera) cvtColor(img2, gray2, CV_BGR2GRAY);
    
    if(!single_camera)
    {            
      initUndistortRectifyMap(K1, D1, R1, P1, img1.size(), CV_32F, lmapx, lmapy);
      initUndistortRectifyMap(K2, D2, R2, P2, img2.size(), CV_32F, rmapx, rmapy);
      remap(img1, imgU1, lmapx, lmapy, cv::INTER_LINEAR);
      remap(img2, imgU2, rmapx, rmapy, cv::INTER_LINEAR);
    }
    else
    {
	  initUndistortRectifyMap(
                K1, D1, Mat(),
                getOptimalNewCameraMatrix(K1, D1, img1.size(), 1, img1.size(), 0), img1.size(),
                CV_32F, lmapx, lmapy);
	}
    
    resize(imgU1, gray1sml, Size(imgU1.cols * 480 /imgU1.rows, 480), 0, 0, INTER_LINEAR);
    if(!single_camera) resize(imgU2, gray2sml, Size(imgU2.cols * 480 /imgU2.rows, 480), 0, 0, INTER_LINEAR);
    imshow("Left_Img", gray1sml);
    if(!single_camera) imshow("Right_Img", gray2sml);
    waitKey(1);
    
    char leftout_filename[100], rightout_filename[100];
    sprintf(leftout_filename, "%s%s%d.%s", leftrectified_dir, leftimg_filename, i, extension);
    if(!single_camera) sprintf(rightout_filename, "%s%s%d.%s", rightrectified_dir, rightimg_filename, i, extension);
    imwrite(leftout_filename, imgU1);
    if(!single_camera) imwrite(rightout_filename, imgU2);
	if(!single_camera) cout << i << ".Images rectified:  " << left_img << "  and  " << right_img << endl;
	else cout << i << ".Images rectified:  " << left_img << endl;
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
  int num_imgs, first_img;
  char* leftrectified_dir;
  char* rightrectified_dir;
  int only_rectify = 0, single_camera_calib = 0, camera_calib_already_done = 0;
  
  static struct poptOption options[] = {
    { "board_width",'w',POPT_ARG_INT,&board_width,0,"Checkerboard width","NUM" },
    { "board_height",'h',POPT_ARG_INT,&board_height,0,"Checkerboard height","NUM" },
    { "square_size",'s',POPT_ARG_FLOAT,&square_size,0,"Size of checkerboard square","NUM" },
    { "first_img",'p',POPT_ARG_INT,&first_img,0,"First checkerboard image number","NUM" },
    { "num_imgs",'n',POPT_ARG_INT,&num_imgs,0,"Last checkerboard image number","NUM" },
    { "leftimg_dir",'L',POPT_ARG_STRING,&leftimg_dir,0,"Directory containing left images","STR" },
    { "rightimg_dir",'R',POPT_ARG_STRING,&rightimg_dir,0,"Directory containing right images","STR" },
    { "leftimg_filename",'l',POPT_ARG_STRING,&leftimg_filename,0,"Left image prefix","STR" },
    { "rightimg_filename",'r',POPT_ARG_STRING,&rightimg_filename,0,"Right image prefix","STR" },
    { "extension",'e',POPT_ARG_STRING,&extension,0,"Image extension","STR" },
    { "out_file",'o',POPT_ARG_STRING,&out_file,0,"Output calibration filename (YML)","STR" },
    { "leftrectified_dir",'d',POPT_ARG_STRING,&leftrectified_dir,0,"Directory to save left rectified images","STR" },
    { "rightrectified_dir",'f',POPT_ARG_STRING,&rightrectified_dir,0,"Directory to save right rectified images","STR" },
    { "only_rectify",'j',POPT_ARG_INT,&only_rectify,0,"Stereo Calibration already done. Only rectify images and not calibrate","INT" },
    { "single_camera_calib",'x',POPT_ARG_INT,&single_camera_calib,0,"Only do single camera calibration","INT" },
    { "camera_calib_already_done",'m',POPT_ARG_INT,&camera_calib_already_done,0,"Single camera calibration already done. Take intrinsic inputs from xml files.","INT" },
    { "leftcalib_file",'u',POPT_ARG_STRING,&leftcalib_file,0,"Left camera calibration","STR" },
    { "rightcalib_file",'v',POPT_ARG_STRING,&rightcalib_file,0,"Right camera calibration","STR" },
    { "BlurThreshold",'t',POPT_ARG_FLOAT,&BlurThreshold,0,"Blurriness Threshold","NUM" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}
  
  bool single_camera = single_camera_calib != 0;
  bool do_camera_calibration = camera_calib_already_done == 0;
  
  namedWindow("Left_Img", CV_WINDOW_AUTOSIZE);
  moveWindow("Left_Img", 100, 20);
  if(!single_camera) namedWindow("Right_Img", CV_WINDOW_AUTOSIZE);
  if(!single_camera) moveWindow("Right_Img", 800, 20);
  namedWindow("Left_Img_Calibration", CV_WINDOW_AUTOSIZE);
  moveWindow("Left_Img_Calibration", 100, 550);
  if(!single_camera) namedWindow("Right_Img_Calibration", CV_WINDOW_AUTOSIZE);
  if(!single_camera) moveWindow("Right_Img_Calibration", 800, 550);
  
  if(only_rectify == 0)
  {
    
    printf("Load Image Points\n");
    
    load_image_points(board_width, board_height, first_img, num_imgs, square_size,
                     leftimg_dir, rightimg_dir, leftimg_filename, rightimg_filename, extension, single_camera);
    
    if(do_camera_calibration)
    {
	  printf("Starting Intrinsics Calibration\n");
      vector< Mat > rvecs1, tvecs1, rvecs2, tvecs2;
      int flag = 0;
      flag |= CV_CALIB_FIX_K4;
      flag |= CV_CALIB_FIX_K5;
      
      calibrateCamera(object_points, left_img_points, img1.size(), K1, D1, rvecs1, tvecs1, flag);
      cout << "Left Camera Calibration error: " << computeReprojectionErrors(object_points, left_img_points, rvecs1, tvecs1, K1, D1) << endl;
      
      if(!single_camera) calibrateCamera(object_points, right_img_points, img2.size(), K2, D2, rvecs2, tvecs2, flag);
      if(!single_camera) cout << "Right Camera Calibration error: " << computeReprojectionErrors(object_points, right_img_points, rvecs2, tvecs2, K2, D2) << endl;
      
      printf("Done Intrinsics Calibration.\n");
    }
    else
    {
	  FileStorage fsl(leftcalib_file, FileStorage::READ);
      FileStorage fsr(rightcalib_file, FileStorage::READ);
      fsl["K"] >> K1;
      fsr["K"] >> K2;
      fsl["D"] >> D1;
      fsr["D"] >> D2;
      fsl.release();
      fsr.release();
      printf("Read Intrinsics Calibration.\n");
	}
    
    if(!single_camera) printf("Starting Extrinsics Calibration...\n");
    
    if(!single_camera)
    {
      Mat R, F, E;
      Vec3d T;
      int flag = 0;
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
      
      printf("Done Extrinsics Calibration. Starting Rectification...\n");
      
      cv::Mat Q;
      stereoRectify(K1, D1, K2, D2, img1.size(), R, T, R1, R2, P1, P2, Q);
      
      fs << "R1" << R1;
      fs << "R2" << R2;
      fs << "P1" << P1;
      fs << "P2" << P2;
      fs << "Q" << Q;
      fs.release();
        
      //cout << "K1: " << K1 << endl;
      //cout << "K2: " << K2 << endl;
      //cout << "D1: " << D1 << endl;
      //cout << "D2: " << D2 << endl;
      //cout << "R1: " << R1 << endl;
      //cout << "R2: " << R2 << endl;
      //cout << "P1: " << P1 << endl;
      //cout << "P2: " << P2 << endl;
      
      printf("Rectification Done. Saved parameters. Starting undistortion...\n");
    }
    else
    {
	  cv::FileStorage fs(out_file, cv::FileStorage::WRITE);
      fs << "board_width" << board_width;
      fs << "board_height" << board_height;
      fs << "square_size" << square_size;
      fs << "K" << K1;
      fs << "D" << D1;
      fs.release();
    }
  }
  else
  {
    cv::FileStorage fs(out_file, cv::FileStorage::READ);
    if(!single_camera)
    {
      fs["K1"] >> K1;
      fs["K2"] >> K2;
      fs["D1"] >> D1;
      fs["D2"] >> D2;
      
      fs["R1"] >> R1;
      fs["R2"] >> R2;
      fs["P1"] >> P1;
      fs["P2"] >> P2;
      
      cout << "K1: " << K1 << endl;
      cout << "K2: " << K2 << endl;
      cout << "D1: " << D1 << endl;
      cout << "D2: " << D2 << endl;
      cout << "R1: " << R1 << endl;
      cout << "R2: " << R2 << endl;
      cout << "P1: " << P1 << endl;
      cout << "P2: " << P2 << endl;
    }
    else
    {
	  fs["K"] >> K1;
      fs["D"] >> D1;
      
      cout << "K: " << K1 << endl;
      cout << "D: " << D1 << endl;
	}
    fs.release();
    
    
    printf("Read rectification parameters. Starting undistortion...\n");
  }
      
  rectify_images(first_img, num_imgs, leftimg_dir, rightimg_dir, leftimg_filename, rightimg_filename, 
                      extension, leftrectified_dir, rightrectified_dir, single_camera);
  
  printf("Saved rectified images\n");
  
  return 0;
}
