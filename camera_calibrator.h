#pragma once

#include <vector>
#include <iostream>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>

std::vector<std::vector<cv::Point3f>> objectPoints;
std::vector<std::vector<cv::Point2f>> imagePoints;
// output Matrices
cv::Mat cameraMatrix;
cv::Mat distCoeffs;



class CameraCalibrator {

	// input points

	// flag to specify how calibration is done
	int flag;
	// used in image undistortion 
	cv::Mat map1, map2;
	bool mustInitUndistort;


public:
	CameraCalibrator() : flag(0), mustInitUndistort(true) {};


	cv::Vec3d triangulate(const cv::Mat &p1, const cv::Mat &p2, const cv::Vec2d &u1, const cv::Vec2d &u2);
	void triangulate(const cv::Mat &p1, const cv::Mat &p2, const std::vector<cv::Vec2d> &pts1, const std::vector<cv::Vec2d> &pts2, std::vector<cv::Vec3d> &pts3D);
	// Open the chessboard images and extract corner points
	int addChessboardPoints(const std::vector<std::string>& filelist, cv::Size & boardSize);
	// Add scene points and corresponding image points
	void addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners);
	// Calibrate the camera
	double calibrate(cv::Size &imageSize);
	// Set the calibration flag
	void setCalibrationFlag(bool radial8CoeffEnabled = false, bool tangentialParamEnabled = false);
	// Remove distortion in an image (after calibration)
	cv::Mat remap(const cv::Mat &image);
	void takeChessboardImages();
	void takeStereoPair();
	void ReconstructPointCloud();

	// Getters
	cv::Mat getCameraMatrix() { return cameraMatrix; }
	cv::Mat getDistCoeffs() { return distCoeffs; }
};

