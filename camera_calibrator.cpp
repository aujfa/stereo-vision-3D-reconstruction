#include "camera_calibrator.h"
#include "get_disparity.cpp"

#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp" 
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/viz.hpp>

using namespace std;
using namespace cv;

std::vector<cv::Mat> rvecs, tvecs;
int cameraIndex1 = 2, 
	cameraIndex2 = 3;

void extractColorFromStereoKeys(Mat img1, Mat img2, std::vector<Vec2d> k1, std::vector<Vec2d> k2,
	std::vector<Vec3b>& color) {
	for (int i = 0; i < k1.size(); i++) {

		Vec3b c1 = img1.at<Vec3b>(k1[i].val[1], k1[i].val[0]);
		Vec3b c2 = img2.at<Vec3b>(k2[i].val[1], k2[i].val[0]);

		int r, g, b;

		r = (c1[0] + c2[0]) / 2;
		g = (c1[1] + c2[1]) / 2;
		b = (c1[2] + c2[2]) / 2;

		Vec3b c;
		c[0] = r;
		c[1] = g;
		c[2] = b;
		color.push_back(c);
	}
}

// Open chessboard images and extract corner points
int CameraCalibrator::addChessboardPoints(
	const std::vector<std::string>& filelist,
	cv::Size & boardSize) {

	// the points on the chessboard
	std::vector<cv::Point2f> imageCorners;
	std::vector<cv::Point3f> objectCorners;


	// 3D Scene Points:
	// Initialize the chessboard corners 
	// in the chessboard reference frame
    // The corners are at 3D location (X,Y,Z)= (i,j,0)
	for (int i = 0; i < boardSize.height; i++) {
		for (int j = 0; j < boardSize.width; j++) {

			objectCorners.push_back(cv::Point3f(i, j, 0.0f));
		}
	}

	// 2D Image points:
	cv::Mat image; // to contain chessboard image
	int successes = 0;
	// for all viewpoints
	for (int i = 0; i < filelist.size(); i++) {

		// Open the image
		image = cv::imread(filelist[i], 0);

		// Get the chessboard corners
		bool found = cv::findChessboardCorners(
			image, boardSize, imageCorners);

		// Get subpixel accuracy on the corners
		cv::cornerSubPix(image, imageCorners,
			cv::Size(5, 5),
			cv::Size(-1, -1),
			cv::TermCriteria(cv::TermCriteria::MAX_ITER +
				cv::TermCriteria::EPS,
				30,    // max number of iterations 
				0.1));     // min accuracy

			 // If we have a good board, add it to our data
		if (imageCorners.size() == boardSize.area()) {

			// Add image and scene points from one view
			addPoints(imageCorners, objectCorners);
			successes++;
		}
		/*
		//Draw the corners
		cv::Mat imageChessboardCorners = cv::imread(filelist[i]);
		cv::drawChessboardCorners(imageChessboardCorners, boardSize, imageCorners, found);
		cv::imshow("Corners on Chessboard", imageChessboardCorners);
		cv::waitKey(1000);
		*/
	}

	return successes;
}

// Add scene points and corresponding image points
void CameraCalibrator::addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners) {

	// 2D image points from one view
	imagePoints.push_back(imageCorners);
	// corresponding 3D scene points
	objectPoints.push_back(objectCorners);

}

// Calibrate the camera
// returns the re-projection error
double CameraCalibrator::calibrate(cv::Size &imageSize)
{
	// undistorter must be reinitialized
	mustInitUndistort = true;

	//Output rotations and translations


	// start calibration
	return
		calibrateCamera(objectPoints, // the 3D points
			imagePoints,  // the image points
			imageSize,    // image size
			cameraMatrix, // output camera matrix
			distCoeffs,   // output distortion matrix
			rvecs, tvecs, // Rs, Ts 
			flag);        // set options
  //          ,CV_CALIB_USE_INTRINSIC_GUESS);

}

cv::Vec3d CameraCalibrator::triangulate(const cv::Mat &p1, const cv::Mat &p2, const cv::Vec2d &u1, const cv::Vec2d &u2) {

	// system of equations assuming image=[u,v] and X=[x,y,z,1]
	// from u(p3.X)= p1.X and v(p3.X)=p2.X
	cv::Matx43d A(u1(0)*p1.at<double>(2, 0) - p1.at<double>(0, 0),
		u1(0)*p1.at<double>(2, 1) - p1.at<double>(0, 1),
		u1(0)*p1.at<double>(2, 2) - p1.at<double>(0, 2),
		u1(1)*p1.at<double>(2, 0) - p1.at<double>(1, 0),
		u1(1)*p1.at<double>(2, 1) - p1.at<double>(1, 1),
		u1(1)*p1.at<double>(2, 2) - p1.at<double>(1, 2),
		u2(0)*p2.at<double>(2, 0) - p2.at<double>(0, 0),
		u2(0)*p2.at<double>(2, 1) - p2.at<double>(0, 1),
		u2(0)*p2.at<double>(2, 2) - p2.at<double>(0, 2),
		u2(1)*p2.at<double>(2, 0) - p2.at<double>(1, 0),
		u2(1)*p2.at<double>(2, 1) - p2.at<double>(1, 1),
		u2(1)*p2.at<double>(2, 2) - p2.at<double>(1, 2));

	cv::Matx41d B(p1.at<double>(0, 3) - u1(0)*p1.at<double>(2, 3),
		p1.at<double>(1, 3) - u1(1)*p1.at<double>(2, 3),
		p2.at<double>(0, 3) - u2(0)*p2.at<double>(2, 3),
		p2.at<double>(1, 3) - u2(1)*p2.at<double>(2, 3));

	// X contains the 3D coordinate of the reconstructed point
	cv::Vec3d X;
	// solve AX=B
	cv::solve(A, B, X, cv::DECOMP_SVD);
	return X;
}

// triangulate a vector of image points
void CameraCalibrator::triangulate(const cv::Mat &p1, const cv::Mat &p2, const std::vector<cv::Vec2d> &pts1, const std::vector<cv::Vec2d> &pts2, std::vector<cv::Vec3d> &pts3D) {

	for (int i = 0; i < pts1.size(); i++) {

		pts3D.push_back(triangulate(p1, p2, pts1[i], pts2[i]));
	}
}

void CameraCalibrator::takeChessboardImages() {
	VideoCapture camCap1(cameraIndex1);  // cameraStream
	waitKey(10000);
	Mat output1;

//	camCap1 >> output1;

	int i = 1;
	while (true) {

		camCap1.read(output1);
		imshow("Chessboard", output1);

		if (waitKey(30) == 27) {

			cout << "Taken chessboard image" << endl;

			imwrite("boards" + std::to_string(i) + ".jpg", output1);
			i++;
		}

		if (waitKey(30) == int("a")) {
			cout << "ende" << endl;
			break;
		}

	}
}

void CameraCalibrator::takeStereoPair() {
	VideoCapture camCap1(cameraIndex1);  // cameraStream
	VideoCapture camCap2(cameraIndex2);  // cameraStream
	waitKey(10000);
	Mat output1, output2;

//	camCap1 >> output1;
//	camCap2 >> output2;

//	cv::Size imageSize1 = output1.size();

	int i = 0;
	while (true) {
		
		camCap1.read(output1);
		camCap2.read(output2);
		imshow("Left", output1);
		imshow("Right", output2);

		if (waitKey(30) == 27) {

//			cout << "tu sam" << endl;
	 
			imwrite("imL.jpg", output1);
			imwrite("imR.jpg", output2);
			i++;
		}

		if (waitKey(30) == int("a")) {
			cout << "ende" << endl;
			break;
		}
	}

}

void CameraCalibrator::ReconstructPointCloud() {

	const std::vector<std::string> files = { "boards1.jpg", "boards2.jpg","boards3.jpg","boards4.jpg","boards5.jpg","boards6.jpg","boards7.jpg","boards8.jpg","boards9.jpg",
											 "boards10.jpg","boards11.jpg","boards12.jpg","boards13.jpg","boards14.jpg","boards15.jpg","boards16.jpg","boards17.jpg",
											 "boards18.jpg","boards19.jpg","boards20.jpg","boards21.jpg","boards22.jpg","boards23.jpg","boards24.jpg","boards25.jpg" };
	cv::Size board_size(5, 8);

	addChessboardPoints(files, board_size);

	cv::Mat img = cv::imread("boards1.jpg");

	cv::Size img_size = img.size();
	calibrate(img_size);
	std::cout << cameraMatrix << endl;


	cv::Mat image1 = cv::imread("imR.jpg");
	cv::Mat image2 = cv::imread("imL.jpg");

	// vector of keypoints and descriptors
	std::vector<cv::KeyPoint> keypoints1;
	std::vector<cv::KeyPoint> keypoints2;
	cv::Mat descriptors1, descriptors2;

	// Construction of the feature detector
	cv::Ptr<cv::Feature2D> ptrFeature2D;

	//case 1, 2 and default are cases with default parameters
	//case 3, 4 and 5 are cases with adjusted parameters
	switch (5) {
	case 1: 
		//default ORB parameters
		ptrFeature2D = cv::ORB::create(500, 1.2, 8, 31, 0, 2, 0, 31, 20);
		break;
	case 2:
		//default SURF parameters
		ptrFeature2D = cv::xfeatures2d::SURF::create(100, 4, 3, false, false);
		break;
	case 3:
		//best SURF parameters
		ptrFeature2D = cv::xfeatures2d::SURF::create(5, 12, 24, false, true);
		break;
	case 4:
		//best ORB parameters
		ptrFeature2D = cv::ORB::create(100000, 1.2, 24, 50, 0, 4, 1, 50, 20);
		break;
	case 5:
		//best SIFT parameters
		ptrFeature2D = cv::xfeatures2d::SIFT::create(0, 32, 0.005, 55.0, 1.2);
		break;
	default:
		//default SIFT parameters
		ptrFeature2D = cv::xfeatures2d::SIFT::create(0, 3, 0.04, 10.0, 1.6);
	}

	// Detection of the features and associated descriptors
	ptrFeature2D->detectAndCompute(image1, cv::noArray(), keypoints1, descriptors1);
	ptrFeature2D->detectAndCompute(image2, cv::noArray(), keypoints2, descriptors2);

	// Match the two image descriptors
	// Construction of the matcher with crosscheck
	cv::BFMatcher matcher(cv::NORM_L2, true);
	std::vector<cv::DMatch> matches;
	matcher.match(descriptors1, descriptors2, matches);

	cv::Mat matchImage;

//	cv::namedWindow("img1");
	cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, matchImage, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	cv::imwrite("matches.jpg", matchImage);

	// Convert keypoints into Point2f
	std::vector<cv::Point2f> points1, points2;

	for (std::vector<cv::DMatch>::const_iterator it = matches.begin(); it != matches.end(); ++it) {
		// Get the position of left keypoints
		float x = keypoints1[it->queryIdx].pt.x;
		float y = keypoints1[it->queryIdx].pt.y;
		points1.push_back(cv::Point2f(x, y));
		// Get the position of right keypoints
		x = keypoints2[it->trainIdx].pt.x;
		y = keypoints2[it->trainIdx].pt.y;
		points2.push_back(cv::Point2f(x, y));
	}

	// Find the essential between image 1 and image 2
	cv::Mat inliers;
	cv::Mat essential = cv::findEssentialMat(points1, points2, cameraMatrix, cv::RANSAC, 0.9, 1.0, inliers);

	cout << essential << endl;

	// recover relative camera pose from essential matrix
	cv::Mat rotation, translation;
	cv::recoverPose(essential, points1, points2, cameraMatrix, rotation, translation, inliers);
	cout << rotation << endl;
	cout << translation << endl;


	// compose projection matrix from R,T
	cv::Mat projection2(3, 4, CV_64F); // the 3x4 projection matrix
	rotation.copyTo(projection2(cv::Rect(0, 0, 3, 3)));
	translation.copyTo(projection2.colRange(3, 4));
	
	// compose generic projection matrix
	cv::Mat projection1(3, 4, CV_64F, 0.); // the 3x4 projection matrix
	cv::Mat diag(cv::Mat::eye(3, 3, CV_64F));
	diag.copyTo(projection1(cv::Rect(0, 0, 3, 3)));
	
	// to contain the inliers
	std::vector<cv::Vec2d> inlierPts1;
	std::vector<cv::Vec2d> inlierPts2;
	
	// create inliers input point vector for triangulation
	int j(0);
	for (int i = 0; i < inliers.rows; i++) {
		if (inliers.at<uchar>(i)) {
			inlierPts1.push_back(cv::Vec2d(points1[i].x, points1[i].y));
			inlierPts2.push_back(cv::Vec2d(points2[i].x, points2[i].y));
		}
	}
	// undistort and normalize the image points
	std::vector<cv::Vec2d> points1u;
	cv::undistortPoints(inlierPts1, points1u, cameraMatrix, distCoeffs);
	std::vector<cv::Vec2d> points2u;
	cv::undistortPoints(inlierPts2, points2u, cameraMatrix, distCoeffs);


	// Triangulation
	std::vector<cv::Vec3d> points3D;
	std::vector<cv::Vec3b> color3D;
	triangulate(projection1, projection2, points1u, points2u, points3D);
	extractColorFromStereoKeys(image1, image2, inlierPts1, inlierPts2, color3D);

	cout << "3D points :" << points3D.size() << endl;

	viz::Viz3d window; //creating a Viz window

	//Displaying the Coordinate Origin (0,0,0)
	window.showWidget("coordinate", viz::WCoordinateSystem());
	window.setBackgroundColor(cv::viz::Color::black());

	//Displaying the 3D points 
	viz::WCloud pc(points3D, color3D);
	pc.setRenderingProperty(cv::viz::POINT_SIZE, 2);
	window.showWidget("points", pc);
	window.spin();
		
}

void blendPictures()
{
	VideoCapture camCap1(cameraIndex1);  // cameraStream
	VideoCapture camCap2(cameraIndex2);  // cameraStream
	waitKey(10000);
	Mat output1, output2, dst;

	camCap1 >> output1;
	camCap2 >> output2;

	cv::Size imageSize1 = output1.size();

	while (true) {
		camCap1.read(output1);
		camCap2.read(output2);
		addWeighted(output1, 0.5, output2, 0.5, 0.0, dst);
		imshow("Linear Blend", dst);
		waitKey(30);
	}
}


int main() {
	
//	getDisparity();

//	blendPictures();
	CameraCalibrator cal;

//	cal.takeChessboardImages();
//	cal.takeStereoPair();
	cal.ReconstructPointCloud();

}
	