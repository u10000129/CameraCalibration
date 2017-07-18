#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include "dirent.h"

using namespace cv;
using namespace std;

// global variable
int nx = 9;
int ny = 6;
int nBoards = 20;

Mat calibration(Mat img, vector<vector<Point3f>> object_points, vector<vector<Point2f>> image_points);

int main()
{
	Size patternSize(nx, ny);
	vector<vector<Point3f>> object_points;
	vector<vector<Point2f>> image_points;
	vector<Point3f> obj;
	vector<Point2f> corners;
	char name[100];

	for (int i = 0; i < ny; i++) {
		for (int j = 0; j < nx; j++) {
			obj.push_back(Point3f(i, j, 0.0f));
		}
	}	
	
	string inputDirectory = "camera_cal";
	DIR *directory = opendir(inputDirectory.c_str());
	struct dirent *_dirent = NULL;
	if (directory == NULL) {
		cout << "Cannot open Input Folder" << endl;
		return -1;
	}
	while ((_dirent = readdir(directory)) != NULL) {
		string fileName = inputDirectory + "\\" + string(_dirent->d_name);
		Mat rawImage = imread(fileName.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
		if (rawImage.data == NULL) {
			cout << "Cannot Open Image" << endl;
			continue;
		}
		bool patternfound = findChessboardCorners(rawImage, patternSize, corners);
		if (patternfound) {
			cornerSubPix(rawImage, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(rawImage, patternSize, Mat(corners), patternfound);
			//imshow("find corner",cur);
			//waitKey();
			image_points.push_back(corners);
			object_points.push_back(obj);
		}
	}
	closedir(directory);

	Mat img = imread("signs_vehicles_xygrad.jpg");
	Mat processedImg = calibration(img, object_points, image_points);

	//imwrite("signs_vehicles_xygrad2.jpg", processedImg);
	imshow("Original", img);
	imshow("Processed", processedImg);
	waitKey();

	return 0;
}

Mat calibration(Mat img, vector<vector<Point3f>> object_points, vector<vector<Point2f>> image_points) {
	Mat intrinsic = Mat(3, 3, CV_32FC1);
	Mat distCoeffs;
	vector<Mat> rvecs;
	vector<Mat> tvecs;
	intrinsic.ptr<float>(0)[0] = 1;
	intrinsic.ptr<float>(1)[1] = 1;
	calibrateCamera(object_points, image_points, img.size(), intrinsic, distCoeffs, rvecs, tvecs);
	Mat processedImg;
	undistort(img, processedImg, intrinsic, distCoeffs);

	imshow("Original", img);
	imshow("Processed", processedImg);
	imwrite("signs_vehicles_xygrad2.jpg", processedImg);

	return processedImg;
}