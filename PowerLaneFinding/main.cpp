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

int main()
{
	Size patternSize(nx, ny);
	vector<vector<Point3f>> object_points;
	vector<vector<Point2f>> image_points;
	vector<Point3f> obj;
	vector<Point2f> corners;
	char name[100];

	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir("camera_cal")) != NULL) {
		/* print all the files and directories within directory */
		while ((ent = readdir(dir)) != NULL) {
			printf("%s\n", ent->d_name);
		}
		closedir(dir);
	}
	else {
		/* could not open directory */
		perror("");
		return EXIT_FAILURE;
	}

	for (int i = 0; i < ny; i++) {
		for (int j = 0; j < nx; j++) {
			obj.push_back(Point3f(i, j, 0.0f));
		}
	}	
	
	for (int i = 1; i <= nBoards; i++) {
		sprintf(name, "camera_cal/calibration%d.jpg", i);
		Mat cur = imread(name);
		cvtColor(cur, cur, CV_BGR2GRAY);
		bool patternfound = findChessboardCorners(cur, patternSize, corners);
		if (patternfound) {
			cornerSubPix(cur, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(cur, patternSize, Mat(corners), patternfound);
			//imshow("find corner",cur);
			//waitKey();
			image_points.push_back(corners);
			object_points.push_back(obj);
		}
	}

	Mat img = imread("signs_vehicles_xygrad.jpg");
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
	imwrite("signs_vehicles_xygrad.jpg", processedImg);
	
	waitKey();

	return 0;
}
