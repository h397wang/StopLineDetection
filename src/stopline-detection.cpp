#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/objdetect.hpp"

#include "helper.h"

#include <iostream>

using namespace cv;
using namespace std;


const int STARTING_FRAME=275;
const string WINDOW_NAME = "Video";

const double L_LOW_MULT=1.65;
const int L_HIGH=255;
const int OPEN_K = 2;

const double rho = 2;// distance resolution in pixels of the Hough grid
const double theta = CV_PI/180; // angular resolution in radians of the Hough grid
const int thresh = 25; // minimum number of votes (intersections in Hough grid cell)
const double min_line_length = 180; //minimum number of pixels making up a line
const double max_line_gap = 20; //maximum gap in pixels between connectable line segments


int main (int argc, char* argv[])
{
	Mat image;

	if (argc != 2)
	{
		cout << "Specify path to image" << endl;
		return -1;
	}
    namedWindow(WINDOW_NAME, WINDOW_NORMAL );
    setWindowProperty(WINDOW_NAME, CV_WND_PROP_FULLSCREEN, 1);
    resizeWindow(WINDOW_NAME, 600,8);
	image = imread(argv[1], CV_LOAD_IMAGE_COLOR);


	detectStopLine(image);
	
	waitKey(0);

	detectSign(image);
	return 0;
}

Mat apply_threshold_v2_hls(Mat& img_init, double L_LOW_MULT, int L_HIGH){
	Mat hls;
	cvtColor(img_init, hls, COLOR_BGR2HLS);

	Mat light_ch;
	extractChannel(hls, light_ch, 1);
	equalizeHist(light_ch,light_ch);

	int w = hls.cols;
	int h = hls.rows;
	int b = h/2; // amount to mask on the image

	Rect rRect(0, b, w, h-b-1);
	Mat mask(h, w, CV_8UC1, cv::Scalar(0));
	mask(rRect)=255;

	//imshow( WINDOW_NAME, mask );
	//waitKey();

	double hls_low_thresh = L_LOW_MULT*((mean(light_ch, mask))[0]);
	Mat binary;
	inRange(light_ch, Scalar(hls_low_thresh), Scalar(L_HIGH),binary);

	Mat element = getStructuringElement( 0, Size( 2*OPEN_K + 1, 2*OPEN_K+1 ), Point( OPEN_K, OPEN_K ) );
	//morphologyEx( binary, binary, MORPH_OPEN, element );
	morphologyEx( binary, binary, MORPH_CLOSE, element );
	//imshow( WINDOW_NAME, binary );

	return binary;
}

void warp_corners(Mat& gray, Point2f vert[], Mat& warped, Mat& M, Mat& Minv) {
	int w = gray.cols;
	int h = gray.rows;
	Size img_size = Size(w, h);

	double scale = 5;
	double hw_ratio = 98.3 / 65.7;
	double left = (834.0 + 1054.0) / 2.0 - 110.0 / scale;
	double right = (834.0 + 1054.0) / 2.0 + 110.0 / scale;
	double base = (1106.0 + 1035.0) / 2.0 + (right - left) * hw_ratio / 2.0;
	double top = (1106.0 + 1035.0) / 2.0 - (right - left) * hw_ratio / 2.0;

	Point2f dst[4];
	dst[0] = Point2f( left, top );
    dst[1] = Point2f( right, top );
    dst[2] = Point2f( left, base );
    dst[3] = Point2f( right, base );

    M = getPerspectiveTransform(vert, dst);
    Minv = getPerspectiveTransform(dst, vert);
	warpPerspective(gray, warped, M, img_size);

}

int detectStopLine(Mat img_init) {

    Mat gray;
    cvtColor(img_init, gray,COLOR_BGR2GRAY);

    cv::Mat binaryMat(gray.size(), gray.type());

    Mat c_img = apply_threshold_v2_hls(img_init, L_LOW_MULT, L_HIGH);

    Mat edges;
	Canny(gray, edges,50, 150);
	edges = 255*(edges > 0);
	//imshow( WINDOW_NAME, edges );
	Mat combined_binary;

	bitwise_and(c_img, edges, combined_binary);

	Mat top_down_edges;
	Mat perspective_M;
	Mat Minv;

	Point2f src[4];
	src[0] = Point2f( 849, 1035 );
	src[1] = Point2f( 1041, 1035 );
	src[2] = Point2f( 834, 1106 );
	src[3] = Point2f( 1054, 1106 );

	Mat warped_out;

	warp_corners(combined_binary, src, top_down_edges, perspective_M, Minv);
	//imshow( WINDOW_NAME, top_down_edges );

	int w = top_down_edges.cols;
	int h = top_down_edges.rows;
	Rect rRect(w/3, 0, w/3, h);
	Mat mask(h, w, CV_8UC1, cv::Scalar(0));
	mask(rRect)=255;

	Mat top;
	bitwise_and(top_down_edges, top_down_edges, top, mask);


	cvtColor(top,warped_out,COLOR_GRAY2BGR);

	vector<Vec4i> lines;

	HoughLinesP(top, lines, rho, theta,thresh,min_line_length, max_line_gap );

	double max_dist = 0;
	Vec4i stopline(0,0,0,0);
	for (size_t i = 0; i < lines.size(); i++ ) {
		Vec4i l = lines[i];
		double dy = l[3]-l[1];
		double dx = l[2]-l[0];

		dy = dy > 0 ? dy:-dy;
		dx = dx > 0 ? dx:-dx;

		double dist = sqrt(dx*dx + dy*dy);

		if((dy < 2.5*dx) && dist > max_dist){
			max_dist = dist;
			stopline = l;
		}

    }

	if (max_dist){
		line( warped_out, Point(stopline[0], stopline[1]), Point(stopline[2], stopline[3]), Scalar(0,0,255), 3, CV_AA);
	}

	cout << stopline[0] << " " << stopline[1] << " " << stopline[2] << " " << stopline[3] << endl;


    imshow( WINDOW_NAME, warped_out );

	if(waitKey(0) == 'q') {
	 	imwrite("color-filter.jpg", c_img);
	 	imwrite("canny.jpg", edges);
	 	imwrite("concat.jpg", combined_binary);
	 	imwrite("mask.jpg", mask);
	}

    return 0;
}
