// helper.cpp

#include "helper.h"


using namespace cv;
using namespace std;

const int OPEN_K = 2;
const int L_HIGH = 255;
const double L_LOW_MULT = 1.65;


enum hls {
	eHue = 0,
	eLightness = 1,
	eSaturation = 2,
};

line_t detectStopLine(const Mat & aImage)
{

	const Mat vImageFilter1 = filter1(aImage);


	line_t vStopLine;
	return vStopLine;
}

Mat filter1(const Mat & aImage)
{
	Mat vImageHLS;
	cvtColor(aImage, vImageHLS, COLOR_BGR2HLS);

	Mat vImageLightness;
	extractChannel(vImageHLS, vImageLightness, eLightness);
	equalizeHist(vImageLightness, vImageLightness);
	
	namedWindow("vImageLightness");
	imshow("vImageLightness", vImageLightness);	


	const int yMin = vImageHLS.rows / 2; // amount to mask on the image
	Rect lowerHalf(0, yMin, vImageHLS.cols, vImageHLS.rows - yMin - 1);
	Mat vMask(vImageHLS.size(), CV_8UC1, cv::Scalar(0));
	vMask(lowerHalf) = 255;

	double hls_low_thresh = L_LOW_MULT*((mean(vImageLightness, vMask))[0]);
	inRange(vImageLightness, Scalar(hls_low_thresh), Scalar(L_HIGH), vImageLightness);

	namedWindow("binary");
	imshow("binary", vImageLightness);	


#if 0
	Mat vImageAdpThresh;
	adaptiveThreshold(vImageLightness
		, vImageLightness
		, 255 // value to assign when threshold is met
		, CV_ADAPTIVE_THRESH_GAUSSIAN_C
		, CV_THRESH_BINARY
		, 5 // block size
		, 1  // constant weight subtracted from mean?
		);

	namedWindow("vImageAdpThresh");
	imshow("vImageAdpThresh", vImageLightness);	
#endif

#if 0
	Mat vImageClosed;
	Mat vMorphKernel = getStructuringElement(0
		, Size( 2*OPEN_K + 1, 2*OPEN_K + 1 )
		, Point( OPEN_K, OPEN_K )
		);
	morphologyEx( vImageLightness, vImageLightness, MORPH_CLOSE, vMorphKernel);

	namedWindow("vImageClosed");
	imshow("vImageClosed", vImageLightness);	
#endif

	Mat ret;
	return ret;
}


