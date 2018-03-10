// helper.h
#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/objdetect.hpp"

using namespace cv;

typedef struct {
    Point2i mPt1;
    Point2i mPt2;
} line_t;

line_t detectStopLine(const Mat & image);

Mat filter1(const Mat & image);

