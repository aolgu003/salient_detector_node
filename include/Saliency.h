#include <iostream>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>

using namespace cv;
using namespace std;

class Saliency {
	public:

		void spectral_saliency(Mat &I, Mat &map);
		void spectral_saliency_multi(Mat &I, Mat &map);
		void MSSS_saliency_multi(Mat &in, Mat &out);
		void find_targets( Mat &I, Mat &map, vector<Rect> &roi, vector<Mat> &targets, double thresh, int MinArea, int MaxArea, int sf, int buffer );
};
