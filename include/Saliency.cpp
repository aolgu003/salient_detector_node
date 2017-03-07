#include "Saliency.h"


void fftShift(Mat image);

void Saliency::spectral_saliency(Mat &I, Mat &map) {
		//expand input image to optimal size by padding array with zeros to create optimal array size
		//for dct transform computation
		Mat padded;                            
		int m = getOptimalDFTSize(I.rows);
		int n = getOptimalDFTSize(I.cols);
		copyMakeBorder(I, padded, 0, m - I.rows, 0, n - I.cols, BORDER_REPLICATE, Scalar::all(0));


		// Create a floating point array for to 
		Mat planes[] = { Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F) }; //Cast padded into a 2d float array 
		Mat complexI;
		merge(planes, 2, complexI);         // Add merge planes to create one Mat object


		/* Begin Residual saliency algorithm: */
		dft(complexI, complexI);            // this way the result may fit in the source matrix

		split(complexI, planes);
		Mat magI, phaseI;
		cartToPolar(planes[0], planes[1], magI, phaseI);

		magI += Scalar::all(1);
		log(magI, magI); // calculate the log representation of the image

		fftShift(magI); //shift the spectrum to be double sided function
		normalize(magI, magI, 0, 1, NORM_MINMAX);
		fftShift(magI);

		Mat magAverage, magRes, Sum, InverseFourier, kernel;

		medianBlur(magI, magAverage, 5); //find the redundant (average) information of the image

		magRes = magI - magAverage; //remode the redundant information
		exp(magRes, magRes); //convert the image from log to normal representation

		polarToCart(magRes, phaseI, planes[0], planes[1]); //convert back to complex image

		merge(planes, 2, Sum);

		//dft(complexI, InverseFourier, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT);
		dft(Sum, InverseFourier, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT); //inverse of the transform
		pow(InverseFourier, 2, InverseFourier); //square the image to remove negative values
		GaussianBlur(InverseFourier, InverseFourier, Size(7, 7), 0); //smooth the image to remove noise
		normalize(InverseFourier, map, 0, 1, NORM_MINMAX);
}

void Saliency::spectral_saliency_multi(Mat &I, Mat &map) {
	Mat I_split[3];	
	Mat c0_salient_map, c1_salient_map, c2_salient_map;

	split(I, I_split);
	
	this->spectral_saliency(I_split[0], c0_salient_map);
	this->spectral_saliency(I_split[1], c1_salient_map);
	this->spectral_saliency(I_split[2], c2_salient_map);

	map = (c0_salient_map + c1_salient_map + c2_salient_map) / 3;
}
void Saliency::MSSS_saliency_multi(Mat &in, Mat &out) {}


// Pass is the saliency map as I
// Pass in the threshold saliency value
// Pass in the maximum area of the potential targets
void Saliency::find_targets( Mat &I, Mat &map, vector<Rect> &roi, vector<Mat> &targets, double thresh, int MinArea, int MaxArea,     int sf, int buffer ) {
	
	Mat binaryImg;
	Mat threshold_output;
	Mat padded;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;



	threshold( map, binaryImg, thresh, 255	, THRESH_BINARY );
	map = binaryImg;
	binaryImg.convertTo(binaryImg, CV_8UC1);

	/// Find contours
  	findContours( binaryImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
	
	/// Approximate contours to polygons + get bounding rects and circles
	vector<vector<Point> > contours_poly( contours.size() );
	Rect tmp_rect;
	vector<Point2f>center( contours.size() );
	vector<float>radius( contours.size() );
	
	int pad = 2 * buffer;
	padded.create(I.rows + 2*pad, I.cols + 2*pad, I.type());
	padded.setTo(Scalar::all(0));	
	I.copyTo(padded(Rect( pad, pad, I.cols, I.rows )));
	cout << "Size of I: " << I.size() << endl;

	for( int i = 0; i < contours.size(); i++ ) { 
		approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
		tmp_rect = boundingRect( Mat(contours_poly[i]) );
		cout << "Area: " << tmp_rect.width * tmp_rect.height << endl;
		if ( MinArea < tmp_rect.height * tmp_rect.width < MaxArea ) {
			roi.push_back(tmp_rect);
			
			tmp_rect.x = tmp_rect.x * sf;
			tmp_rect.y = tmp_rect.y * sf;
			tmp_rect.width = tmp_rect.width * sf + 2*buffer;
			tmp_rect.height = tmp_rect.height * sf + 2*buffer;

			cout << tmp_rect.x << " " << tmp_rect.y << " " << tmp_rect.width << " " << tmp_rect.height << endl;			
			targets.push_back(padded(tmp_rect));
		}
	}
}


void fftShift(Mat image) {
	// rearrange the quadrants of Fourier image  so that the origin is at the image center
	int cx = image.cols / 2;
	int cy = image.rows / 2;

	Mat q0(image, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
	Mat q1(image, Rect(cx, 0, cx, cy));  // Top-Right
	Mat q2(image, Rect(0, cy, cx, cy));  // Bottom-Left
	Mat q3(image, Rect(cx, cy, cx, cy)); // Bottom-Right

	Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
	q0.copyTo(tmp);
	q3.copyTo(q0);
	tmp.copyTo(q3);

	q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
	q2.copyTo(q1);
	tmp.copyTo(q2);
}

