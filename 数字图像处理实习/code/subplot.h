#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include<opencv2/opencv.hpp>


using namespace cv;
using namespace std;

void Image_Show(Mat img1, Mat img2, int type)
{
	Mat img11, img22;
	resize(img1, img11, Size(), 0.5, 0.5);
	resize(img2, img22, Size(), 0.5, 0.5);


	int syn_row = max(img11.rows, img22.rows);
	int col1 = img11.cols;
	int col2 = img22.cols;

	int syn_col = col1 + col2 + 1;

	Mat syn_img(syn_row, syn_col, CV_MAKETYPE(CV_8U, type), Scalar(0, 0, 0));

	img11.colRange(0, col1).copyTo(syn_img.colRange(0, col1));
	img22.colRange(0, col2).copyTo(syn_img.colRange(col1 + 1, syn_col));

	imshow("contrast_image", syn_img);
	waitKey();
}