#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include<stdio.h>
#include<math.h>

using namespace std;
using namespace cv;

void Rgb2His(double r, double g, double b, double &h, double &i, double &s) {
	//归一化
	double rn = r, gn = g, bn = b;
	rn = rn / 255.0; gn = gn / 255.0; bn = bn / 255.0;
	double rad = acos((rn - gn + rn - bn) / (2 * sqrt((rn - gn) *(rn - gn) + (rn - bn)*(gn - bn))));
	if (gn >= bn) {
		h = rad;
	}
	else {
		h = 2 * CV_PI - rad;
	}
	i = (rn + gn + bn) / 3;
	if (i < 0)  i = 0;
	else if (i > 1) i = 1.0;

	if (i == 0 || max(max(r, g), b) == min(min(r, g), b)) {
		s = 0;
		h = 0;
	}
	else {
		double rad = acos((rn - gn + rn - bn) / (2 * (sqrt((rn - gn) *(rn - gn) + (rn - bn)*(gn - bn)) + 1e-5)));
		if (gn >= bn) {
			h = rad;
		}
		else {
			h = 2 * CV_PI - rad;
		}
		s = 1 - (min(min(rn, gn), bn) * 3) / (rn + gn + bn);
	}

}
void His2Rgb(double &r, double &g, double &b, double h, double i, double s) {

	double rn = 0, gn = 0, bn = 0;
	if (h >= 0 && h < CV_PI / 3 * 2) {
		double x = i * (1 - s);
		double y = i * (1 + (s*cos(h)) / cos(CV_PI / 3 - h));
		double z = 3 * i - x - y;
		bn = x; rn = y; gn = z;
	}
	else if (h >= CV_PI / 3 * 2 && h < CV_PI / 3 * 4) {
		h = h - CV_PI / 3 * 2;
		double x = i * (1 - s);
		double y = i * (1 + (s*cos(h)) / cos(CV_PI / 3 - h));
		double z = 3 * i - x - y;
		rn = x; gn = y; bn = z;
	}
	else if (h >= CV_PI / 3 * 4 && h < CV_PI * 2) {
		h = h - CV_PI / 3 * 4;
		double x = i * (1 - s);
		double y = i * (1 + (s*cos(h)) / cos(CV_PI / 3 - h));
		double z = 3 * i - x - y;
		gn = x; bn = y; rn = z;
	}
	r = rn * 255; g = gn * 255; b = bn * 255;

}

double* Strech(double *I, Mat img,int row,int col)
{
	double *data = new double[row*col];
	for (int i = 0; i < row; i++)
		for (int j = 0; j < col; j++)
		{
			data[i*col + j] = img.at<uchar>(i, j);
		}

	double Imax = 0;
	double Imin = 1;


	double Imgmax = 0;
	double Imgmin = 255;

	//找最大最小值
	for (int k = 0; k < row*col; k++)
	{
			if (I[k] > Imax)
			{
				Imax = I[k];
			}
			if (I[k] < Imin)
			{
				Imin = I[k];
			}

			if (data[k]>Imgmax)
			{
				Imgmax = data[k];
			}
			if (data[k] < Imgmin)
			{
				Imgmin = data[k];
			}
		}

	double *dest;
	dest = new double[row*col];

	for(int k=0;k<row;k++)
		for (int j = 0; j < col; j++)
		{
			dest[k*col + j] = (img.at<uchar>(k, j) - Imgmin) / (Imgmax+Imgmin)
				              *(Imax - Imin) + Imin;
		}

	delete []data;

	return dest;
}

void HISfuse(const char *path1, const char *path2)
{
	Mat img_ori = imread(path1,0);
	Mat img_tar = imread(path2);

	int row = img_ori.rows;
	int col = img_ori.cols;

	double *h_vec,*io_vec,*id_vec,*s_vec;   //o 灰色图像  d彩色图像
	h_vec = new double[row*col];
	id_vec = new double[row*col];
	s_vec = new double[row*col];

	for(int k=0;k<row;k++)
		for (int j = 0; j < col; j++)
		{
			double b = img_tar.at<Vec3b>(k, j)[0];
			double g = img_tar.at<Vec3b>(k, j)[1];
			double r = img_tar.at<Vec3b>(k, j)[2];

			Rgb2His(r, g, b, h_vec[k*col + j], id_vec[k*col + j], s_vec[k*col + j]);
		}

	io_vec = Strech(id_vec, img_ori, row, col);

	Mat img(img_tar.size(), img_tar.type());
	double r=0; double g = 0; double b = 0;
	for (int k = 0; k < row; k++)
		for (int j = 0; j < col; j++)
		{
			His2Rgb(r,g,b,h_vec[k*col + j], io_vec[k*col + j], s_vec[k*col + j]);
			img.at<Vec3b>(k, j)[0] = b;
			img.at<Vec3b>(k, j)[1] = g;
			img.at<Vec3b>(k, j)[2] = r;
		}

	imwrite("C:\\Users\\yyc\\Desktop\\his.bmp", img);
	
}
