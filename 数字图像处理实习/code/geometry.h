#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include<math.h>

#define pi 3.1415926535

using namespace cv;
using namespace std;

Mat Translation(Mat img,int length,char direction1,char direction2)
{
	
	Mat img_t(img.size(), img.type());

	if (direction1 == 'p')
	{
		if (direction2 == 'r')
		for (int k = 0; k < img.channels(); k++)
			for (int i = 0; i < img.rows; i++)
				for (int j = 0; j < img.cols-length; j++)
				{
				    img_t.at<Vec3b>(i, j + length)[k] = img.at<Vec3b>(i, j)[k];
				}
		else if(direction2=='l')
			for (int k = 0; k < img.channels(); k++)
				for (int i = 0; i < img.rows; i++)
					for (int j = 0; j < img.cols-length; j++)
					{
						img_t.at<Vec3b>(i, j)[k] = img.at<Vec3b>(i, j + length)[k];
					}
	}
	else if (direction1 == 'v')
	{
		if(direction2=='d')
		for (int k = 0; k < img.channels(); k++)
			for (int i = 0; i < img.rows-length; i++)
				for (int j = 0; j < img.cols ; j++)
				{
					img_t.at<Vec3b>(i + length, j)[k] = img.at<Vec3b>(i, j)[k];
				}
		else if(direction2 == 'u')
			for (int k = 0; k < img.channels(); k++)
				for (int i = 0; i < img.rows - length; i++)
					for (int j = 0; j < img.cols; j++)
					{
						img_t.at<Vec3b>(i, j)[k] = img.at<Vec3b>(i + length, j)[k];
					}
	}
	imshow("", img_t);
	waitKey(6000);
	return img_t;
}


//双线性插值缩放
Mat Scaling(Mat img,double x_ratio,double y_ratio)
{
	int row_s = img.rows*x_ratio;
	int col_s = img.cols*y_ratio;

	Mat img_s(row_s, col_s, img.type());

	double xf, yf;  //小数部分
	int    xo, yo;  //原图对应的xy坐标
	int    xoo, yoo;//原图近临点xy坐标


	for(int k=0;k<img.channels();k++)
		for(int i=0;i<row_s;i++)
			for (int j = 0; j < col_s; j++)
			{
				xo = int(i / x_ratio);   yo = int(j / y_ratio);
				xoo = xo + 1;            yoo = yo + 1;
				xf = i / x_ratio - xo;   yf = j / y_ratio - yo;

				//边界判断
				if (xoo > img.rows - 1)
					xoo = xo - 1;
				if (yoo > img.cols - 1)
					yoo = yo - 1;

				img_s.at<Vec3b>(i, j)[k] = int(img.at<Vec3b>(xo, yo)[k] * (1 - xf)*(1 - yf) +
					                           img.at<Vec3b>(xoo, yo)[k] * xf*(1 - yf) +
					                           img.at<Vec3b>(xo, yoo)[k] * (1 - xf)* yf +
					                           img.at<Vec3b>(xoo, yoo)[k] * xf*yf);
			}
	
	imshow("After Scaling", img_s);
	waitKey(6000);
	return img_s;
}

Mat Rotation(Mat img,double alpha)
{
	Mat img_r(1.414*img.rows,1.414*img.cols, img.type());
	
	int xx, yy;    //图像旋转之后的对应坐标
	double beta = alpha + atan(img.rows / img.cols);// 旋转角
	double len = sqrt(img.rows*img.rows + img.cols*img.cols)/2;//斜边


	for(int k=0;k<img.channels();k++)
		for(int i=0;i<img.rows;i++)
			for (int j = 0; j < img.cols; j++)
			{
				xx = i * cos(alpha) - j * sin(alpha) + img_r.rows/2-len*cos(beta);
				yy = i * sin(alpha) + j * cos(alpha) + img_r.cols/2-len*sin(beta);

				if (xx < img_r.rows && yy < img_r.cols && xx>0 && yy>0)
					img_r.at<Vec3b>(xx, yy)[k] = img.at<Vec3b>(i, j)[k];
				    
			}
	
	imshow("After Rotating", img_r);
	waitKey(6000);
	return img_r;
}