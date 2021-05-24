#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include<opencv2/opencv.hpp>


using namespace cv;
using namespace std; 

//高通滤波和低通滤波模板滤波器
//const double HPFtemplate[9] = { 0,-1,0,-1,5,-1,0,-1,0 };
const double HPFtemplate[9] = { 1,-2,1,-2,5,-2,1,-2,1 };
const double LPFtemplate[9] = { 1,1,1,1,0,1,1,1,1 }; 

int Multi(Mat img, int i, int j,int k,int channel,int filter)   //filter1代表高通滤波器 2代表低通滤波器
{
	int ans=0;
	if (filter == 1)
	{
		if (channel > 1)
		{
			ans += (img.at<Vec3b>(i - 1, j - 1)[k] * HPFtemplate[0]);
			ans += (img.at<Vec3b>(i - 1, j)[k] * HPFtemplate[1]);
			ans += (img.at<Vec3b>(i - 1, j + 1)[k] * HPFtemplate[2]);
			ans += (img.at<Vec3b>(i, j - 1)[k] * HPFtemplate[3]);
			ans += (img.at<Vec3b>(i, j)[k] * HPFtemplate[4]);
			ans += (img.at<Vec3b>(i, j + 1)[k] * HPFtemplate[5]);
			ans += (img.at<Vec3b>(i + 1, j - 1)[k] * HPFtemplate[6]);
			ans += (img.at<Vec3b>(i + 1, j)[k] * HPFtemplate[7]);
			ans += (img.at<Vec3b>(i + 1, j + 1)[k] * HPFtemplate[8]);
		}
		else
		{
			ans += (img.at<uchar>(i - 1, j - 1) * HPFtemplate[0]);
			ans += (img.at<uchar>(i - 1, j) * HPFtemplate[1]);
			ans += (img.at<uchar>(i - 1, j + 1) * HPFtemplate[2]);
			ans += (img.at<uchar>(i, j - 1) * HPFtemplate[3]);
			ans += (img.at<uchar>(i, j) * HPFtemplate[4]);
			ans += (img.at<uchar>(i, j + 1) * HPFtemplate[5]);
			ans += (img.at<uchar>(i + 1, j - 1) * HPFtemplate[6]);
			ans += (img.at<uchar>(i + 1, j) * HPFtemplate[7]);
			ans += (img.at<uchar>(i + 1, j + 1) * HPFtemplate[8]);
		}
	}
	else if (filter == 2)
	{
		if (channel > 1)
		{
			ans += (img.at<Vec3b>(i - 1, j - 1)[k] * LPFtemplate[0]/8);
			ans += (img.at<Vec3b>(i - 1, j)[k] * LPFtemplate[1]/8);
			ans +=(img.at<Vec3b>(i - 1, j + 1)[k] * LPFtemplate[2]/8);
			ans += (img.at<Vec3b>(i, j - 1)[k] * LPFtemplate[3]/8);
			ans += (img.at<Vec3b>(i, j)[k] * LPFtemplate[4]/8);
			ans += (img.at<Vec3b>(i, j + 1)[k] * LPFtemplate[5]/8);
			ans +=(img.at<Vec3b>(i + 1, j - 1)[k] * LPFtemplate[6]/8);
			ans += (img.at<Vec3b>(i + 1, j)[k] * LPFtemplate[7]/8);
			ans += (img.at<Vec3b>(i + 1, j + 1)[k] * LPFtemplate[8]/8);
		}
		else
		{
			ans += (img.at<uchar>(i - 1, j - 1) * LPFtemplate[0]/8);
			ans += (img.at<uchar>(i - 1, j) * LPFtemplate[1]/8);
			ans += (img.at<uchar>(i - 1, j + 1) * LPFtemplate[2]/8);
			ans += (img.at<uchar>(i, j - 1) * LPFtemplate[3]/8);
			ans += (img.at<uchar>(i, j) * LPFtemplate[4]/8);
			ans += (img.at<uchar>(i, j + 1) * LPFtemplate[5]/8);
			ans += (img.at<uchar>(i + 1, j - 1) * LPFtemplate[6]/8);
			ans += (img.at<uchar>(i + 1, j) * LPFtemplate[7]/8);
			ans += (img.at<uchar>(i + 1, j + 1) * LPFtemplate[8]/8);
		}
	}
	return ans;
}

Mat HPF(const char *path)
{
	Mat img = imread(path,0);
	
	Mat img_HPF(img.size(), img.type());

	int row = img.rows;
	int col = img.cols;
	int channel = img.channels();

	if (channel > 1)   //如果是彩色图像
	{
		//不对边缘点进行处理
		for (int k = 0; k < channel; k++)
			for (int i = 1; i < row - 1; i++)
				for (int j = 1; j < col - 1; j++)
				{
					img_HPF.at<Vec3b>(i, j)[k] = saturate_cast<uchar>(Multi(img, i, j, k, channel, 1));
				}
	}
	else
	{
			for (int i = 1; i < row - 1; i++)
				for (int j = 1; j < col - 1; j++)
				{
					img_HPF.at<uchar>(i, j) = saturate_cast<uchar>(Multi(img, i, j, 1, channel, 1));
				}
	}
	Image_Show(img,img_HPF,channel);
	return img_HPF;
}

Mat LPF(const char *path)
{
	Mat img = imread(path,0);

	Mat img_LPF(img.size(), img.type());

	int row = img.rows;
	int col = img.cols;
	int channel = img.channels();
	int channel2 = img_LPF.channels();

	if (channel > 1)   //如果是彩色图像
	{
		//不对边缘点进行处理
		for (int k = 0; k < channel; k++)
			for (int i = 1; i < row - 1; i++)
				for (int j = 1; j < col - 1; j++)
				{
					img_LPF.at<Vec3b>(i, j)[k] = saturate_cast<uchar>(Multi(img, i, j, k, channel, 2));
				}
	}
	else
	{
		for (int i = 1; i < row - 1; i++)
			for (int j = 1; j < col - 1; j++)
			{
				img_LPF.at<uchar>(i, j) = saturate_cast<uchar>(Multi(img, i, j, 1, channel, 2));
			}
	}
	Image_Show(img, img_LPF, channel);
	return img_LPF;
}