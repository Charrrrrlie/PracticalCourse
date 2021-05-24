#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include<opencv2/opencv.hpp>

using namespace cv;
using namespace std; 

void LinearTrans(const char *path,double k,double b)
{
	Mat img = imread(path);
	Mat trans_img(img.size(), img.type());
	
	int row = img.rows;
	int col = img.cols;
	

	int imtype = img.channels();
	if (imtype == 1)     //如果是灰度图像
	{
		for(int i=0;i<row;i++)
			for (int j = 0; j < col; j++)
			{
				trans_img.at<uchar>(i, j) = saturate_cast<uchar>(img.at<uchar>(i, j)*k + b);
			}
    }
   else if (imtype == 3)//如果是彩色图像
   {
		for(int l=0;l<imtype;l++)
		  for(int i=0;i<row;i++)
			for (int j = 0; j < col; j++)
			{
				trans_img.at<Vec3b>(i, j)[l] = saturate_cast<uchar>(img.at<Vec3b>(i, j)[l]*k + b);
			}
   }

   Image_Show(img,trans_img,imtype);
}

