#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include<opencv2/opencv.hpp>


using namespace cv;
using namespace std; 

//加权融合法
void Fuse(const char *path1, const char *path2)    //path1 高精度灰度图 path2低精度彩色图
{
	Mat img_c = imread(path2);
	Mat img_p = imread(path1,0);
	Mat img(img_c.size(), img_c.type());
	Mat img_hp = HPF(path1);
	Mat img_lp = LPF(path1);


	for(int k=0;k<img_c.channels();k++)
		for(int i=0;i<img_c.rows;i++)
			for (int j = 0; j < img_c.cols; j++)
			{
				if (img_lp.at<uchar>(i, j) == 0)
					img_lp.at<uchar>(i, j) = 1;
				img.at<Vec3b>(i, j)[k] = img_c.at<Vec3b>(i, j)[k] + img_c.at<Vec3b>(i, j)[k]*
					         (img_p.at<uchar>(i, j) - img_lp.at<uchar>(i, j))/ img_lp.at<uchar>(i, j);
			}

	Image_Show(img_c, img,img.channels());
		
}
