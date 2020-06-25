#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<math.h>
#include<algorithm>

#define Pi 3.1415926

#define T1  80  //类间最大误差法调高的阈值 用作HSV去噪
#define T2  40  //类间最大误差法调高的阈值 用作C1C2C3去噪

#define HSV 0              //用于不同type进行相似的函数操作
#define C1C2C3 1

using namespace std;
using namespace cv;

void Rgb2Hsv(Mat r, Mat g, Mat b,Mat *h,Mat *s,Mat *v);  //HSV空间转换
Mat Shadow(Mat h, Mat s, Mat v);                         //HSV阴影识别
Mat CCC(Mat r, Mat g, Mat b);                           // C1C2C3空间转换
Mat Segment(Mat img,int type);                          //二值化处理
void Merge(Mat img,int type);                          //小面积去除 大面积合并

int main()
{
	Mat img = imread("../../../实习数据/第三部分 阴影检测/Color.bmp");

	vector<Mat> channel;
	split(img, channel);        //channel  B G R

	Mat h(img.size(),CV_64FC1), s(img.size(), CV_64FC1), v(img.size(), CV_64FC1);

	

	Rgb2Hsv(channel[2], channel[1], channel[0], &h, &s, &v);  
	
	Mat M=Shadow(h, s, v);
	Mat M_binary = Segment(M,HSV);    //最大类间误差法分割
	Merge(M_binary, HSV);
	
	Mat C = CCC(channel[2], channel[1], channel[0]);
	Mat C_binary = Segment(C,C1C2C3);    //最大类间误差法分割
	Merge(C_binary, C1C2C3);

	return 0;
}


void Rgb2Hsv(Mat r, Mat g, Mat b, Mat *h, Mat *s, Mat *v)
{
	r.convertTo(r, CV_64FC1);
	g.convertTo(g, CV_64FC1);
	b.convertTo(b, CV_64FC1);

	Mat temp1(r.size(),CV_64FC1),temp2(r.size(),CV_64FC1);

	//  V=1/3 *(R+G+B)
	add(r, g, temp1);
	add(temp1, b, temp1);	
	divide(temp1, 3, *v);

	//  S=1-min(R,G,B)/V
	for(int i=0;i<r.rows;i++)
		for (int j = 0; j < r.cols; j++)
		{
			double val =0;
			val = min(r.at<double>(i, j),g.at<double>(i, j));
			temp2.at<double>(i, j) = min(val, b.at<double>(i, j));
		}

	divide(temp2, *v, temp2);
	subtract(1, temp2, *s);


	//  H = θ   360-θ
	for(int i=0;i<r.rows;i++)
		for (int j = 0; j < r.cols; j++)
		{
			double valr = r.at<double>(i, j);
			double valg = g.at<double>(i, j);
			double valb = b.at<double>(i, j);

			if (valb <= valg)
			{
				h->at<double>(i, j) = 180 / Pi * acos(0.5*(valr - valg + valr - valb)
					                  / sqrt((valr - valg)*(valr - valg) + (valr - valb)*(valg - valb)));
			}
			else
			{
				h->at<double>(i, j) = 360-180 / Pi * acos(0.5*(valr - valg + valr - valb)
					                  / sqrt((valr - valg)*(valr - valg) + (valr - valb)*(valg - valb)));
			}

		}


}

Mat Shadow(Mat h, Mat s, Mat v)
{
	Mat M;

	Mat res1, res2;  // 分子分母

	subtract(s, v, res1);
	add(h, s, res2);
	add(res2, v, res2);

	divide(res1, res2, M);

	return M;
}

Mat CCC(Mat r, Mat g, Mat b)
{
	r.convertTo(r, CV_64FC1);
	g.convertTo(g, CV_64FC1);
	b.convertTo(b, CV_64FC1);

	Mat C3(r.size(),CV_64FC1);

	for(int i=0;i<r.rows;i++)
		for (int j = 0; j < r.cols; j++)
		{
			C3.at<double>(i, j) = atan(b.at<double>(i, j) / max(r.at<double>(i, j), g.at<double>(i, j)));
		}

	return C3;
}

Mat Segment(Mat img,int type)
{
	Mat out(img.size(), CV_8UC1);

	double temp = 0;
	double max = 0;
	double min = 255;

	//最大类间误差法

	//灰度拉伸 从0-255
	for (int i = 0; i < img.rows; i++)
		for (int j = 0; j < img.cols; j++)
		{
			temp = img.at<double>(i, j);
			if (temp > max)
			{
				max = temp;
			}
			if (temp < min)
			{
				min = temp;
			}
		}
	double scale = 255 / (max - min);

	for (int i = 0; i < img.rows; i++)
		for (int j = 0; j < img.cols; j++)
		{
			out.at<uchar>(i, j) = (unsigned char)((int)((img.at<double>(i, j) - min)*scale));
		}

	//动态计算阈值
	int thre = 0;
	double f1 = 0; double f2 = 0;         //前后景灰度频数
	double s_temp = 0; double s = 0;     //方差
	double avg1 = 0; double avg2 = 0;    //前后景分别均值

	for (int t = 0; t < 256; t++)          //找每一个阈值
	{
		f1 = 0;
		f2 = 0;
		avg1 = 0;
		avg2 = 0;

		for (int i = 0; i < out.rows; i++)
			for (int j = 0; j < out.cols; j++)
			{
				int val = (int)(out.at<uchar>(i, j));
				if (val < t)
				{
					f1++;
					avg1 += val;
				}
				else
				{
					f2++;
					avg2 += val;
				}
			}
		avg1 /= f1;
		avg2 /= f2;
		s_temp = f1 * f2*pow((avg1 - avg2), 2) / pow((out.rows - out.cols), 2);

		if (s_temp > s)    //找到动态规划后的阈值
		{
			thre = t;
			s = s_temp;
		}
	}

	Mat res(out.size(), out.type());

	//二值化
	if (type == 0)
	{
		for (int i = 0; i < res.rows; i++)
			for (int j = 0; j < res.cols; j++)
			{
				int val = (int)(out.at<uchar>(i, j));
				if (val > thre + T1)   //////////阈值调大
				{
					res.at<uchar>(i, j) = 255;
				}
				else
				{
					res.at<uchar>(i, j) = 0;
				}
			}
		//imshow("合并前基于HSV空间的阴影检测", res);
		//waitKey(600);
		//imwrite("合并前基于HSV空间的阴影检测.jpg", res);
	}
	else if (type == 1)
	{
		for (int i = 0; i < res.rows; i++)
			for (int j = 0; j < res.cols; j++)
			{
				int val = (int)(out.at<uchar>(i, j));
				if (val > thre + T2)   //////////阈值调大
				{
					res.at<uchar>(i, j) = 255;
				}
				else
				{
					res.at<uchar>(i, j) = 0;
				}
			}
		//imshow("合并前基于C1C2C3空间的阴影检测", res);
		//waitKey(600);
		//imwrite("合并前基于C1C2C3空间的阴影检测.jpg", res);
	}
	
	return res;
}

void Merge(Mat img, int type)
{
	Mat temp=img.clone();

	if (type == 0)
	{
		//腐蚀和膨胀操作形成连通域
		Mat element1 = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
		Mat element2 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));

		dilate(temp, temp, element1, Point(-1, -1), 6);
		dilate(temp, temp, element2, Point(-1, -1), 3);
		erode(temp, temp, element1, Point(-1, -1), 1);

		//寻找连通区域
		Mat res = temp.clone();
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		findContours(res, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

		for (int i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i]) < 100)    //选取连通区域的阈值
			{
				drawContours(res, contours, i, Scalar(0), -1);
			}
		}

		imshow("合并后基于HSV空间的阴影检测", res);
		waitKey(600);
	}
	else if (type == 1)
	{
		//腐蚀和膨胀操作形成连通域
		Mat element1 = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
		Mat element2 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));

		dilate(temp, temp, element1, Point(-1, -1), 2);
		dilate(temp, temp, element2, Point(-1, -1), 1);
		erode(temp, temp, element1, Point(-1, -1), 1);

		//寻找连通区域
		Mat res = temp.clone();
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		findContours(res, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

		for (int i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i]) < 100)    //选取连通区域的阈值
			{
				drawContours(res, contours, i, Scalar(0), -1);
			}
		}

		imshow("合并后基于C1C2C3空间的阴影检测", res);
		waitKey(0);
	}

}