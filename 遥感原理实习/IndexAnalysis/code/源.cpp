#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <string>

#define  L 0.5                //随着植被密度变化的常参数

//用于可视化显示
#define VI_NDVI 0             //归一化植被指数
#define VI_SAVI 1             //土壤调节植被指数
#define WI_MNDWI 2            //修改归一化水体指数
#define WI_AWEI 3             //自动水体提取指数
#define LI_NDBI 4             //归一化裸地指数
#define LI_IBI  5             //建筑用地指数


using namespace cv;
using namespace std;

Mat Normalize(Mat r1, Mat r2);      //归一化指数
Mat SAVI(Mat r, Mat nr);      //土壤调节植被指数
Mat AWEI(Mat g, Mat nr, Mat s1, Mat s2); //自动水体提取指数
Mat IBI(Mat ndbi, Mat savi, Mat mndwi); //建筑用地指数

void Col_Display(Mat b, Mat g, Mat r);       //全彩色
void Display(Mat VI, int type);        //伪彩色可视化 并文件输出
void Binary(Mat img,int type);                 //二值可视化  并文件输出  

int main()
{
	Mat img_b, img_g, img_r, img_nr, img_mr, img_dr;     //波段
	Mat img_NV, img_SV,img_MW,img_AW,img_NL,img_IL;      //指数

	//相对路径
	string path0 = "../../../实习数据/第二部分 专题指数/";
	string path[7] = { "tm1.tif","tm2.tif","tm3.tif","tm4.tif","tm5.tif","tm6.tif","tm7.tif" };

	//蓝波段
	img_b = imread(path0+path[0], 0);
	//绿波段
	img_g = imread(path0 + path[1], 0);
	//红波段
	img_r = imread(path0 + path[2], 0);
	//近红外波段
	img_nr = imread(path0 + path[3], 0);
    //中红外波段 SWIR1
	img_mr = imread(path0 + path[4], 0);
	//远红外波段 SWIR2
	img_dr = imread(path0 + path[6], 0);



	//归一化植被指数
	img_NV = Normalize(img_r, img_nr);
	//Display(img_NV, VI_NDVI);
	Binary(img_NV, VI_NDVI);

	//土壤调节植被指数
	img_SV = SAVI(img_r, img_nr);
	//Display(img_SV, VI_SAVI);
	Binary(img_SV, VI_SAVI);

	//修改归一化水体指数
	img_MW = Normalize(img_dr, img_g);
	//Display(img_MW, WI_MNDWI);
	Binary(img_MW, WI_MNDWI);

	//自动水体提取指数
	img_AW = AWEI(img_g, img_nr, img_mr, img_dr);
	//Display(img_AW, WI_AWEI);
	Binary(img_AW, WI_AWEI);

	//归一化裸地指数
	img_NL = Normalize(img_nr, img_mr);
	//Display(img_NL, LI_NDBI);
	Binary(img_NL, LI_NDBI);

	//建筑用地指数
	img_IL = IBI(img_NL, img_SV, img_MW);
	//Display(img_IL, LI_IBI);
	Binary(img_IL, LI_IBI);

	//真彩色合成
	Col_Display(img_b, img_g, img_r);

	return 0;
}

Mat Normalize(Mat r1, Mat r2)     //归一化  (r2-r1)/(r2+r1)
{
	Mat res1(r1.size(), CV_64FC1);   //分子
	Mat res2(r1.size(), CV_64FC1);   //分母

	subtract(r2, r1, res1);
	add(r2, r1, res2);

	Mat img(r1.size(), CV_64FC1);    //计算得到的指数 用double存储

	res1.convertTo(res1, CV_64FC1);   //add与subtract会导致res的类型不为doule
	res2.convertTo(res2, CV_64FC1);   // 此处进行强制转换

	divide(res1, res2, img);

	return img;
}

Mat SAVI(Mat r, Mat nr)
{
	Mat res1(r.size(), CV_64FC1);   //分子
	Mat res2(r.size(), CV_64FC1);   //分母

	subtract(nr, r, res1);
	add(nr, r, res2);
	add(res2, L, res2);

	res1.convertTo(res1, CV_64FC1);   //add与subtract会导致res的类型不为doule
	res2.convertTo(res2, CV_64FC1);   // 此处进行强制转换

	Mat img(r.size(), CV_64FC1);    //计算得到的指数

	divide(res1, res2, img);
	multiply(L + 1, img, img);

	return img;
}

Mat AWEI(Mat g, Mat nr, Mat s1, Mat s2)
{
	Mat res1(g.size(), CV_64FC1);   //分子
	Mat res2(g.size(), CV_64FC1);   //分母
	Mat res3(g.size(), CV_64FC1);

	nr.convertTo(nr, CV_64FC1);
	s2.convertTo(s2, CV_64FC1);

	subtract(g, s1, res1);
	multiply(res1, 4, res1);
	res1.convertTo(res1, CV_64FC1);

	multiply(nr, 0.25, res2);
	multiply(s2, 2.75, res3);
	add(res2, res3, res2);

	Mat img(g.size(), CV_64FC1);

	divide(res1, res2, img);

	return img;
}

Mat IBI(Mat ndbi, Mat savi, Mat mndwi)
{
	Mat res1(ndbi.size(), CV_64FC1);

	add(savi, mndwi, res1);
	divide(res1, 2.0, res1);

	Mat img = Normalize(res1, ndbi);

	return img;
}

void Display(Mat Index, int type)
{

	Mat out(Index.size(), CV_8UC3);

	if (type == 0 || type == 1)          //植被
	{
		for (int i = 0; i < Index.rows; i++)
			for (int j = 0; j < Index.cols; j++)
			{
				if (Index.at<double>(i, j) > 0)
				{
					out.at<Vec3b>(i, j)[0] = 0;
					out.at<Vec3b>(i, j)[1] = saturate_cast<uchar>(Index.at<double>(i, j) * 255);   //添加绿色
					out.at<Vec3b>(i, j)[2] = 0;
				}
				else
				{
					out.at<Vec3b>(i, j)[0] = 0;
					out.at<Vec3b>(i, j)[1] = 0;
					out.at<Vec3b>(i, j)[2] = 0;
				}
			}
	}
	else if (type == 2 || type == 3)      //水体
	{
		for (int i = 0; i < Index.rows; i++)
			for (int j = 0; j < Index.cols; j++)
			{
				if (Index.at<double>(i, j) > 0)
				{
					out.at<Vec3b>(i, j)[0] = saturate_cast<uchar>(Index.at<double>(i, j) * 255);  //添加蓝色
					out.at<Vec3b>(i, j)[1] = 0;
					out.at<Vec3b>(i, j)[2] = 0;
				}
				else
				{
					out.at<Vec3b>(i, j)[0] = 0;
					out.at<Vec3b>(i, j)[1] = 0;
					out.at<Vec3b>(i, j)[2] = 0;
				}
			}
	}
	else if (type == 4 || type == 5)      //土地
	{
		for (int i = 0; i < Index.rows; i++)
			for (int j = 0; j < Index.cols; j++)
			{
				if (Index.at<double>(i, j) > 0)
				{
					out.at<Vec3b>(i, j)[0] = 0;
					out.at<Vec3b>(i, j)[1] = saturate_cast<uchar>(Index.at<double>(i, j) * 255);  //添加黄色
					out.at<Vec3b>(i, j)[2] = saturate_cast<uchar>(Index.at<double>(i, j) * 255);
				}
				else
				{
					out.at<Vec3b>(i, j)[0] = 0;
					out.at<Vec3b>(i, j)[1] = 0;
					out.at<Vec3b>(i, j)[2] = 0;
				}
			}
	}
	
	if (type == 0)    //归一化植被指数
	{
		imshow("归一化植被指数", out);
		waitKey(600);
		//imwrite("归一化植被指数.jpg", out);
	}
	else if (type == 1)   //土壤调节植被指数
	{
		imshow("土壤调节植被指数", out);
		waitKey(600);
		//imwrite("土壤调节植被指数.jpg", out);
	}      
	else if (type == 2)  //修改归一化水体指数
	{
		imshow("修改归一化水体指数", out);
		waitKey(600);
		//imwrite("修改归一化水体指数.jpg", out);
	}
	else if (type == 3)  //自动水体提取指数
	{
		imshow("自动水体提取指数", out);
		waitKey(600);
		//imwrite("自动水体提取指数.jpg", out);
	}
	else if (type == 4)  //归一化裸地指数
	{
		imshow("归一化裸地指数", out);
		waitKey(600);
		//imwrite("归一化裸地指数.jpg", out);
	}
	else if (type == 5)  //建筑用地指数
	{
		imshow("建筑用地指数", out);
		waitKey(0);
		//imwrite("建筑用地指数.jpg", out);
	}


}

void Col_Display(Mat b, Mat g, Mat r)
{
	Mat img(b.size(), CV_8UC3);

	/*for(int i=0;i<b.rows;i++)
		for (int j = 0; j < b.cols; j++)
		{
			img.at<Vec3b>(i, j)[0] = r.at<uchar>(i, j);
			img.at<Vec3b>(i, j)[1] = b.at<uchar>(i, j);
			img.at<Vec3b>(i, j)[2] = g.at<uchar>(i, j);
		}*/
	Mat channel[3] = { b,g,r };

	merge(channel, 3, img);

	imshow("真彩色合成", img);
	waitKey(0);
}

void Binary(Mat img,int type)
{
	Mat out(img.size(),CV_8UC1);
	
	double temp = 0;
	double max = 0;
	double min = 255;

	//最大类间误差法
	
	//灰度拉伸 从0-255
	for(int i=0;i<img.rows;i++)
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

	for(int i=0;i<img.rows;i++)
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

		for(int i=0;i<out.rows;i++)
			for (int j = 0; j < out.cols; j++)
			{
				int val = (int)(out.at<uchar>(i, j));
				if(val<t)
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
		s_temp = f1 * f2*pow((avg1 - avg2), 2)/pow((out.rows-out.cols),2);

		if (s_temp > s)    //找到动态规划后的阈值
		{
			thre = t;
			s = s_temp;
		}
	}

	Mat res(out.size(), out.type());

	//二值化
	for(int i=0;i<res.rows;i++)
		for (int j = 0; j < res.cols; j++)
		{
			int val = (int)(out.at<uchar>(i, j));
			if (val > thre)
			{
				res.at<uchar>(i, j) = 255;
			}
			else
			{
				res.at<uchar>(i, j) = 0;
			}
		}


	if (type == 0)    //归一化植被指数
	{
		imshow("归一化植被指数", res);
		waitKey(600);
		//imwrite("归一化植被指数_binary.jpg", out);
	}
	else if (type == 1)   //土壤调节植被指数
	{
		imshow("土壤调节植被指数", res);
		waitKey(600);
		//imwrite("土壤调节植被指数_binary.jpg", out);
	}
	else if (type == 2)  //修改归一化水体指数
	{
		imshow("修改归一化水体指数", res);
		waitKey(600);
		//imwrite("修改归一化水体指数_binary.jpg", res);
	}
	else if (type == 3)  //自动水体提取指数
	{
		imshow("自动水体提取指数", res);
		waitKey(600);
		//imwrite("自动水体提取指数_binary.jpg", res);
	}
	else if (type == 4)  //归一化裸地指数
	{
		imshow("归一化裸地指数", res);
		waitKey(600);
		//imwrite("归一化裸地指数_binary.jpg", res);
	}
	else if (type == 5)  //建筑用地指数
	{
		imshow("建筑用地指数", res);
		waitKey(0);
		//imwrite("建筑用地指数_binary.jpg", res);
	}
}