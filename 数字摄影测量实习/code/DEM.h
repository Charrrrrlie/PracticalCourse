#pragma once

#include <opencv2/highgui.hpp>  
#include <opencv2/core.hpp>  

#include<vector>
#include<iostream>
#include<fstream>
#include<string>

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/Texture2D>
#include <osg/ShapeDrawable>
#include<osg/LineWidth>

struct PtLoc {
	double X;
	double Y;
	double Z;
	int in;//进入网格的边号
	int col;
	int row;//所在边的序号
	int HK;//0为横边，1为数边
};


struct CLine {
	std::vector<PtLoc> pt;
	int shape = 0;//0为开曲线，1为闭曲线
};


class DEM
{
public:
	DEM(std::string path, std::string image_path);    //构造函数里面初始化
	~DEM();

	std::vector<std::vector<double>> GetNearestHeight();//无效值处理
	std::vector<std::vector<double>> Interp();     //匹配正射影像和DEM
	osg::Node* Create_DEM_lines(std::string img_path, std::string dem_path); //可视化等高线
	osg::Node* Create_DEM(std::string img_path, std::string dem_path); //可视化DEM

	std::vector<std::vector<double>> StateMatHk(int t);
	std::vector<std::vector<double>> StateMatVk(int t);

	void GetlinePt(std::vector<std::vector<double>>& Hk, std::vector<std::vector<double>>& Vk, double Z, int ni, int nj, CLine& line);
	void getline(std::vector<std::vector<double>> Hk, std::vector<std::vector<double>> Vk, double Z, std::vector<CLine>& Line);
	void savelines(std::string dir, std::vector<CLine> Line, int k);
	void nlines(std::string dir, double dz);



private:
	std::vector<std::vector<double>> height_mat;   //存数据
	std::vector<std::vector<double>> dense_height_mat; //插值后的密集格网
	std::vector<std::pair<int, int>> invalid_val; //DEM无效值
	std::vector<std::vector<CLine>> AllLines;//存所有的线

	int col;
	int row;
	double A;
	double X0, Y0;    //起始点实际地面坐标      
	double dx, dy;    //格网点在实际地面间距

	double max_z, min_z;

	double delz = 5;	//设置等高线间隔 默认5米

	int image_col, image_row;
};
