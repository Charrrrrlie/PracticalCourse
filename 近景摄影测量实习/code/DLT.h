#pragma once
#include<Eigen/Dense>
#include<vector>
#include<unordered_set>
#include<unordered_map>
#include<iostream>
#include<fstream>
#include <iomanip>
#include"struct.h"

class DLT
{
public:
	DLT(vector<PhotoPt> pht,unordered_map<int, GroundPt> gcp,int check_num,string out_path); //检查点个数
	DLT(vector<PhotoPt> pht, unordered_map<int, GroundPt> gcp, unordered_set<int> seed,string out_path); //检查点编号初始化

	Eigen::MatrixXd LeastSquare(Eigen::MatrixXd A, Eigen::MatrixXd L);
	unordered_set<int> Random(int min, int max, int n);    //随机取检查点

	double Judge(vector<double> L);   //跳出值计算
	void Init(int num);       //L参数初值  选择前n个数求解
	void Calculate(int times);  //直接线性变换解算

	pair<InterElement,ExterElement> GetElement();  //计算内外方位元素
	void Display();               //可视化
	vector<double> GetDistort();                   //返回畸变参数
	unordered_set<int> GetCheckNum();              //返回检查点点号
	vector<PhotoPt> GetCheckPhoPts();              //返回检查点像方坐标
	vector<GroundPt> GetCheckGroundPts();          //返回检查点物方坐标
	vector<PhotoPt> GetUnknownPts();               //返回未知点像方坐标
	vector<double> GetL();                         //返回L系数

	

	//物方点解算!!!!
	vector<GroundPt> ForGround(vector<PhotoPt> pht_left, vector<PhotoPt> pht_right
		            , vector<double> left_L, vector<double> right_L,
				vector<double> left_distort, vector<double> right_distort,
			pair<InterElement, ExterElement> left_element, pair<InterElement, ExterElement> right_element);
	
	void ExtCheck(DLT left_photo, DLT right_photo); //外精度检查
	vector<GroundPt> CalUnknown(DLT left_photo, DLT right_photo); //未知点解算

	void Check();      //重投影误差 精度检查
	void PrecisionDisplay(Eigen::MatrixXd A, Eigen::MatrixXd X, Eigen::MatrixXd L, int num);   //精度输出
	~DLT();


private:
	vector<GroundPt> ground_pts_ctrl;
	vector<GroundPt> ground_pts_check;
	vector<PhotoPt> pho_pts_ctrl;
	vector<PhotoPt> pho_pts_check;
	vector<PhotoPt> unknownpts;
	string out_dir;
	vector<double> L;   //直接线性变换参数
	vector<double> distort; // 畸变参数
};