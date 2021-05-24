#pragma once
#include<iostream>
#include<fstream>
#include <iomanip>
#include<Eigen/Dense>
#include<Eigen/Sparse>
#include<vector>

#define CTRL 0;
#define UNKNOWN 1;
#define PI 3.1415926;

using namespace std;

struct mypair   //匹配点
{
	int left, right;
};

struct resPoint   //地面点
{
	int type;
	int num;
	double X, Y, Z;
};

struct phoPoint   //像点
{
	int num;
	double x, y;
};

struct Point
{
	int type;  //类型
	int num;   //点号
	double g_x, g_y, g_z; //地面坐标
	double p_x, p_y;  //像片坐标
};

struct UnknowPT
{
	int type;
	int photo_num;    //像片号
	int num;
	double pxl_x, pxl_y;   //像素坐标
	double pho_x, pho_y;   //像片坐标
};

struct ExterElement
{
	int photo_num=0;    //像片号
	double phi, omega, kappa, Xs, Ys, Zs;
};

struct InterElement
{
	double x, y, f;
	int m;   
};

struct RelativeElement
{
	double phi, omega, kappa, u, v;
};

struct AbsoluteElement
{
	double lambda, phi, omega, kappa, dx, dy, dz;
};

class PhotoGram
{
public:
	PhotoGram(char *path_int, char *path_gcp);
	void setInt(char *path);   //读取内方位元素
	void setGCP(char *path);    //读取地面控制点

	Eigen::MatrixXd adj(Eigen::MatrixXd A, Eigen::MatrixXd L);
	Eigen::MatrixXd calRotate(ExterElement temp);

	vector<resPoint> getGCP();
	InterElement getInt();
	

protected:
	InterElement int_element;
	vector<resPoint> gcp;	
};

class RSanction: public PhotoGram
{

public:
	RSanction(char *path_int, char *path_realgcp,char *path_photogcp);
	void init();      //给定初值

	Eigen::MatrixXd diff(Point pt,ExterElement temp);    // 系数矩阵
	Eigen::MatrixXd resi(Point pt, ExterElement temp);    // 误差向量
	
	void calRSanction(int times,int photo_num); 
	void show();

	ExterElement getExt();
	void setphoGCP(char *path);

private:
	ExterElement initial;
	ExterElement res;
	Eigen::MatrixXd rotate;
	vector<phoPoint> p_gcp;   //单像片坐标
};

class FSanction :public RSanction
{
public:
	FSanction(char *path_int, char *path_rgcp,char*path_pgcp, char *path_ext ,int count,int type);
	void setUP(char *path,int photo_num,int time);    //读取连接点 像片号 次数
	
	void setExt(char *path);   //读取外方位元素
	double angleTrans(double a);
	

	void point_prj(int point_num);   //点投影系数法
	Eigen::MatrixXd diff(Point pt, ExterElement temp);
	Eigen::MatrixXd resi(Point pt, ExterElement temp);

	void calFSanction(int times,int point_num);
private:
	vector<vector<UnknowPT>> unknown_pt;
	vector<ExterElement> ext_element;
	int photo_count;
	int type;
	Eigen::MatrixXd rotate;

	resPoint init;
	resPoint res;
};

class IOrientation
{
public:
	IOrientation(char *path);
	Eigen::MatrixXd calculate();
private:
	int num;
	Eigen::MatrixXd params;
	Eigen::MatrixXd origin;
	Eigen::MatrixXd change;
};

class ROrientation:public PhotoGram
{
public:
	ROrientation(char *path_int, char *path_gcp,char *path_left,char*path_right);
	void setPoints(char *path, int type);
	
	void calculate(int times);
	vector<resPoint> translate();
	void show();

	Eigen::MatrixXd diff(UnknowPT l, UnknowPT r);
	Eigen::MatrixXd resi(UnknowPT l, UnknowPT r);

	resPoint point_prj(double x1,double y1,double z1,double x2,double y2,double z2);
	vector<mypair> match();
	
private:

	double Bx = 0.08;   //Bx常数
	Eigen::MatrixXd loc;    //右片空间模型坐标
	vector<resPoint> respt;  //右片同名点空间辅助坐标
	vector<UnknowPT> left, right; 
	Eigen::MatrixXd rotate;   //右片旋转矩阵
	RelativeElement res;      //连续法相对定向参数
	vector<mypair> match_res;  // 左右片点匹配结果
};

class AOrientation :public ROrientation
{
public:
	AOrientation(char *path_int, char *path_gcp,char *path_model, char *path_left, char*path_right);
	void getModel(char *path);
	
	void calculate(int times);
	vector<resPoint> translate();

	vector<resPoint> gravitilize(vector<resPoint> pts);
	Eigen::MatrixXd diff(resPoint pt);
	Eigen::MatrixXd resi(resPoint pt_model, resPoint pt_gravi);
	
private:
	resPoint gravity;   //仅用XYZ成员存重心化结果
	vector<resPoint> model;
	AbsoluteElement res;
};

class Bundle: public PhotoGram
{
public:
	Bundle(char* path_int, char* path_gcp,char*path_init_ext, char* path_init_pt, int count);
	void setPt(char* path, int photo_num, int time);  //读入所有像片中的点
	void setExt(char* path);      //读入所有像片的外方位元素
	void setPoints(char* path);    //设置地面点与像片点的对应关系
	
	void calculate(int times);    //光束法平差计算
	void show(char* path1,char *path2);    

	Eigen::MatrixXd adj(Eigen::MatrixXd A, Eigen::MatrixXd L);   //稀疏矩阵求解方法

	Eigen::MatrixXd diff(Point pt, ExterElement temp);
	Eigen::MatrixXd resi(Point pt, ExterElement temp);
	void init();
	vector<mypair> search(int num);    //寻找像片点与地面点的匹配

	double angleTrans(double a);   //WuCAPs格式转换 正反变换
	double invTrans(double len);

private:
	vector<vector<UnknowPT>> pts;  
	int pt_num=0;    //所有像片点数

	vector<ExterElement> element;   //输出
	vector<resPoint> res;
	
	vector<resPoint> init_pt;
	vector<int> point;   //点顺序——-点号
};