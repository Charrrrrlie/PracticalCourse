#pragma once
#include<Eigen/Dense>

#define ROW 2848   //原相片行
#define COL 4272   //原相片列
#define pix (5.1966*1e-3) //像元大小
using namespace std;

struct GroundPt
{
	double x, y, z;
};

struct PhotoPt
{
	int num;
	double x, y;
};

struct ExterElement
{
	Eigen::MatrixXd R;
	double Xs, Ys, Zs;
	double phi, omega, kappa;
};

struct InterElement
{
	double x0, y0, f; 
};