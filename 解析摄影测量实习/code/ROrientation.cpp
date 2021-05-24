#include"Gram.h"

ROrientation::ROrientation(char *path_int, char *path_gcp, char *path_left, char*path_right):PhotoGram(path_int,path_gcp)
{
	setPoints(path_left, 0);
	setPoints(path_right, 1);
}

void ROrientation::setPoints(char *path,int type)
{

	FILE *fp;
	errno_t err = fopen_s(&fp, path, "r");

	fscanf_s(fp, "%*[^\n]");     //单独读取第一行

	int up_num;
	fscanf_s(fp, "%d", &up_num);

	UnknowPT temp;
	vector<resPoint> check = getGCP();

	temp.photo_num = type;
	temp.type = UNKNOWN;

	for (int i = 0; i < up_num; i++)
	{
		int valid;     //判断量测精度是否达标
		fscanf_s(fp, "%d %lf %lf %lf %lf %d", &temp.num, &temp.pxl_x, &temp.pxl_y, &temp.pho_x, &temp.pho_y, &valid);


		for (int j = 0; j < check.size(); j++)
		{
			if (temp.num == check[j].num)   //判断是否为地面控制点跳出
			{
				temp.type = CTRL;
				break;
			}
		}

		if (valid == 128)
		{
			temp.pho_x /= 1000;
			temp.pho_y /= 1000;
			if (type == 0)
			{
				left.push_back(temp);
				temp.type = UNKNOWN;
			}
			else if (type == 1)
			{
				right.push_back(temp);
				temp.type = UNKNOWN;
			}
			
		}

	}
	fclose(fp);
}

void ROrientation::calculate(int times)
{
	cout << "--------------" << "相对定向" << "--------------" << endl;

	match_res = match();
	ExterElement temp;
	res.kappa = res.omega = res.phi = res.u = res.v = 0;

	Eigen::MatrixXd A(match_res.size(), 5);
	Eigen::MatrixXd L(match_res.size(), 1);

	for (int t = 0; t < times; t++)
	{
		temp.phi = res.phi; 
		temp.omega = res.omega;
		temp.kappa = res.kappa;

		rotate = calRotate(temp);

		for (int i = 0; i < match_res.size(); i++)
		{
			Eigen::MatrixXd a=diff(left[match_res[i].left], right[match_res[i].right]);
			Eigen::MatrixXd l = resi(left[match_res[i].left], right[match_res[i].right]);

			A.block<1, 5>(i, 0) = a;
			L.block<1, 1>(i, 0) = l;
		}

		Eigen::MatrixXd delta = adj(A, L);		

		res.phi += delta(0, 0);
		res.omega += delta(1, 0);
		res.kappa += delta(2, 0);
		res.u += delta(3, 0);
		res.v += delta(4, 0);

		if (abs(delta(0, 0)) < 3e-5 && abs(delta(1, 0)) < 3e-5 && abs(delta(2, 0)) < 3e-5
			&& abs(delta(3, 0)) < 3e-5 && abs(delta(4, 0)) < 3e-5)
		{
			cout <<endl<< "相对定向收敛!"<<"      "<<"迭代次数:" <<t << endl;
			break;
		}
		
	}
}

vector<resPoint> ROrientation::translate()
{
	loc.resize(right.size(), 4);
	InterElement element = getInt();
	
	Eigen::MatrixXd trans(1, 3);  //Bx By Bz 平移
	trans(0, 0) = Bx;
	trans(0,1)= Bx * res.u;
	trans(0,2) = Bx * res.v;
	
	Eigen::MatrixXd rot(3, 3);  //相对旋转
	ExterElement t;
	t.phi = res.phi; t.omega = res.omega; t.kappa = res.kappa;
	rot = calRotate(t);

	Eigen::MatrixXd temp(3, 1);
	for (int i = 0; i < right.size(); i++)
	{
		temp(0, 0) = right[i].pho_x; temp(1, 0) = right[i].pho_y; temp(2, 0) = -element.f;
		loc.block(i, 1, 1, 3) = (rot*rotate * temp).transpose()+trans;
		loc(i, 0) = right[i].num;
	}

	//求解空间辅助坐标
	
	for (int i = 0; i < match_res.size(); i++)
	{
		resPoint temp = point_prj(left[match_res[i].left].pho_x, left[match_res[i].left].pho_y, -element.f,
			loc(match_res[i].right, 1), loc(match_res[i].right, 2), loc(match_res[i].right, 3));
		temp.num = left[match_res[i].left].num;
		temp.type = left[match_res[i].left].type;
		respt.push_back(temp);
	}

	return respt;
}

void ROrientation::show()
{
	cout << endl << "相对定向参数:" << endl;
	cout << res.phi << "   " << res.omega << "   " << res.kappa << "    " << res.u << "    " << res.v << endl;
	cout << endl << "相对定向同名点空间辅助坐标" << endl;
	
	resPoint temp;
	for (int i = 0; i < match_res.size(); i++)
	{
		temp = respt[i];
		cout << setw(8) << temp.num << " " << setw(11) << temp.X << " " << setw(11) << temp.Y << " " << setw(11) << temp.Z << endl;
	}
}

Eigen::MatrixXd ROrientation::diff(UnknowPT l, UnknowPT r)
{
	Eigen::MatrixXd diff_res(1, 5);
	Eigen::MatrixXd left(3,1), right(3,1);
	InterElement element = getInt();

	left(0, 0) = l.pho_x;
	left(1, 0) = l.pho_y;
	left(2, 0) = -element.f;

	right(0, 0) = r.pho_x;
	right(1, 0) = r.pho_y;
	right(2, 0) = -element.f;

	//只有右片乘旋转矩阵
	right = rotate * right;

	double By = Bx * res.u;
	double Bz = Bx * res.v;

	Eigen::MatrixXd temp(3, 3);
	temp(0, 0) = Bx; temp(0, 1) = By; temp(0, 2) = Bz;
	temp.block<1, 3>(1, 0) = left.transpose();
	
	//d phi
	temp(2, 0) = -right(2, 0); temp(2, 1) = 0; temp(2, 2) = right(0, 0);
	diff_res(0, 0) = temp.determinant();

	//d omgea
	temp(2, 0) = -right(1, 0)*sin(res.phi);
	temp(2, 1) =right(0,0)*sin(res.phi)-right(2,0)*cos(res.phi); 
	temp(2, 2) = right(1, 0)*cos(res.phi);
	diff_res(0, 1) = temp.determinant();

	//d kappa
	temp(2, 0) = -right(1, 0)*cos(res.phi)*cos(res.omega)-right(2,0)*sin(res.omega);
	temp(2, 1) = right(0, 0)*cos(res.phi)*cos(res.omega) + right(2, 0)*sin(res.phi)*cos(res.omega);
	temp(2, 2) = right(0,0)*sin(res.omega)-right(1, 0)*sin(res.phi)*cos(res.omega);
	diff_res(0, 2) = temp.determinant();

	//d u
	diff_res(0, 3) = Bx * (left(2, 0)*right(0, 0) - left(0, 0)*right(2, 0));

	//d v
	diff_res(0, 4) = Bx * (left(0, 0)*right(1, 0) - left(1, 0)*right(0, 0));
	   
	return diff_res;
}

Eigen::MatrixXd ROrientation::resi(UnknowPT l, UnknowPT r)
{
	Eigen::MatrixXd resi_res(1,1);
	Eigen::MatrixXd left(3, 1), right(3, 1);
	InterElement element = getInt();

	left(0, 0) = l.pho_x;
	left(1, 0) = l.pho_y;
	left(2, 0) = -element.f;

	right(0, 0) = r.pho_x;
	right(1, 0) = r.pho_y;
	right(2, 0) = -element.f;

	//只有右片乘旋转矩阵
	right = rotate * right;

	double By = Bx * res.u;
	double Bz = Bx * res.v;

	Eigen::MatrixXd temp(3, 3);

	temp(0, 0) = Bx; temp(0, 1) = By; temp(0, 2) = Bz;
	temp.block<1, 3>(1, 0) = left.transpose();
	temp.block<1, 3>(2, 0) = right.transpose();

	resi_res(0, 0) = -temp.determinant();

	return resi_res;
}

vector<mypair> ROrientation::match()
{
	vector<mypair> pair;
	for (int i = 0; i < left.size(); i++)
	{
		for (int j = 0; j < right.size(); j++)
		{
			if (left[i].num == right[j].num)
			{
				mypair temp;
				temp.left = i; temp.right = j;
				pair.push_back(temp);
			}
		}
	}
	return pair;
}

resPoint ROrientation::point_prj(double x1, double y1, double z1,double x2, double y2,double z2)
{
	resPoint ans;
	double By = Bx * res.u;
	double Bz = Bx * res.v;

	double N1 = (Bx*z2 - Bz * x2) / (x1*z2 - z1*x2);
	double N2 = (Bx*z1 - Bz * x1) / (x1*z2 - z1 * x2);

	ans.X = x1 * N1;
	ans.Y = 0.5*(y1*N1 + y2 * N2 + By);
	ans.Z = N1 * z1;

	return ans;

}