#include"DLT.h"

#define TIMES 10 //迭代次数

using namespace std;

unordered_set<int> DLT::Random(int min, int max, int n)
{
	unordered_set<int> res;
	srand((unsigned)time(NULL));
	while (res.size() < n)
	{
		int t = ((max - min + 1) * rand() / (RAND_MAX + 1.0) + min);
		res.insert(t);
	}
	return res;
}

DLT::DLT(vector<PhotoPt> pht, unordered_map<int, GroundPt> gcp,int check_num, string out_path)
{
	int n = pht.size();
	vector<GroundPt> temp_gt;
	vector<PhotoPt> temp_pt;
	for (int i = 0; i < n; i++)
	{
		if (gcp.count(pht[i].num))
		{
			temp_gt.push_back(gcp[pht[i].num]);  //map[num] :XYZ  map.insert(num,XYZ)
			temp_pt.push_back(pht[i]);
		}
		else
		{
			unknownpts.push_back(pht[i]);
		}
	}
	unordered_set<int> seed=Random(0, temp_gt.size() - 1, check_num);

	for (int i = 0; i < temp_gt.size(); i++)
	{
		if (seed.count(i))
		{
			ground_pts_check.push_back(temp_gt[i]);
			pho_pts_check.push_back(temp_pt[i]);
		}
		else
		{
			ground_pts_ctrl.push_back(temp_gt[i]);
			pho_pts_ctrl.push_back(temp_pt[i]);
		}
	}

	seed.clear();
	temp_gt.clear();
	temp_pt.clear();

	out_dir = out_path;
	L.resize(11);
	distort.resize(5);
}

DLT::DLT(vector<PhotoPt> pht, unordered_map<int, GroundPt> gcp, unordered_set<int> check_num, string out_path)
{
	int n = pht.size();
	vector<GroundPt> temp_gt;
	vector<PhotoPt> temp_pt;
	for (int i = 0; i < n; i++)
	{
		if (gcp.count(pht[i].num))
		{
			temp_gt.push_back(gcp[pht[i].num]);  //map[num] :XYZ  map.insert(num,XYZ)
			temp_pt.push_back(pht[i]);
		}
		else
		{
			unknownpts.push_back(pht[i]);
		}
	}

	for (int i = 0; i < temp_gt.size(); i++)
	{
		if (check_num.count(temp_pt[i].num))
		{
			ground_pts_check.push_back(temp_gt[i]);
			pho_pts_check.push_back(temp_pt[i]);
		}
		else
		{
			ground_pts_ctrl.push_back(temp_gt[i]);
			pho_pts_ctrl.push_back(temp_pt[i]);
		}
	}

	temp_gt.clear();
	temp_pt.clear();

	out_dir = out_path;
	L.resize(11);
	distort.resize(5);
}

DLT::~DLT()
{
	ground_pts_ctrl.clear();
	ground_pts_check.clear();
	pho_pts_check.clear();
	pho_pts_ctrl.clear();
	unknownpts.clear();

	L.clear();
	distort.clear();
}

Eigen::MatrixXd DLT::LeastSquare(Eigen::MatrixXd A, Eigen::MatrixXd L)
{
	Eigen::MatrixXd temp = A.transpose() * A;
	if (abs(temp.determinant()) < 1e-9)
	{
		std::cout << "奇异矩阵！特征值为：" <<temp.determinant()<< endl;
	}
	return temp.inverse() * A.transpose()*L;
}

double DLT::Judge(vector<double>L)
{
	double garma = 1 / (L[8] * L[8] + L[9] * L[9] + L[10] * L[10]);
	double x0 = -(L[0] * L[8] + L[1] * L[9] + L[2] * L[10])*garma;   
	double y0 = -(L[4] * L[8] + L[5] * L[9] + L[6] * L[10])*garma;

	double A = (L[0] * L[0] + L[1] * L[1] + L[2] * L[2]) * garma - x0 * x0;
	double B = (L[4] * L[4] + L[5] * L[5] + L[6] * L[6]) * garma - y0 * y0;
	double C = (L[0] * L[4] + L[1] * L[5] + L[2] * L[6]) * garma - x0 * y0;

	double beta = sqrt(C * C / A / B);
	beta = C>0 ? asin(-beta):asin(beta);

	double fx = cos(beta) * sqrt(A);

	return fx;
	
}

void DLT::Init(int num)
{
	Eigen::MatrixXd A(num * 2, 11);
	Eigen::MatrixXd Cons(num * 2, 1);
	A.fill(0);

	for (int i = 0; i < num; i++)
	{
		A(2*i, 0) = ground_pts_ctrl[i].x;
		A(2*i, 1) = ground_pts_ctrl[i].y;
		A(2*i, 2) = ground_pts_ctrl[i].z;
		A(2*i, 3) = 1;

		A(2 * i, 8) = ground_pts_ctrl[i].x * pho_pts_ctrl[i].x;
		A(2 * i, 9) = ground_pts_ctrl[i].y * pho_pts_ctrl[i].x;
		A(2 * i, 10) = ground_pts_ctrl[i].z * pho_pts_ctrl[i].x;

		A(2*i+1, 4) = ground_pts_ctrl[i].x;
		A(2*i+1, 5) = ground_pts_ctrl[i].y;
		A(2*i+1, 6) = ground_pts_ctrl[i].z;
		A(2*i+1, 7) = 1;

		A(2 * i+1, 8) = ground_pts_ctrl[i].x * pho_pts_ctrl[i].y;
		A(2 * i+1, 9) = ground_pts_ctrl[i].y * pho_pts_ctrl[i].y;
		A(2 * i+1, 10) = ground_pts_ctrl[i].z * pho_pts_ctrl[i].y;

		Cons(2 * i, 0) = -pho_pts_ctrl[i].x;
		Cons(2 * i + 1, 0) = -pho_pts_ctrl[i].y;
	}
	
	Eigen::MatrixXd val = LeastSquare(A, Cons);

	for (int i = 0; i < L.size(); i++)
	{
		L[i] = val(i, 0);
	}

	/*Eigen::MatrixXd temp_val = A * val + Cons;
	for (int i = 0; i < num*2; i++)
	{
		cout << temp_val(i, 0) << endl;
	}
	int a;*/
 }

pair<InterElement,ExterElement> DLT::GetElement()
{

	double garma= 1/(L[8] * L[8] + L[9] * L[9] + L[10] * L[10]);
	double x0 = -(L[0] * L[8] + L[1] * L[9] + L[2] * L[10]) * garma;
	double y0 = -(L[4] * L[8] + L[5] * L[9] + L[6] * L[10]) * garma;

	double A = (L[0] * L[0] + L[1] * L[1] + L[2] * L[2]) * garma - x0 * x0;
	double B = (L[4] * L[4] + L[5] * L[5] + L[6] * L[6]) * garma - y0 * y0;
	double C = (L[0] * L[4] + L[1] * L[5] + L[2] * L[6]) * garma - x0 * y0;

	double ds = sqrt(A/B)-1;
	double beta = sqrt(C * C / A / B);
	beta = C > 0 ? asin(-beta) : asin(beta);   //取和C相反的值

	double fx= cos(beta) * sqrt(A);
	double fy = fx / (1 + ds);

	InterElement int_element;
	int_element.x0 = x0;
	int_element.y0 = y0;
	int_element.f = (fx + fy) / 2;   //乱取个平均！


	Eigen::MatrixXd ext(3, 3);
	ext << L[0],L[1],L[2],
		   L[4],L[5],L[6],
		   L[8],L[9],L[10];
	Eigen::MatrixXd vec(3, 1);
	vec << -L[3], -L[7], -1;

	Eigen::MatrixXd line = ext.inverse() * vec; //外方位线元素

	ExterElement ext_element;
	ext_element.Xs = line(0, 0);
	ext_element.Ys = line(1, 0);
	ext_element.Zs = line(2, 0);

	//定义式求b2会超界
	//Eigen::MatrixXd mat(3, 4);    //公式7-2-3
	//mat << L[0], L[1], L[2], L[3],
	//	L[4], L[5], L[6], L[7],
	//	L[8], L[9], L[10], 1;

	//Eigen::MatrixXd param(3, 3);
	//param << fx, -fx * tan(beta), -x0,
	//	0, fx / (1 + ds) / cos(beta), -y0,
	//	0, 0, 1;

	//ext_element.R = (param.inverse() * mat * garma).block<3, 3>(0, 0).transpose();

	double a3 = sqrt(garma) * L[8];
	double b3 = sqrt(garma) * L[9];
	double c3 = sqrt(garma) * L[10];
	double a2 = (sqrt(garma) * L[4] + a3 * sqrt(garma)) * (1 + ds) * cos(beta)/fx;
	double b2 = (L[5] * sqrt(garma) + b3 * y0) * (1 + ds) * cos(beta) / fx;
	double b1 = ((L[1] * sqrt(garma)) + b3 * x0 + b2 * fx * tan(beta)) / fx;

	ext_element.phi = atan(-a3 / c3);
	ext_element.omega = asin(-b3);
	ext_element.kappa = atan(b1/b2);

	//cout << ext_element.R*ext_element.R.inverse();
	return make_pair(int_element, ext_element);

}

unordered_set<int> DLT::GetCheckNum()
{
	unordered_set<int> check_num;
	for (int i = 0; i < pho_pts_check.size(); i++)
	{
		check_num.insert(pho_pts_check[i].num);
	}
	return check_num;
}

vector<double> DLT::GetDistort()
{
	return distort;
}

vector<PhotoPt>  DLT::GetCheckPhoPts()
{
	return pho_pts_check;
}

vector<GroundPt>  DLT::GetCheckGroundPts()
{
	return ground_pts_check;
}

vector<PhotoPt>  DLT::GetUnknownPts()
{
	return unknownpts;
}

vector<double>  DLT::GetL()
{
	return L;
}

vector<GroundPt> DLT::CalUnknown(DLT left_photo, DLT right_photo)
{
	vector<PhotoPt> pht_left_temp = left_photo.GetUnknownPts();
	vector<PhotoPt> pht_right = right_photo.GetUnknownPts();

	vector<PhotoPt> pht_left;
	//可能未知点不在两张像片上
	for (int j = 0; j < pht_right.size(); j++)
	{
		for (int i = 0; i < pht_left_temp.size(); i++)


		{
			if (pht_left_temp[i].num == pht_right[j].num)
			{
				pht_left.push_back(pht_left_temp[i]);
				break;
			}
		}
	}
	pht_left_temp.clear();
	cout << endl << "*************待定点计算*************" << endl;
	cout << "待求点点号:";
	for (auto i = 0; i < pht_left.size(); i++)
	{
		cout << pht_left[i].num << " ";
	}
	cout << endl << endl;

	int size = pht_right.size();
	pair<InterElement, ExterElement> left_element = left_photo.GetElement();
	pair<InterElement, ExterElement> right_element = right_photo.GetElement();
	vector<double> left_distort = left_photo.GetDistort();
	vector<double> right_distort = right_photo.GetDistort();
	vector<double> left_L = left_photo.GetL();
	vector<double> right_L = right_photo.GetL();

	vector<GroundPt> infered = ForGround(pht_left, pht_right, left_L, right_L, left_distort, right_distort
		, left_element, right_element);
	
	//找与52号点间的距离
	double x_52, y_52, z_52;
	for (int i = 0; i < infered.size(); i++)
	{
		if (pht_left[i].num == 52)
		{
			x_52 = infered[i].x;
			y_52 = infered[i].y;
			z_52 = infered[i].z;
		}
	}


	std::cout << "点号" << setw(12) << "X" <<setw(12) << "Y" << setw(18) << "Z" <<setw(20)<<"与52号点距离" <<endl;
	for (int i = 0; i < infered.size(); i++)
	{
		double distance = sqrt(pow(infered[i].x-x_52,2)+ pow(infered[i].y - y_52,2)+ pow(infered[i].z - z_52,2));
		
		//倒着输出
		std::cout << fixed << setprecision(6)
			<<pht_left[i].num<<"\t"
			<< setw(10) << -infered[i].z << "\t" << setw(10) << infered[i].x<<"\t"
			<< setw(10) <<infered[i].y<<"\t"<<setw(10)<<distance<<endl;
	}

	return infered ;
}

void DLT::ExtCheck(DLT left_photo, DLT right_photo)
{
	//vector<GroundPt> gcp_left=left_photo.GetCheckGroundPts();
	vector<GroundPt> gcp = right_photo.GetCheckGroundPts();
	vector<PhotoPt> pht_left_temp = left_photo.GetCheckPhoPts();
	vector<PhotoPt> pht_right = right_photo.GetCheckPhoPts();

	vector<PhotoPt> pht_left;
	//可能检查点不在两张像片上
	
	for (int j = 0; j < pht_right.size(); j++)
	{
		for (int i = 0; i < pht_left_temp.size(); i++)
		{
			if (pht_left_temp[i].num == pht_right[j].num)
			{
				pht_left.push_back(pht_left_temp[i]);
				break;
			}
		}
	}
	pht_left_temp.clear();
	cout << endl << "*************检查点计算*************" << endl;
	cout << "检查点点号:";
	for (auto i =0; i<pht_left.size(); i++)
	{
		cout << pht_left[i].num << " ";
	}
	cout << endl << endl;

	int size = pht_right.size();
	pair<InterElement, ExterElement> left_element = left_photo.GetElement();
	pair<InterElement, ExterElement> right_element = right_photo.GetElement();
	vector<double> left_distort = left_photo.GetDistort();
	vector<double> right_distort = right_photo.GetDistort();
	vector<double> left_L = left_photo.GetL();
	vector<double> right_L = right_photo.GetL();

	vector<GroundPt> infered= ForGround(pht_left, pht_right,left_L,right_L,left_distort,right_distort
											, left_element, right_element);

	if (infered.size() != size)
	{
		cout << "未知点计算出错" << endl;
		return;
	}

	double d_X = 0; double d_Y = 0; double d_Z = 0;
	std::cout << endl << "检查点:" << endl;
	std::cout << setw(10) << "点号" << setw(10) << "精度" << endl;
	for (int i = 0; i < size; i++)
	{
		std::cout << fixed << setprecision(6)
			<< setw(10) << pht_right[i].num << " "  << setw(10)<<abs(gcp[i].x - infered[i].x) << " "
			<<setw(10) << abs(gcp[i].y - infered[i].y) <<" "<< setw(10) << abs(gcp[i].z - infered[i].z) << endl;

		d_X += abs(gcp[i].x - infered[i].x);
		d_Y += abs(gcp[i].y - infered[i].y);
		d_Z += abs(gcp[i].z - infered[i].z);
	}
	d_X /= size;
	d_Y /= size;
	d_Z /= size;
	std::cout <<endl<< "检查点总精度:" << endl;
	std::cout << fixed << setprecision(6)
		<< "Dx  " << d_X << " "
		<< "Dy  " << d_Y << " "
		<< "Dz  " << d_Z << endl << endl;
}

void DLT::Calculate(int times)
{
	double thred;
	int t = 0;

	int size = ground_pts_ctrl.size();
	Eigen::MatrixXd M(size * 2, 16);
	Eigen::MatrixXd W(size * 2, 1);

	M.fill(0);

	double fx = 0;

	cout << "*************直接线性变换求解*************" << endl;

	while (t < times)
	{

		for (int i = 0; i < size; i++)
		{
			double A = L[8] * ground_pts_ctrl[i].x + L[9] * ground_pts_ctrl[i].y + L[10] * ground_pts_ctrl[i].z + 1;
			double x0 = -(L[0] * L[8] + L[1] * L[9] + L[2] * L[10]) / (L[8] * L[8] + L[9] * L[9] + L[10] * L[10]);
			double y0 = -(L[4] * L[8] + L[5] * L[9] + L[6] * L[10]) / (L[8] * L[8] + L[9] * L[9] + L[10] * L[10]);

			double r = sqrt(pow(pho_pts_ctrl[i].x - x0, 2) + pow(pho_pts_ctrl[i].y - y0, 2));

			M(2 * i, 0) = -ground_pts_ctrl[i].x / A;
			M(2 * i, 1) = -ground_pts_ctrl[i].y / A;
			M(2 * i, 2) = -ground_pts_ctrl[i].z / A;
			M(2 * i, 3) = -1.0 / A;
			M(2 * i, 8) = -ground_pts_ctrl[i].x * pho_pts_ctrl[i].x / A;
			M(2 * i, 9) = -ground_pts_ctrl[i].y * pho_pts_ctrl[i].x / A;
			M(2 * i, 10) = -ground_pts_ctrl[i].z * pho_pts_ctrl[i].x / A;

			M(2 * i + 1, 4) = -ground_pts_ctrl[i].x / A;
			M(2 * i + 1, 5) = -ground_pts_ctrl[i].y / A;
			M(2 * i + 1, 6) = -ground_pts_ctrl[i].z / A;
			M(2 * i + 1, 7) = -1.0 / A;
			M(2 * i + 1, 8) = -ground_pts_ctrl[i].x * pho_pts_ctrl[i].y / A;
			M(2 * i + 1, 9) = -ground_pts_ctrl[i].y * pho_pts_ctrl[i].y / A;
			M(2 * i + 1, 10) = -ground_pts_ctrl[i].z * pho_pts_ctrl[i].y / A;

			M(2 * i, 11) = -(pho_pts_ctrl[i].x - x0) * pow(r, 2);
			M(2 * i, 12) = -(pho_pts_ctrl[i].x - x0) * pow(r, 4);
			M(2 * i, 13) = -(pho_pts_ctrl[i].x - x0) * pow(r, 6);
			M(2 * i, 14) = -(2 * pow(pho_pts_ctrl[i].x - x0, 2) + pow(r, 2));
			M(2 * i, 15) = -(2 * (pho_pts_ctrl[i].x - x0) * (pho_pts_ctrl[i].y - y0));

			M(2 * i + 1, 11) = -(pho_pts_ctrl[i].y - y0) * pow(r, 2);
			M(2 * i + 1, 12) = -(pho_pts_ctrl[i].y - y0) * pow(r, 4);
			M(2 * i + 1, 13) = -(pho_pts_ctrl[i].y - y0) * pow(r, 6);
			M(2 * i + 1, 14) = -(2 * (pho_pts_ctrl[i].x - x0) * (pho_pts_ctrl[i].y - y0));
			M(2 * i + 1, 15) = -(2 * pow(pho_pts_ctrl[i].y - y0, 2) + pow(r, 2));


			W(2 * i, 0) = pho_pts_ctrl[i].x / A;
			W(2 * i + 1, 0) = pho_pts_ctrl[i].y / A;
		}

		Eigen::MatrixXd val = LeastSquare(M, W);

		for (int i = 0; i < 11; i++)
		{
			L[i] = val(i, 0);
		}
		for (int j = 0; j < 5; j++)
		{
			distort[j] = val(11 + j, 0);
		}

		double temp_fx = Judge(L);

		if (abs(temp_fx - fx) < 1e-3)
		{
			//cout << val << endl;
			cout << "\tDLT Success in " << t << " times!" << endl << endl;

			//结果输出
			ofstream f(out_dir);
		
			Eigen::MatrixXd X = M * val - W;
		
			for (int ii = 0; ii < pho_pts_ctrl.size(); ii++)
			{
				f << pho_pts_ctrl[ii].num << " " << pho_pts_ctrl[ii].x << " " <<
					pho_pts_ctrl[ii].y << " " << X(2 * ii, 0) << " " << X(2 * ii + 1, 0) << endl;
			}
			f.close();

			PrecisionDisplay(M, val, W, size);
			break;
		}
		else
		{
			fx = temp_fx;
			t++;
		}

	}

	GetElement();
}

vector<GroundPt> DLT::ForGround(vector<PhotoPt> pht_left, vector<PhotoPt> pht_right
	, vector<double> left_L, vector<double> right_L, vector<double> left_distort,vector<double> right_distort,
	pair<InterElement, ExterElement> left_element, pair<InterElement, ExterElement> right_element)

{
	int size = pht_right.size();
	vector<GroundPt> infered;

	//畸变改正  
	for (int i = 0; i < size; i++)
	{
		double r_left = sqrt(pow(pht_left[i].x - left_element.first.x0, 2) + pow(pht_left[i].y - left_element.first.y0, 2));
		double dx_left = (pht_left[i].x - left_element.first.x0) * (left_distort[0] * pow(r_left, 2) + left_distort[1] * pow(r_left, 4) + left_distort[2] * pow(r_left, 6))
			+ left_distort[3] * (pow(r_left, 2) + 2 * pow(pht_left[i].x - left_element.first.x0, 2))
			+ 2 * left_distort[4] * (pht_left[i].x - left_element.first.x0) * (pht_left[i].y - left_element.first.y0);
		double dy_left = (pht_left[i].y - left_element.first.y0) * (left_distort[0] * pow(r_left, 2) + left_distort[1] * pow(r_left, 4) + left_distort[2] * pow(r_left, 6))
			+ left_distort[4] * (pow(r_left, 2) + 2 * pow(pht_left[i].y - left_element.first.y0, 2))
			+ 2 * left_distort[3] * (pht_left[i].x - left_element.first.x0) * (pht_left[i].y - left_element.first.y0);

		double r_right = sqrt(pow(pht_right[i].x - right_element.first.x0, 2) + pow(pht_right[i].y - right_element.first.y0, 2));
		double dx_right = (pht_right[i].x - right_element.first.x0) * (right_distort[0] * pow(r_right, 2) + right_distort[1] * pow(r_right, 4) + right_distort[2] * pow(r_right, 6))
			+ right_distort[3] * (pow(r_right, 2) + 2 * pow(pht_right[i].x - right_element.first.x0, 2))
			+ 2 * right_distort[4] * (pht_right[i].x - right_element.first.x0) * (pht_right[i].y - right_element.first.y0);
		double dy_right = (pht_right[i].y - right_element.first.y0) * (right_distort[0] * pow(r_right, 2) + right_distort[1] * pow(r_right, 4) + right_distort[2] * pow(r_right, 6))
			+ right_distort[4] * (pow(r_right, 2) + 2 * pow(pht_right[i].y - right_element.first.y0, 2))
			+ 2 * right_distort[3] * (pht_right[i].x - right_element.first.x0) * (pht_right[i].y - right_element.first.y0);

		pht_left[i].x += dx_left;
		pht_left[i].y += dy_left;

		pht_right[i].x += dx_right;
		pht_right[i].y += dy_right;

	}


	//逐点结算
	for (int i = 0; i < size; i++)
	{
		GroundPt res;
		Eigen::MatrixXd N(4, 3);
		Eigen::MatrixXd Q(4, 1);
		int times = 0;

		//求解近似值
		Eigen::MatrixXd init_A(4, 3);
		Eigen::MatrixXd init_L(4, 1);
		init_A << left_L[0] + left_L[8] * pht_left[i].x, left_L[1] + left_L[9] * pht_left[i].x, left_L[2] + left_L[10] * pht_left[i].x,
			left_L[4] + left_L[8] * pht_left[i].y, left_L[5] + left_L[9] * pht_left[i].y, left_L[6] + left_L[10] * pht_left[i].y,
			right_L[0] + right_L[8] * pht_right[i].x, right_L[1] + right_L[9] * pht_right[i].x, right_L[2] + right_L[10] * pht_right[i].x,
			right_L[4] + right_L[8] * pht_right[i].y, right_L[5] + right_L[9] * pht_right[i].y, right_L[6] + right_L[10] * pht_right[i].y;
		init_L << -(left_L[3] + pht_left[i].x), -(left_L[7] + pht_left[i].y),
			-(right_L[3] + pht_right[i].x), -(right_L[7] + pht_right[i].y);

		Eigen::MatrixXd init = (init_A.transpose() * init_A).inverse() * init_A.transpose() * init_L;
		/*	cout << gcp[i].x << " " << gcp[i].y << " " << gcp[i].z << endl;
			cout << init<<endl;*/
		res.x = init(0, 0); res.y = init(1, 0); res.z = init(2, 0);

		while (times < TIMES)  //TIMES
		{
			double A_left;
			double A_right;

			A_left = left_L[8] * res.x + left_L[9] * res.y + left_L[10] * res.z + 1;
			A_right = right_L[8] * res.x + right_L[9] * res.y + right_L[10] * res.z + 1;

			{
				N(0, 0) = -(left_L[0] + left_L[8] * pht_left[i].x) / A_left;
				N(0, 1) = -(left_L[1] + left_L[9] * pht_left[i].x) / A_left;
				N(0, 2) = -(left_L[2] + left_L[10] * pht_left[i].x) / A_left;

				N(1, 0) = -(left_L[4] + left_L[8] * pht_left[i].y) / A_left;
				N(1, 1) = -(left_L[5] + left_L[9] * pht_left[i].y) / A_left;
				N(1, 2) = -(left_L[6] + left_L[10] * pht_left[i].y) / A_left;

				N(2, 0) = -(right_L[0] + right_L[8] * pht_right[i].x) / A_right;
				N(2, 1) = -(right_L[1] + right_L[9] * pht_right[i].x) / A_right;
				N(2, 2) = -(right_L[2] + right_L[10] * pht_right[i].x) / A_right;

				N(3, 0) = -(right_L[4] + right_L[8] * pht_right[i].y) / A_right;
				N(3, 1) = -(right_L[5] + right_L[9] * pht_right[i].y) / A_right;
				N(3, 2) = -(right_L[6] + right_L[10] * pht_right[i].y) / A_right;

				Q(0, 0) = (left_L[3] + pht_left[i].x) / A_left;
				Q(1, 0) = (left_L[7] + pht_left[i].y) / A_left;
				Q(2, 0) = (right_L[3] + pht_right[i].x) / A_right;
				Q(3, 0) = (right_L[7] + pht_right[i].y) / A_right;
			}

			//防止输出一大串奇异矩阵警告
			Eigen::MatrixXd val = (N.transpose() * N).inverse() * N.transpose() * Q;//DLT::LeastSquare(N, Q);

			//更新
			if (abs(res.x - val(0, 0)) < 1e-2
				&& abs(res.y - val(1, 0)) < 1e-2
				&& abs(res.z - val(2, 0)) < 1e-2)
			{
				infered.push_back(res);
				break;
			}

			res.x = val(0, 0);
			res.y = val(1, 0);
			res.z = val(2, 0);
			times++;
		}

	}
	return infered;
}

void DLT::Check()
{
	int size = ground_pts_check.size();

	pair<InterElement, ExterElement> element_pair = GetElement();
	InterElement element = element_pair.first;
	vector<double> var(2);  //方差


	for (int i = 0; i < size; i++)
	{
		double r = sqrt(pow(pho_pts_check[i].x - element.x0, 2) + pow(pho_pts_check[i].y - element.y0, 2));

		//distort = { 0.0001026855412169485, -2.45513140909504e-07,0,6.306783883098746e-05,1.236671554277367e-05 };


		//畸变改正
		double dx =(pho_pts_check[i].x - element.x0) * (distort[0] * pow(r, 2) + distort[1] * pow(r, 4) + distort[2] * pow(r, 6))
			+ distort[3] * (pow(r, 2) + 2 * pow(pho_pts_check[i].x - element.x0, 2))
			+ 2 * distort[4] * (pho_pts_check[i].x - element.x0) * (pho_pts_check[i].y - element.y0);
		double dy =(pho_pts_check[i].y - element.y0) * (distort[0] * pow(r, 2) + distort[1] * pow(r, 4) + distort[2] * pow(r, 6))
			+ distort[4] * (pow(r, 2) + 2 * pow(pho_pts_check[i].y - element.y0, 2))
			+ 2 * distort[3] * (pho_pts_check[i].x - element.x0) * (pho_pts_check[i].y - element.y0);
		
		double x = pho_pts_check[i].x+dx;
		double y = pho_pts_check[i].y+dy;

		double vx = -(L[0] * ground_pts_check[i].x + L[1] * ground_pts_check[i].y + L[2] * ground_pts_check[i].z + L[3]) /
			(L[8] * ground_pts_check[i].x + L[9] * ground_pts_check[i].y + L[10] * ground_pts_check[i].z + 1)-x;

		double vy= -(L[4] * ground_pts_check[i].x + L[5] * ground_pts_check[i].y + L[6] * ground_pts_check[i].z + L[7]) /
			(L[8] * ground_pts_check[i].x + L[9] * ground_pts_check[i].y + L[10] * ground_pts_check[i].z + 1)-y;

		var[0] += abs(vx);
		var[1] += abs(vy);
	}

	var[0] /= size;
	var[1] /= size;

	cout << "重投影误差精度：" << endl
		<< " x方向 " << var[0] << "mm   约" << double(var[0] / (pix)) << "像素" << endl
		<< " y方向 " << var[1] << "mm   约" << double(var[1] / (pix)) << "像素" << endl;
}

void DLT::PrecisionDisplay(Eigen::MatrixXd A, Eigen::MatrixXd X, Eigen::MatrixXd L, int num)
{
	Eigen::MatrixXd V = A * X - L;
	double v = (V.transpose() * V)(0, 0);
	double sigma = sqrt(v / (num - 8));
	cout << "观测值中误差(sigma)：" << sigma << endl << endl;

	Eigen::MatrixXd Q = (A.transpose() * A).inverse(); //A.transpose() * (A * A.transpose()).inverse() * A;
	//cout << Q;

	cout << "未知数解算理论精度：" << endl;
	cout << fixed << setprecision(6)
		<< setw(16) << "L1: " << sigma * sqrt(Q(0, 0)) << endl
		<< setw(16) << "L2: " << sigma * sqrt(Q(1, 1)) << endl
		<< setw(16) << "L3: " << sigma * sqrt(Q(2, 2)) << endl
		<< setw(16) << "L4: " << sigma * sqrt(Q(3, 3)) << endl
		<< setw(16) << "L5: " << sigma * sqrt(Q(4, 4)) << endl
		<< setw(16) << "L6: " << sigma * sqrt(Q(5, 5)) << endl
		<< setw(16) << "L7: " << sigma * sqrt(Q(6, 6)) << endl
		<< setw(16) << "L8: " << sigma * sqrt(Q(7, 7)) << endl
		<< setw(16) << "L9: " << sigma * sqrt(Q(8, 8)) << endl
		<< setw(16) << "L10: " << sigma * sqrt(Q(9, 9)) << endl
		<< setw(16) << "L11: " << sigma * sqrt(Q(10, 10)) << endl
		<< setw(16) << "k1: " << sigma * sqrt(Q(11, 11)) << endl
		<< setw(16) << "k2: " << sigma * sqrt(Q(12, 12)) << endl
		<< setw(16) << "k3: " << sigma * sqrt(Q(13, 13)) << endl
		<< setw(16) << "p1: " << sigma * sqrt(Q(14, 14)) << endl
		<< setw(16) << "p2: " << sigma * sqrt(Q(15, 15)) << endl;

}

void DLT::Display()
{
	cout << endl;
	cout << "--------直接线性变换L系数--------" << endl;
	cout << fixed << setprecision(6)
		<< setw(8) <<L[0]<< " " << L[1] << " " << L[2] << " "
		<< L[3] << " " << L[4] << " " << L[5] << " " 
		<< L[6] << " " << L[7] << " " << L[8] << " " 
		<< L[9] << " " << L[10] << endl<<endl;

	pair<InterElement, ExterElement> element_pair = GetElement();
	cout << "--------外方位元素--------" << endl;
	
	cout << setw(10) << "Xs" << setw(10) << "Ys" << setw(10) << "Zs" << endl;
	cout<< fixed << setprecision(6) 
		<< setw(12) << element_pair.second.Xs << " "
		<< setw(12) << element_pair.second.Ys << " "
		<< setw(12) << element_pair.second.Zs  << endl<<endl;

	cout << setw(10) << "phi" << setw(10) << "omega" << setw(10) << "kappa" << endl;
	cout<< fixed << setprecision(6)
		<< setw(12) << element_pair.second.phi<<" "
		<< setw(12) << element_pair.second.omega<<" "
		<< setw(12) << element_pair.second.kappa <<endl<<endl;

	cout << "--------内方位元素-------" << endl;
	cout << setw(10) << "x0" << setw(10) << "y0" << setw(10) << "f" << endl;
	cout << fixed << setprecision(6)
		<< setw(12) << element_pair.first.x0 << " "
		<< setw(12) << element_pair.first.y0 << " "
		<< setw(12) << element_pair.first.f << endl<<endl;

	cout << "--------畸变参数(k1,k2,k3,p1,p2)--------" << endl;
	cout << fixed << setprecision(12)
		<< setw(8) << distort[0] << " " << distort[1] << " " << distort[2] << " "
		<< distort[3] << " " << distort[4] << endl<<endl<<endl;
}