#include"RSanction.h"

using namespace std;

unordered_set<int> RSanction::Random(int min, int max, int n)
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

RSanction::RSanction(vector<PhotoPt> pht, unordered_map<int, GroundPt> gcp, int check_num, string outpath)
{
	int n = pht.size();
	vector<GroundPt> temp_gt;
	vector<PhotoPt> temp_pt;
	for (int i = 0; i < n; i++)
	{
		if (gcp.count(pht[i].num))
		{
			temp_gt.push_back(gcp[pht[i].num]);
			temp_pt.push_back(pht[i]);
		}
		else
		{
			unknownpts.push_back(pht[i]);
		}
	}
	unordered_set<int> seed = Random(0, temp_gt.size() - 1, check_num);

	//cout << "Check Point Numbers:";
	//for (auto i = seed.begin(); i != seed.end(); i++)
	//{
	//	cout << temp_pt[*i].num << " ";
	//}
	//cout << endl << endl;

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

	//给zls的demo
	//ofstream f("../../../zmj.txt");
	//for (int i = 0; i < ground_pts_ctrl.size(); i++)
	//{
	//	f << pho_pts_ctrl[i].x << "\t" << pho_pts_ctrl[i].y << "\t"
	//		<< ground_pts_ctrl[i].x << "\t" << ground_pts_ctrl[i].y << "\t" << ground_pts_ctrl[i].z << endl;
	//}
	out_dir = outpath;
	seed.clear();
	temp_gt.clear();
	temp_pt.clear();
}

RSanction::~RSanction()
{
	ground_pts_ctrl.clear();
	ground_pts_check.clear();
	pho_pts_ctrl.clear();
	pho_pts_check.clear();
	unknownpts.clear();

	distort.clear();
}

void RSanction::locTransform()
{
	int n1 = ground_pts_ctrl.size();
	int n2 = ground_pts_check.size();
	GroundPt temp;
	for (int i = 0; i < n1; i++)
	{
		temp.x = ground_pts_ctrl[i].y;
		temp.y = ground_pts_ctrl[i].z;
		temp.z = -ground_pts_ctrl[i].x;
		ground_pts_ctrl[i] = temp;
	}
	for (int i = 0; i < n2; i++)
	{
		temp.x = ground_pts_check[i].y;
		temp.y = ground_pts_check[i].z;
		temp.z = -ground_pts_check[i].x;
		ground_pts_check[i] = temp;
	}

}

void RSanction::init()
{
	
	distort.resize(5);

	ext_element.Xs = Xs0;
	ext_element.Ys = Ys0;
	ext_element.Zs = Zs0;
	ext_element.phi = Phi0;
	ext_element.kappa = Kappa0;
	ext_element.omega = Omega0;

	ext_element.R = calRotate(ext_element);

	int_element.f = f0;
	int_element.x0 = 0;
	int_element.y0 = 0;
}

void RSanction::init_from_DLT(InterElement int_element,ExterElement ext_element,vector<double>distort)
{
	this->int_element = int_element;
	this->ext_element = ext_element;
	this->distort = distort;
}

Eigen::MatrixXd RSanction::LeastSquare(Eigen::MatrixXd A, Eigen::MatrixXd L)
{
	Eigen::MatrixXd temp = A.transpose() * A;
	//for zls
	//cout << A << endl<<endl;
	//cout << L << endl<<endl;
	//cout << temp << endl<<endl;   //At A
	//cout << A.transpose() * L << endl << endl;
	double det=temp.determinant();
	if (abs(det) < 1e-4)
	{
		cout << "奇异矩阵！特征值为：" <<det<<endl;
	}
	//Eigen::MatrixXd zls = A.transpose() * L;
	return temp.inverse()* A.transpose()* L;
}

Eigen::MatrixXd RSanction::calRotate(ExterElement temp)
{
	Eigen::MatrixXd rotate;
	Eigen::MatrixXd r_phi = Eigen::MatrixXd::Zero(3, 3);
	Eigen::MatrixXd r_omega = Eigen::MatrixXd::Zero(3, 3);
	Eigen::MatrixXd r_kappa = Eigen::MatrixXd::Zero(3, 3);

	r_phi(1, 1) = 1;
	r_phi(0, 0) = cos(temp.phi);    r_phi(0, 2) = -sin(temp.phi);
	r_phi(2, 0) = sin(temp.phi);    r_phi(2, 2) = cos(temp.phi);

	r_omega(0, 0) = 1;
	r_omega(1, 1) = cos(temp.omega);    r_omega(1, 2) = -sin(temp.omega);
	r_omega(2, 1) = sin(temp.omega);    r_omega(2, 2) = cos(temp.omega);

	r_kappa(2, 2) = 1;
	r_kappa(0, 0) = cos(temp.kappa);    r_kappa(0, 1) = -sin(temp.kappa);
	r_kappa(1, 0) = sin(temp.kappa);    r_kappa(1, 1) = cos(temp.kappa);

	rotate = r_phi * r_omega * r_kappa;
	return rotate;
}

Eigen::MatrixXd RSanction::diff(PhotoPt pho_pt, GroundPt ground_pt)
{
	Eigen::MatrixXd A(2, 14);
	Eigen::MatrixXd xyz(3, 1);

	xyz(0, 0) = ground_pt.x - ext_element.Xs;
	xyz(1, 0) = ground_pt.y - ext_element.Ys;
	xyz(2, 0) = ground_pt.z - ext_element.Zs;

	xyz = ext_element.R.inverse() * xyz;
	//cout << xyz;
	A(0, 0) = 1 / xyz(2, 0) * (ext_element.R(0, 0) * int_element.f + ext_element.R(0, 2) * (pho_pt.x - int_element.x0));
	A(0, 1) = 1 / xyz(2, 0) * (ext_element.R(1, 0) * int_element.f + ext_element.R(1, 2) * (pho_pt.x - int_element.x0));
	A(0, 2) = 1 / xyz(2, 0) * (ext_element.R(2, 0) * int_element.f + ext_element.R(2, 2) * (pho_pt.x - int_element.x0));
	A(1, 0) = 1 / xyz(2, 0) * (ext_element.R(0, 1) * int_element.f + ext_element.R(0, 2) * (pho_pt.y - int_element.y0));
	A(1, 1) = 1 / xyz(2, 0) * (ext_element.R(1, 1) * int_element.f + ext_element.R(1, 2) * (pho_pt.y - int_element.y0));
	A(1, 2) = 1 / xyz(2, 0) * (ext_element.R(2, 1) * int_element.f + ext_element.R(2, 2) * (pho_pt.y - int_element.y0));

	A(0, 3) = (pho_pt.y - int_element.y0) * sin(ext_element.omega) - ((pho_pt.x - int_element.x0) / int_element.f *
		((pho_pt.x - int_element.x0) * cos(ext_element.kappa) - (pho_pt.y - int_element.y0) * sin(ext_element.kappa)) 
		+ int_element.f * cos(ext_element.kappa)) * cos(ext_element.omega);
	A(0, 4) = -int_element.f * sin(ext_element.kappa) - (pho_pt.x - int_element.x0) / int_element.f *
		((pho_pt.x - int_element.x0) * sin(ext_element.kappa) + (pho_pt.y - int_element.y0) * cos(ext_element.kappa));
	A(0, 5) = pho_pt.y - int_element.y0;

	A(1, 3) = -(pho_pt.x - int_element.x0) * sin(ext_element.omega) - ((pho_pt.y - int_element.y0) / int_element.f *
		((pho_pt.x - int_element.x0) * cos(ext_element.kappa) - (pho_pt.y - int_element.y0) * sin(ext_element.kappa)) 
		- int_element.f * sin(ext_element.kappa)) * cos(ext_element.omega);
	A(1, 4) = -int_element.f * cos(ext_element.kappa) - (pho_pt.y - int_element.y0) / int_element.f *
		((pho_pt.x - int_element.x0) * sin(ext_element.kappa) + (pho_pt.y - int_element.y0) * cos(ext_element.kappa));
	A(1, 5) = -(pho_pt.x - int_element.x0);

	A(0, 6) = (pho_pt.x - int_element.x0) / int_element.f;
	A(0, 7) = 1;
	A(0, 8) = 0;
	A(1, 6) = (pho_pt.y - int_element.y0) / int_element.f;
	A(1, 7) = 0;
	A(1, 8) = 1;

	double r = sqrt(pow(pho_pt.x - int_element.x0, 2) + pow(pho_pt.y - int_element.y0, 2));

	A(0, 9) = -(pho_pt.x - int_element.x0) * pow(r, 2);
	A(0, 10) = -(pho_pt.x - int_element.x0) * pow(r, 4);
	A(0, 11) = -(pho_pt.x - int_element.x0) * pow(r, 6);
	A(0, 12) = -(pow(r, 2) + 2 * pow(pho_pt.x - int_element.x0,2));
	A(0, 13) = -2 * (pho_pt.x - int_element.x0) * (pho_pt.y - int_element.y0);

	A(1, 9) = -(pho_pt.y - int_element.y0) * pow(r, 2);
	A(1, 10) = -(pho_pt.y - int_element.y0) * pow(r, 4);
	A(1, 11) = -(pho_pt.y - int_element.y0) * pow(r, 6);
	A(1, 12) =-2 * (pho_pt.x - int_element.x0) * (pho_pt.y - int_element.y0);
	A(1, 13) = -(pow(r, 2) + 2 * pow(pho_pt.y - int_element.y0,2));
	//cout << A<<endl;
	return A;

}

Eigen::MatrixXd RSanction::resi(PhotoPt pho_pt, GroundPt ground_pt)
{

	Eigen::MatrixXd xyz(3, 1);
	Eigen::MatrixXd L(2, 1);
	xyz(0, 0) = ground_pt.x - ext_element.Xs;
	xyz(1, 0) = ground_pt.y - ext_element.Ys;
	xyz(2, 0) = ground_pt.z - ext_element.Zs;

	xyz = ext_element.R.inverse() * xyz;

	double r = sqrt(pow(pho_pt.x - int_element.x0, 2) + pow(pho_pt.y - int_element.y0, 2));
	double dx = (pho_pt.x - int_element.x0) * (distort[0] * pow(r, 2) + distort[1] * pow(r, 4) + distort[2] * pow(r, 6))
			+ distort[3] * (pow(r, 2) + 2 * pow(pho_pt.x - int_element.x0, 2))
			+ 2 * distort[4] * (pho_pt.x - int_element.x0) * (pho_pt.y - int_element.y0);
	double dy = (pho_pt.y - int_element.y0) * (distort[0] * pow(r, 2) + distort[1] * pow(r, 4) + distort[2] * pow(r, 6))
			+ distort[4] * (pow(r, 2) + 2 * pow(pho_pt.y - int_element.y0, 2))
			+ 2 * distort[3] * (pho_pt.x - int_element.x0) * (pho_pt.y - int_element.y0);

	L(0, 0) = pho_pt.x + dx - int_element.x0 + int_element.f * xyz(0, 0) / xyz(2, 0);
	L(1, 0) = pho_pt.y + dy - int_element.y0 + int_element.f * xyz(1, 0) / xyz(2, 0);

	return L;
}

void RSanction::check(Eigen::MatrixXd A, Eigen::MatrixXd X,Eigen::MatrixXd L, int num)
{
	Eigen::MatrixXd V = A * X - L;
	double v= (V.transpose() * V)(0,0);
	double sigma = sqrt(v / (num - 7));
	cout << "观测值中误差(sigma)：" <<sigma<<endl<<endl;

	Eigen::MatrixXd Q = (A.transpose() * A).inverse(); //A.transpose() * (A * A.transpose()).inverse() * A;
	//cout << Q;

	cout << "未知数解算理论精度：" << endl;
	cout << fixed << setprecision(6)
		<< setw(16) << "Xs: " << sigma * sqrt(Q(0, 0))<<endl
		<< setw(16) << "Ys: " << sigma * sqrt(Q(1, 1)) << endl
		<< setw(16) << "Zs: " << sigma * sqrt(Q(2, 2)) << endl
		<< setw(16) << "phi: " << sigma * sqrt(Q(3, 3)) << endl
		<< setw(16) << "omega: " << sigma * sqrt(Q(4, 4)) << endl
		<< setw(16) << "kappa: " << sigma * sqrt(Q(5, 5)) << endl
		<< setw(16) << "x0: " << sigma * sqrt(Q(7, 7)) << endl
		<< setw(16) << "y0: " << sigma * sqrt(Q(8, 8)) << endl
		<< setw(16) << "f: " << sigma * sqrt(Q(6, 6)) << endl
		<< setw(16) << "k1: " << sigma * sqrt(Q(9, 9)) << endl
		<< setw(16) << "k2: " << sigma * sqrt(Q(10, 10)) << endl
		<< setw(16) << "k3: " << sigma * sqrt(Q(11, 11)) << endl
		<< setw(16) << "p1: " << sigma * sqrt(Q(12, 12)) << endl
		<< setw(16) << "p2: " << sigma * sqrt(Q(13, 13)) << endl;
	
}

void RSanction::calRSanction(int times)
{
	int n = pho_pts_ctrl.size();
	Eigen::MatrixXd A(n * 2, 14);   //14
	Eigen::MatrixXd L(n * 2, 1);

	cout <<endl<<endl<< "*************后方交会求解*************" << endl;
	
	for (int i = 0; i < times; i++)
	{

		ext_element.R = calRotate(ext_element);
		//cout << ext_element.R;
		for (int j = 0; j < n; j++)
		{
			Eigen::MatrixXd a = diff(pho_pts_ctrl[j], ground_pts_ctrl[j]);
			Eigen::MatrixXd l = resi(pho_pts_ctrl[j], ground_pts_ctrl[j]);
			
			A.block<2, 14>(j * 2, 0) = a;
			L.block<2, 1>(j * 2, 0) = l;
		}

		Eigen::MatrixXd delta = LeastSquare(A, L);
		//cout << delta << endl << endl;

		ext_element.Xs += delta(0, 0);
		ext_element.Ys += delta(1, 0);
		ext_element.Zs += delta(2, 0);
		ext_element.phi += delta(3, 0);
		ext_element.omega += delta(4, 0);
		ext_element.kappa += delta(5, 0);
		
		int_element.f += delta(6, 0);
		int_element.x0 += delta(7, 0);
		int_element.y0 += delta(8, 0);

		distort[0] += delta(9, 0);
		distort[1] += delta(10, 0);
		distort[2] += delta(11, 0);
		distort[3] += delta(12, 0);
		distort[4] += delta(13, 0);
		
		if (abs(delta(12, 0)) < 3e-6 && abs(delta(13, 0)) < 3e-6
			&&abs(delta(9, 0)) < 3e-6 && abs(delta(10, 0)) < 3e-6 && abs(delta(11, 0)) < 3e-6 &&
			abs(delta(6, 0)) < 3e-6 && abs(delta(7, 0)) < 3e-6 && abs(delta(8, 0)) < 3e-6 &&
			abs(delta(3, 0)) < 3e-6 && abs(delta(4, 0)) < 3e-6 && abs(delta(5, 0)) < 3e-6
			&& abs(delta(0, 0)) < 3e-6 && abs(delta(1, 0)) < 3e-6 && abs(delta(2, 0)) < 3e-6)
		{
			cout <<"\t后方交会收敛!" << " " << "迭代次数:" << i << endl << endl;
			
			//结果输出
			ofstream f(out_dir);

			Eigen::MatrixXd X = A * delta - L;

			for (int ii = 0; ii < pho_pts_ctrl.size(); ii++)
			{
				f << pho_pts_ctrl[ii].num << " " << pho_pts_ctrl[ii].x << " " <<
					pho_pts_ctrl[ii].y << " " << X(2 * ii, 0) << " " << X(2 * ii + 1, 0) << endl;
			}
			f.close();

			
			check(A, delta, L, n);
			break;
		}

	}

}

void RSanction::Display()
{
	cout << endl;
	cout << "--------外方位元素--------" << endl;

	cout << setw(10) << "Xs" << setw(10) << "Ys" << setw(10) << "Zs" << endl;
	cout << fixed << setprecision(6)
		<< setw(12) << ext_element.Xs << " "
		<< setw(12) << ext_element.Ys << " "
		<< setw(12) << ext_element.Zs << endl << endl;

	cout << setw(10) << "phi" << setw(10) << "omega" << setw(10) << "kappa" << endl;
	cout << fixed << setprecision(6)
		<< setw(12) << ext_element.phi << " "
		<< setw(12) << ext_element.omega << " "
		<< setw(12) << ext_element.kappa << endl << endl;

	cout << "--------内方位元素-------" << endl;
	cout << setw(10) << "x0" << setw(10) << "y0" << setw(10) << "f" << endl;
	cout << fixed << setprecision(6)
		<< setw(12) << int_element.x0 << " "
		<< setw(12) << int_element.y0 << " "
		<< setw(12) << int_element.f << endl << endl;

	cout << "--------畸变参数(k1,k2,k3,p1,p2)--------" << endl;
	cout << fixed << setprecision(12)
		<< setw(8) << distort[0] << " " << distort[1] << " " << distort[2] << " "
		<< distort[3] << " " << distort[4] << endl;
}