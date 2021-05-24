#include"Gram.h"

Bundle::Bundle(char* path_int, char* path_gcp, char* path_init_ext, char* path_init_pt, int count)
	:PhotoGram(path_int, path_gcp)
{
	pts.resize(count);
	setExt(path_init_ext);
	setPoints(path_init_pt);
}

double Bundle::angleTrans(double a)
{
	int d = (int)a;
	double m = (int)((a - d) * 100);
	double s = ((a - d) * 100 - m) * 100;

	double res = (d + m / 60 + s / 3600) / 180 * PI;

	return res;
}

double Bundle::invTrans(double len)
{
	len = len * 180 / PI;
	double d = (int)len;
	double m = (int)((len - d) * 60);
	double s = (len - d - m / 60) * 3600;

	return d + m / 100 + s / 10000;
}

void Bundle::setPt(char* path, int photo_num, int time)
{
	FILE* fp;
	errno_t err = fopen_s(&fp, path, "r");

	fscanf_s(fp, "%*[^\n]");     //单独读取第一行

	int up_num;
	fscanf_s(fp, "%d", &up_num);

	UnknowPT temp;
	vector<resPoint> check = getGCP();

	temp.photo_num = photo_num;
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
			pts[time].push_back(temp);
			temp.type = UNKNOWN;
			pt_num++;
		}

	}
	fclose(fp);
}

void Bundle::setPoints(char* path)
{
	FILE* fp;
	errno_t err = fopen_s(&fp, path, "r");

	int p_num;
	fscanf_s(fp, "%d", &p_num);

	resPoint temp;
	temp.type = UNKNOWN;

	vector<resPoint> check = getGCP();

	for (int i = 0; i < p_num; i++)
	{
		fscanf_s(fp, "%d %lf %lf %lf",
			&temp.num, &temp.X, &temp.Y, &temp.Z);
		for (int j = 0; j < check.size(); j++)
		{
			if (check[j].num == temp.num)
			{
				temp.type = CTRL;
				break;
			}
		}

		init_pt.push_back(temp);
		temp.type = UNKNOWN;
	}
	fclose(fp);
}

void Bundle::setExt(char* path)
{

	FILE* fp;
	errno_t err = fopen_s(&fp, path, "r");

	int p_num;
	fscanf_s(fp, "%d", &p_num);

	ExterElement temp;

	for (int i = 0; i < p_num; i++)
	{
		fscanf_s(fp, "%d %lf %lf %lf %lf %lf %lf",
			&temp.photo_num, &temp.phi, &temp.omega, &temp.kappa, &temp.Xs, &temp.Ys, &temp.Zs);

		temp.phi = angleTrans(temp.phi);
		temp.omega = angleTrans(temp.omega);
		temp.kappa = angleTrans(temp.kappa);

		element.push_back(temp);
	}
	fclose(fp);
}

void Bundle::init()
{
	//Search 遍历地面点
	for (int i = 0; i < pts.size(); i++)
	{
		for (int j = 0; j < pts[i].size(); j++)
		{
			int flag = 0;
			for (int k = 0; k < point.size(); k++)
			{
				if (pts[i][j].num == point[k])

				{
					flag = 1;
					break;
				}
			}
			if (flag == 0)
			{
				point.push_back(pts[i][j].num);
			}

		}
	}

	for (int i = 0; i < point.size(); i++)
	{
		for (int j = 0; j < init_pt.size(); j++)
		{
			if (point[i] == init_pt[j].num)
			{
				res.push_back(init_pt[j]);
			}
		}
	}

}

void Bundle::calculate(int times)
{
	init();

	cout <<endl<<endl<< "--------------" << "光束法平差" << "--------------" << endl;


	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(pt_num * 2, point.size() * 3 + pts.size() * 6);
	Eigen::MatrixXd L(pt_num * 2, 1);
	Eigen::MatrixXd a(2, 9);
	Eigen::MatrixXd l(2, 1);

	Point temp_pt;

	for (int t = 0; t < times; t++)
	{
		int pt_count = 0;
		for (int i = 0; i < point.size(); i++)
		{
			vector<mypair> my = search(point[i]);  //地面点对应的像片号和点号

			for (int j = 0; j < my.size(); j++)
			{
				temp_pt.type = pts[my[j].left][my[j].right].type;
				temp_pt.p_x = pts[my[j].left][my[j].right].pho_x;
				temp_pt.p_y = pts[my[j].left][my[j].right].pho_y;
				temp_pt.g_x = res[i].X;
				temp_pt.g_y = res[i].Y;
				temp_pt.g_z = res[i].Z;

				a = diff(temp_pt, element[my[j].left]);   // dx dy dz dxs dys dzs dphi domega dkappa 
				l = resi(temp_pt, element[my[j].left]);

				A.block(2 * pt_count, i * 3, 2, 3) = a.block<2, 3>(0, 0);
				A.block(2 * pt_count, point.size() * 3 + my[j].left * 6, 2, 6) = a.block<2, 6>(0, 3);
				L.block(2 * pt_count, 0, 2, 1) = l;

				pt_count++;

			}

		}
		Eigen::MatrixXd delta = adj(A, L);
		//cout << L;
		int ii = 0;
		for (; ii < point.size(); ii++)
		{
			res[ii].X += delta(3 * ii, 0);
			res[ii].Y += delta(3 * ii + 1, 0);
			res[ii].Z += delta(3 * ii + 2, 0);
		}

		for (int j = 0; j < element.size(); j++)    // 22*3+6*6=102  
		{
			element[j].Xs += delta(3 * ii + 6 * j, 0);
			element[j].Ys += delta(3 * ii + 6 * j + 1, 0);
			element[j].Zs += delta(3 * ii + 6 * j + 2, 0);
			element[j].phi += delta(3 * ii + 6 * j + 3, 0);
			element[j].omega += delta(3 * ii + 6 * j + 4, 0);
			element[j].kappa += delta(3 * ii + 6 * j + 5, 0);
		}

		int flag = 0;
		for (int count = 0; count < delta.size(); count++)
		{
			if (delta(count) > 3e-5)
			{
				flag = 1;
				break;
			}
		}
		if (flag == 0)
		{
			cout << endl << "光束法平差收敛!" << "      " << "迭代次数:" << t << endl << endl;
			break;
		}
	}
}

void Bundle::show(char* path1, char* path2)// WuCAPS工程格式输出
{
	ofstream OutExt(path1);
	OutExt << element.size() << endl;

	cout << "外方位元素求解结果:" << endl;
	cout << "像片号 " << "λ φ ω κ dx dy dz" << endl;

	for (int i = 0; i < element.size(); i++)
	{
		cout << fixed << setprecision(6)
			<< setw(8) << element[i].photo_num << "  "
			<< setw(11) << invTrans(element[i].phi) << "  " << setw(11) << invTrans(element[i].omega) << "  "
			<< setw(11) << invTrans(element[i].kappa) << "  " << setw(11) << element[i].Xs << "  "
			<< setw(11) << element[i].Ys << "  " << setw(11) << element[i].Zs
			<< endl;

		OutExt << fixed << setprecision(6)
			<< setw(8) << element[i].photo_num << "  "
			<< setw(11) << invTrans(element[i].phi) << "  " << setw(11) << invTrans(element[i].omega) << "  "
			<< setw(11) << invTrans(element[i].kappa) << "  " << setw(11) << element[i].Xs << "  "
			<< setw(11) << element[i].Ys << "  " << setw(11) << element[i].Zs
			<< endl;
	}
	OutExt.close();

	ofstream OutPts(path2);
	OutPts << res.size() << endl;

	cout << endl << endl << "待求点物空间坐标：" << endl;

	for (int i = 0; i < res.size(); i++)
	{
		cout << fixed << setprecision(6)
			<< setw(8) << res[i].num << "  "
			<< setw(11) << invTrans(res[i].X) << "  " << setw(11) << invTrans(res[i].Y) << "  "
			<< setw(11) << invTrans(res[i].Z) << endl;

		OutPts << fixed << setprecision(6)
			<< setw(8) << res[i].num << "  "
			<< setw(11) << invTrans(res[i].X) << "  " << setw(11) << invTrans(res[i].Y) << "  "
			<< setw(11) << invTrans(res[i].Z) << endl;
	}
	OutPts.close();

	cout << endl << endl << "输出文件路径" << endl;
	cout << "外方位元素：" << path1 << endl;
	cout << "点坐标：" << path2 << endl;
}

vector<mypair> Bundle::search(int num)
{
	vector<mypair> match;
	mypair temp;   //  photo_num ,point_num;

	for (int i = 0; i < pts.size(); i++)
	{
		for (int j = 0; j < pts[i].size(); j++)
		{
			if (pts[i][j].num == num)
			{
				temp.left = i;
				temp.right = j;
				match.push_back(temp);
			}
		}
	}
	return match;
}

Eigen::MatrixXd Bundle::adj(Eigen::MatrixXd A, Eigen::MatrixXd L)
{
	Eigen::SparseMatrix<double> param;
	Eigen::SparseMatrix<double> temp;

	Eigen::MatrixXd ans;

	param = A.sparseView();
	param.makeCompressed();
	
	temp = param.transpose() * param;

	//ans = A.transpose() * A;

	//ofstream out("../../a.txt");
	//for (int i = 0; i < A.rows(); i++)
	//{
	//	for (int j = 0; j < A.cols(); j++)
	//	{
	//		out << A(i,j) << " ";
	//	}
	//	out << endl;
	//}

	//out.close();

	Eigen::SparseQR<Eigen::SparseMatrix<double>,Eigen::AMDOrdering<int>> solver;
	
	solver.compute(temp);

	ans=solver.solve(param.transpose() * L);  
	//cout << ans;
	return ans;
}

Eigen::MatrixXd Bundle::diff(Point pt, ExterElement temp)
{
	Eigen::MatrixXd a(2, 9);

	InterElement element = getInt();

	Eigen::MatrixXd rotate = calRotate(temp);

	Eigen::MatrixXd xyz(3, 1);

	xyz(0, 0) = pt.g_x - temp.Xs;
	xyz(1, 0) = pt.g_y - temp.Ys;
	xyz(2, 0) = pt.g_z - temp.Zs;

	xyz = rotate.inverse() * xyz;

	a(0, 0) = -1 / xyz(2, 0) * (rotate(0, 0) * element.f + rotate(0, 2) * (pt.p_x - element.x));
	a(0, 1) = -1 / xyz(2, 0) * (rotate(0, 0) * element.f + rotate(0, 2) * (pt.p_x - element.x));
	a(0, 2) = -1 / xyz(2, 0) * (rotate(2, 0) * element.f + rotate(2, 2) * (pt.p_x - element.x));
	a(1, 0) = -1 / xyz(2, 0) * (rotate(0, 1) * element.f + rotate(0, 2) * (pt.p_y - element.y));
	a(1, 1) = -1 / xyz(2, 0) * (rotate(1, 1) * element.f + rotate(1, 2) * (pt.p_y - element.y));
	a(1, 2) = -1 / xyz(2, 0) * (rotate(2, 1) * element.f + rotate(2, 2) * (pt.p_y - element.y));

	a(0, 3) = 1 / xyz(2, 0) * (rotate(0, 0) * element.f + rotate(0, 2) * (pt.p_x - element.x));
	a(0, 4) = 1 / xyz(2, 0) * (rotate(1, 0) * element.f + rotate(1, 2) * (pt.p_x - element.x));
	a(0, 5) = 1 / xyz(2, 0) * (rotate(2, 0) * element.f + rotate(2, 2) * (pt.p_x - element.x));
	a(1, 3) = 1 / xyz(2, 0) * (rotate(0, 1) * element.f + rotate(0, 2) * (pt.p_y - element.y));
	a(1, 4) = 1 / xyz(2, 0) * (rotate(1, 1) * element.f + rotate(1, 2) * (pt.p_y - element.y));
	a(1, 5) = 1 / xyz(2, 0) * (rotate(2, 1) * element.f + rotate(2, 2) * (pt.p_y - element.y));

	a(0, 6) = (pt.p_y - element.y) * sin(temp.omega) - ((pt.p_x - element.x) / element.f *
		((pt.p_x - element.x) * cos(temp.kappa) - (pt.p_y - element.y) * sin(temp.kappa)) + element.f * cos(temp.kappa)) * cos(temp.omega);
	a(0, 7) = -element.f * sin(temp.kappa) - (pt.p_x - element.x) / element.f *
		((pt.p_x - element.x) * sin(temp.kappa) + (pt.p_y - element.y) * cos(temp.kappa));
	a(0, 8) = pt.p_y - element.y;

	a(1, 6) = -(pt.p_x - element.x) * sin(temp.omega) - ((pt.p_y - element.y) / element.f *
		((pt.p_x - element.x) * cos(temp.kappa) - (pt.p_y - element.y) * sin(temp.kappa)) - element.f * sin(temp.kappa)) * cos(temp.omega);
	a(1, 7) = -element.f * cos(temp.kappa) - (pt.p_y - element.y) / element.f *
		((pt.p_x - element.x) * sin(temp.kappa) + (pt.p_y - element.y) * cos(temp.kappa));
	a(1, 8) = -(pt.p_x - element.x);

	return a;
}

Eigen::MatrixXd Bundle::resi(Point pt, ExterElement temp)
{
	InterElement element = getInt();

	Eigen::MatrixXd rotate = calRotate(temp);

	Eigen::MatrixXd xyz(3, 1);
	Eigen::MatrixXd L(2, 1);
	xyz(0, 0) = pt.g_x - temp.Xs;
	xyz(1, 0) = pt.g_y - temp.Ys;
	xyz(2, 0) = pt.g_z - temp.Zs;

	xyz = rotate.inverse() * xyz;

	L(0, 0) = pt.p_x - element.x + element.f * xyz(0, 0) / xyz(2, 0);
	L(1, 0) = pt.p_y - element.y + element.f * xyz(1, 0) / xyz(2, 0);

	return L;
}

