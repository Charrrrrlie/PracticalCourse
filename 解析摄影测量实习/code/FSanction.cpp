#include"Gram.h"

FSanction::FSanction(char *path_int, char *path_rgcp,char*path_pgcp, char *path_ext, int count, int type)
	     :RSanction(path_int, path_rgcp,path_pgcp)
{
	photo_count = count;
	unknown_pt.resize(count);
	this->type = type;
	setExt(path_ext);
}

void FSanction::setUP(char *path,int photo_num,int time)
{
	FILE *fp;
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
		fscanf_s(fp, "%d %lf %lf %lf %lf %d", &temp.num, &temp.pxl_x, &temp.pxl_y ,&temp.pho_x, &temp.pho_y, &valid);
		
		
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
			unknown_pt[time].push_back(temp);
			temp.type = UNKNOWN;
		}
		
	}
	fclose(fp);
}

double FSanction::angleTrans(double a)
{
	int d = (int)a;
	double m = (int)((a - d)*100);
	double s = ((a - d) * 100 - m)*100;

	double res = (d + m / 60 + s / 3600)/180*PI;
	
	return res;
}

void FSanction::setExt(char *path)
{
	if (this->type == 0)   //从后方交会得到
	{

	}
	else if (this->type == 1) //从文件读取
	{
		FILE *fp;
		errno_t err = fopen_s(&fp, path, "r");

		int p_num;
		fscanf_s(fp, "%d", &p_num);

		ExterElement temp;

		for (int i = 0; i < p_num; i++)
		{
			fscanf_s(fp, "%d %lf %lf %lf %lf %lf %lf",
				      &temp.photo_num,&temp.phi,&temp.omega,&temp.kappa,&temp.Xs,&temp.Ys,&temp.Zs);

			temp.phi=angleTrans(temp.phi);
			temp.omega = angleTrans(temp.omega);
			temp.kappa = angleTrans(temp.kappa);

			ext_element.push_back(temp);
		}
		fclose(fp);
	}
	
}

void FSanction::calFSanction(int point_num,int times)
{
	cout << "--------------" << "前方交会" << "--------------" << endl;
	cout << "同名点点号：" << point_num<<endl;
	//初值
	point_prj(point_num);

	res = init;
	
	vector<int> pho_num;
	vector<int> poi_num;

	//找某一点在多少像片上出现过
	for (int i = 0; i < unknown_pt.size(); i++)
	{
		for (int j = 0; j < unknown_pt[i].size(); j++)
		{
			if (unknown_pt[i][j].num == point_num)
			{
				pho_num.push_back(i);
				poi_num.push_back(j);
			}
		}
	}

	cout << "包含像片：";
	for (int i = 0; i < pho_num.size(); i++)
	{
		cout<< unknown_pt[pho_num[i]][0].photo_num << " ";
	}
	cout << endl;


	Eigen::MatrixXd A(pho_num.size() * 2, 3);
	Eigen::MatrixXd L(pho_num.size() * 2, 1);


	for (int t = 0; t < times; t++)
	{
		for (int i = 0; i < pho_num.size(); i++)
		{
		
			//格式化
			Point temp_pt;
			temp_pt.g_x = res.X;
			temp_pt.g_y = res.Y; 
			temp_pt.g_z = res.Z;
			temp_pt.p_x=  unknown_pt[pho_num[i]][poi_num[i]].pho_x;
			temp_pt.p_y = unknown_pt[pho_num[i]][poi_num[i]].pho_y;
			
			rotate = calRotate(ext_element[pho_num[i]]);

			Eigen::MatrixXd a = diff(temp_pt, ext_element[pho_num[i]]);
			Eigen::MatrixXd l = resi(temp_pt, ext_element[pho_num[i]]);

			A.block<2, 3>(i * 2, 0) = a;
			L.block<2, 1>(i * 2, 0) = l;
		}

		Eigen::MatrixXd delta = adj(A, L);
		
		res.X += delta(0, 0);
		res.Y += delta(1, 0);
		res.Z += delta(2, 0);

		if (abs(delta(0, 0)) < 3e-5 && abs(delta(1, 0)) < 3e-5 && abs(delta(2, 0)) < 3e-5)
		{
			cout << endl<< "前方交会收敛!" << "      " << "迭代次数:" << t << endl;
			break;
		}

	}
	res.type = unknown_pt[pho_num[0]][poi_num[0]].type;
	cout << endl<< "物方空间坐标：" << endl;
	cout << res.X << "   " << res.Y << "   " << res.Z << endl<<endl;	
}

void FSanction::point_prj(int point_num)   
{
	int times = 0;
	int l_photo,r_photo;
	int l_point, r_point;
	for (int i = 0; i < unknown_pt.size(); i++)
	{
		for (int j = 0; j < unknown_pt[i].size(); j++)
		{
			if (unknown_pt[i][j].num == point_num)
			{
				times++;
				if (times == 1)
				{
					l_photo = i;
					l_point = j;
				}
				else if (times == 2)
				{
					r_photo = i;
					r_point = j;
					break;
				}
			}
		}
	}
	Eigen::MatrixXd left_rot = calRotate(ext_element[l_photo]);
	Eigen::MatrixXd right_rot= calRotate(ext_element[r_photo]);

	Eigen::MatrixXd l(3, 1);    //左片 XYZ
	Eigen::MatrixXd r(3, 1);    //右片 XYZ

	l(0, 0) = unknown_pt[l_photo][l_point].pho_x;
	l(1, 0) = unknown_pt[l_photo][l_point].pho_y;
	l(2, 0) = -getInt().f;

	r(0, 0) = unknown_pt[r_photo][r_point].pho_x;
	r(1, 0) = unknown_pt[r_photo][r_point].pho_y;
	r(2, 0) = -getInt().f;

	l = left_rot * l;
	r = right_rot * r;

	double  Bx = ext_element[r_photo].Xs-ext_element[l_photo].Xs;
	double  By = ext_element[r_photo].Ys-ext_element[l_photo].Ys;
	double  Bz = ext_element[r_photo].Zs-ext_element[l_photo].Zs;

	double N1 = (Bx*r(2, 0) - Bz * r(0, 0)) / (l(0, 0)*r(2, 0) - l(2, 0)*r(0, 0));
	double N2 = (Bx*l(2, 0) - Bz * l(0, 0)) / (l(0, 0)*r(2, 0) - l(2, 0)*r(0, 0));

	init.num = unknown_pt[l_photo][l_point].num;
	init.X = ext_element[l_photo].Xs + N1 * l(0, 0);
	init.Y = ext_element[l_photo].Ys + 0.5*(N1 * l(1, 0) + N2 * r(1, 0) + By);
	init.Z = ext_element[l_photo].Zs + N1 * l(2, 0);

	//cout << endl<<init.X << " " << init.Y << " "<<init.Z << endl;
	
}

Eigen::MatrixXd FSanction::diff(Point pt, ExterElement temp)
{
	InterElement element = getInt();
	
	Eigen::MatrixXd A(2, 3);
	Eigen::MatrixXd xyz(3, 1);

	xyz(0, 0) = pt.g_x - temp.Xs;
	xyz(1, 0) = pt.g_y - temp.Ys;
	xyz(2, 0) = pt.g_z - temp.Zs;

	xyz = rotate.inverse()*xyz;

	A(0, 0) = -1 / xyz(2, 0)*(rotate(0, 0)*element.f + rotate(0, 2)*(pt.p_x - element.x));
	A(0, 1) = -1 / xyz(2, 0)*(rotate(1, 0)*element.f + rotate(1, 2)*(pt.p_x - element.x));
	A(0, 2) = -1 / xyz(2, 0)*(rotate(2, 0)*element.f + rotate(2, 2)*(pt.p_x - element.x));
	A(1, 0) = -1 / xyz(2, 0)*(rotate(0, 1)*element.f + rotate(0, 2)*(pt.p_y - element.y));
	A(1, 1) = -1 / xyz(2, 0)*(rotate(1, 1)*element.f + rotate(1, 2)*(pt.p_y - element.y));
	A(1, 2) = -1 / xyz(2, 0)*(rotate(2, 1)*element.f + rotate(2, 2)*(pt.p_y - element.y));

	return A;
}

Eigen::MatrixXd FSanction::resi(Point pt, ExterElement temp)
{
	InterElement element = getInt();

	Eigen::MatrixXd xyz(3, 1);
	Eigen::MatrixXd L(2, 1);
	xyz(0, 0) = pt.g_x - temp.Xs;
	xyz(1, 0) = pt.g_y - temp.Ys;
	xyz(2, 0) = pt.g_z - temp.Zs;

	xyz = rotate.inverse()*xyz;

	L(0, 0) = pt.p_x - element.x + element.f* xyz(0, 0) / xyz(2, 0);
	L(1, 0) = pt.p_y - element.y + element.f* xyz(1, 0) / xyz(2, 0);

	return L;
}