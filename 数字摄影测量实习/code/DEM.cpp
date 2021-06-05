#include"DEM.h"

#define START_X_IMAGE 1365.2400000000
#define START_Y_IMAGE 297.01800000000
#define DX_IMAGE 1
#define DY_IMAGE 1

using namespace std;
using namespace cv;

DEM::DEM(string path, string image_path)
{
	fstream f(path);

	f >> X0 >> Y0 >> A >> dx >> dy >> col >> row;

	max_z = 0;
	min_z = INT_MAX;

	for (int i = 0; i < row; i++)
	{
		vector<double> temp;
		for (int j = 0; j < col; j++)
		{
			double temp_z;
			f >> temp_z;
			if (temp_z < -9999)
			{
				temp.push_back(temp_z);
				invalid_val.push_back(make_pair(i, j));
				continue;
			}
			max_z = max(temp_z, max_z);
			min_z = min(temp_z, min_z);
			temp.push_back(temp_z + 1e-5);
		}
		height_mat.push_back(temp);
	}
	f.close();

	max_z = int(max_z);
	min_z = int(min_z) + 1;

	Mat image = imread(image_path);

	image_col = image.cols;
	image_row = image.rows;

	//处理无效值 方便后续处理
	height_mat = GetNearestHeight();

}

DEM::~DEM()
{
	height_mat.clear();
}

vector<vector<double>> DEM::GetNearestHeight() {

	vector<vector<double>> mat = height_mat;

	for (int i = 0; i < invalid_val.size(); i++)
	{
		int temp_row = invalid_val[i].first;
		int temp_col = invalid_val[i].second;
		int search_row = temp_row;
		int search_col = temp_col;   //搜索开始位置

		//在图像左上角部分
		if (temp_row <= row / 2 && temp_col <= col / 2)
		{
			while (search_col < col)  //沿行搜索
			{
				if (mat[search_row][search_col] > -9999)
				{
					mat[temp_row][temp_col] = mat[search_row][search_col];
					break;
				}
				search_col++;
			}
			if (search_col != col) continue;
			search_col = temp_col;
			while (search_row < row)
			{
				if (mat[search_row][search_col] > -9999)
				{
					mat[temp_row][temp_col] = mat[search_row][search_col];
					break;
				}
				search_row++;
			}

		}
		//图像右上角部分
		else if (temp_row <= row / 2 && temp_col > col / 2)
		{
			while (search_col >= 0)
			{
				if (mat[search_row][search_col] > -9999)
				{
					mat[temp_row][temp_col] = mat[search_row][search_col];
					break;
				}
				search_col--;
			}
			if (search_col != col) continue;
			search_col = temp_col;
			while (search_row < row)
			{
				if (mat[search_row][search_col] > -9999)
				{
					mat[temp_row][temp_col] = mat[search_row][search_col];
					break;
				}
				search_row++;
			}
		}

		//图像左下角部分
		else if (temp_row > row / 2 && temp_col <= col / 2)
		{
			while (search_col < col)  //沿行搜索
			{
				if (mat[search_row][search_col] > -9999)
				{
					mat[temp_row][temp_col] = mat[search_row][search_col];
					break;
				}
				search_col++;
			}
			if (search_col != col) continue;
			search_col = temp_col;
			while (search_row >= 0)
			{
				if (mat[search_row][search_col] > -9999)
				{
					mat[temp_row][temp_col] = mat[search_row][search_col];
					break;
				}
				search_row--;
			}
		}

		//图像右下角部分
		else if (temp_row > row / 2 && temp_col > col / 2)
		{
			while (search_col >= 0)
			{
				if (mat[search_row][search_col] > -9999)
				{
					mat[temp_row][temp_col] = mat[search_row][search_col];
					break;
				}
				search_col--;
			}
			if (search_col != col) continue;
			search_col = temp_col;
			while (search_row < row)
			{
				if (mat[search_row][search_col] > -9999)
				{
					mat[temp_row][temp_col] = mat[search_row][search_col];
					break;
				}
				search_row++;
			}
		}
	}

	return mat;
}

vector<vector<double>> DEM::Interp()
{
	int step_x = dx / DX_IMAGE;
	int step_y = dy / DY_IMAGE;

	vector<vector<double>> res(image_row, vector<double>(image_col));

	for (int i = 0; i < image_row; i++)
	{
		//cout << i << endl;
		for (int j = 0; j < image_col; j++)
		{
			int temp_col = (j / step_x);
			int temp_row = (i / step_y);

			double dxx, dyy;

			dxx = (j - temp_col * step_x) / step_x;
			dyy = (i - temp_row * step_y) / step_y;

			double val;
			//cout << "j" << j << endl;
			//双线性
			if (temp_col == col - 1 && temp_row == row - 1)
			{
				val = height_mat[temp_row][temp_col];
			}
			else if (temp_col == col - 1 && temp_row < row - 1)
			{
				val = height_mat[temp_row][temp_col] * dyy + height_mat[temp_row + 1][temp_col] * (1 - dyy);
			}

			else if (temp_row == row - 1 && temp_col < col - 1)
			{
				val = height_mat[temp_row][temp_col] * dxx + height_mat[temp_row][temp_col + 1] * (1 - dxx);
			}
			else
			{
				val = height_mat[temp_row][temp_col] * dxx * dyy + dyy * (1 - dxx) * height_mat[temp_row][temp_col + 1]
					+ height_mat[temp_row + 1][temp_col] * dxx * (1 - dyy)
					+ height_mat[temp_row + 1][temp_col + 1] * (1 - dxx) * (1 - dyy);
			}

			res[i][j] = val;
		}
	}

	dense_height_mat = res;
	return res;
}

vector<vector<double>> DEM::StateMatHk(int t) {
	//求最大最小高程

	double z_min, z_max;
	z_min = min_z;
	z_max = max_z;
	int k = (z_max - z_min) / delz;
	cout << z_max << "   " << z_min << endl;
	std::vector<std::vector<double>> Hk;
	double zk = z_min + t * delz;
	for (int i = 0; i < row - 1; i++)//Hk列向相减，row少1
	{
		vector<double> temp;
		for (int j = 0; j < col; j++)
		{
			double temph;
			if (height_mat[i][j] > 0 && height_mat[i + 1][j] > 0 && (height_mat[i][j] - zk) * (height_mat[i + 1][j] - zk) < 0)
				temph = 1;
			else temph = 0;
			temp.push_back(temph);
		}
		Hk.push_back(temp);
	}
	cout << "第" << t << "条等高线的" << "Hk矩阵计算完成" << endl;
	return Hk;
}

vector<vector<double>> DEM::StateMatVk(int t) {
	//求最大最小高程

	double z_min, z_max;
	z_min = min_z;
	z_max = max_z;
	int k = (z_max - z_min) / delz;
	std::vector<std::vector<double>> Vk;
	//设置等高线间隔（最高与最低相差440米）
	double zk = z_min + t * delz;
	for (int i = 0; i < row; i++)
	{
		vector<double> temp;
		for (int j = 0; j < col - 1; j++)//Vk行向相减，col少1
		{
			double temph;
			if (height_mat[i][j] > 0 && height_mat[i][j + 1] > 0 && (height_mat[i][j] - zk) * (height_mat[i][j + 1] - zk) < 0)
				temph = 1;
			else temph = 0;
			temp.push_back(temph);
		}
		Vk.push_back(temp);
	}
	cout << "第" << t << "条等高线的" << "Vk矩阵计算完成" << endl;
	return Vk;
}

void DEM::GetlinePt(vector<vector<double>>& Hk, vector<vector<double>>& Vk, double Z, int ni, int nj, CLine& line)
{
	PtLoc pt;
	pt = line.pt[0];
	int count = 0;
	while (count != -1)
	{//顺时针判断
		if (pt.in == 4)
		{
			if (nj == col - 1 || Hk[ni][nj + 1] + Vk[ni][nj] + Vk[ni + 1][nj] == 0)
				break;
			if (Hk[ni][nj + 1] + Vk[ni][nj] + Vk[ni + 1][nj] == 3)
			{
				double zmean = (height_mat[ni][nj] + height_mat[ni][nj + 1] + height_mat[ni + 1][nj] + height_mat[ni + 1][nj + 1]) / 4.0;
				if ((height_mat[ni][nj] - Z) * (zmean - Z) < 0)
				{
					Vk[ni][nj] = 0;
					pt.in = 1;
					pt.X = X0 + nj * dx + (Z - height_mat[ni][nj]) / (height_mat[ni][nj + 1] - height_mat[ni][nj]) * dx;
					pt.Y = Y0 + ni * dy;
					pt.Z = Z;
					pt.HK = 1;
					pt.row = ni;
					pt.col = nj;
					line.pt.push_back(pt);
				}
				else
				{
					Vk[ni + 1][nj] = 0;
					pt.in = 3;
					pt.X = X0 + nj * dx + (Z - height_mat[ni + 1][nj]) / (height_mat[ni + 1][nj + 1] - height_mat[ni + 1][nj]) * dx;
					pt.Y = Y0 + (ni + 1) * dy;
					pt.Z = Z;
					pt.HK = 1;
					pt.row = ni + 1;
					pt.col = nj;
					line.pt.push_back(pt);
					ni += 1;
				}
			}
			else
			{
				if (Vk[ni][nj] == 1)
				{
					pt.in = 1;
					pt.X = X0 + nj * dx + (Z - height_mat[ni][nj]) / (height_mat[ni][nj + 1] - height_mat[ni][nj]) * dx;
					pt.Y = Y0 + ni * dy;
					pt.Z = Z;
					pt.HK = 1;
					pt.row = ni;
					pt.col = nj;
					line.pt.push_back(pt);
					Vk[ni][nj] = 0;

				}
				else if (Hk[ni][nj + 1] == 1)
				{
					pt.in = 4;
					pt.X = X0 + (nj + 1) * dx;
					pt.Y = Y0 + ni * dy + dy * (Z - height_mat[ni][nj + 1]) / (height_mat[ni + 1][nj + 1] - height_mat[ni][nj + 1]);
					pt.Z = Z;
					pt.HK = 0;
					pt.row = ni;
					pt.col = nj + 1;
					line.pt.push_back(pt);
					Hk[ni][nj + 1] = 0;
					nj += 1;
				}
				else if (Vk[ni + 1][nj] == 1)
				{
					pt.in = 3;
					pt.X = X0 + nj * dx + (Z - height_mat[ni + 1][nj]) / (height_mat[ni + 1][nj + 1] - height_mat[ni + 1][nj]) * dx;
					pt.Y = Y0 + (ni + 1) * dy;
					pt.Z = Z;
					pt.HK = 1;
					pt.row = ni + 1;
					pt.col = nj;
					line.pt.push_back(pt);
					Vk[ni + 1][nj] = 0;
					ni += 1;
				}

			}
		}
		else if (pt.in == 3)
		{
			if (ni == row - 1 || Hk[ni][nj] + Hk[ni][nj + 1] + Vk[ni + 1][nj] == 0)
				break;
			if (Hk[ni][nj] + Hk[ni][nj + 1] + Vk[ni + 1][nj] == 3)
			{
				double zmean = (height_mat[ni][nj] + height_mat[ni][nj + 1] + height_mat[ni + 1][nj] + height_mat[ni + 1][nj + 1]) / 4.0;
				if ((height_mat[ni][nj + 1] - Z) * (zmean - Z) < 0)
				{
					pt.in = 4;
					pt.X = X0 + (nj + 1) * dx;
					pt.Y = Y0 + ni * dy + dy * (Z - height_mat[ni][nj + 1]) / (height_mat[ni + 1][nj + 1] - height_mat[ni][nj + 1]);
					pt.Z = Z;
					pt.HK = 0;
					pt.row = ni;
					pt.col = nj + 1;
					line.pt.push_back(pt);
					Hk[ni][nj + 1] = 0;
					nj += 1;
				}
				else
				{
					pt.in = 2;
					pt.X = X0 + (nj)*dx;
					pt.Y = Y0 + ni * dy + dy * (Z - height_mat[ni][nj]) / (height_mat[ni + 1][nj] - height_mat[ni][nj]);
					pt.Z = Z;
					pt.HK = 0;
					pt.row = ni;
					pt.col = nj;
					line.pt.push_back(pt);
					Hk[ni][nj] = 0;
				}
			}
			else
			{
				if (Hk[ni][nj + 1] == 1)
				{
					pt.in = 4;
					pt.X = X0 + (nj + 1) * dx;
					pt.Y = Y0 + ni * dy + dy * (Z - height_mat[ni][nj + 1]) / (height_mat[ni + 1][nj + 1] - height_mat[ni][nj + 1]);
					pt.Z = Z;
					pt.HK = 0;
					pt.row = ni;
					pt.col = nj + 1;
					line.pt.push_back(pt);
					Hk[ni][nj + 1] = 0;
					nj += 1;
				}
				else if (Vk[ni + 1][nj] == 1)
				{
					pt.in = 3;
					pt.X = X0 + nj * dx + (Z - height_mat[ni + 1][nj]) / (height_mat[ni + 1][nj + 1] - height_mat[ni + 1][nj]) * dx;
					pt.Y = Y0 + (ni + 1) * dy;
					pt.Z = Z;
					pt.HK = 1;
					pt.row = ni + 1;
					pt.col = nj;
					line.pt.push_back(pt);
					Vk[ni + 1][nj] = 0;
					ni += 1;
				}
				else if (Hk[ni][nj] == 1)
				{
					pt.in = 2;
					pt.X = X0 + (nj)*dx;
					pt.Y = Y0 + ni * dy + dy * (Z - height_mat[ni][nj]) / (height_mat[ni + 1][nj] - height_mat[ni][nj]);
					pt.Z = Z;
					pt.HK = 0;
					pt.row = ni;
					pt.col = nj;
					line.pt.push_back(pt);
					Hk[ni][nj] = 0;

				}
			}

		}
		else if (pt.in == 2)
		{
			if (nj == 0 || Vk[ni + 1][nj - 1] + Hk[ni][nj - 1] + Vk[ni][nj - 1] == 0)
				break;
			if (Vk[ni + 1][nj - 1] + Hk[ni][nj - 1] + Vk[ni][nj - 1] == 3)
			{
				double zmean = (height_mat[ni][nj - 1] + height_mat[ni][nj] + height_mat[ni + 1][nj - 1] + height_mat[ni + 1][nj]) / 4.0;
				if ((height_mat[ni + 1][nj] - Z) * (zmean - Z) < 0)
				{
					pt.in = 3;
					pt.X = X0 + (nj - 1) * dx + (Z - height_mat[ni + 1][nj - 1]) / (height_mat[ni + 1][nj] - height_mat[ni + 1][nj - 1]) * dx;
					pt.Y = Y0 + (ni + 1) * dy;
					pt.Z = Z;
					pt.HK = 1;
					pt.row = ni + 1;
					pt.col = nj - 1;
					line.pt.push_back(pt);
					Vk[ni + 1][nj - 1] = 0;
					ni += 1;
					nj -= 1;
				}
				else
				{
					pt.in = 1;
					pt.X = X0 + (nj - 1) * dx + (Z - height_mat[ni][nj - 1]) / (height_mat[ni][nj] - height_mat[ni][nj - 1]) * dx;
					pt.Y = Y0 + (ni)*dy;
					pt.Z = Z;
					pt.HK = 1;
					pt.row = ni;
					pt.col = nj - 1;
					line.pt.push_back(pt);
					Vk[ni][nj - 1] = 0;
					nj -= 1;
				}
			}
			else
			{
				if (Vk[ni + 1][nj - 1] == 1)
				{
					pt.in = 3;
					pt.X = X0 + (nj - 1) * dx + (Z - height_mat[ni + 1][nj - 1]) / (height_mat[ni + 1][nj] - height_mat[ni + 1][nj - 1]) * dx;
					pt.Y = Y0 + (ni + 1) * dy;
					pt.Z = Z;
					pt.HK = 1;
					pt.row = ni + 1;
					pt.col = nj - 1;
					line.pt.push_back(pt);
					Vk[ni + 1][nj - 1] = 0;
					ni += 1;
					nj -= 1;
				}
				else if (Hk[ni][nj - 1] == 1)
				{
					pt.in = 2;
					pt.X = X0 + (nj - 1) * dx;
					pt.Y = Y0 + ni * dy + dy * (Z - height_mat[ni][nj - 1]) / (height_mat[ni + 1][nj - 1] - height_mat[ni][nj - 1]);
					pt.Z = Z;
					pt.HK = 0;
					pt.row = ni;
					pt.col = nj - 1;
					line.pt.push_back(pt);
					Hk[ni][nj - 1] = 0;
					nj -= 1;
				}
				else if (Vk[ni][nj - 1] == 1)
				{
					pt.in = 1;
					pt.X = X0 + (nj - 1) * dx + (Z - height_mat[ni][nj - 1]) / (height_mat[ni][nj] - height_mat[ni][nj - 1]) * dx;
					pt.Y = Y0 + (ni)*dy;
					pt.Z = Z;
					pt.HK = 1;
					pt.row = ni;
					pt.col = nj - 1;
					line.pt.push_back(pt);
					Vk[ni][nj - 1] = 0;
					nj -= 1;
				}
			}
		}
		else if (pt.in == 1)
		{
			if (ni == 0 || Hk[ni - 1][nj] + Vk[ni - 1][nj] + Hk[ni - 1][nj + 1] == 0)
				break;
			if (Hk[ni - 1][nj] + Vk[ni - 1][nj] + Hk[ni - 1][nj + 1] == 3)
			{
				double zmean = (height_mat[ni - 1][nj] + height_mat[ni - 1][nj + 1] + height_mat[ni][nj] + height_mat[ni][nj + 1]) / 4.0;
				if ((height_mat[ni][nj] - Z) * (zmean - Z) < 0)
				{
					pt.in = 2;
					pt.X = X0 + (nj)*dx;
					pt.Y = Y0 + (ni - 1) * dy + dy * (Z - height_mat[ni - 1][nj]) / (height_mat[ni][nj] - height_mat[ni - 1][nj]);
					pt.Z = Z;
					pt.HK = 0;
					pt.row = ni - 1;
					pt.col = nj;
					line.pt.push_back(pt);
					Hk[ni - 1][nj] = 0;
					ni -= 1;
				}
				else
				{
					pt.in = 4;
					pt.X = X0 + (nj + 1) * dx;
					pt.Y = Y0 + (ni - 1) * dy + dy * (Z - height_mat[ni - 1][nj + 1]) / (height_mat[ni][nj + 1] - height_mat[ni - 1][nj + 1]);
					pt.Z = Z;
					pt.HK = 0;
					pt.row = ni - 1;
					pt.col = nj + 1;
					line.pt.push_back(pt);
					Hk[ni - 1][nj + 1] = 0;
					ni -= 1;
					nj += 1;
				}
			}
			else
			{
				if (Hk[ni - 1][nj] == 1)
				{
					pt.in = 2;
					pt.X = X0 + (nj)*dx;
					pt.Y = Y0 + (ni - 1) * dy + dy * (Z - height_mat[ni - 1][nj]) / (height_mat[ni][nj] - height_mat[ni - 1][nj]);
					pt.Z = Z;
					pt.HK = 0;
					pt.row = ni - 1;
					pt.col = nj;
					line.pt.push_back(pt);
					Hk[ni - 1][nj] = 0;
					ni -= 1;
				}
				else if (Vk[ni - 1][nj] == 1)
				{
					pt.in = 1;
					pt.X = X0 + nj * dx + (Z - height_mat[ni - 1][nj]) / (height_mat[ni - 1][nj + 1] - height_mat[ni - 1][nj]) * dx;
					pt.Y = Y0 + (ni - 1) * dy;
					pt.Z = Z;
					pt.HK = 1;
					pt.row = ni - 1;
					pt.col = nj;
					line.pt.push_back(pt);
					Vk[ni - 1][nj] = 0;
					ni -= 1;
				}
				else if (Hk[ni - 1][nj + 1] == 1)
				{
					pt.in = 4;
					pt.X = X0 + (nj + 1) * dx;
					pt.Y = Y0 + (ni - 1) * dy + dy * (Z - height_mat[ni - 1][nj + 1]) / (height_mat[ni][nj + 1] - height_mat[ni - 1][nj + 1]);
					pt.Z = Z;
					pt.HK = 0;
					pt.row = ni - 1;
					pt.col = nj + 1;
					line.pt.push_back(pt);
					Hk[ni - 1][nj + 1] = 0;
					ni -= 1;
					nj += 1;
				}
			}
		}
	}
}

void DEM::getline(vector<vector<double>> Hk, vector<vector<double>> Vk, double Z, vector<CLine>& Lines)    //Vk行 Hk列
{
	vector<CLine> Line;
	//判断开闭曲线 先列后行 
	//Hk开曲线
	for (int i = 0; i < row - 1; i++)
	{
		if (Hk[i][0] == 1)
		{
			CLine line;
			Hk[i][0] = 0;
			line.shape = 0;
			PtLoc pt;
			pt.X = X0;
			pt.Y = Y0 + i * dy + dy * (Z - height_mat[i][0]) / (height_mat[i + 1][0] - height_mat[i][0]);
			pt.Z = Z;
			pt.in = 4;
			pt.HK = 0;
			pt.row = i;
			pt.col = 0;
			line.pt.push_back(pt);

			//搜索下一个等高线点
			int ni = i, nj = 0;
			GetlinePt(Hk, Vk, Z, ni, nj, line);
			Line.push_back(line);

		}
	}
	for (int i = 0; i < row - 1; i++)
	{
		if (Hk[i][col - 1] == 1) {
			CLine line;
			Hk[i][col - 1] = 0;
			line.shape = 0;
			PtLoc pt;
			pt.X = X0 + (col - 1) * dx;
			pt.Y = Y0 + i * dy + dy * (Z - height_mat[i][col - 1]) / (height_mat[i + 1][col - 1] - height_mat[i][col - 1]);
			pt.Z = Z;
			pt.in = 2;
			pt.HK = 0;
			pt.row = i;
			pt.col = col - 1;
			line.pt.push_back(pt);

			int ni = i, nj = col - 1;
			GetlinePt(Hk, Vk, Z, ni, nj, line);
			Line.push_back(line);
		}
	}


	//Vk开曲线
	for (int j = 0; j < col - 1; j++)
	{
		if (Vk[0][j] == 1)
		{
			CLine line;
			Vk[0][j] = 0;
			line.shape = 0;
			PtLoc pt;
			pt.X = X0 + j * dx + dx * (Z - height_mat[0][j]) / (height_mat[0][j + 1] - height_mat[0][j]);
			pt.Y = Y0;
			pt.Z = Z;
			pt.in = 3;
			pt.HK = 1;
			pt.row = 0;
			pt.col = j;
			line.pt.push_back(pt);

			//搜索下一个等高线点
			int ni = 0, nj = j;
			GetlinePt(Hk, Vk, Z, ni, nj, line);
			Line.push_back(line);
		}
	}

	for (int j = 0; j < col - 1; j++)
	{
		if (Vk[row - 1][j] == 1) {
			CLine line;
			Vk[row - 1][j] = 0;
			line.shape = 0;
			PtLoc pt;
			pt.X = X0 + j * dx + dx * (Z - height_mat[row - 1][j]) / (height_mat[row - 1][j + 1] - height_mat[row - 1][j]);
			pt.Y = Y0 + (row - 1) * dy;
			pt.Z = Z;
			pt.in = 1;
			pt.HK = 1;
			pt.row = row - 1;
			pt.col = j;
			line.pt.push_back(pt);

			int ni = row - 1, nj = j;
			GetlinePt(Hk, Vk, Z, ni, nj, line);
			Line.push_back(line);
		}
	}

	//Hk闭曲线
	for (int j = col - 1; j >= 0; j--)
	{
		for (int i = 0; i < row - 1; i++)
		{
			if (Hk[i][j] == 1)
			{
				CLine line;
				Hk[i][j] = 1;//起点仍为1
				line.shape = 1;//闭曲线
				PtLoc pt;
				pt.X = X0 + j * dx;
				pt.Y = Y0 + i * dy + dy * (Z - height_mat[i][j]) / (height_mat[i + 1][j] - height_mat[i][j]);
				pt.Z = Z;
				pt.in = 2;//顺时针跟踪闭曲线
				pt.HK = 0;
				pt.row = i;
				pt.col = j;
				line.pt.push_back(pt);

				int ni = i, nj = j;
				GetlinePt(Hk, Vk, Z, ni, nj, line);
				Line.push_back(line);
			}
		}
	}


	for (int j = col - 2; j >= 0; j--)
	{
		for (int i = 0; i < row; i++)
		{
			if (Vk[i][j] == 1)
			{
				CLine line;
				Vk[i][j] = 1;//起点仍为1
				line.shape = 1;//闭曲线
				PtLoc pt;
				pt.X = X0 + j * dx + (Z - height_mat[i][j]) / (height_mat[i][j + 1] - height_mat[i][j]) * dx;
				pt.Y = Y0 + i * dy;
				pt.Z = Z;
				pt.in = 1;//ni时针跟踪闭曲线
				pt.HK = 1;
				pt.row = i;
				pt.col = j;
				line.pt.push_back(pt);

				int ni = i, nj = j;
				GetlinePt(Hk, Vk, Z, ni, nj, line);
				Line.push_back(line);

			}
		}
	}

	///
	for (int i = 0; i < Line.size(); i++)
	{
		if (Line[i].pt.size() < 12)
			Line[i].shape = -1;
	}

	//
	int a = 0;
	for (int n = 0; n < Line.size(); n++)
	{
		CLine l1;
		if (Line[n].shape == 0)
		{
			l1.shape = 0;
			for (int i = 0; i < Line[n].pt.size(); i++)
			{
				a = 0;
				for (int m = 0; m < invalid_val.size(); m++)
				{
					int nr = invalid_val[m].first;
					int nc = invalid_val[m].second;
					if (Line[n].pt[i].HK == 0)
					{
						if ((nr == Line[n].pt[i].row && nc == Line[n].pt[i].col) || (nr == Line[n].pt[i].row + 1 && nc == Line[n].pt[i].col))
							a = 1;
					}
					else
					{
						if ((nr == Line[n].pt[i].row && nc == Line[n].pt[i].col) || (nr == Line[n].pt[i].row && nc == Line[n].pt[i].col + 1))
							a = 1;
					}
				}
				if (a == 0)
					l1.pt.push_back(Line[n].pt[i]);
			}
			Lines.push_back(l1);
		}
		else if (Line[n].shape == 1)
		{
			CLine l2;
			a = 0;
			int b = 0;
			for (int i = 0; i < Line[n].pt.size(); i++)
			{
				b = 0;
				for (int m = 0; m < invalid_val.size(); m++)
				{
					int nr = invalid_val[m].first;
					int nc = invalid_val[m].second;
					if (Line[n].pt[i].HK == 0)
					{
						if ((nr == Line[n].pt[i].row && nc == Line[n].pt[i].col) || (nr == Line[n].pt[i].row + 1 && nc == Line[n].pt[i].col))
						{
							a = 1;
						}
						else
						{
							if (a == 0)
								b = 1;
							else if (a == 1)
								b = 2;
						}
					}
					else
					{
						if ((nr == Line[n].pt[i].row && nc == Line[n].pt[i].col) || (nr == Line[n].pt[i].row && nc == Line[n].pt[i].col + 1))
						{
							a = 1;
						}
						else
						{
							if (a == 0)
								b = 1;
							else if (a == 1)
								b = 2;
						}

					}
				}
				if (b == 1)
					l1.pt.push_back(Line[n].pt[i]);
				else if (b == 2)
					l2.pt.push_back(Line[n].pt[i]);


			}
			if (l2.pt.size() == 0)
			{
				l1.shape = 1;
				Lines.push_back(l1);
			}
			else
			{
				l2.shape = 0;
				l2.pt.insert(l2.pt.end(), l1.pt.begin(), l1.pt.end());
				Lines.push_back(l2);
			}
		}
	}



	/*
	string dir = "../../data/DPEX_Data09/points.txt";
	ofstream outfile(dir);
	double X = 0, Y = 0;
	for (int i = 0; i < 340; i++) {
		for (int j = 0; j < 255; j++) {
			if (Hk[i][j] == 1) {
				X = X0 + j * dx;
				Y = Y0 + i * dy + dy * (Z - height_mat[i][j]) / (height_mat[i + 1][j] - height_mat[i][j]);
				outfile << X << "   " << Y << endl;
			}
		}
	}

	for (int i = 0; i < 341; i++) {
		for (int j = 0; j < 254; j++) {
			if (Vk[i][j] == 1) {
				X = X0 + j * dx + (Z - height_mat[i][j]) / (height_mat[i][j + 1] - height_mat[i][j])*dx;
				Y = Y0 + i * dy;
				outfile << X << "   " << Y << endl;
			}
		}
	}
	outfile.close();*/

}

void DEM::nlines(string dir, double dz)
{
	delz = dz;
	int k = (max_z - min_z) / delz;
	for (int i = 1; i <= k; i++)
	{
		vector<vector<double>> Hk;
		vector<vector<double>> Vk;
		Hk = StateMatHk(i);
		Vk = StateMatVk(i);
		double Z = min_z + i * delz;
		vector<CLine> Lines;
		getline(Hk, Vk, Z, Lines);
		savelines(dir, Lines, i);
		AllLines.push_back(Lines);
	}
}

void DEM::savelines(string dir, vector<CLine> Line, int k)
{
	ofstream outfile(dir);
	for (int i = 0; i < Line.size(); i++)
	{
		for (int j = 0; j < Line[i].pt.size(); j++)
		{
			if (Line[i].shape > -1)
				outfile << Line[i].pt[j].X << "   " << Line[i].pt[j].Y << "   " << Line[i].pt[j].Z << "   ";
		}
		outfile << endl;
	}
	outfile.close();
}

osg::Node* DEM::Create_DEM_lines(std::string img_path, std::string dem_path)
{
	//定义并设置高度文件
	osg::ref_ptr<osg::HeightField> heightField = new osg::HeightField();
	heightField->allocate(image_col, image_row);			//申请空间
	heightField->setOrigin(osg::Vec3(START_X_IMAGE, START_Y_IMAGE, 0));			//起始位置	
	heightField->setXInterval(DX_IMAGE);			//间距X
	heightField->setYInterval(DY_IMAGE);			//间距Y
	heightField->setSkirtHeight(1.0f);

	//填充高度值
	for (int r = 0; r < image_row; r++)
	{
		for (int c = 0; c < image_col; c++)
		{
			heightField->setHeight(c, r, dense_height_mat[r][c]);
		}
	}

	//节点
	osg::Geode* geode = new osg::Geode();
	//osg::ref_ptr<osg::ShapeDrawable> heightShape = new osg::ShapeDrawable(heightField.get());
	//geode->addDrawable(heightShape);
	int a = AllLines.size();
	for (int i = 0; i < a; i++)
	{
		for (int j = 0; j < AllLines[i].size(); j++)
		{
			osg::ref_ptr<osg::Vec3Array> vex = new osg::Vec3Array;
			for (int k = 0; k < AllLines[i][j].pt.size(); k++) {
				vex->push_back(osg::Vec3(AllLines[i][j].pt[k].X, AllLines[i][j].pt[k].Y, AllLines[i][j].pt[k].Z));
			}
			osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
			geometry->setVertexArray(vex.get());
			//设置颜色数组
			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
			for (int k = 0; k < AllLines[i][j].pt.size(); k++) {
				colors->push_back(osg::Vec4(1.0, 1.0, 0.2, 0.5));
			}
			geometry->setColorArray(colors);
			geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
			osg::ref_ptr<osg::PrimitiveSet> primitiveSet = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, AllLines[i][j].pt.size());
			geometry->addPrimitiveSet(primitiveSet);
			//设置线宽
			osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(1.0);
			geometry->getOrCreateStateSet()->setAttribute(lw, osg::StateAttribute::ON);

			geode->addDrawable(geometry.get());

		}

	}



	/*
	//设置顶点数组
	osg::ref_ptr<osg::Vec3Array> vex = new osg::Vec3Array;
	vex->push_back(osg::Vec3(-3.0, 0.0, 0.0));
	vex->push_back(osg::Vec3(3.0, 0.0, 0.0));
	vex->push_back(osg::Vec3(0.0, 5.0, 0.0));
	vex->push_back(osg::Vec3(0.0, 0.0, 5.0));

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
	geometry->setVertexArray(vex);

	//设置颜色数组
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;

	colors->push_back(osg::Vec4(1.0, 1.0, 0.0, 0.5));
	colors->push_back(osg::Vec4(1.0, 1.0, 0.0, 0.5));
	colors->push_back(osg::Vec4(1.0, 1.0, 0.0, 0.5));
	colors->push_back(osg::Vec4(1.0, 1.0, 0.0, 0.5));
	geometry->setColorArray(colors);
	geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

	osg::ref_ptr<osg::PrimitiveSet> primitiveSet = new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, 0, 4);
	geometry->addPrimitiveSet(primitiveSet);

	//设置线宽
	osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(6.0);
	geometry->getOrCreateStateSet()->setAttribute(lw, osg::StateAttribute::ON);

	geode->addDrawable(geometry);
	*/
	////设置纹理
	//osg::ref_ptr<osg::Image> texImage = osgDB::readImageFile(img_path);
	//osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
	//tex->setImage(texImage);
	//tex->setDataVariance(osg::Object::DYNAMIC);

	////渲染状态
	//osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
	//stateset->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
	//geode->setStateSet(stateset.get());

	return geode;
}

osg::Node* DEM::Create_DEM(std::string img_path, std::string dem_path)
{
	//定义并设置高度文件
	osg::ref_ptr<osg::HeightField> heightField = new osg::HeightField();
	heightField->allocate(image_col, image_row);			//申请空间
	heightField->setOrigin(osg::Vec3(START_X_IMAGE, START_Y_IMAGE, 0));			//起始位置	
	heightField->setXInterval(DX_IMAGE);			//间距X
	heightField->setYInterval(DY_IMAGE);			//间距Y
	heightField->setSkirtHeight(1.0f);

	//填充高度值
	for (int r = 0; r < image_row; r++)
	{
		for (int c = 0; c < image_col; c++)
		{
			heightField->setHeight(c, r, dense_height_mat[r][c]);
		}
	}

	//节点
	osg::Geode* geode = new osg::Geode();
	osg::ref_ptr<osg::ShapeDrawable> heightShape = new osg::ShapeDrawable(heightField.get());
	geode->addDrawable(heightShape);

	//设置纹理
	osg::ref_ptr<osg::Image> texImage = osgDB::readImageFile(img_path);
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
	tex->setImage(texImage);
	tex->setDataVariance(osg::Object::DYNAMIC);

	//渲染状态
	osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
	stateset->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
	geode->setStateSet(stateset.get());

	return geode;
}