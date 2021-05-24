#include"Gram.h"

IOrientation::IOrientation(char *path)
{
	//origin 2n*6
	//change 2n*1
	FILE *fp;
	errno_t err = fopen_s(&fp, path, "r");

	for (int i = 0; i < 2; i++)  //读取文件头
	{
		double temp;
		fscanf_s(fp, "%lf %lf %lf %lf %lf %lf", &temp, &temp, &temp, &temp, &temp, &temp);
	}


	int frame_num;
	fscanf_s(fp, "%d", &frame_num);

	change.resize(2*frame_num, 1);
	origin.resize(2*frame_num, 6);
	this->num = frame_num;


	for (int i = 0; i < frame_num; i++)
	{
		double temp;
		origin(2 * i + 1, 3) = 1; origin(2 * i + 1, 0) = 0;  origin(2 * i + 1, 1) = 0; origin(2 * i + 1, 2) = 0;
		origin(2 * i, 0) = 1;  origin(2 * i, 3) = 0; origin(2 * i, 4) = 0; origin(2 * i,5 ) = 0;
		fscanf_s(fp, "%lf %lf %lf %lf %lf %lf", &change(2*i,0), &change(2*i+1,0), &origin(2 * i, 1), &origin(2 * i, 2), &temp,&temp);
		origin(2 * i + 1, 4) = origin(2 * i , 1) ;
		origin(2 * i + 1, 5) = origin(2 * i , 2) ;
	}
	fclose(fp);

}

Eigen::MatrixXd IOrientation::calculate()
{
	cout << "--------------" << "内定向" << "--------------" << endl;
	params.resize(6, 1);
	params = (origin.transpose()*origin).inverse()*origin.transpose()*change;

	cout << endl<<"内定向仿射变换参数:" << endl;
	cout << params.block<3, 1>(0, 0).transpose() << endl;
	cout << params.block<3, 1>(3, 0).transpose() << endl;

	cout << endl << "内定向中误差Dxy:" << endl;
	cout << (change - origin * params).transpose()*(change - origin * params)<<endl<<endl;
	
	return params;
}