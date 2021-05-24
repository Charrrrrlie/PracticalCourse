#include "Gram.h"

void PhotoGram::setInt(char* path)
{
	FILE *fp;
	errno_t err=fopen_s(&fp, path, "r");

	while (!feof(fp))
	{
		fscanf_s(fp, "%lf %lf %lf %d", &int_element.x, &int_element.y, &int_element.f,&int_element.m);
	}

	int_element.x /= 1000;
	int_element.y /= 1000;
	int_element.f /= 1000;

	fclose(fp);
}

void PhotoGram::setGCP(char *path)
{
	FILE *fp;
	errno_t err = fopen_s(&fp, path, "r");
	
	int gcp_num;
	fscanf_s(fp, "%d", &gcp_num);

	resPoint temp;
	
	for(int i=0;i<gcp_num;i++)
	{
		fscanf_s(fp, "%d %lf %lf %lf", &temp.num, &temp.X,&temp.Y,&temp.Z);
		gcp.push_back(temp);
	}
	fclose(fp);
}

InterElement PhotoGram::getInt()
{
	return int_element;
}

vector<resPoint> PhotoGram::getGCP()
{
	return gcp;
}

Eigen::MatrixXd PhotoGram::adj(Eigen::MatrixXd A, Eigen::MatrixXd L)
{
	return (A.transpose()*A).inverse()*A.transpose()*L;
}

Eigen::MatrixXd PhotoGram::calRotate(ExterElement temp)
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

	rotate = r_phi * r_omega*r_kappa;
	return rotate;
}

PhotoGram::PhotoGram(char *path_int, char *path_gcp)
{
	setInt(path_int);
	setGCP(path_gcp);
}

