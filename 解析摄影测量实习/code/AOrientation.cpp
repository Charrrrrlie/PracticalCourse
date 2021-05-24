#include"Gram.h"

AOrientation::AOrientation(char *path_int, char *path_gcp, char *path_model,char *path_left, char*path_right)
	:ROrientation(path_int,path_gcp,path_left,path_right)
{
	getModel(path_model);
}

void AOrientation::calculate(int times)
{
	cout << endl << endl;
	cout<<"--------------" << "绝对定向" << "--------------" << endl;
	vector<resPoint> ground = gravitilize(getGCP());
	res = { 1,0,0,0,0,0,0 };

	Eigen::MatrixXd A(ground.size() * 3, 7);
	Eigen::MatrixXd L(ground.size() * 3, 1);

	for (int t = 0; t < times; t++)
	{
		
		for (int i = 0; i < ground.size(); i++)
		{
			Eigen::MatrixXd a=diff(model[i]);
			Eigen::MatrixXd l=resi(model[i],ground[i]);

			A.block<3, 7>(i * 3, 0) = a;
			L.block<3, 1>(i * 3, 0) = l;
		}

		Eigen::MatrixXd delta = adj(A, L);

		res.dx += delta(0, 0);
		res.dy += delta(1, 0);
		res.dz += delta(2, 0);
		res.lambda+= delta(3, 0);
		res.phi += delta(4, 0);
		res.omega += delta(5, 0);
		res.kappa += delta(6, 0);


		if (abs(delta(0, 0)) < 3e-5 && abs(delta(1, 0)) < 3e-5 && abs(delta(2, 0)) < 3e-5
			&& abs(delta(3, 0)) < 3e-5 && abs(delta(4, 0)) < 3e-5 && abs(delta(5, 0)) < 3e-5 
			&& abs(delta(6, 0)) < 3e-5)
		{
			cout << endl << "绝对定向收敛!" << "      " << "迭代次数:" << t << endl;
			break;
		}
	}

	res.dx += gravity.X;  res.dy += gravity.Y; res.dz += gravity.Z;

	cout << endl << "绝对定向参数" << endl;
	cout << setw(8) << "λ φ ω κ dx dy dz" << endl;
	cout << setw(8) << fixed << setprecision(6) <<res.lambda << " " << res.phi << " " << res.omega << " " << res.kappa << " "
		<< res.dx << " " << res.dy << " " << res.dz << endl;
	
	
	cout << endl << "精度:" <<scientific<< ((A*((A.transpose()*A)*A.transpose()*L) - L).transpose()
		   *(A*((A.transpose()*A)*A.transpose()*L) - L) / (3 * ground.size()-7))<< endl;
}

vector<resPoint> AOrientation::translate()
{
	Eigen::MatrixXd trans(3, 1);
	Eigen::MatrixXd rot(3, 3);
	ExterElement r;
	r.phi = res.phi; r.omega = res.omega; r.kappa = res.kappa;
	trans(0, 0) = res.dx; trans(1, 0) = res.dy; trans(2, 0) = res.dz;
	rot = calRotate(r);
	
	vector<resPoint> trans_ans;

	Eigen::MatrixXd xyz(3, 1);
	cout << endl << "定向点坐标" << endl;
	for (int i = 0; i < model.size(); i++)
	{
		resPoint temp;
		xyz(0, 0) = model[i].X; xyz(1, 0) = model[i].Y; xyz(2, 0) = model[i].Z;
		xyz = rot * xyz*res.lambda + trans;

		temp = { model[i].type,model[i].num,xyz(0,0),xyz(1,0),xyz(2,0)};
		trans_ans.push_back(temp);

		
		cout << setw(8) << fixed << setprecision(6) <<model[i].num<<"    "<<setw(11)<< temp.X << "   " << setw(11) << temp.Y << "   " << setw(11) << temp.Z<<endl;
	}
	
	return trans_ans;
}

Eigen::MatrixXd AOrientation::diff(resPoint pt)
{
	Eigen::MatrixXd A=Eigen::MatrixXd::Zero(3,7);

	ExterElement ext;
	ext.phi = res.phi; ext.omega = res.omega; ext.kappa = res.kappa;

	Eigen::MatrixXd rot = calRotate(ext);

	Eigen::MatrixXd  xyz(3, 1);
	xyz(0, 0) = pt.X; 
	xyz(1, 0) = pt.Y;
	xyz(2, 0) = pt.Z;

	xyz = rot * xyz;

	A.col(0) << 1, 0, 0;
	A.col(1) << 0, 1, 0;
	A.col(2) << 0, 0, 1;
	A.col(3) << xyz(0, 0), xyz(1, 0), xyz(2, 0);
	A.col(4) << -res.lambda*xyz(2, 0), 0, res.lambda*xyz(0, 0);
	A.col(5) << -res.lambda*xyz(1, 0)*sin(res.phi),
		res.lambda*(xyz(0, 0)*sin(res.phi) - xyz(2, 0)*cos(res.phi)),
		res.lambda*xyz(1, 0)*cos(res.phi);
	A.col(6) << -res.lambda*xyz(1, 0)*cos(res.phi)*cos(res.omega) - res.lambda*xyz(2, 0)*sin(res.omega),
		res.lambda*xyz(0, 0)*cos(res.phi)*cos(res.omega) - res.lambda*xyz(2, 0)*sin(res.phi)*cos(res.omega),
		res.lambda*xyz(0, 0)*sin(res.omega) - res.lambda*xyz(1, 0)*sin(res.phi)*cos(res.omega);
	return A;
}

Eigen::MatrixXd AOrientation::resi(resPoint pt_model,resPoint pt_gravi)
{
	Eigen::MatrixXd L(3, 1);
	
	ExterElement ext;
	ext.phi = res.phi; ext.omega = res.omega; ext.kappa = res.kappa;

	Eigen::MatrixXd rot = calRotate(ext);

	Eigen::MatrixXd  xyz(3, 1);
	xyz(0, 0) = pt_model.X;
	xyz(1, 0) = pt_model.Y;
	xyz(2, 0) = pt_model.Z;

	xyz = rot * xyz;

	L(0, 0) = pt_gravi.X - res.lambda*xyz(0, 0) - res.dx;
	L(1, 0) = pt_gravi.Y - res.lambda*xyz(1, 0) - res.dy;
	L(2, 0) = pt_gravi.Z - res.lambda*xyz(2, 0) - res.dz;

	return L;
}

void AOrientation::getModel(char *path)
{
	FILE *fp;
	errno_t err = fopen_s(&fp, path, "r");

	int gcp_num;
	fscanf_s(fp, "%d", &gcp_num);

	resPoint temp;
	temp.type = UNKNOWN;

	for (int i = 0; i < gcp_num; i++)
	{
		fscanf_s(fp, "%d %lf %lf %lf", &temp.num, &temp.X, &temp.Y, &temp.Z);
		model.push_back(temp);
	}
	fclose(fp);
}

vector<resPoint> AOrientation::gravitilize(vector<resPoint> pts)
{
	vector<resPoint> gravi_pts(pts.size());
	for (int i = 0; i < pts.size(); i++)
	{
		gravity.X += pts[i].X;
		gravity.Y += pts[i].Y;
		gravity.Z += pts[i].Z;
	}

	gravity.X /= pts.size();
	gravity.Y /= pts.size();
	gravity.Z /= pts.size();

	for (int i = 0; i < pts.size(); i++)
	{
		gravi_pts[i].X = pts[i].X - gravity.X;
		gravi_pts[i].Y = pts[i].Y - gravity.Y;
		gravi_pts[i].Z = pts[i].Z - gravity.Z;
	}

	return gravi_pts;
}