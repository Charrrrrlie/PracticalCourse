#include"Gram.h"
#include<string.h>
#include<stdlib.h>

#define FROM_RSanction 0
#define FROM_File 1

//内定向
void interOrientation(char* path)
{
	IOrientation io(path);
	io.calculate();
}

//后方交会
void rearSanction(char *path_int,char*realgcp,char*photogcp)
{
	RSanction rs(path_int, realgcp, photogcp);
	rs.calRSanction(6, 319);
	rs.show();
}

//前方交会
void forwardSanction(char *path_int, char*path_gcp1,char*path_gcp2, char*path_ext, int count, int type, int times,int point)
{
	FSanction fs(path_int, path_gcp1,path_gcp2, path_ext, count, type);

	char path_unknown[40];
	int list[6] = { 321,320,319,332,333,334 };

	for (int i = 0; i < 6; i++)
	{
		sprintf_s(path_unknown, "%s%d%s", "../../tieEdit/0", list[i], ".tif.dpg.pxy");
		fs.setUP(path_unknown, list[i], i);
	}

	fs.calFSanction(point, times);
}

//相对定向
void relativeOrientation(char *path_int,char*path_gcp,char*path_l,char*path_r)
{
	ROrientation d(path_int, path_gcp,path_l,path_r);
	d.calculate(5);
	d.translate();
	d.show();
}

int main()
{
	char path_int[30] = "../../tieEdit/data1int.txt";
	char path_ext[30] = "../../tieEdit/exterior.txt";
	char path_iorin[30]= "../../tieEdit/0319.tif.siop";
	char path_demoint[30] = "../../tieEdit/interior.txt";
	char path_demogcp[30] = "../../tieEdit/realGCP.txt";
	char path_demogcp2[30] = "../../tieEdit/photoGCP.txt";
	char path_gcps[40]= "../../tieEdit/GCP.txt";

	////内定向
	interOrientation(path_iorin);

	////后方交会
	rearSanction(path_demoint, path_demogcp,path_demogcp2);
	
	////前方交会
	forwardSanction(path_int, path_gcps,path_demogcp2, path_ext, 6, FROM_File, 20, 6);

	////相对定向
	char path_l[] = "../../tieEdit/0320.tif.dpg.pxy";
	char path_r[] = "../../tieEdit/0319.tif.dpg.pxy";
	
	relativeOrientation(path_int, path_gcps, path_l, path_r);

	////绝对定向
	char path_model[] = "../../tieEdit/ao_model.txt";
	char path_ground[]= "../../tieEdit/ao.txt";

	AOrientation e(path_int, path_ground, path_model,path_l,path_r);
	e.calculate(10);
	e.translate();

	////光束法
	char path_init_ext[] = "../../tieEdit/bundle_init_ext.txt";
	char path_init_pt[] = "../../tieEdit/bundle_init_pt.txt";
	char path_out_ext[] = "../../tieEdit/bundle_ext.ext";
	char path_out_pts[] = "../../tieEdit/bundle_pt.pt";

	Bundle adjst(path_int, path_gcps,path_init_ext,path_init_pt, 6);

	char path_unknown[40];
	int list[6] = { 321,320,319,332,333,334 };

	for (int i = 0; i < 6; i++)
	{
		sprintf_s(path_unknown, "%s%d%s", "../../tieEdit/0", list[i], ".tif.dpg.pxy");
		adjst.setPt(path_unknown, list[i], i);
	}

	adjst.calculate(6);
	adjst.show(path_out_ext, path_out_pts);


	return 0;
}




