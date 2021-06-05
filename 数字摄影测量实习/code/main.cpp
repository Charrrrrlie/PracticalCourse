#include <osgViewer/Viewer>
#include <osgDB/ReadFile>

#include"DEM.h"

using namespace std;

int main(int argc, char** argv)
{
	string dem_path = "../../../data/DPEX_Data09/rock1.ddem";   //demÂ·¾¶
	string image_path = "../../../data/DPEX_Data11/DEMphoto4.bmp";  //ÎÆÀíÍ¼Æ¬Â·¾¶
	string path = "../../../res/lines.txt";
	DEM dem(dem_path, image_path);
	dem.nlines(path, 5);
	dem.Interp();
	osgViewer::Viewer viewer;
	osg::Node* model = dem.Create_DEM_lines(image_path,dem_path);

	if (model == nullptr)
	{
		printf("ohno");
	}

	viewer.setSceneData(model);
	return viewer.run();

	//return 0;
}