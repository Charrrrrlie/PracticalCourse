#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  

#define def_row 1024
#define def_col 1024
#define def_band 1

using namespace cv;
using namespace std;

void RawRead(const char *Sourcepath,const char *Targetpath)
{
    //∂¡»Îraw

	FILE *fp;
	fp = fopen(Sourcepath, "rb");
	
	if (fp == NULL)
	{
		cout << "Error.Can not read Raw files" << endl;
	}

	unsigned char* mat;
	mat = new unsigned char [def_band*def_col*def_row];
	
	fread(mat, sizeof(unsigned char), def_band*def_col*def_row, fp);
	

	fclose(fp);
	
	//–¥»Îbmp

	if (def_band != 1)
	{
		Mat img(def_row, def_col, CV_8UC3, Scalar(0, 0, 0));
		
	    for (int i = 0; i < def_row; i++)
			for (int j = 0; j < def_col; j++)
				for (int k = 3; k < 6; k++)
				{
			     //img.data[(i*def_col + j) * 3 + k] = mat[(i+def_row*k)* def_col + j];     //BSQ
			    //img.data[(i*def_col + j) * 3 + k] = mat[(i*def_col+j)* def_band + k];     //BIP
				img.data[(i*def_col +j)*3+k] = mat[i*def_col*def_band +k* def_col +j];     //BIL
			    }


		imwrite(Targetpath, img);
		imshow("BMP image", img);
		waitKey(6000);
	}
	else
	{
		Mat img(def_row, def_col, CV_8UC1, Scalar(0, 0, 0));
		
		memcpy(img.data, mat, def_row*def_col);
		imwrite(Targetpath, img);
		imshow("BMP image", img);
		waitKey(6000);
	}
	
	delete[] mat;

}
