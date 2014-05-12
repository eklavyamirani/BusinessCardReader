#include<assert.h>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <fstream>
#include <iostream>
using namespace std;


int main(int argc,char* argv[])
{

	tesseract::TessBaseAPI api;
api.Init("", "eng", tesseract::OEM_DEFAULT);
api.SetPageSegMode(static_cast<tesseract::PageSegMode>(7));
api.SetOutputName("out");

char* imageName=argv[1];
	PIX   *pixs = pixRead(imageName);
	assert(pixs->data);
	

STRING text_out;

api.ProcessPages(imageName, NULL, 0, &text_out);
string text=text_out.string();
ofstream OcredText;
OcredText.open ("D:\\OcredText.txt",ios_base::app);
OcredText << text.c_str();  cout<<"\n"<<pixs->yres;
OcredText.close();
cout<<text_out.string();
system("pause");
	return 0;
}