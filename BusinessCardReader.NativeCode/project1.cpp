#include<iostream>
#include<string>
using namespace std;

int main(int argc,char* argv[])
{
	string imagename="D:\\bc8.jpg";
	string setpath="D:\\swt\\x64\\Release\\swt.exe ";
	string path=setpath+imagename;
	string output="D:\\Project1\\Project1\\";
	int i=system(path.c_str());
	for(int k=0;k<i;k++)
	{
		string imageloc="D:\\tess\\Release\\tess.exe "+ output +"image" + to_string(k)+".jpg";
		system(imageloc.c_str());
	}
		system("pause");
return 0;
}