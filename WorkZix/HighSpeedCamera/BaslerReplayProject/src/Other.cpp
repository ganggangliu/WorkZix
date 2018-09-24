#include "Other.h"

using namespace std;

vector<string> string_split(string& szIn, string& szSeperator)
{
	vector<string> VecOut;
	vector<size_t> VecSep;
	size_t nPos = 0;
	while(1)
	{
		nPos = szIn.find(szSeperator, nPos);
		if (nPos == string::npos)
		{
			break;
		}
		VecSep.push_back(nPos);
		nPos++;
		if (nPos >= szIn.length())
		{
			break;
		}
	}
	int nStart = 0;
	int nEnd = 0;
	for (size_t i = 0; i < VecSep.size(); i++)
	{
		if (i == 0)
		{
			nStart = 0;
			nEnd = VecSep[i];
		}
		else
		{
			nStart = nEnd+1;
			nEnd = VecSep[i];
		}
		int nLen = nEnd - nStart;
		string szItem = szIn.substr(nStart, nLen);
		VecOut.push_back(szItem);
	}
	if (VecSep.size() > 0)
	{
		nStart = nEnd+1;
		nEnd = szIn.length();
		int nLen = nEnd - nStart;
		if (nLen >= 0)
		{
			string szItem = szIn.substr(nStart, nLen);
			VecOut.push_back(szItem);
		}
	}


	return VecOut;
}

int WriteKmlFile(std::string szFilePath, std::string szKmlName,
	std::vector<cv::Point3d>& Path, cv::Scalar color/* = CV_RGB(255,0,0)*/)
{
	if (Path.size() <= 0)
	{
		return -1;
	}

	char szColor[64] = {0};
	sprintf(szColor, "ff%02x%02x%02x", uchar(color.val[0]), uchar(color.val[1]), uchar(color.val[2]));

	ofstream fsKlm(szFilePath);
	fsKlm << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << endl;
	fsKlm << "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" xmlns:atom=\"http://www.w3.org/2005/Atom\">" << endl;
	fsKlm << "<Document>" << endl;
//	fsKlm << "<name>ublox.kml</name>" << endl;
	fsKlm << "<name>" << szKmlName << "</name>" << endl;;

	fsKlm << "<StyleMap id=\"msn_ylw-pushpin\">" << endl;
	fsKlm << "<Pair>" << endl;
	fsKlm << "<key>normal</key>" << endl;
	fsKlm << "<styleUrl>#sn_ylw-pushpin</styleUrl>" << endl;
	fsKlm << "</Pair>" << endl;
	fsKlm << "</StyleMap>" << endl;
	fsKlm << "<Style id=\"sn_ylw-pushpin\">" << endl;
	fsKlm << "<LineStyle>" << endl;
//	fsKlm << "<color>ff0000ff</color>" << endl;
	fsKlm << "<color>";
	fsKlm << szColor;
	fsKlm << "</color>" << endl;
	fsKlm << "<width>2</width>" << endl;
	fsKlm << "</LineStyle>" << endl;
	fsKlm << "</Style>" << endl;

	fsKlm << "<Placemark>" << endl;
	fsKlm << "<name>ublox</name>" << endl;
	fsKlm << "<styleUrl>#msn_ylw-pushpin</styleUrl>" << endl;
	fsKlm << "<LineString>" << endl;
	fsKlm << "<tessellate>1</tessellate>" << endl;
	fsKlm << "<coordinates>" << endl;

	for (unsigned int i = 0; i < Path.size(); i++)
	{
		char szLine[1024] = {0};
		sprintf(szLine, "%.7f,%.7f,0", Path[i].x,Path[i].y);
		if (i != 0)
		{
			fsKlm << ",";
		}
		fsKlm << szLine;
//		printf(szLine);
//		printf("\n");
	}
	fsKlm << endl;

	fsKlm << "</coordinates>" << endl;
	fsKlm << "</LineString>" << endl;
	fsKlm << "</Placemark>" << endl;
	fsKlm << "</Document>" << endl;
	fsKlm << "</kml>" << endl;

	fsKlm.close();

	return Path.size();
}

#if WIN32
#include <windows.h>
#include <io.h>
#include <direct.h>

using namespace std;

int getFiles( string path, string exd, vector<string>& files )
{
    //文件句柄
    long   hFile   =   0;
    //文件信息
    struct _finddata_t fileinfo;
    string pathName, exdName;

    if (0 != strcmp(exd.c_str(), ""))
    {
        exdName = "\\*." + exd;
    }
    else
    {
        exdName = "\\*";
    }

    if((hFile = _findfirst(pathName.assign(path).append(exdName).c_str(),&fileinfo)) !=  -1)
    {
        do
        {
            //如果是文件夹中仍有文件夹,迭代之
            //如果不是,加入列表
            // 不推荐使用，硬要使用的话，需要修改else 里面的语句
            /*if((fileinfo.attrib &  _A_SUBDIR))
            {
                if(strcmp(fileinfo.name,".") != 0  &&  strcmp(fileinfo.name,"..") != 0)
                    getFiles( pathName.assign(path).append("\\").append(fileinfo.name), exd, files );
            }
            else */
            {
                if(strcmp(fileinfo.name,".") != 0  &&  strcmp(fileinfo.name,"..") != 0)
                    //files.push_back(pathName.assign(path).append("\\").append(fileinfo.name)); // 要得到绝对目录使用该语句
                    //如果使用
                    files.push_back(fileinfo.name); // 只要得到文件名字使用该语句
            }
        }while(_findnext(hFile, &fileinfo)  == 0);
        _findclose(hFile);
    }

    return files.size();
}

uint64_t getCurrentTime_ms()
{
	return ::GetTickCount(); 
}

int makeDir(std::string szPath)
{
	return _mkdir(szPath.c_str());
}

#else

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <string.h>
#include <cstring>
#include <stdlib.h>
#include <vector>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>

using namespace std;

int getFiles(string path, string exd, vector<string>& files)
{
	//目录
	DIR *dp;
	//获取dir目录具体的文件信息，名字，长度，指针地址等
	struct dirent *dirp;
	if ((dp = opendir(path.c_str())) == NULL)
	{
		perror("opendir");
	}
	int strL = 0;
	string szFilePath;
	while ((dirp = readdir(dp)) != NULL)
	{
		if (strcmp(dirp->d_name, ".") == 0 || strcmp(dirp->d_name, "..") == 0)
			continue;
		int size = strlen(dirp->d_name);
        int sizeSep = exd.length();
		char* pszSep = dirp->d_name + ( size - sizeSep);
        if (strcmp(pszSep, exd.c_str())!=0)
			continue;
		szFilePath = string(dirp->d_name);
		files.push_back(szFilePath);
	}

	closedir(dp);

	return strL;
}

uint64_t getCurrentTime_ms()
{
	struct timeval t;
	gettimeofday(&t, NULL);
	return ((uint64_t)t.tv_sec)*1000 + (uint64_t)t.tv_usec/1000;
}

int makeDir(std::string szPath)
{
    return mkdir(szPath.c_str(), S_IRWXU|S_IRWXG|S_IROTH|S_IXOTH);
}

#endif
