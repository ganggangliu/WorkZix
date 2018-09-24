#ifndef OTHER_H
#define OTHER_H

#include<iostream>
#include<stdio.h>
#include<unistd.h>
#include<dirent.h>
#include<string.h>
#include<cstring>
#include<stdlib.h>

using namespace std;

std::vector<std::string> string_split(std::string& szIn, std::string& szSeperator)
{
	std::vector<std::string> VecOut;
	std::vector<size_t> VecSep;
	size_t nPos = 0;
	while(1)
	{
		nPos = szIn.find(szSeperator, nPos);
		if (nPos == std::string::npos)
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
		std::string szItem = szIn.substr(nStart, nLen);
		VecOut.push_back(szItem);
	}
	if (VecSep.size() > 0)
	{
		nStart = nEnd+1;
		nEnd = szIn.length();
		int nLen = nEnd - nStart;
		if (nLen >= 0)
		{
			std::string szItem = szIn.substr(nStart, nLen);
			VecOut.push_back(szItem);
		}
	}


	return VecOut;
}

//void getFiles( string path, string exd, vector<string>& files )
/*
参数 第一解析目录地址  第二 截取后存放的数组   第三分隔符即解析后缀
方法返回数组实际长度，注意size-3部分其中3是分隔符长度
解析失败返回-1
*/
int getFiles(string path, char* split, vector<string>& files)
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
        int sizeSep = strlen(split);
        char* pszSep = dirp->d_name + ( size - sizeSep);
        if (strcmp(pszSep, split)!=0)
            continue;
        szFilePath = string(dirp->d_name);
        files.push_back(szFilePath);
    }

    closedir(dp);

    return strL;
}

#endif
