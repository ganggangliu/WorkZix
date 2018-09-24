#ifndef OTHER_H
#define OTHER_H

#include <string>
#include <vector>
#include <cstdint>
#include "OpencvInc.h"

std::vector<std::string> string_split(std::string& szIn, std::string& szSeperator);

int getFiles( std::string path, std::string exd, std::vector<std::string>& files );

uint64_t getCurrentTime_ms();

int makeDir(std::string szPath);

int WriteKmlFile(std::string szFilePath, std::string szKmlName, 
	std::vector<cv::Point3d>& Path, cv::Scalar color = CV_RGB(255,0,0));

#endif
