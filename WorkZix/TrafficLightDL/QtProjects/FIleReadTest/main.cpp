#include <iostream>
#include <fstream>

#include "OpenCVInc.h"

using namespace std;

int main1()
{
    ifstream fs;
    fs.open("/home/zix-kotei/data/vs2013.4_ult_chs.raw");

    fs.seekg(0, _S_end);
    int64_t nnn = fs.tellg();

    return 0;
}

