#include <string>
#include "UbloxReader.h"

int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		printf("Path not found!");
		getchar();
	}
	std::string szPath(argv[1]);
	if (szPath.back() != '/' && szPath.back() != '\\')
	{
		szPath += "/";
	}
	CUbloxReader Reader;
	Reader.ReadLog(szPath+"ublox");
	Reader.Ublox2KmlFileWithPop(szPath+"ublox/ublox.kml", CV_RGB(255,0,0));

	printf("Finished!");
	getchar();

	return 0;
}

