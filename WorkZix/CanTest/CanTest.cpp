#include "CanOpr.h"

int main(int argc, char* argv[])
{
	CCanParam Param;
	Param.nDevType = CAN_KVASER;
	Param.nDevInd = 0;
	Param.sChannel[0].nChannleInd = 0;
	Param.sChannel[0].nBaudRate = CAN_BAUDRATE_500K;
	CCanOpr Can;
	Can.Init(Param);
	Can.Start();

	return 0;
}

