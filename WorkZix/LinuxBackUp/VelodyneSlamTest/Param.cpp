#include "Param.h"

VelodyneParam::VelodyneParam()
{
    dTreadSlope = 10.0;
    CalibT.x = 0;
    CalibT.y = 1.6;
    CalibT.z = 0;
    CalibR.x = 0;
    CalibR.y = 0;
    CalibR.z = 0;
    dVehicleWidthHalf = 1600.0;
    dVehicleLen = 3000.0;
    dPathChangeToleratePer = 1.0;
    dPathChangeDist = 12.0;
    nStableCont = 3;
    nStableValid = 3;
}
