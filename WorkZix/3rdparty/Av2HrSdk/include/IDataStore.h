
#ifndef _I_DATA_STORE_H_
#define _I_DATA_STORE_H_

#include "Av2HR_define.h"

class IDataStore
{
public:
    virtual av2hr_e Av2HR_getPosition(uint8_t index, av2hr_position_t* position) = 0;
    virtual av2hr_e Av2HR_getStub(av2hr_pathid_t pathId, av2hr_offset_t offset, uint8_t index, av2hr_stub_t* stub) = 0;
    virtual av2hr_e Av2HR_getProfileRange(uint16_t type, av2hr_pathid_t pathId, av2hr_offset_t* X0, av2hr_offset_t* X1) = 0;
    virtual av2hr_e Av2HR_getProfile(uint16_t type, av2hr_pathid_t pathId, av2hr_offset_t offset, av2hr_profile_desc_t* pd) = 0;

    virtual av2hr_e Av2HR_getLocation(uint16_t* contryCode, uint16_t* regionCode) = 0;
    virtual av2hr_e Av2HR_getDrivingSide(bool_t* drivingSideRight) = 0;
    virtual av2hr_e Av2HR_getSpeedUnit(bool_t* speedUnitMPH) = 0;
    virtual av2hr_e Av2HR_getProtoclVersion(uint8_t* major, uint8_t* minor, uint8_t* subminor) = 0;
    virtual av2hr_e Av2HR_getHardwareVersion(uint16_t* hardwareVersion) = 0;
    virtual av2hr_e Av2HR_getMapProvider(av2hr_mapprovider_e* mapProvider) = 0;
    virtual av2hr_e Av2HR_getMapVersion(uint16_t* mapVersionYear, uint8_t* mapVersionQuarter) = 0;
    virtual av2hr_e Av2HR_initializeReconstructor() = 0;
};

extern "C" IDataStore * GetDataStoreSingleton();

#endif // _I_DATA_STORE_H_
