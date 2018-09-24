#pragma once

#include "Av2HR_define.h"

class IMessageObserver
{
public:
    virtual av2hr_e Av2HR_onNewPosition(const av2hr_position_t* p) = 0;
    virtual av2hr_e Av2HR_onNewSegment(const av2hr_segment_t* p) = 0;
    virtual av2hr_e Av2HR_onNewStub(const av2hr_stub_t* p) = 0;
    virtual av2hr_e Av2HR_onNewProfile(const av2hr_profile_t* p) = 0;
    virtual av2hr_e Av2HR_onNewMetaData(const av2hr_metadata_t* p) = 0;
    virtual av2hr_e Av2HR_onMissingMessage(const av2hr_msgtype_e msgType) = 0;
};

extern "C" IMessageObserver* GetDataStoreObserverSingleton();
