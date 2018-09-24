#ifndef _I_MESSAGE_CODEC_H_
#define _I_MESSAGE_CODEC_H_

#include "Av2HR_define.h"

class IMessageCodec
{
public:
    virtual av2hr_e Av2HR_MC_onMessage(const av2hr_canframe_t* can) = 0;
};

extern "C" IMessageCodec* GetMessageCodecSingleton();

#endif  // _I_MESSAGE_CODEC_H_
