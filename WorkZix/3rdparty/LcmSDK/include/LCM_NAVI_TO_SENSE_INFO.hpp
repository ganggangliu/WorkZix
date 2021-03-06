/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __LCM_NAVI_TO_SENSE_INFO_hpp__
#define __LCM_NAVI_TO_SENSE_INFO_hpp__

#include <vector>
#include "LCM_NAVI_LINE.hpp"
#include "LCM_NAVI_PATH.hpp"
#include "LCM_POINT2D_D.hpp"
#include "LCM_NAVI_OBJECT.hpp"


class LCM_NAVI_TO_SENSE_INFO
{
    public:
        int32_t    MsgInd;
        int32_t    ContLine;
        std::vector< LCM_NAVI_LINE > Lines;
        int32_t    ContPath;
        std::vector< LCM_NAVI_PATH > Paths;
        LCM_POINT2D_D RefLocale;
        float      RefHeading;
        int32_t    ContObj;
        std::vector< LCM_NAVI_OBJECT > Objs;

    public:
		inline LCM_NAVI_TO_SENSE_INFO();
        inline int encode(void *buf, int offset, int maxlen) const;
        inline int getEncodedSize() const;
        inline int decode(const void *buf, int offset, int maxlen);
        inline static int64_t getHash();
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static int64_t _computeHash(const __lcm_hash_ptr *p);
};

LCM_NAVI_TO_SENSE_INFO::LCM_NAVI_TO_SENSE_INFO()
{
	ContLine = 0;
	ContPath = 0;
	ContObj = 0;
}

int LCM_NAVI_TO_SENSE_INFO::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int LCM_NAVI_TO_SENSE_INFO::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int LCM_NAVI_TO_SENSE_INFO::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t LCM_NAVI_TO_SENSE_INFO::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* LCM_NAVI_TO_SENSE_INFO::getTypeName()
{
    return "LCM_NAVI_TO_SENSE_INFO";
}

int LCM_NAVI_TO_SENSE_INFO::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->MsgInd, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->ContLine, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->ContLine; a0++) {
        tlen = this->Lines[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->ContPath, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->ContPath; a0++) {
        tlen = this->Paths[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = this->RefLocale._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->RefHeading, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->ContObj, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->ContObj; a0++) {
        tlen = this->Objs[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int LCM_NAVI_TO_SENSE_INFO::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->MsgInd, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->ContLine, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    this->Lines.resize(this->ContLine);
    for (int a0 = 0; a0 < this->ContLine; a0++) {
        tlen = this->Lines[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->ContPath, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    this->Paths.resize(this->ContPath);
    for (int a0 = 0; a0 < this->ContPath; a0++) {
        tlen = this->Paths[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = this->RefLocale._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->RefHeading, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->ContObj, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    this->Objs.resize(this->ContObj);
    for (int a0 = 0; a0 < this->ContObj; a0++) {
        tlen = this->Objs[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int LCM_NAVI_TO_SENSE_INFO::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->ContLine; a0++) {
        enc_size += this->Lines[a0]._getEncodedSizeNoHash();
    }
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->ContPath; a0++) {
        enc_size += this->Paths[a0]._getEncodedSizeNoHash();
    }
    enc_size += this->RefLocale._getEncodedSizeNoHash();
    enc_size += __float_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->ContObj; a0++) {
        enc_size += this->Objs[a0]._getEncodedSizeNoHash();
    }
    return enc_size;
}

int64_t LCM_NAVI_TO_SENSE_INFO::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == LCM_NAVI_TO_SENSE_INFO::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, (void*)LCM_NAVI_TO_SENSE_INFO::getHash };

    int64_t hash = 0xd4e91b89897ce7c5LL +
         LCM_NAVI_LINE::_computeHash(&cp) +
         LCM_NAVI_PATH::_computeHash(&cp) +
         LCM_POINT2D_D::_computeHash(&cp) +
         LCM_NAVI_OBJECT::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

#endif
