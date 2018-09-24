/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __LCM_MOBILEYE_INFO_hpp__
#define __LCM_MOBILEYE_INFO_hpp__

#include <vector>
#include "LCM_MOBILEYE_LINE.hpp"
#include "LCM_MOBILEYE_LINE.hpp"
#include "LCM_MOBILEYE_OBJECT.hpp"


class LCM_MOBILEYE_INFO
{
    public:
        int64_t    FrameId;
        LCM_MOBILEYE_LINE LeftLine;
        LCM_MOBILEYE_LINE RightLine;
        int32_t    nContObj;
        std::vector< LCM_MOBILEYE_OBJECT > ObjList;

    public:
		inline LCM_MOBILEYE_INFO();
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

LCM_MOBILEYE_INFO::LCM_MOBILEYE_INFO()
{
	nContObj = 0;
	ObjList.clear();
}

int LCM_MOBILEYE_INFO::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int LCM_MOBILEYE_INFO::decode(const void *buf, int offset, int maxlen)
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

int LCM_MOBILEYE_INFO::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t LCM_MOBILEYE_INFO::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* LCM_MOBILEYE_INFO::getTypeName()
{
    return "LCM_MOBILEYE_INFO";
}

int LCM_MOBILEYE_INFO::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->FrameId, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->LeftLine._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->RightLine._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->nContObj, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->nContObj; a0++) {
        tlen = this->ObjList[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int LCM_MOBILEYE_INFO::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->FrameId, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->LeftLine._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->RightLine._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->nContObj, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    this->ObjList.resize(this->nContObj);
    for (int a0 = 0; a0 < this->nContObj; a0++) {
        tlen = this->ObjList[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int LCM_MOBILEYE_INFO::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += this->LeftLine._getEncodedSizeNoHash();
    enc_size += this->RightLine._getEncodedSizeNoHash();
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->nContObj; a0++) {
        enc_size += this->ObjList[a0]._getEncodedSizeNoHash();
    }
    return enc_size;
}

int64_t LCM_MOBILEYE_INFO::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == LCM_MOBILEYE_INFO::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, (void*)LCM_MOBILEYE_INFO::getHash };

    int64_t hash = 0x126993ed17bae934LL +
         LCM_MOBILEYE_LINE::_computeHash(&cp) +
         LCM_MOBILEYE_LINE::_computeHash(&cp) +
         LCM_MOBILEYE_OBJECT::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

#endif