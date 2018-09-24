/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __LCM_NAVI_PATH_hpp__
#define __LCM_NAVI_PATH_hpp__

#include <vector>
#include "LCM_POINT2D_F.hpp"


class LCM_NAVI_PATH
{
    public:
        int32_t    PathId;
        int32_t    PathType;
        int32_t    ContPathPoint;
        std::vector< LCM_POINT2D_F > Path;
        int32_t    IndLeftLine;
        int32_t    IndRightLine;

    public:
		inline LCM_NAVI_PATH();
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

LCM_NAVI_PATH::LCM_NAVI_PATH()
{
	ContPathPoint = 0;
}

int LCM_NAVI_PATH::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int LCM_NAVI_PATH::decode(const void *buf, int offset, int maxlen)
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

int LCM_NAVI_PATH::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t LCM_NAVI_PATH::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* LCM_NAVI_PATH::getTypeName()
{
    return "LCM_NAVI_PATH";
}

int LCM_NAVI_PATH::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->PathId, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->PathType, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->ContPathPoint, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->ContPathPoint; a0++) {
        tlen = this->Path[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->IndLeftLine, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->IndRightLine, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int LCM_NAVI_PATH::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->PathId, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->PathType, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->ContPathPoint, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    this->Path.resize(this->ContPathPoint);
    for (int a0 = 0; a0 < this->ContPathPoint; a0++) {
        tlen = this->Path[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->IndLeftLine, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->IndRightLine, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int LCM_NAVI_PATH::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->ContPathPoint; a0++) {
        enc_size += this->Path[a0]._getEncodedSizeNoHash();
    }
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    return enc_size;
}

int64_t LCM_NAVI_PATH::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == LCM_NAVI_PATH::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, (void*)LCM_NAVI_PATH::getHash };

    int64_t hash = 0xfd40c8b6db15748bLL +
         LCM_POINT2D_F::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

#endif