/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __Av2HR_ADM_OUTLINE_hpp__
#define __Av2HR_ADM_OUTLINE_hpp__

#include <vector>
#include "Av2HR_ADM_POINT.hpp"


class Av2HR_ADM_OUTLINE
{
    public:
        int8_t     m_iLaneLineIndex;
        int8_t     m_iOutlineType;
        int8_t     m_iOutlineProperty;
        int16_t    m_usPointNum;
        std::vector< Av2HR_ADM_POINT > m_pointString;

    public:
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

int Av2HR_ADM_OUTLINE::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int Av2HR_ADM_OUTLINE::decode(const void *buf, int offset, int maxlen)
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

int Av2HR_ADM_OUTLINE::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t Av2HR_ADM_OUTLINE::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* Av2HR_ADM_OUTLINE::getTypeName()
{
    return "Av2HR_ADM_OUTLINE";
}

int Av2HR_ADM_OUTLINE::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->m_iLaneLineIndex, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->m_iOutlineType, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->m_iOutlineProperty, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->m_usPointNum, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->m_usPointNum; a0++) {
        tlen = this->m_pointString[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int Av2HR_ADM_OUTLINE::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->m_iLaneLineIndex, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->m_iOutlineType, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->m_iOutlineProperty, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->m_usPointNum, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    this->m_pointString.resize(this->m_usPointNum);
    for (int a0 = 0; a0 < this->m_usPointNum; a0++) {
        tlen = this->m_pointString[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int Av2HR_ADM_OUTLINE::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int8_t_encoded_array_size(NULL, 1);
    enc_size += __int8_t_encoded_array_size(NULL, 1);
    enc_size += __int8_t_encoded_array_size(NULL, 1);
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->m_usPointNum; a0++) {
        enc_size += this->m_pointString[a0]._getEncodedSizeNoHash();
    }
    return enc_size;
}

int64_t Av2HR_ADM_OUTLINE::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == Av2HR_ADM_OUTLINE::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, (void*)Av2HR_ADM_OUTLINE::getHash };

    int64_t hash = 0x51570acadf2e706aLL +
         Av2HR_ADM_POINT::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

#endif
