/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __Av2HR_ADM_VEHICLE_STATE_hpp__
#define __Av2HR_ADM_VEHICLE_STATE_hpp__



class Av2HR_ADM_VEHICLE_STATE
{
    public:
        int64_t    m_tmSinceEpoch;
        int32_t    m_iLongitude;
        int32_t    m_iLatitude;
        int32_t    m_iAltitude;
        int32_t    m_iSpeed;
        int32_t    m_iHeading;
        int32_t    m_iStatus;

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

int Av2HR_ADM_VEHICLE_STATE::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int Av2HR_ADM_VEHICLE_STATE::decode(const void *buf, int offset, int maxlen)
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

int Av2HR_ADM_VEHICLE_STATE::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t Av2HR_ADM_VEHICLE_STATE::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* Av2HR_ADM_VEHICLE_STATE::getTypeName()
{
    return "Av2HR_ADM_VEHICLE_STATE";
}

int Av2HR_ADM_VEHICLE_STATE::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->m_tmSinceEpoch, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->m_iLongitude, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->m_iLatitude, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->m_iAltitude, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->m_iSpeed, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->m_iHeading, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->m_iStatus, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int Av2HR_ADM_VEHICLE_STATE::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->m_tmSinceEpoch, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->m_iLongitude, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->m_iLatitude, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->m_iAltitude, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->m_iSpeed, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->m_iHeading, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->m_iStatus, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int Av2HR_ADM_VEHICLE_STATE::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    return enc_size;
}

int64_t Av2HR_ADM_VEHICLE_STATE::_computeHash(const __lcm_hash_ptr *)
{
    int64_t hash = 0xf0a3615e30619606LL;
    return (hash<<1) + ((hash>>63)&1);
}

#endif