/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __lcmtypes_CAN_Data_hpp__
#define __lcmtypes_CAN_Data_hpp__


namespace lcmtypes
{

class CAN_Data
{
    public:
        int16_t    N[8];
        int16_t    L[8];
        int16_t    C[8];
        int16_t    P[8];
        int16_t    M[8];
        int16_t    W[8];
        float      Distance[8];
        float      Rel_Speed[8];
        float      X_location[8];
        int16_t    Percent[8];
        int8_t     NewFlag[8];

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

int CAN_Data::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int CAN_Data::decode(const void *buf, int offset, int maxlen)
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

int CAN_Data::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t CAN_Data::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* CAN_Data::getTypeName()
{
    return "CAN_Data";
}

int CAN_Data::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->N[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->L[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->C[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->P[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->M[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->W[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->Distance[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->Rel_Speed[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->X_location[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->Percent[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_encode_array(buf, offset + pos, maxlen - pos, &this->NewFlag[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int CAN_Data::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->N[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->L[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->C[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->P[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->M[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->W[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->Distance[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->Rel_Speed[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->X_location[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->Percent[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_decode_array(buf, offset + pos, maxlen - pos, &this->NewFlag[0], 8);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int CAN_Data::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int16_t_encoded_array_size(NULL, 8);
    enc_size += __int16_t_encoded_array_size(NULL, 8);
    enc_size += __int16_t_encoded_array_size(NULL, 8);
    enc_size += __int16_t_encoded_array_size(NULL, 8);
    enc_size += __int16_t_encoded_array_size(NULL, 8);
    enc_size += __int16_t_encoded_array_size(NULL, 8);
    enc_size += __float_encoded_array_size(NULL, 8);
    enc_size += __float_encoded_array_size(NULL, 8);
    enc_size += __float_encoded_array_size(NULL, 8);
    enc_size += __int16_t_encoded_array_size(NULL, 8);
    enc_size += __boolean_encoded_array_size(NULL, 8);
    return enc_size;
}

int64_t CAN_Data::_computeHash(const __lcm_hash_ptr *)
{
    int64_t hash = 0xa3f4e14669ccb546LL;
    return (hash<<1) + ((hash>>63)&1);
}

}

#endif
