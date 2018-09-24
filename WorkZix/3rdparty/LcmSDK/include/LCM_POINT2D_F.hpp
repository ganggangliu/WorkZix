/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __LCM_POINT2D_F_hpp__
#define __LCM_POINT2D_F_hpp__



class LCM_POINT2D_F
{
    public:
        float      x;
        float      y;

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

int LCM_POINT2D_F::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int LCM_POINT2D_F::decode(const void *buf, int offset, int maxlen)
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

int LCM_POINT2D_F::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t LCM_POINT2D_F::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* LCM_POINT2D_F::getTypeName()
{
    return "LCM_POINT2D_F";
}

int LCM_POINT2D_F::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->x, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->y, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int LCM_POINT2D_F::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->x, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->y, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int LCM_POINT2D_F::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __float_encoded_array_size(NULL, 1);
    enc_size += __float_encoded_array_size(NULL, 1);
    return enc_size;
}

int64_t LCM_POINT2D_F::_computeHash(const __lcm_hash_ptr *)
{
    int64_t hash = 0x7c2d87baacd686e3LL;
    return (hash<<1) + ((hash>>63)&1);
}

#endif
