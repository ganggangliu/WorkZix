#ifndef __SOPAS_BINARY_FRAMER_H__
#define __SOPAS_BINARY_FRAMER_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "Framer.h"

/*======================================================================================*/
/*! \brief Implements SOPAS-binary-framing

    \ingroup common

    This class implements binary framing which is more complex than ASCII framing: 
    The data to be sent is started by four STX-characters followed by the payload length
    (4 byte) and a XOR checksum built over the payload.
    
    \note Invalid frames and characters inbetween are discarded until a complete frame
      with correct CRC can be decoded.
 */
/*======================================================================================*/
class CSopasBinaryFramer : public CFramer
{
public:
  CSopasBinaryFramer(void);
    //!< Constructor allocates storage for frame data.
  ~CSopasBinaryFramer(void);
    //!< Constructor deletes storage for frame data.

  virtual CBinaryDataStream* AddFraming(const CBinaryDataStream& rUnframedData);
    //!< Implementation of CFramer interface: Add framing to data.
  virtual CBinaryDataStream* RemoveFraming(CBinaryDataStream& rFramedData);
    //!< Implementation of CFramer interface: Remove framing from data.

private:
  CSopasBinaryFramer(const CSopasBinaryFramer&);
    //!< Do not allow copying
  CSopasBinaryFramer& operator=(const CSopasBinaryFramer&);
    //!< Do not allow copying

  //! Internal state-machine for decoding.
  enum EFrameState {WAIT_FOR_STX, LEN_BYTES, DATA_BYTES, CRC_BYTE};

  EFrameState m_eState;
    //!< Current decoding state.
  unsigned int m_ulStxCtr;
    //!< Count STX characters (4 start a frame)
  unsigned char m_aLengthArray[4];
    //!< Store length indicator of frame for decoding
  unsigned int m_ulLengthIndex;
    //!< Current index within #m_aLengthArray.
  CBinaryDataStream* m_pUnframedData;
    //!< Unframed data storage.
  unsigned char m_Crc;
    //!< Calculated CRC for unframed data.

};

#endif