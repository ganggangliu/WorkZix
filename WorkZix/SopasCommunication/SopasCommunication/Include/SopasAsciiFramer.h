#ifndef __SOPAS_ASCII_FRAMER_H__
#define __SOPAS_ASCII_FRAMER_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "Framer.h"

/*======================================================================================*/
/*! \brief Implements SOPAS-ASCII-framing

    \ingroup common

    This class implements ASCII-framing which is quite simple: 
    The data to be sent is enclosed by a STX- and an ETX-character. There are some
    limitations, see note below.
    
    \note Because of the simple framings the following limitations apply:
      - No binary data should be contained within a frame. If the data to be framed 
        even contains a STX- or ETX-character an invalid frame might be produced. 
        <em>This is not checked right now!</em>
      - The size of a received frame cannot be known in advance. So this implementation
        defines a maximum frame size of #ASCII_RECEIVE_BUFFER_SIZE. If any frame
        bigger than this is received <em>the framer will stop to remove framings and 
        will never recover from this!</em> You have to adjust the buffer size in that
        case. The telegram sizes can be obtained from the device documentation so a safe
        buffer size can be selected for the according device.
 */
/*======================================================================================*/
class CSopasAsciiFramer : public CFramer
{
public:
  CSopasAsciiFramer(void);
    //!< Constructor allocates storage for frame data.
  ~CSopasAsciiFramer(void);
    //!< Constructor deletes storage for frame data.

  virtual CBinaryDataStream* AddFraming(const CBinaryDataStream& rUnframedData);
    //!< Implementation of CFramer interface: Add framing to data.
  virtual CBinaryDataStream* RemoveFraming(CBinaryDataStream& rFramedData);
    //!< Implementation of CFramer interface: Remove framing from data.

private:
  CSopasAsciiFramer(const CSopasAsciiFramer&);
    //!< Do not allow copying
  CSopasAsciiFramer& operator=(const CSopasAsciiFramer&);
    //!< Do not allow copying

  static const int ASCII_RECEIVE_BUFFER_SIZE = 100000; 
    //!< Receive buffer size is not known in advance.
    //!< Must be big enough to contain the biggest frame
    //!<  you expect to receive.

  //! Internal state-machine for decoding.
  enum EFrameState { WAIT_FOR_STX, DATA_BYTES, BUFFER_FULL };    

  EFrameState m_eState;
    //!< Current decoding state.
  unsigned char* m_pFrameDataStorage;
    //!< Storage for partial received data until frame is complete.
  int m_curDataPos;
    //!< Current position within data storage.
};

#endif