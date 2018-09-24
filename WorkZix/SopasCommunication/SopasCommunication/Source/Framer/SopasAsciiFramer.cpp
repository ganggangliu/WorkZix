/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "SopasAsciiFramer.h"
#include "BinaryDataStream.h"
#include <memory>

/*======================================================================================*/
/*! The internal state is reset so that the next received STX-character will 
    start a new frame.
 */
/*======================================================================================*/
CSopasAsciiFramer::CSopasAsciiFramer(void) :
  m_eState(WAIT_FOR_STX), m_pFrameDataStorage(0), m_curDataPos(0)
{
  // Max bufsize may not be enough for ASCII but we don't know bufsize...
  m_pFrameDataStorage = new unsigned char[ASCII_RECEIVE_BUFFER_SIZE];         
}

CSopasAsciiFramer::~CSopasAsciiFramer(void)
{
  delete[] m_pFrameDataStorage;
}

/*======================================================================================*/
/*! This method just takes all data written to <em>rUnframedData</em> and
    puts a STX in front and an ETX behind the data.

    \note <em>rUnframedData</em> should not contain STX- or ETX-characters.
      This would cause an invalid frame and the receiver wouldn't be able
      to remove the framing correctly. This is not checked here!

    \param rUnframedData A complete message to be sent. 

    \return The data including framing. Has to be deleted by the caller later.
 */
/*======================================================================================*/
CBinaryDataStream* CSopasAsciiFramer::AddFraming(const CBinaryDataStream& rUnframedData)
{
  CBinaryDataStream* pResult = 0;

  // Create output data stream. Add space for STX and ETX
  std::auto_ptr<CBinaryDataStream> coFramedData(
    new CBinaryDataStream(rUnframedData.GetUsedLength() + 2));

  // Add STX, copy unframed data and add ETY
  bool bSuccess = coFramedData->WriteToStream(0x02);
  bSuccess &= coFramedData->WriteToStream(rUnframedData);
  bSuccess &= coFramedData->WriteToStream(0x03);

  // Serialization should be successful because stream size is big enough. Check anyway:
  if(bSuccess == true)
  {
    // Transfer ownership, so that data is not deleted when function returns
    pResult = coFramedData.release();
  }

  return pResult;
}

/*======================================================================================*/
/*! This method just processes all data until it detects a frame enclosed by STX and ETX.
    The data inside the frame will be returned.

    This function is able to decode partial frames or multiple frames as required by the 
    CFramer interface. Keep extracting single frames until all data of \e rFramedData 
    has been processed.

    \note There is a limitation on frame size, see note for CSopasAsciiFramer.

    \param rFramedData A received data block. Needn't contain a whole frame or might even
      contain multiple frames. The read position is advanced accordingly until a frame
      has been decoded or all data has been processed.

    \return The first frame included in the given data.
      Has to be deleted by the caller later. \e null if no valid frame could be decoded yet.
 */
/*======================================================================================*/
CBinaryDataStream* CSopasAsciiFramer::RemoveFraming(CBinaryDataStream& rFramedData)
{  
  std::auto_ptr<CBinaryDataStream> coUnframedData;

  bool bFrameDecoded = false;

  // Repeat until no more input data is available
  // or a complete frame has been decoded
  while((rFramedData.IsReadPosAtEnd() == false) && (bFrameDecoded == false))  
  {
    // Note: Stream data is available (checked above), so no error checking is done here!
    unsigned char data = static_cast<unsigned char>(rFramedData.ReadChar());

    switch(m_eState)
    {
    case WAIT_FOR_STX:      
      if(data == 0x02)
      {
        m_eState = DATA_BYTES;        
      }      
      break;
    case DATA_BYTES:
      if(data == 0x03)
      {
        // Etx detected. Copy all payload data to return value
        coUnframedData.reset(new CBinaryDataStream(m_curDataPos));
        coUnframedData->WriteToStream(m_pFrameDataStorage, m_curDataPos);
        // Reset state
        m_curDataPos = 0;
        m_eState = WAIT_FOR_STX;
        bFrameDecoded = true; // End method call
      }
      else
      {
        // Check if buffer is big enough
        if(m_curDataPos < ASCII_RECEIVE_BUFFER_SIZE)
        {
          m_pFrameDataStorage[m_curDataPos] = data;
          m_curDataPos++;
        }
        else
        {
          // Stop further processing. Buffer size must be increased!
          // Note that the framer will not recover from this state!
          m_eState = BUFFER_FULL;
        }
      }
      break;
    case BUFFER_FULL:
      // If buffer is full, decoding is stopped.
      // Buffer size for this framing cannot be known in advance so
      // the buffer size must be increased in the header file.

      // Every cycle a byte is read until stream is at end
      // Program will interpret it as timeout because no more data is deframed
      break;
    default:
      break;
    }
  }

  // Transfer ownership, so that data is not deleted when function returns
  return coUnframedData.release();
}
