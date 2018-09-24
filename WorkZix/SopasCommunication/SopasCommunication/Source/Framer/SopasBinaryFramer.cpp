/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "SopasBinaryFramer.h"
#include "BinaryDataStream.h"
#include <memory>

/*======================================================================================*/
/*! The internal state is reset so that the next 4 received STX-character will 
    start a new frame.
 */
/*======================================================================================*/
CSopasBinaryFramer::CSopasBinaryFramer(void) : 
  m_eState(WAIT_FOR_STX), m_ulStxCtr(0), m_ulLengthIndex(0), 
  m_pUnframedData(0), m_Crc(0)
{
}

CSopasBinaryFramer::~CSopasBinaryFramer(void)
{
  delete m_pUnframedData;
}

/*======================================================================================*/
/*! This method takes all data written to \e rUnframedData and
    encloses it with a frame out of STX, data length and checksum.

    \param rUnframedData A complete message to be sent.

    \return The data including framing. Has to be deleted by the caller later.
 */
/*======================================================================================*/
CBinaryDataStream* CSopasBinaryFramer::AddFraming(const CBinaryDataStream& rUnframedData)
{
  CBinaryDataStream* pResult = 0;

  // Create output data stream. Add space for 4 byte stx, 4 byte len, 1 byte crc
  std::auto_ptr<CBinaryDataStream> coFramedData(
    new CBinaryDataStream(rUnframedData.GetUsedLength() + 9));

  // Add STX
  bool bSuccess = true;
  for(unsigned int i = 0; i < 4; ++i)
  {
    bSuccess &= coFramedData->WriteToStream(0x02);
  }

  // Length of data, msb first
  unsigned int ulLength = rUnframedData.GetUsedLength();
  for(unsigned int i = 0; i < 4; ++i)
  {
    unsigned char lengthByte = static_cast<unsigned char>((ulLength & 0xFF000000) >> 24);
    bSuccess &= coFramedData->WriteToStream(lengthByte);
    ulLength <<= 8;
  }

  // Copy payload data
  bSuccess &= coFramedData->WriteToStream(rUnframedData);

  // XOR-Checksum
  unsigned char checksum = 0;
  const unsigned char* pRawData = rUnframedData.GetData();
  for(unsigned int i = 0; i < rUnframedData.GetUsedLength(); ++i)
  {
    checksum ^= pRawData[i];
  }
  bSuccess &= coFramedData->WriteToStream(checksum);

  // Serialization should be successful because stream size is big enough. Check anyway:
  if(bSuccess == true)
  {
    // Transfer ownership, so that data is not deleted when function returns
    pResult = coFramedData.release();
  }

  return pResult;
}

/*======================================================================================*/
/*! This method just processes all data until it detects a start of frame (4 times STX).
    The data inside the frame will be returned when data of according length (encoded
    into framing) and valid checksum has been detected.

    While inside a frame no new frame can be started (even if 4 time STX occur). A frame
    is finished when the according number of bytes have been extracted (length encoded
    into framing). When the checksum is invalid the decoded frame is discarded and
    the framer waits for the next frame start.

    This function is able to decode partial frames or multiple frames as required by the 
    CFramer interface. Keep extracting single frames until all data of \e rFramedData 
    has been processed.

    \param rFramedData A received data block. Needn't contain a whole frame or might even
      contain multiple frames. The read position is advanced accordingly until a frame
      has been decoded or all data has been processed.

    \return The first frame included in the given data.
      Has to be deleted by the caller later. \e null if no valid frame could be decoded yet.
 */
/*======================================================================================*/
CBinaryDataStream* CSopasBinaryFramer::RemoveFraming(CBinaryDataStream& rFramedData)
{
  CBinaryDataStream* pUnframedRetVal = 0;

  // Repeat until no more input data is available
  // or a complete frame has been decoded
  while((rFramedData.IsReadPosAtEnd() == false) && (pUnframedRetVal == 0))  
  {
    // Note: Stream data is available (checked above), so no error checking is done here!
    unsigned char data = static_cast<unsigned char>(rFramedData.ReadChar());

    switch(m_eState)
    {
    case WAIT_FOR_STX:
      if(data != 0x02)
      {
        m_ulStxCtr = 0;
      }
      else
      {
        m_ulStxCtr++;
      }
      if(m_ulStxCtr == 4)
      {
        m_ulStxCtr = 0;							
        m_eState = LEN_BYTES;
      }
      break;
    case LEN_BYTES:
      m_aLengthArray[m_ulLengthIndex] = data;
      m_ulLengthIndex++;
      if(m_ulLengthIndex == 4)
      {
        unsigned int ulUnframedDataLength = 0;
        for(int j = 0; j < 4; ++j)
        {
          ulUnframedDataLength <<= 8;
          ulUnframedDataLength += m_aLengthArray[j];								
        }
        m_pUnframedData = new CBinaryDataStream(ulUnframedDataLength);
        m_ulLengthIndex = 0;
        m_eState = DATA_BYTES;
      }
      break;
    case DATA_BYTES:
      (void)m_pUnframedData->WriteToStream(data);
      m_Crc ^= data;        
      if(m_pUnframedData->IsWritePosAtEnd() == true)
      {							
        m_eState = CRC_BYTE;
      }
      break;
    case CRC_BYTE:
      if(data == m_Crc)
      {
        // Valid crc, return unframed data
        pUnframedRetVal = m_pUnframedData; 
      }
      else
      {
        // Invalid crc, discard unframed data
        delete m_pUnframedData;
      }
      // Reset state
      m_pUnframedData = 0;
      m_Crc = 0;
      m_eState = WAIT_FOR_STX;
      break;
    default:
      break;
    }
  }
  
  // All bytes of inData processed?
  return pUnframedRetVal;
}
