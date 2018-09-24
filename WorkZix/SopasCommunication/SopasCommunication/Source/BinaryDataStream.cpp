/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "BinaryDataStream.h"
#include <memory.h>

/*======================================================================================*/
/*! The given size cannot be changed afterwards. This size is the maximum number
    of bytes that can be stored. If fewer bytes were written to the stream
    only those bytes can be read.

    \param size The maximum size of the stream in bytes.
 */
/*======================================================================================*/
CBinaryDataStream::CBinaryDataStream(unsigned int size) :
  m_pStream(new unsigned char[size]), m_ulStreamSize(size), m_ulStreamWritePos(0), m_ulStreamReadPos(0)
{
}

CBinaryDataStream::~CBinaryDataStream(void)
{
  delete[] m_pStream;
}

/*======================================================================================*/
/*! The first character in the buffer starts at index 0.

    \return The index of the byte that will be read by the next read command.
 */
/*======================================================================================*/
unsigned int CBinaryDataStream::GetReadIndex(void) const
{
  return m_ulStreamReadPos;
}

/*======================================================================================*/
/*! Use this function if you need to read the whole stream contents on your own.
    The read pointer of the stream will not be affected. The whole stream data is 
    returned so use GetUsedLength() to determine the end of written data!

    \attention Do only use when really necessary and use the data for a short time only.
      When any write function is called this data is changed. When the stream is deleted
      the returned data becomes invalid! This is \e not a copy of the stream buffer so 
      don't delete it outside!

    \return The whole internal buffer of the stream.
 */
/*======================================================================================*/
unsigned const char* CBinaryDataStream::GetData(void) const
{
  return m_pStream;
}

/*======================================================================================*/
/*! The character is appended at the current write position unless the stream would 
    exceed its maximum size. The write pointer is advanced accordingly if successful.

    \param data The character to write to the stream.

    \return \e true if the character could be appended. \e false if the stream
      size would be exceeded by this operation.
 */
/*======================================================================================*/
bool CBinaryDataStream::WriteToStream(const unsigned char data)
{
  bool retVal = false;
  if(m_ulStreamWritePos + 1 <= m_ulStreamSize)
  {
    retVal = true;
    m_pStream[m_ulStreamWritePos] = data;
    m_ulStreamWritePos++;
  }

  return retVal;
}

/*======================================================================================*/
/*! The character array is appended at the current write position unless the stream would 
    exceed its maximum size. The write pointer is advanced accordingly if successful.

    \param pData The character array to write to the stream. Must contain at least
      \e length characters. This cannot be checked!
    \param length The number of bytes in \e pData to be written to the stream.

    \return \e true if the characters could be appended. \e false if the stream
      size would be exceeded by this operation.
 */
/*======================================================================================*/
bool CBinaryDataStream::WriteToStream(const unsigned char* pData, int length)
{
  bool retVal = false;
  if((pData != 0) && (m_ulStreamWritePos + length <= m_ulStreamSize))
  {
    retVal = true;    
    (void*)memcpy(&m_pStream[m_ulStreamWritePos], pData, length);    
    m_ulStreamWritePos += length;
  }

  return retVal;
}

/*======================================================================================*/
/*! The string is appended at the current write position unless the stream would 
    exceed its maximum size. The write pointer is advanced accordingly if successful.
    The whole string is written to the stream.

    \param rSrc The string to write to the stream. 

    \return \e true if the characters could be appended. \e false if the stream
      size would be exceeded by this operation.
 */
/*======================================================================================*/
bool CBinaryDataStream::WriteToStream(const std::string& rSrc)
{
  bool retVal = false;
  // Copy string into byte array.
  if(m_ulStreamWritePos + rSrc.length() <= m_ulStreamSize)
  {
    const char* pSrc = rSrc.c_str();
    memcpy(&m_pStream[m_ulStreamWritePos], pSrc, rSrc.length());
    m_ulStreamWritePos += static_cast<unsigned int>(rSrc.length()); // Advance stream position
    retVal = true;
  }
  return retVal;
}

/*======================================================================================*/
/*! All written contents of the other stream are appended at the current write position 
    of this stream unless this stream would exceed its maximum size. The write pointer 
    is advanced accordingly if successful.

    \note The other stream is appended completely, regardless of its read pointer 
      position. Its read pointer remains unchanged.

    \param rOtherStream The other stream to append to this stream. 

    \return \e true if the characters could be appended. \e false if the stream
      size would be exceeded by this operation.
 */
/*======================================================================================*/
bool CBinaryDataStream::WriteToStream(const CBinaryDataStream& rOtherStream)
{
  return WriteToStream(rOtherStream.GetData(), rOtherStream.GetUsedLength());
}

/*======================================================================================*/
/*! \return The used size of this stream.
 */
/*======================================================================================*/
unsigned int CBinaryDataStream::GetUsedLength(void) const
{
  return m_ulStreamWritePos;
}

/*======================================================================================*/
/*! The given number of characters are read from the stream at the current read 
    position and returned as string. This function will not read beyond bytes that have 
    been written by the stream's write functions even if the stream size is bigger. 
    The read pointer is advanced accodingly if all bytes could be read.

    \note The characters may even contain \e null characters. std::string is able
      to store the whole data of given length but further processing may have problems
      with it.

    \param length The number of bytes to read as string.

    \return The extracted string or an empty string if not enough bytes
      are available.
 */
/*======================================================================================*/
std::string CBinaryDataStream::ReadString(int length)
{
  if((m_ulStreamReadPos + length <= m_ulStreamSize) && (length > 0))
  {    
    const char* p = reinterpret_cast<const char*>(&m_pStream[m_ulStreamReadPos]);
    m_ulStreamReadPos += length; // Advance read pointer
    return std::string(p, length); // Extract string until given length
  }
  else
  {
    return "";
  }
}

/*======================================================================================*/
/*! Finds the next read index of the character \e c. The search is started at the current
    read position. For example use this function to find a separator up to which data has 
    to be extracted from this stream. The first character in the buffer starts at index 0.

    This function does not affect the read pointer. 

    \param c The character to find.

    \return The next index of the character \e c.
      If the character cannot be found the return value will be the position
      behind the last element written to this stream.
 */
/*======================================================================================*/
unsigned int CBinaryDataStream::FindIndexOfNext(const unsigned char c) const
{
  unsigned int i;
  for (i = m_ulStreamReadPos; i < m_ulStreamWritePos; ++i)
  {
    if (m_pStream[i] == c)
    {
      break;
    }
  }
  // Points to separator's index or to index behind last written element
  return i;
}

/*======================================================================================*/
/*! This function will not read beyond bytes that have been written by the stream's 
    write functions even if the stream size is bigger. The read pointer is advanced
    accordingly if the byte could be read.

    \return The character read or \e -1 if read pointer is already at the end of data.
 */
/*======================================================================================*/
int CBinaryDataStream::ReadChar(void)
{
  // Write routines check that no writing is done beyond stream
  // Reading beyond written data is not allowed
  int data = -1;
  if(m_ulStreamReadPos < m_ulStreamWritePos)
  {
    data = m_pStream[m_ulStreamReadPos];
    m_ulStreamReadPos++;
  }
  return data;
}

/*======================================================================================*/
/*! This function will not peek beyond bytes that have been written by the stream's 
    write functions even if the stream size is bigger. The read pointer is not affected
    by this function.

    \return The character that would be read next or \e -1 if read pointer is already 
    at the end of data.
 */
/*======================================================================================*/
int CBinaryDataStream::PeekChar(void) const
{
  // Write routines check that no writing is done beyond stream
  // Reading beyond written data is not allowed
  int data = -1;
  if(m_ulStreamReadPos < m_ulStreamWritePos)
  {
    data = m_pStream[m_ulStreamReadPos];
  }
  return data;
}

/*======================================================================================*/
/*! Returns <em>a copy</em> of the given number of bytes read from the current read position. 
    This function will not read beyond bytes that have been written by the stream's 
    write functions even if the stream size is bigger. The read pointer is advanced
    accordingly if all bytes could be read.

    \note The returned value must be deleted by the caller (use delete[]).

    \param length The number of bytes to read.

    \return A copy (on the heap) of the characters read or \e null if not enough bytes 
      are available to read.    
 */
/*======================================================================================*/
unsigned char* CBinaryDataStream::ReadBytes(int length)
{
  // Write routines check that no writing is done beyond stream
  // Reading beyond written data is not allowed
  unsigned char* pData = 0;
    
  if(m_ulStreamReadPos + length <= m_ulStreamWritePos)
  {
    pData = new unsigned char[length];
    (void*)memcpy(pData, &m_pStream[m_ulStreamReadPos], length);
    m_ulStreamReadPos += length;
  }

  return pData;
}

void CBinaryDataStream::ResetReadPointer(void)
{
  m_ulStreamReadPos = 0;
}

/*======================================================================================*/
/*! \return \e true if the read pointer is at the end of all data written to the stream
      or \e false otherwise. Also see ResetReadPointer().
 */
/*======================================================================================*/
bool CBinaryDataStream::IsReadPosAtEnd(void) const
{
  // Write routines check that no writing is done beyond stream
  // Reading beyond written data is not allowed
  return (m_ulStreamReadPos >= m_ulStreamWritePos);
}

/*======================================================================================*/
/*! \return \e true if the write pointer is at the end of data buffer (size determined 
      by constructor) or \e false otherwise.
 */
/*======================================================================================*/
bool CBinaryDataStream::IsWritePosAtEnd(void) const
{
  return (m_ulStreamWritePos >= m_ulStreamSize);
}
