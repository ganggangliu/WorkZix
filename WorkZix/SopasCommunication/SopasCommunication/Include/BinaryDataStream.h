#ifndef __BINARY_DATA_STREAM_H__
#define __BINARY_DATA_STREAM_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include <string>

/*======================================================================================*/
/*! \brief Stream for binary data storage

    \ingroup common

    This class stores binary data as a stream. It provides functions for writing
    different types of data to the stream and reading it from the stream. The class 
    manages a read and a write pointer which are advanced accordingly when reading
    or writing data to the stream.

    So this class helps to easily deserialize and serialize data and write it to
    or read it from a communication channel.

    The class is also secured against reading or writing beyond the end of the stream.
 */
/*======================================================================================*/
class CBinaryDataStream
{
public:
  explicit CBinaryDataStream(unsigned int size);
    //!< Creates a stream of given size.
  ~CBinaryDataStream(void);
    //!< Deletes internal buffer.

  bool WriteToStream(const unsigned char data);
    //!< Writes a single character to the stream.
  bool WriteToStream(const unsigned char* pData, int length);
    //!< Writes a character array of given length to the stream.
  bool WriteToStream(const std::string& rSrc);
    //!< Writes a string to the stream.
  bool WriteToStream(const CBinaryDataStream& rOtherStream);
    //!< Writes another stream to this stream.

  unsigned int GetReadIndex(void) const;
    //!< Gets current read pointer position.
  bool IsReadPosAtEnd(void) const;
    //!< Checks if all data contained in the stream has been read.
  bool IsWritePosAtEnd(void) const;
    //!< Checks if more data can be written to the stream.

  std::string ReadString(int length);
    //!< Reads a string of given length from the stream.
  int ReadChar(void);
    //!< Reads a single character from the stream.
  unsigned char* ReadBytes(int length);
    //!< Reads a character array of given length from the stream.
  int PeekChar(void) const;
    //!< Looks at the next character that will be read.
  unsigned int FindIndexOfNext(const unsigned char c) const;  
    //!< Finds next read index of given character.

  unsigned const char* GetData(void) const;
    //!< Gets read access to internal buffer.
  unsigned int GetUsedLength(void) const;
    //!< Gets number of bytes written to this stream.
  void ResetReadPointer(void);
    //!< Sets read pointer to the start of the stream.

private:
  CBinaryDataStream(const CBinaryDataStream&);
    //!< Do not allow copying
  CBinaryDataStream& operator=(const CBinaryDataStream&);
    //!< Do not allow copying

  unsigned char *const m_pStream;
    //!< The underlying buffer of the stream.
  const unsigned int m_ulStreamSize;
    //!< The size of the stream.
  unsigned int m_ulStreamWritePos;
    //!< The current write position of the stream.
  unsigned int m_ulStreamReadPos;
    //!< The current read position of the stream.
};

#endif