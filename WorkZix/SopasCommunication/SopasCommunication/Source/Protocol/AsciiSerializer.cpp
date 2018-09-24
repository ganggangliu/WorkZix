/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "AsciiSerializer.h"
#include "BinaryDataStream.h"
#include <memory>

CAsciiSerializer::CAsciiSerializer(void)
{
}

/*======================================================================================*/
/*! See SerializeSimpleType() for details which serializes all simple types.

    \param rOutputStream The data stream to write the value to.
    \param value The value to be written to the stream.

    \return \e true if the value could be written to the stream. \e false otherwise 
      (e.g. stream is too small).
 */
/*======================================================================================*/
bool CAsciiSerializer::Serialize(CBinaryDataStream& rOutputStream, SOPAS_Int8 value)
{
  // Use template function with according data type
  return SerializeSimpleType(rOutputStream, value);
}

/*======================================================================================*/
/*! \copydetails CAsciiSerializer::Serialize(CBinaryDataStream&,SOPAS_Int8)             */
/*======================================================================================*/
bool CAsciiSerializer::Serialize(CBinaryDataStream& rOutputStream, SOPAS_Int16 value)
{
  // Use template function with according data type
  return SerializeSimpleType(rOutputStream, value);
}

/*======================================================================================*/
/*! \copydetails CAsciiSerializer::Serialize(CBinaryDataStream&,SOPAS_Int8)             */
/*======================================================================================*/
bool CAsciiSerializer::Serialize(CBinaryDataStream& rOutputStream, SOPAS_Int32 value)
{
  // Use template function with according data type
  return SerializeSimpleType(rOutputStream, value);
}

/*======================================================================================*/
/*! \copydetails CAsciiSerializer::Serialize(CBinaryDataStream&,SOPAS_Int8)             */
/*======================================================================================*/
bool CAsciiSerializer::Serialize(CBinaryDataStream& rOutputStream, SOPAS_UInt8 value)
{
  // Use template function with according data type
  return SerializeSimpleType(rOutputStream, value);
}

/*======================================================================================*/
/*! \copydetails CAsciiSerializer::Serialize(CBinaryDataStream&,SOPAS_Int8)             */
/*======================================================================================*/
bool CAsciiSerializer::Serialize(CBinaryDataStream& rOutputStream, SOPAS_UInt16 value)
{
  // Use template function with according data type
  return SerializeSimpleType(rOutputStream, value);
}

/*======================================================================================*/
/*! \copydetails CAsciiSerializer::Serialize(CBinaryDataStream&,SOPAS_Int8)             */
/*======================================================================================*/
bool CAsciiSerializer::Serialize(CBinaryDataStream& rOutputStream, SOPAS_UInt32 value)
{
  // Use template function with according data type
  return SerializeSimpleType(rOutputStream, value);
}

/*======================================================================================*/
/*! \copydetails CAsciiSerializer::Serialize(CBinaryDataStream&,SOPAS_Int8)             */
/*======================================================================================*/
bool CAsciiSerializer::Serialize(CBinaryDataStream& rOutputStream, SOPAS_Float value)
{
  // Use template function with according data type
  return SerializeSimpleType(rOutputStream, value);
}

/*======================================================================================*/
/*! \copydetails CAsciiSerializer::Serialize(CBinaryDataStream&,SOPAS_Int8)             */
/*======================================================================================*/
bool CAsciiSerializer::Serialize(CBinaryDataStream& rOutputStream, SOPAS_Double value)
{
  // Use template function with according data type
  return SerializeSimpleType(rOutputStream, value);
}

/*======================================================================================*/
/*! This function uses the given datatype of \e value to detect the number of bytes to 
    be written to the stream. A binary representation of the value is calculated
    and written as hex string to the stream. The string will have the maximum possible 
    length of the data type i.e. a #SOPAS_Int16 with value 1 will be written as "0001".    
    
    See WriteHexStringToStream() for more details.

    \note If this function fails the write pointer of <em>pStream</em> will have been 
      advanced. Cancel further processing in that case. 

    \param rOutputStream The data stream to write the value to.
    \param value The value to be written to the stream.

    \return \e true if the value could be written to the stream. \e false otherwise 
      (e.g. stream is too small).
 */
/*======================================================================================*/
template<class T>
bool CAsciiSerializer::SerializeSimpleType(CBinaryDataStream& rOutputStream, T value)
{    
  // Convert value to binary data bytes
  // Stream has needed size, return value is ignored here
  CBinaryDataStream binaryStream(sizeof(T));
  (void)binaryStream.WriteToStream(reinterpret_cast<unsigned char*>(&value), sizeof(T));
  
  return WriteHexStringToStream(rOutputStream, binaryStream);
}

/*======================================================================================*/
/*! A fix string is determined by its length but may contain spaces.
    This function writes the string to the stream. The length is defined by the
    #SOPAS_FixString object. If the actual character data is shorter it is padded with
    trailing zero characters (binary).
    
    \note The string should not contain binary characters as COLA-A-protocol usually
      uses an ASCII-framing. This function will fail if the string contains the 
      framing characters 0x02 (STX) or 0x03 (ETX).

    \param rOutputStream The data stream to write the value to.
    \param rValue The value to be written to the stream.

    \return \e true if the value could be written to the stream. \e false otherwise 
      (e.g. stream is too small).
 */
/*======================================================================================*/
bool CAsciiSerializer::Serialize(CBinaryDataStream& rOutputStream, 
  const SOPAS_FixString &rValue)
{
  bool bSuccess = true; // May be set to false below

  if(rValue.uiLength > 0)
  {
    // Check that no invalid characters are contained (i.e. STX and ETX)
    if((rValue.data.find(0x02) == std::string::npos) && (rValue.data.find(0x03) == std::string::npos))
    {
      // Add separator and string
      bSuccess &= rOutputStream.WriteToStream(" ");      
      // Length is limited to FixString length
      bSuccess &= rOutputStream.WriteToStream(rValue.GetClippedString()); 
      // Add 0 if data is too short
      int lLengthDifference = static_cast<int>(rValue.uiLength - rValue.data.length());
      for(int i = 0; i < lLengthDifference; ++i)
      {
        bSuccess &= rOutputStream.WriteToStream(static_cast<const unsigned char>(0));
      }
    }
    else
    {
      bSuccess = false;
    }
  }
  
  return bSuccess;
}

/*======================================================================================*/
/*! This function writes binary data to the stream using byte order needed by SOPAS 
    (Big Endian).

    \param rOutputStream The data stream to write the value to.
    \param rBinaryStream Binary representation of the value to write. All bytes written
      to this stream are converted to a hex string.

    \return \e true if the value could be written to the stream. \e false otherwise 
      i.e. \e pStream is too small.
 */
/*======================================================================================*/
bool CAsciiSerializer::WriteHexStringToStream(CBinaryDataStream& rOutputStream, const CBinaryDataStream& rBinaryStream)
{
  static const unsigned char hexDigitArray[] = {'0', '1', '2', '3', '4', '5', '6',
    '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

  // Hex string takes two characters per byte
  // Use complete space for datatype, easier to implement. Also add separator  
  int hexDataLength = (rBinaryStream.GetUsedLength() * 2) + 1;
  unsigned char* pHexDataBuffer = new unsigned char[hexDataLength]; // auto_ptr not suitable for arrays
  
  // Build hexstring and convert from Little Endian (Intel) to Big Endian (Sopas)
  int dstIndex = hexDataLength - 1;
  const unsigned char* pBinaryData = rBinaryStream.GetData();  
  for(unsigned int i = 0; i < rBinaryStream.GetUsedLength(); ++i)
  {
    int index = pBinaryData[i] & 0x0F;
    pHexDataBuffer[dstIndex--] = hexDigitArray[index];
    index = (pBinaryData[i] >> 4) & 0x0F;
    pHexDataBuffer[dstIndex--] = hexDigitArray[index];
  }
  // Add separator
  pHexDataBuffer[dstIndex] = ' ';

  // Copy hexstring to stream and return success
  bool bRetVal = rOutputStream.WriteToStream(pHexDataBuffer, hexDataLength);
  delete[] pHexDataBuffer; // Cleanup
  return bRetVal;
}

/*======================================================================================*/
/*! The value is calculated easily because this serializer implementation always 
    uses the maximum number of bytes for each datatype. Also COLA-A inserts one separator. 
    So the actual value is not needed for this implementation (only the type is). 
    This might change in later releases so it's better not to supply a dummy value here.    

    \param value The value to be serialized.

    \return The number of bytes needed to store the given data as a serialized stream.
 */
/*======================================================================================*/
template<class T>
size_t CAsciiSerializer::GetSimpleSerializedSize(T) const
{
  // ASCII serialization is done by creating a hex string for
  // all data bytes (including leading zeros).
  // A separator is added for each parameter

  // The parameter value is unused right now but would be needed
  // if serialization is done without leading zeros.
  return (sizeof(T) * 2) + 1;
}

/*======================================================================================*/
/*! See GetSerializedSize() for details which calculates the needed size for all 
    simple types.

    \param value The value to be serialized.

    \return The number of bytes needed to store the given data as a serialized stream.

    \note The serialized length may depend on the actual value of \e value. This
      is protocol and implementation specific. So do not use a dummy value with
      just the correct type but use the actual value to be serialized instead.
 */
/*======================================================================================*/
size_t CAsciiSerializer::GetSerializedSize(SOPAS_Int8 value) const
{
  // Use template with according data type
  return GetSimpleSerializedSize(value);
}

/*======================================================================================*/
/*! \copydetails CAsciiSerializer::GetSerializedSize(SOPAS_Int8)const                   */
/*======================================================================================*/
size_t CAsciiSerializer::GetSerializedSize(SOPAS_Int16 value) const
{
  // Use template with according data type
  return GetSimpleSerializedSize(value);
}

/*======================================================================================*/
/*! \copydetails CAsciiSerializer::GetSerializedSize(SOPAS_Int8)const                   */
/*======================================================================================*/
size_t CAsciiSerializer::GetSerializedSize(SOPAS_Int32 value) const
{
  // Use template with according data type
  return GetSimpleSerializedSize(value);
}

/*======================================================================================*/
/*! \copydetails CAsciiSerializer::GetSerializedSize(SOPAS_Int8)const                   */
/*======================================================================================*/
size_t CAsciiSerializer::GetSerializedSize(SOPAS_UInt8 value) const
{
  // Use template with according data type
  return GetSimpleSerializedSize(value);
}

/*======================================================================================*/
/*! \copydetails CAsciiSerializer::GetSerializedSize(SOPAS_Int8)const                   */
/*======================================================================================*/
size_t CAsciiSerializer::GetSerializedSize(SOPAS_UInt16 value) const
{
  // Use template with according data type
  return GetSimpleSerializedSize(value);
}

/*======================================================================================*/
/*! \copydetails CAsciiSerializer::GetSerializedSize(SOPAS_Int8)const                   */
/*======================================================================================*/
size_t CAsciiSerializer::GetSerializedSize(SOPAS_UInt32 value) const
{
  // Use template with according data type
  return GetSimpleSerializedSize(value);
}

/*======================================================================================*/
/*! \copydetails CAsciiSerializer::GetSerializedSize(SOPAS_Int8)const                   */
/*======================================================================================*/
size_t CAsciiSerializer::GetSerializedSize(SOPAS_Float value) const
{
  // Use template with according data type
  return GetSimpleSerializedSize(value);
}

/*======================================================================================*/
/*! \copydetails CAsciiSerializer::GetSerializedSize(SOPAS_Int8)const                   */
/*======================================================================================*/
size_t CAsciiSerializer::GetSerializedSize(SOPAS_Double value) const
{
  // Use template with according data type
  return GetSimpleSerializedSize(value);
}

/*======================================================================================*/
/*! It will return the number of characters including separator unless the string is
    empty. In that case the function will return 0. This is necessary because 
    #SOPAS_FlexString might contain an empty string wich would not be serialized.

    \param rValue The string to be serialized.

    \return The number of bytes needed to store the given data as a serialized stream.

    \note The serialized length depends on the actual value of \e value. So do not use 
      a dummy value with just the correct type but use the actual value to be serialized 
      instead.
 */
/*======================================================================================*/
size_t CAsciiSerializer::GetSerializedSize(const SOPAS_FixString& rValue) const
{
  // String is serialized using whole length. Separator is added
  size_t ulLength = 0;

  if(rValue.uiLength > 0)
  {
    // Separator is only added if string isn't empty
    ulLength = 1 + rValue.uiLength;
  }
  
  return ulLength;
}

