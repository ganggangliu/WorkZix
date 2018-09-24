/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "BinarySerializer.h"
#include "BinaryDataStream.h"
#include <memory>

CBinarySerializer::CBinarySerializer(void)
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
bool CBinarySerializer::Serialize(CBinaryDataStream& rOutputStream, SOPAS_Int8 value)
{
  // Use template function with according data type
  return SerializeSimpleType(rOutputStream, value);
}

/*======================================================================================*/
/*! \copydetails CBinarySerializer::Serialize(CBinaryDataStream&,SOPAS_Int8)             */
/*======================================================================================*/
bool CBinarySerializer::Serialize(CBinaryDataStream& rOutputStream, SOPAS_Int16 value)
{
  // Use template function with according data type
  return SerializeSimpleType(rOutputStream, value);
}

/*======================================================================================*/
/*! \copydetails CBinarySerializer::Serialize(CBinaryDataStream&,SOPAS_Int8)             */
/*======================================================================================*/
bool CBinarySerializer::Serialize(CBinaryDataStream& rOutputStream, SOPAS_Int32 value)
{
  // Use template function with according data type
  return SerializeSimpleType(rOutputStream, value);
}

/*======================================================================================*/
/*! \copydetails CBinarySerializer::Serialize(CBinaryDataStream&,SOPAS_Int8)             */
/*======================================================================================*/
bool CBinarySerializer::Serialize(CBinaryDataStream& rOutputStream, SOPAS_UInt8 value)
{
  // Use template function with according data type
  return SerializeSimpleType(rOutputStream, value);
}

/*======================================================================================*/
/*! \copydetails CBinarySerializer::Serialize(CBinaryDataStream&,SOPAS_Int8)             */
/*======================================================================================*/
bool CBinarySerializer::Serialize(CBinaryDataStream& rOutputStream, SOPAS_UInt16 value)
{
  // Use template function with according data type
  return SerializeSimpleType(rOutputStream, value);
}

/*======================================================================================*/
/*! \copydetails CBinarySerializer::Serialize(CBinaryDataStream&,SOPAS_Int8)             */
/*======================================================================================*/
bool CBinarySerializer::Serialize(CBinaryDataStream& rOutputStream, SOPAS_UInt32 value)
{
  // Use template function with according data type
  return SerializeSimpleType(rOutputStream, value);
}

/*======================================================================================*/
/*! \copydetails CBinarySerializer::Serialize(CBinaryDataStream&,SOPAS_Int8)             */
/*======================================================================================*/
bool CBinarySerializer::Serialize(CBinaryDataStream& rOutputStream, SOPAS_Float value)
{
  // Use template function with according data type
  return SerializeSimpleType(rOutputStream, value);
}

/*======================================================================================*/
/*! \copydetails CBinarySerializer::Serialize(CBinaryDataStream&,SOPAS_Int8)             */
/*======================================================================================*/
bool CBinarySerializer::Serialize(CBinaryDataStream& rOutputStream, SOPAS_Double value)
{
  // Use template function with according data type
  return SerializeSimpleType(rOutputStream, value);
}

/*======================================================================================*/
/*! This function uses the given datatype of \e value to detect the number of bytes to 
    be written to the stream. A binary representation of the value is calculated,
    converted to Big Endian and written to the stream. The string will have the maximum 
    possible length of the data type i.e. a #SOPAS_Int16 with value 1 will be written 
    as 0x0001.
    
    \note If this function fails the write pointer of <em>pStream</em> will have been 
      advanced. Cancel further processing in that case. 

    \param rOutputStream The data stream to write the value to.
    \param value The value to be written to the stream.

    \return \e true if the value could be written to the stream. \e false otherwise 
      (e.g. stream is too small).
 */
/*======================================================================================*/
template<class T>
bool CBinarySerializer::SerializeSimpleType(CBinaryDataStream& rOutputStream, T value)
{    
  // Convert value to binary data bytes. Convert Endian representation
  bool bSuccess = true;
  unsigned char* pValue = reinterpret_cast<unsigned char*>(&value);
  for(int i = 0; i < sizeof(T); ++i)
  {
    bSuccess &= rOutputStream.WriteToStream(pValue[sizeof(T) - i - 1]);
  }

  return bSuccess;
}

/*======================================================================================*/
/*! A fix string is determined by its length but may contain spaces.
    This function writes the string to the stream. The length is defined by the
    #SOPAS_FixString object. If the actual character data is shorter it is padded with
    trailing zero characters (binary).
    
    \param rOutputStream The data stream to write the value to.
    \param rValue The value to be written to the stream.

    \return \e true if the value could be written to the stream. \e false otherwise 
      (e.g. stream is too small).
 */
/*======================================================================================*/
bool CBinarySerializer::Serialize(CBinaryDataStream& rOutputStream, 
  const SOPAS_FixString &rValue)
{
  bool bSuccess = true; // May be set to false below

  if(rValue.uiLength > 0)
  {
    // Length is limited to FixString length
    bSuccess &= rOutputStream.WriteToStream(rValue.GetClippedString()); 
    // Add 0 if data is too short
    int lLengthDifference = static_cast<int>(rValue.uiLength - rValue.data.length());
    for(int i = 0; i < lLengthDifference; ++i)
    {
      bSuccess &= rOutputStream.WriteToStream(static_cast<const unsigned char>(0));
    }
  }
  
  return bSuccess;
}

/*======================================================================================*/
/*! The value is calculated easily because this serializer implementation always 
    uses the maximum number of bytes for each datatype. So the actual value is not 
    needed for COLA-B, only the type is. 

    \param value The value to be serialized.

    \return The number of bytes needed to store the given data as a serialized stream.
 */
/*======================================================================================*/
template<class T>
size_t CBinarySerializer::GetSimpleSerializedSize(T) const
{
  // Binary serialization uses the exact amount of storage in memory.
  // No separators are used because size of each variable is known.

  return sizeof(T);
}

/*======================================================================================*/
/*! See GetSerializedSize() for details which calculates the needed size for all 
    simple types.

    \param value The value to be serialized.

    \return The number of bytes needed to store the given data as a serialized stream.

    \note The serialized length may depend on the actual value of \e value for other
      protocols. So do not use a dummy value with just the correct type but use the 
      actual value to be serialized instead.
 */
/*======================================================================================*/
size_t CBinarySerializer::GetSerializedSize(SOPAS_Int8 value) const
{
  // Use template with according data type
  return GetSimpleSerializedSize(value);
}

/*======================================================================================*/
/*! \copydetails CBinarySerializer::GetSerializedSize(SOPAS_Int8)const                   */
/*======================================================================================*/
size_t CBinarySerializer::GetSerializedSize(SOPAS_Int16 value) const
{
  // Use template with according data type
  return GetSimpleSerializedSize(value);
}

/*======================================================================================*/
/*! \copydetails CBinarySerializer::GetSerializedSize(SOPAS_Int8)const                   */
/*======================================================================================*/
size_t CBinarySerializer::GetSerializedSize(SOPAS_Int32 value) const
{
  // Use template with according data type
  return GetSimpleSerializedSize(value);
}

/*======================================================================================*/
/*! \copydetails CBinarySerializer::GetSerializedSize(SOPAS_Int8)const                   */
/*======================================================================================*/
size_t CBinarySerializer::GetSerializedSize(SOPAS_UInt8 value) const
{
  // Use template with according data type
  return GetSimpleSerializedSize(value);
}

/*======================================================================================*/
/*! \copydetails CBinarySerializer::GetSerializedSize(SOPAS_Int8)const                   */
/*======================================================================================*/
size_t CBinarySerializer::GetSerializedSize(SOPAS_UInt16 value) const
{
  // Use template with according data type
  return GetSimpleSerializedSize(value);
}

/*======================================================================================*/
/*! \copydetails CBinarySerializer::GetSerializedSize(SOPAS_Int8)const                   */
/*======================================================================================*/
size_t CBinarySerializer::GetSerializedSize(SOPAS_UInt32 value) const
{
  // Use template with according data type
  return GetSimpleSerializedSize(value);
}

/*======================================================================================*/
/*! \copydetails CBinarySerializer::GetSerializedSize(SOPAS_Int8)const                   */
/*======================================================================================*/
size_t CBinarySerializer::GetSerializedSize(SOPAS_Float value) const
{
  // Use template with according data type
  return GetSimpleSerializedSize(value);
}

/*======================================================================================*/
/*! \copydetails CBinarySerializer::GetSerializedSize(SOPAS_Int8)const                   */
/*======================================================================================*/
size_t CBinarySerializer::GetSerializedSize(SOPAS_Double value) const
{
  // Use template with according data type
  return GetSimpleSerializedSize(value);
}

/*======================================================================================*/
/*! It will return the number of characters needed for the fix string.

    \param rValue The string to be serialized.

    \return The number of bytes needed to store the given data as a serialized stream.

    \note The serialized length depends on the actual value of \e value. So do not use 
      a dummy value with just the correct type but use the actual value to be serialized 
      instead.
 */
/*======================================================================================*/
size_t CBinarySerializer::GetSerializedSize(const SOPAS_FixString& rValue) const
{
  return rValue.uiLength;
}

