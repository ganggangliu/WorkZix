/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "BinaryDeserializer.h"
#include <math.h>
#include "AsciiParser.h"
#include "BinaryDataStream.h"

CBinaryDeserializer::CBinaryDeserializer(void)
{
}

/*======================================================================================*/
/*! See GetSimpleType() for details which deserializes all simple types.

    \param rInputStream The data stream to extract the values from.
    \param[out] rResult The deserialized value is written to this variable.
      Only valid when this function returns <em>true</em>.

    \return <em>true</em> if a value of according type could be extracted from the 
      stream. <em>false</em> otherwise.
 */
/*======================================================================================*/
bool CBinaryDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_Int8& rResult)
{
  // Use template with according data type
  return GetSimpleType(rInputStream, rResult);
}

/*======================================================================================*/
/*! \copydetails CBinaryDeserializer::Deserialize(CBinaryDataStream&,SOPAS_Int8&)        */
/*======================================================================================*/
bool CBinaryDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_Int16& rResult)
{
  // Use template with according data type
  return GetSimpleType(rInputStream, rResult);
}

/*======================================================================================*/
/*! \copydetails CBinaryDeserializer::Deserialize(CBinaryDataStream&,SOPAS_Int8&)        */
/*======================================================================================*/
bool CBinaryDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_Int32& rResult)
{
  // Use template with according data type
  return GetSimpleType(rInputStream, rResult);
}

/*======================================================================================*/
/*! \copydetails CBinaryDeserializer::Deserialize(CBinaryDataStream&,SOPAS_Int8&)        */
/*======================================================================================*/
bool CBinaryDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_UInt8& rResult)
{
  // Use template with according data type
  return GetSimpleType(rInputStream, rResult);
}

/*======================================================================================*/
/*! \copydetails CBinaryDeserializer::Deserialize(CBinaryDataStream&,SOPAS_Int8&)        */
/*======================================================================================*/
bool CBinaryDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_UInt16& rResult)
{
  // Use template with according data type
  return GetSimpleType(rInputStream, rResult);
}

/*======================================================================================*/
/*! \copydetails CBinaryDeserializer::Deserialize(CBinaryDataStream&,SOPAS_Int8&)        */
/*======================================================================================*/
bool CBinaryDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_UInt32& rResult)
{
  // Use template with according data type
  return GetSimpleType(rInputStream, rResult);
}

/*======================================================================================*/
/*! \copydetails CBinaryDeserializer::Deserialize(CBinaryDataStream&,SOPAS_Int8&)        */
/*======================================================================================*/
bool CBinaryDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_Float& rResult)
{
  // Use template with according data type
  return GetSimpleType(rInputStream, rResult);
}

/*======================================================================================*/
/*! \copydetails CBinaryDeserializer::Deserialize(CBinaryDataStream&,SOPAS_Int8&)        */
/*======================================================================================*/
bool CBinaryDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_Double& rResult)
{
  // Use template with according data type
  return GetSimpleType(rInputStream, rResult);
}

/*======================================================================================*/
/*! This function uses the given datatype of <em>pResult</em> to detect the 
    number of bytes to be extracted from the stream. This function works for all built-in
    numerical types. This function will only fail if not enough data could be read from 
    the stream for the given type.

    \note If this function fails the read pointer of <em>pStream</em> will have been 
      advanced. Cancel further processing in that case.    

    \param rInputStream The data stream to extract the values from.
    \param[out] rResult The deserialized value is written to this variable.
      Only valid when this function returns <em>true</em>.

    \return <em>true</em> if a value of according type could be extracted from the 
      stream. <em>false</em> otherwise.
 */
/*======================================================================================*/
template<class T>
bool CBinaryDeserializer::GetSimpleType(CBinaryDataStream& rInputStream, T& rResult)
{
  bool bRetVal = false;
  // Hex entry, create target byte array of according size and initialize with 0
  // The whole array is reinterpreted afterwards
  std::auto_ptr<unsigned char> pBinaryData(rInputStream.ReadBytes(sizeof(T)));
  unsigned char* pData = pBinaryData.get();    
  if(pData != 0)
  {
    // Convert from Big Endian (SOPAS) to Little Endian (Intel)
    unsigned char pDataReversed[sizeof(T)];
    for(int i = 0; i < sizeof(T); ++i)
    {
      pDataReversed[i] = pData[sizeof(T) - i - 1];
    }
    rResult = *(reinterpret_cast<T*>(pDataReversed)); // Copy value
    bRetVal = true;
  }

  return bRetVal;
}

/*======================================================================================*/
/*! A fix string is determined by its length but may contain spaces.
    This function tries to read the according number of characters and checks that
    the next variable in the stream is separated by a space.

    \param rInputStream The data stream to extract the values from.
    \param[in, out] rResult The deserialized value is written to this variable.
      Only valid when this function returns <em>true</em>. The length of the fix string
      is determined by this parameter.

    \return <em>true</em> if a string of according length could be extracted from the 
      stream. <em>false</em> otherwise.
 */
/*======================================================================================*/
bool CBinaryDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_FixString& rResult)
{
  // Fixed string
  bool bRetVal = false;  

  if(rResult.uiLength > 0)
  {
      // String is directly part of the data, may contain spaces. Extract by length only
      rResult.data = rInputStream.ReadString(static_cast<int>(rResult.uiLength));

      // If string is empty and length not 0 then an error occurred
      bRetVal = !rResult.data.empty();
  }
  else
  {
    // Empty string just returns true (may be flex string of length 0)
    bRetVal = true;
  }

  return bRetVal;
}
