/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "AsciiDeserializer.h"
#include <math.h>
#include "AsciiParser.h"
#include "BinaryDataStream.h"

CAsciiDeserializer::CAsciiDeserializer(void)
{
}

/*======================================================================================*/
/*! See GetSimpleType() for details which deserializes all simple types.

    \param rInputStream The data stream to extract the values from.
    \param[out] rResult The deserialized value is written to this pointer.
      Only valid when this function returns <em>true</em>.

    \return <em>true</em> if a value of according type could be extracted from the 
      stream. <em>false</em> otherwise.
 */
/*======================================================================================*/
bool CAsciiDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_Int8& rResult)
{
  // Use template with according data type
  return GetSimpleType(rInputStream, rResult);
}

/*======================================================================================*/
/*! \copydetails CAsciiDeserializer::Deserialize(CBinaryDataStream&,SOPAS_Int8&)        */
/*======================================================================================*/
bool CAsciiDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_Int16& rResult)
{
  // Use template with according data type
  return GetSimpleType(rInputStream, rResult);
}

/*======================================================================================*/
/*! \copydetails CAsciiDeserializer::Deserialize(CBinaryDataStream&,SOPAS_Int8&)        */
/*======================================================================================*/
bool CAsciiDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_Int32& rResult)
{
  // Use template with according data type
  return GetSimpleType(rInputStream, rResult);
}

/*======================================================================================*/
/*! \copydetails CAsciiDeserializer::Deserialize(CBinaryDataStream&,SOPAS_Int8&)        */
/*======================================================================================*/
bool CAsciiDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_UInt8& rResult)
{
  // Use template with according data type
  return GetSimpleType(rInputStream, rResult);
}

/*======================================================================================*/
/*! \copydetails CAsciiDeserializer::Deserialize(CBinaryDataStream&,SOPAS_Int8&)        */
/*======================================================================================*/
bool CAsciiDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_UInt16& rResult)
{
  // Use template with according data type
  return GetSimpleType(rInputStream, rResult);
}

/*======================================================================================*/
/*! \copydetails CAsciiDeserializer::Deserialize(CBinaryDataStream&,SOPAS_Int8&)        */
/*======================================================================================*/
bool CAsciiDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_UInt32& rResult)
{
  // Use template with according data type
  return GetSimpleType(rInputStream, rResult);
}

/*======================================================================================*/
/*! \copydetails CAsciiDeserializer::Deserialize(CBinaryDataStream&,SOPAS_Int8&)        */
/*======================================================================================*/
bool CAsciiDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_Float& rResult)
{
  // Use template with according data type
  return GetSimpleType(rInputStream, rResult);
}

/*======================================================================================*/
/*! \copydetails CAsciiDeserializer::Deserialize(CBinaryDataStream&,SOPAS_Int8&)        */
/*======================================================================================*/
bool CAsciiDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_Double& rResult)
{
  // Use template with according data type
  return GetSimpleType(rInputStream, rResult);
}

/*======================================================================================*/
/*! This function uses the given datatype of <em>pResult</em> to detect the maximum
    number of bytes to be extracted from the stream. This function works for all built-in
    numerical types. 
    
    It tries to deserialize the next value enclosed in spaces. If the value starts with
    a sign CAsciiParser::Parse() is used. Otherwise ConvertFromHexString() is called. This 
    function will fail if the stream contains invalid characters or is too long for the
    given datatype.

    \note If this function fails the read pointer of <em>pStream</em> will have been 
      advanced. Cancel further processing in that case.    

    \param rInputStream The data stream to extract the values from.
    \param[out] rResult The deserialized value is written to this pointer.
      Only valid when this function returns <em>true</em>.

    \return <em>true</em> if a value of according type could be extracted from the 
      stream. <em>false</em> otherwise.
 */
/*======================================================================================*/
template<class T>
bool CAsciiDeserializer::GetSimpleType(CBinaryDataStream& rInputStream, T& rResult)
{
  bool validNumberFound = false;
  rResult = 0; // Initialize to zero

  // Check for leading separator
  if(rInputStream.ReadChar() == ' ')
  {

    // Find end of field (separated by space or end of data)
    int endIndex = rInputStream.FindIndexOfNext(' ');

    // Check if entry is hex or decimal
    int sign = rInputStream.PeekChar(); // Do not change read pointer of stream
    if ((sign == '+') || (sign == '-'))
    {
      // Decimal entry, convert to string first and parse
      const std::string& value = rInputStream.ReadString(endIndex - rInputStream.GetReadIndex());

      // Use template based parser for simple types
      CAsciiParser<T> coParser;
      validNumberFound = coParser.Parse(value.c_str(), rResult);
    }
    else
    {
      // Hex entry, create target byte array of according size and initialize with 0
      // The whole array is reinterpreted afterwards
      unsigned char* pBinaryData = new unsigned char[sizeof(T)];
      memset(pBinaryData, 0, sizeof(T));
      // Convert hex string to binary data
      if (ConvertFromHexString(rInputStream, endIndex, pBinaryData, sizeof(T)) == true)
      {
        // Conversion successful, binaryData contains the binary value.
        // Interpret binary data as according type and copy value        
        rResult = *(reinterpret_cast<T*>(pBinaryData)); // Copy value
        validNumberFound = true;
      }
      delete[] pBinaryData;
    }
  }

  return validNumberFound;
}

/*======================================================================================*/
/*! This function converts an ASCII-hex-string to its binary representation which 
    can be cast to a built-in datatype afterwards. 

    \param rInputStream The data stream to extract the values from.
    \param endIndex The index up to which data should be extracted from the stream.
    \param[in, out] pBinaryData Existing byte array of size <em>binaryLength</em>. 
      Will containing the binary representation of the hex-string when this functions 
      returns <em>true</em>.
    \param binaryLength The length of the binary representation.
      Determines whether data to be extracted isn't too big.

    \return <em>true</em> if a hex-value of according size could be extracted and converted 
      from the stream. <em>false</em> otherwise.
 */
/*======================================================================================*/
bool CAsciiDeserializer::ConvertFromHexString(CBinaryDataStream& rInputStream, int endIndex, unsigned char* pBinaryData, int binaryLength)
{
  bool retVal = false; 
  int startIndex = rInputStream.GetReadIndex();
  // Check for valid index within source data and that resulting binary array has according length            
  if(ceil((double)(endIndex - startIndex) / 2) <= binaryLength)
  {
    // Read bytes from stream (gets a copy)
    const unsigned char* pData = rInputStream.ReadBytes(endIndex - startIndex);
    if(pData != 0)
    {
      retVal = true; // assume correct data, can be set to false by GetNibble

      int hexIndex = 0;
      int shift = 0;
      for (int i = endIndex - startIndex - 1; i >= 0; --i)
      {
        unsigned char nibble;
        retVal &= GetNibble(pData[i], nibble);
        pBinaryData[hexIndex] |= (unsigned char)(nibble << shift);
        hexIndex += (shift >> 2);
        shift ^= 4;
      }
      delete[] pData;
    }
  }
  return retVal;
}

/*======================================================================================*/
/*! Converts one character from hex to its binary value.    

    \param data The hex character to convert.
    \param[out] rNibble The converted value (4-bit nibble). Only valid when this
      function returns <em>true</em>.

    \return <em>true</em> if <em>data</em> contained a valid hex-value.
      <em>false</em> otherwise.
 */
/*======================================================================================*/
bool CAsciiDeserializer::GetNibble(unsigned char data, unsigned char& rNibble)
{
  rNibble = 0;
  bool validHexData = true;
  if ((data >= '0') && (data <= '9'))
  {
    rNibble = (unsigned char)(data - '0');
  }
  else if ((data >= 'A') && (data <= 'F'))
  {
    rNibble = (unsigned char)(data - 'A' + 10);
  }
  else if ((data >= 'a') && (data <= 'f'))
  {
    rNibble = (unsigned char)(data - 'a' + 10);
  }
  else
  {
    validHexData = false;
  }
  return validHexData;
}

/*======================================================================================*/
/*! A fix string is determined by its length but may contain spaces.
    This function tries to read the according number of characters and checks that
    the next variable in the stream is separated by a space.

    \param rInputStream The data stream to extract the values from.
    \param[in, out] rResult The deserialized value is written to this pointer.
      Only valid when this function returns <em>true</em>. The length of the fix string
      is determined by this parameter.

    \return <em>true</em> if a string of according length could be extracted from the 
      stream. <em>false</em> otherwise.
 */
/*======================================================================================*/
bool CAsciiDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_FixString& rResult)
{
  // Fixed string
  bool retVal = false;  

  if(rResult.uiLength > 0)
  {
    // Must start with separator
    if(rInputStream.ReadChar() == ' ')
    {
      // String is directly part of the data, may contain spaces. Extract by length only
      rResult.data = rInputStream.ReadString(static_cast<int>(rResult.uiLength));

      // If string is empty and length not 0 then an error occurred
      retVal = !rResult.data.empty();

      // Check that a separator is behind the string (or end of stream)
      int nextChar = rInputStream.PeekChar();
      retVal &= ((nextChar == -1) || (nextChar == ' '));
    }
  }
  else
  {
    // Empty string just returns true (may be flex string of length 0)
    retVal = true;
  }

  return retVal;
}
