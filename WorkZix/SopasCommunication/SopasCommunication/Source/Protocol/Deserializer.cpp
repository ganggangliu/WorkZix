/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "Deserializer.h"
#include "BinaryDataStream.h"

/*======================================================================================*/
/*! This helper function makes use of specialized overloads in the derived classes 
    to extract a length indicator and the string itself.

    \param rInputStream The data stream to extract the values from.
    \param rResult The flex string the deserialized values are written to. 
      Only valid when this function returns <em>true</em>.

    \return <em>true</em> if a string of according length could be extracted from the 
      stream. <em>false</em> otherwise.
 */
/*======================================================================================*/
bool CDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_FlexString& rResult)
{
  bool bRetVal = false;  

  // Find length indicator (separated by space)
  SOPAS_UInt16 length;
  if (Deserialize(rInputStream, length) == true)
  {
    // Deserialize fixed string using detected length
    SOPAS_FixString coTmpData(length);
    bRetVal = Deserialize(rInputStream, coTmpData);
    rResult = coTmpData.data; // Copy fixed string
  }

  return bRetVal;
}

/*======================================================================================*/
/*! This helper function deserializes a fixed length array and delegates bit extraction
    to the SOPAS_XByte::Deserialize() function. Look for further limitations there.

    \param rInputStream The data stream to extract the values from.
    \param rResult The XByte the deserialized values are written to. 
      Only valid when this function returns <em>true</em>.

    \return <em>true</em> if a XByte with all defined fields could be extracted from the 
      stream. <em>false</em> otherwise.

    \note The structure of the XByte <em>(pResult)</em> must have been defined before.
      See SOPAS_XByte::AddBitfield(). All bitfields are deserialized in the order they
      were created.
 */
/*======================================================================================*/
bool CDeserializer::Deserialize(CBinaryDataStream& rInputStream, SOPAS_XByte& rResult)
{
  // A XByte is an array of UInt8. Deserialize it and delegate bit extraction 
  // to SOPAS_XByte-class.
  bool bRetVal = false;

  SOPAS_UInt16 uiLength = rResult.GetTotalByteLength();
  SOPAS_UInt8* pArray = new SOPAS_UInt8[uiLength]; // auto_ptr not suitable for arrays
  if(DeserializeArray(rInputStream, uiLength, pArray) == true)
  {
    // Extract bits out of array
    bRetVal = rResult.Deserialize(uiLength, pArray);
  }
  delete[] pArray;

  return bRetVal;
}