/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "Serializer.h"
#include "BinaryDataStream.h"

/*======================================================================================*/
/*! This helper function delegates bit serialization to the SOPAS_XByte::Serialize() 
    function and writes the result as a fixed length array of #SOPAS_UInt8 to the stream. 
    Look for further limitations there.

    \param rOutputStream The data stream to write the values to.
    \param rValue The XByte to be serialized.

   \return <em>true</em> if the values could be written to the stream. <em>false</em> 
      otherwise (e.g. stream is too small).

    \note The structure of the XByte <em>(rValue)</em> must have been defined before.
      See SOPAS_XByte::AddBitfield(). All bitfields are serialized in the order they
      were added.
 */
/*======================================================================================*/
bool CSerializer::Serialize(CBinaryDataStream& rOutputStream, const SOPAS_XByte& rValue)
{
  // A XByte is an array of UInt8. Deserialize it and delegate bit extraction 
  // to SOPAS_XByte-class.
  bool bRetVal = false;

  SOPAS_UInt16 uiLength = rValue.GetTotalByteLength();
  SOPAS_UInt8* pArray = new SOPAS_UInt8[uiLength]; // auto_ptr not suitable for arrays
  // Serialize bits to array
  if(rValue.Serialize(uiLength, pArray) == true)
  {
    // Serialize array to stream
    bRetVal = SerializeArray(rOutputStream, uiLength, pArray);
  }
  delete[] pArray;

  return bRetVal;
}

/*======================================================================================*/
/*! This function adds up the number of bytes needed to store the according bitfields 
    within the given XByte. XBytes are serialized as fixed arrays so the according
    GetSerializedSizeArray() function is used.
 
    \param rValue The SOPAS_XByte containing the values to be serialized. 

    \return The number of bytes needed to store the given data as a serialized stream.

    \note The structure of the XByte <em>(rValue)</em> must have been defined before.
      See SOPAS_XByte::AddBitfield(). All bitfields are serialized in the order they
      were added.
 */
/*======================================================================================*/
size_t CSerializer::GetSerializedSize(const SOPAS_XByte& rValue) const
{
  // XBytes are serialized as array of USInt8
  SOPAS_UInt16 uiLength = rValue.GetTotalByteLength();

  // The serialization might save some space in later implementations
  // by skipping leading zeroes. Therefore the value might be examined.
  
  // To look at the value the XByte would have to be serialized already.
  // Use the maximum value instead here. Value might be slightly too big 
  // but never too small.
  SOPAS_UInt8* pArray = new SOPAS_UInt8[uiLength]; // auto_ptr not suitable for arrays
  (void*)memset(pArray, 0xFF, sizeof(SOPAS_UInt8) * uiLength);

  size_t uiSerializedSize = GetSerializedSizeArray(uiLength, pArray);
  delete[] pArray; // Cleanup
  return uiSerializedSize;
}

/*======================================================================================*/
/*! This helper function makes use of specialized overloads in the derived classes 
    to write the length indicator and the string itself.

    \param rOutputStream The data stream to write the values to.
    \param rValue The flex string to be serialized.

    \return <em>true</em> if the values could be written to the stream. <em>false</em> 
      otherwise (e.g. stream is too small).
 */
/*======================================================================================*/
bool CSerializer::Serialize(CBinaryDataStream& rOutputStream, 
  const SOPAS_FlexString &rValue)
{
  bool bSuccess = true; // May be set to false below
  
  // Add string length (serialized as UInt16)
  SOPAS_UInt16 uiLength = static_cast<SOPAS_UInt16>(rValue.length());
  bSuccess &= Serialize(rOutputStream, uiLength);

  // Serialize FixString
  SOPAS_FixString fixString(uiLength, rValue);
  bSuccess &= Serialize(rOutputStream, fixString);

  return bSuccess;
}

/*======================================================================================*/
/*! This function adds up the number of bytes needed to store the string length counter
    and the string itself.

    It uses the according overloads of GetSerializedSize() for #SOPAS_UInt16 and 
    SOPAS_FixString. See according inherited implementation of simple types for details.
    
    \param rValue The #SOPAS_FlexString containing the value to be serialized.

    \return The number of bytes needed to store the given data as a serialized stream.

    \note The serialized length may depend on the actual value of <em>rValue</em>. This
      is protocol and implementation specific. So do not use a dummy <em>rValue</em> with
      just the correct type but use the actual value to be serialized instead.
 */
/*======================================================================================*/
size_t CSerializer::GetSerializedSize(const SOPAS_FlexString& rValue) const
{
  // String is serialized as length indicator + string  
  SOPAS_UInt16 uiLength = static_cast<SOPAS_UInt16>(rValue.length());
  SOPAS_FixString fixString(uiLength);
 
  return GetSerializedSize(uiLength) + GetSerializedSize(fixString);
}
