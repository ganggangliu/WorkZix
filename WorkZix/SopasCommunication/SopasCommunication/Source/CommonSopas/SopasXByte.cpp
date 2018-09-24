/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/
#include "SopasXByte.h"
#include "BinaryDataStream.h"

/*======================================================================================*/
/*! The value is stored as <em>unsigned int</em> internally and casted accordingly.
    The minimum and maximum values are calculated by taking the sign and the bitwidth 
    into account.

    \param rName The name of the bitfield.
    \param value The initial value of the bitfield.
    \param isSigned Signed or unsigned bitfield.
    \param width The width in bit of this bitfield.
 */
/*======================================================================================*/
SOPAS_XByte::SBitfield::SBitfield(const std::string& rName, unsigned int value, 
                                  bool isSigned, unsigned int width)
: coName(rName), ulValue(value), bIsSigned(isSigned), ulWidth(width)
{
  // Store minimum and maximum values as unsigned bitmasks.
  // They are casted accordingly for a range check, depending on signed/unsigned
  if(isSigned == true)
  {
    ulMinValue = 1 << (width - 1);
    ulMaxValue = ulMinValue - 1;
    ulMinValue = ~ulMinValue + 1; // 1-complement, bitmask of 32-bit negative number.
  }
  else
  {
    ulMinValue = 0;
    ulMaxValue = 0xFFFFFFFF >> (32 - width);
  }
}

/*======================================================================================*/
/*! The default constructor is needed for std::vector.
 */
/*======================================================================================*/

SOPAS_XByte::SBitfield::SBitfield(void) : coName(""), ulValue(0), bIsSigned(false), ulWidth(1),
  ulMinValue(0), ulMaxValue(1)
{
}

SOPAS_XByte::SOPAS_XByte(void) : m_ulTotalBitWidth(0)
{
}

/*======================================================================================*/
/*! A byte array of according length is able to contain the serialized
    information of all contained bitfields.

    \return Length of all bitfields in bytes.
 */
/*======================================================================================*/
SOPAS_UInt16 SOPAS_XByte::GetTotalByteLength(void) const
{
  unsigned int ulLength = m_ulTotalBitWidth / 8;
  if((m_ulTotalBitWidth % 8) != 0)
  {
    ulLength++;
  }

  return static_cast<SOPAS_UInt16>(ulLength);
}

/*======================================================================================*/
/*! Creates a signed bitfield with given name and bit width. Access to the bitfield is
    done by using the given name.

    Access to the stored value can be gained by using GetSignedValue() and 
    SetSignedValue().

    \note Using GetUnsignedValue() or SetUnsignedValue() on this bitfield will cause
    an error.

    \param rName The name of the bitfield (case sensitive).
    \param ulWidth The width of the bitfield in bits.

    \return <em>true</em> if the bitfield was successfully added. <em>false</em> if the bit width
      exceeds 32 bit or a bitfield with given name already exists.
 */
/*======================================================================================*/
bool SOPAS_XByte::AddSignedBitfield(const std::string& rName, unsigned int ulWidth)
{
  return AddBitfield(rName, ulWidth, true);
}

/*======================================================================================*/
/*! Creates an unsigned bitfield with given name and bit width. Access to the bitfield is
    done by using the given name.

    Access to the stored value can be gained by using GetUnsignedValue() and 
    SetUnsignedValue().

    \note Using GetSignedValue() or SetSignedValue() on this bitfield will cause
    an error.

    \param rName The name of the bitfield (case sensitive).
    \param ulWidth The width of the bitfield in bits.

    \return <em>true</em> if the bitfield was successfully added. <em>false</em> if the bit width
      exceeds 32 bit or a bitfield with given name already exists.
 */
/*======================================================================================*/
bool SOPAS_XByte::AddUnsignedBitfield(const std::string& rName, unsigned int ulWidth)
{
  return AddBitfield(rName, ulWidth, false);
}

/*======================================================================================*/
/*! Helper function, see AddUnsignedBitfield() and AddSignedBitfield() for details.    

    \param rName The name of the bitfield (case sensitive).
    \param ulWidth The width of the bitfield in bits.
    \param isSigned Signed or unsigned bitfield?

    \return <em>true</em> if the bitfield was successfully added. <em>false</em> if the bit width
      exceeds 32 bit or a bitfield with given name already exists.
 */
/*======================================================================================*/
bool SOPAS_XByte::AddBitfield(const std::string& rName, unsigned int ulWidth, bool isSigned)
{
  bool bRetVal = false;
  // Check for valid bitfield width. Must fit into int/unsigned int
  if((0 < ulWidth) && (ulWidth <= 32))
  {
    // Check that no bitfield with given name has already been created
    std::vector<SBitfield>::iterator coElement = FindBitfield(rName);

    if(coElement == m_coBitfieldVector.end())
    {
      // Element not found, create it
      m_coBitfieldVector.push_back(SBitfield(rName, 0, isSigned, ulWidth));
      m_ulTotalBitWidth += ulWidth;
      bRetVal = true;
    }
  }

  return bRetVal;
}

/*======================================================================================*/
/*! Tries to find the bitfield by comparing the name (case sensitive).
    
    \param rName The name of the bitfield to find.

    \return Iterator to the bitfield found. If no bitfield with given name could be
    found the iterator will point to m_coBitfieldVector.end().
 */
/*======================================================================================*/
std::vector<SOPAS_XByte::SBitfield>::iterator SOPAS_XByte::FindBitfield(const std::string& rName)
{
  std::vector<SBitfield>::iterator iter = m_coBitfieldVector.begin();
  while(iter != m_coBitfieldVector.end())
  {
    if(iter->coName == rName)
    {
      break;
    }
    ++iter;
  }

  return iter;
}

/*======================================================================================*/
/*! Tries to find the bitfield by comparing the name (case sensitive).
    
    \param rName The name of the bitfield to find.

    \return Const iterator to the bitfield found. If no bitfield with given name could be
    found the iterator will point to m_coBitfieldVector.end().
 */
/*======================================================================================*/
std::vector<SOPAS_XByte::SBitfield>::const_iterator SOPAS_XByte::FindBitfield(const std::string& rName) const
{
  std::vector<SBitfield>::const_iterator iter = m_coBitfieldVector.begin();
  while(iter != m_coBitfieldVector.end())
  {
    if(iter->coName == rName)
    {
      break;
    }
    ++iter;
  }

  return iter;
}

/*======================================================================================*/
/*! Tries to find the bitfield and gets the stored value as <em>signed int</em>.
        
    \param rName The name of the bitfield.
    \param[out] rValue The value is stored in this output parameter. <em>The value
      is only valid if this function returns true</em>.

    \return <em>true</em> if a <em>signed</em> bitfield with according name has been 
    found. <em>false</em> otherwise.
 */
/*======================================================================================*/
bool SOPAS_XByte::GetSignedValue(const std::string& rName, int& rValue) const
{
  bool bRetVal = false;

  // Try to find according bitfield and check that it's signed
  std::vector<SBitfield>::const_iterator coElement = FindBitfield(rName);  
  if(coElement != m_coBitfieldVector.end() && coElement->bIsSigned == true)
  {
    // Convert unsigned to signed regarding bit width
    rValue = static_cast<int>(coElement->ulValue);
    bRetVal = true;
  }

  return bRetVal;
}

/*======================================================================================*/
/*! Tries to find the bitfield and gets the stored value as <em>unsigned int</em>.
        
    \param rName The name of the bitfield.
    \param[out] rValue The value is stored in this output parameter. <em>The value
      is only valid if this function returns true</em>.

    \return <em>true</em> if an <em>unsigned</em> bitfield with according name has been 
    found. <em>false</em> otherwise.
 */
/*======================================================================================*/
bool SOPAS_XByte::GetUnsignedValue(const std::string& rName, unsigned int& rValue) const
{
  bool bRetVal = false;

  // Try to find according bitfield and check that it's signed
  std::vector<SBitfield>::const_iterator coElement = FindBitfield(rName);  
  if(coElement != m_coBitfieldVector.end() && coElement->bIsSigned == false)
  {
    // Convert unsigned to signed regarding bit width
    rValue = coElement->ulValue;
    bRetVal = true;
  }

  return bRetVal;
}

/*======================================================================================*/
/*! Tries to find the bitfield and sets its value accordingly.
        
    \param rName The name of the bitfield.
    \param lValue The value is stored to the bitfield.

    \return <em>true</em> if a <em>signed</em> bitfield with according name has been 
    found and the value (signed) will fit into the bit width of the field. 
    <em>false</em> otherwise.
 */
/*======================================================================================*/
bool SOPAS_XByte::SetSignedValue(const std::string& rName, int lValue)
{
  bool bRetVal = false;

  // Try to find according bitfield and check that it's signed
  std::vector<SBitfield>::iterator coElement = FindBitfield(rName);  
  if(coElement != m_coBitfieldVector.end() && coElement->bIsSigned == true)
  {
    // Does value fit into bit width?
    if((static_cast<int>(coElement->ulMinValue) <= lValue) && (lValue <= static_cast<int>(coElement->ulMaxValue)))
    {
      // Just store as unsigned int. Is interpreted accordingly by get-function
      coElement->ulValue = static_cast<unsigned int>(lValue);
      bRetVal = true;
    }
  }

  return bRetVal;
}

/*======================================================================================*/
/*! Tries to find the bitfield and sets its value accordingly.
        
    \param rName The name of the bitfield.
    \param ulValue The value is stored to the bitfield.

    \return <em>true</em> if an <em>unsigned</em> bitfield with according name has been 
    found and the value will fit into the bit width of the field. 
    <em>false</em> otherwise.
 */
/*======================================================================================*/
bool SOPAS_XByte::SetUnsignedValue(const std::string& rName, unsigned int ulValue)
{
  bool bRetVal = false;

  // Try to find according bitfield and check that it's signed
  std::vector<SBitfield>::iterator coElement = FindBitfield(rName);  
  if(coElement != m_coBitfieldVector.end() && coElement->bIsSigned == false)
  {
    // Does value fit into bit width?
    if((coElement->ulMinValue <= ulValue) && (ulValue <= coElement->ulMaxValue))
    {
      // Just store as unsigned int. Is interpreted accordingly by get-function
      coElement->ulValue = ulValue;
      bRetVal = true;
    }
  }

  return bRetVal;
}

/*======================================================================================*/
/*! XBytes are serialized as array of #SOPAS_UInt8. This is just a helper function, use
    \link CSerializer::Serialize(CBinaryDataStream&, const SOPAS_XByte&)
      CSerializer::Serialize()\endlink instead.

    This function writes all bitfields to memory without any gaps so that they can be 
    serialized as array of #SOPAS_UInt8. The bitfields are serialized in
    the order they were added during initialization.
    
    \note The binary data array must be big enough. Use GetTotalByteLength to dimension it
      accordingly.
    \note This implementation is limited to bitfields of 32 bit.
        
    \param uiLength The length of the binary array.
    \param pArray Pointer to the binary array where the bitfields are written to 
    (consecutively without gaps).

    \return <em>true</em> if all bitfields could be written to pArray.
    <em>false</em> otherwise.
 */
/*======================================================================================*/
bool SOPAS_XByte::Serialize(SOPAS_UInt16 uiLength, SOPAS_UInt8* pArray) const
{
  bool bRetVal = false;

  // XByte is serialized as array of USInt so just extract bytes.  
  if(pArray != 0)
  {
    // Clear array for bit operations
    (void*)memset(&pArray[0], 0, sizeof(SOPAS_UInt8) * uiLength);

    // The bytes are stored as Little Endian (low byte first).
    // So we start at the first bytes and insert bits until we are at the end.
    unsigned int ulCurrentByteIndex = 0;
    unsigned int ulCurrentBitIndex = 0; // Bit position within target array
    std::vector<SBitfield>::const_iterator coBitfieldIter = m_coBitfieldVector.begin();
    while((ulCurrentByteIndex < uiLength) && (coBitfieldIter != m_coBitfieldVector.end()))
    {
      // Insert current bitfield into data
      // Negative (signed) data may have leading ones. 
      // They are clipped because masks are applied until field width is reached.
      unsigned int ulValue = coBitfieldIter->ulValue;
      unsigned int ulBitsToInsert = coBitfieldIter->ulWidth;

      while(ulBitsToInsert > 0)
      {
        // Build a mask at current bit position within one byte
        unsigned char ucMask = 0xFF;
        unsigned int ulAvailableBits = 8 - ulCurrentBitIndex;
        unsigned int ulBitsForMask = std::min(ulAvailableBits, ulBitsToInsert);
        // Mask only contains the number of bits to insert.
        ucMask >>= (8 - ulBitsForMask);        
        // Extract bits of value that fit into current byte.
        unsigned char ucClippedValue = static_cast<unsigned char>(ulValue) & ucMask;
        // Shift value so that lower bits can be masked again next time.
        ulValue >>= ulBitsForMask;
        // Masked bits have to be moved to according position
        ucClippedValue <<= ulCurrentBitIndex;

        // Insert bits into array
        pArray[ulCurrentByteIndex] |= ucClippedValue;

        // Total number of bits to be inserted is decreased
        ulBitsToInsert -= ulBitsForMask;
        // Bit position is adjusted. On wrap-around the next byte is selected
        ulCurrentBitIndex += ulBitsForMask;
        if(ulCurrentBitIndex >= 8)
        {
          ulCurrentBitIndex = 0;
          ulCurrentByteIndex++;
        }
      }

      // Advance to next bitfield
      ++coBitfieldIter;
    }
    
    // If all bitfields were inserted into the bitfield iterator should be at the end
    if(coBitfieldIter == m_coBitfieldVector.end())
    {
      bRetVal = true;
    }
  }

  return bRetVal;
}


/*======================================================================================*/
/*! XBytes are serialized as array of #SOPAS_UInt8. This is just a helper function, use
    \link CDeserializer::Deserialize(CBinaryDataStream&,SOPAS_XByte&)
      CDeserializer::Deserialize()\endlink instead.

    This function extracts all bitfields from memory where they are without any gaps 
    (deserialized as #SOPAS_UInt8 array). The bitfields are extracted in the order 
    they were added during initialization.
    
    \note The binary data array must contain enough data for all bitfields.
    \note This implementation is limited to bitfields of 32 bit.
        
    \param uiLength The length of the binary array.
    \param pArray Pointer to the binary array containing all bitfields consecutively in 
    memory without gaps.

    \return <em>true</em> if all bitfields could be extracted from pArray.
    <em>false</em> otherwise.
 */
/*======================================================================================*/
bool SOPAS_XByte::Deserialize(SOPAS_UInt16 uiLength, const SOPAS_UInt8* pArray)
{
  bool bRetVal = false;

  // XByte is serialized as array of USInt so just extract bytes.  
  if(pArray != 0)
  {
    // The bytes are stored as Little Endian (low byte first).
    // So we start at the first bytes and extract bits until we are at the end.
    unsigned int ulCurrentByteIndex = 0;
    unsigned int ulCurrentBitIndex = 0;
    std::vector<SBitfield>::iterator coBitfieldIter = m_coBitfieldVector.begin();
    while((ulCurrentByteIndex < uiLength) && (coBitfieldIter != m_coBitfieldVector.end()))
    {
      // Extract current bitfield out of data
      unsigned int ulValue = 0;
      unsigned int ulBitsToExtract = coBitfieldIter->ulWidth;
      unsigned int ulTargetBit = 0;

      while(ulBitsToExtract > 0)
      {
        // Build a mask at current bit position within one byte
        unsigned char ucMask = 0xFF;
        unsigned int ulAvailableBits = 8 - ulCurrentBitIndex;
        unsigned int ulBitsForMask = std::min(ulAvailableBits, ulBitsToExtract);
        // Mask only contains the number of bits to extract.
        ucMask >>= (8 - ulBitsForMask);
        // Bits have to be moved to according position
        ucMask <<= ulCurrentBitIndex;

        // Apply byte mask and shift accordingly to get partial result of this byte
        unsigned int ulByteValue = ((pArray[ulCurrentByteIndex] & ucMask) >> ulCurrentBitIndex);
        ulValue |= (ulByteValue << ulTargetBit);
        ulTargetBit += ulBitsForMask;

        // Total number of bits to be extracted is decreased
        ulBitsToExtract -= ulBitsForMask;
        // Bit position is adjusted. On wrap-around the next byte is selected
        ulCurrentBitIndex += ulBitsForMask;
        if(ulCurrentBitIndex >= 8)
        {
          ulCurrentBitIndex = 0;
          ulCurrentByteIndex++;
        }
      }

      // Assign extracted value. Check sign:
      if((coBitfieldIter->bIsSigned) && (ulValue > coBitfieldIter->ulMaxValue))
      {
        // Negative values: Extend leading 1 for int32
        unsigned int ulOnesMask = 0xFFFFFFFF;
        ulOnesMask >>= coBitfieldIter->ulWidth;
        ulOnesMask <<= coBitfieldIter->ulWidth;
        ulValue = ulOnesMask | ulValue;
      }
      coBitfieldIter->ulValue = ulValue;

      // Advance to next bitfield
      ++coBitfieldIter;
    }

    // If all bitfields were extracted the bitfield iterator should be at the end
    if(coBitfieldIter == m_coBitfieldVector.end())
    {
      bRetVal = true;
    }
  }  

  return bRetVal;
}

void SOPAS_XByte::Clear(void)
{
  for(std::vector<SBitfield>::iterator iter = m_coBitfieldVector.begin(); 
    iter != m_coBitfieldVector.end(); ++iter)
  {
    iter->ulValue = 0;    
  }
}