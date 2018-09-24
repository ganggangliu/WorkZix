#ifndef __SOPAS_XBYTE_H__
#define __SOPAS_XBYTE_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include <string>
#include <vector>
#include "SopasBasicTypes.h"

// Forward declarations
class CBinaryDataStream;

/*======================================================================================*/
/*! \ingroup datatypes

    \brief Bitfields for SOPAS serialization and deserialization.
   
    Bitfield types and their memory layout are treated compiler dependent. So SOPAS
    defines an own type for bitfields named XByte. This implementation is a wrapper
    for bitfields that can be serialized and deserialized accordingly.

    A SOPAS_XByte is basically a structure containing signed and unsigned bitfields of
    according bitlength. The structure has to be built by adding named fields (see 
    AddSignedBitfield() and AddUnsignedBitfield()).

    The fields can be signed or unsigned. If you get a value from the bitfield it
    is converted to <em>int</em> or <em>unsigned int</em> respectively. 
    The minimum and maximum values are checked when setting a bitfield's value.

    \note The bitfields must be added in the same order they have to be serialized or
    deserialized.
 */
/*======================================================================================*/
class SOPAS_XByte
{
public:
  SOPAS_XByte(void);
    //!< Creates an empty structure for bitfields.

  bool AddSignedBitfield(const std::string& rName, unsigned int ulWidth);
    //!< Adds a signed bitfield to the structure.
  bool AddUnsignedBitfield(const std::string& rName, unsigned int ulWidth);  
    //!< Adds an unsigned bitfield to the structure.

  bool GetSignedValue(const std::string& rName, int& rValue)  const;
    //!< Gets the value of a signed bitfield contained in this structure.
  bool GetUnsignedValue(const std::string& rName, unsigned int& rValue) const;
    //!< Gets the value of an unsigned bitfield contained in this structure.

  bool SetSignedValue(const std::string& rName, int lValue);
    //!< Sets the value of a signed bitfield contained in this structure.
  bool SetUnsignedValue(const std::string& rName, unsigned int ulValue);
    //!< Sets the value of an unsigned bitfield contained in this structure.

  bool Deserialize(SOPAS_UInt16 uiLength, const SOPAS_UInt8* pArray);
    //!< Helper function to deserialize bitfields from binary data.
  bool Serialize(SOPAS_UInt16 uiLength, SOPAS_UInt8* pArray) const;
    //!< Helper function to serialize bitfields into binary data.

  SOPAS_UInt16 GetTotalByteLength(void) const;
    //!< Gets the number of bytes necessary to store all bitfields within this structure.
  void Clear(void);
    //!< Sets contents of all contained bitfields to zero.

private:
  SOPAS_XByte(const SOPAS_XByte&);
    //!< Do not allow copying
  SOPAS_XByte& operator=(const SOPAS_XByte&);
    //!< Do not allow copying

  //! Contains all information necessary for each bitfield.
  struct SBitfield
  {
    SBitfield(void);
      //!< Default constructor sets everything to zero.
    SBitfield(const std::string& rName, unsigned int value, bool isSigned, unsigned int width);
      //!< Constructor sets all information accordingly.

    std::string coName;
      //!< The name of the bitfield.
    unsigned int ulValue;    
      //!< The value of the bitfield stored as unsigned int.
    bool bIsSigned;
      //!< Signed or unsigned bitfield?
    unsigned int ulWidth;
      //!< Width in bit.
    unsigned int ulMinValue;
      //!< Minimum value of bitfield.
    unsigned int ulMaxValue;
      //!< Maximum value of bitfield.
  };

  // Helper functions
  bool AddBitfield(const std::string& rName, unsigned int ulWidth, bool isSigned);
    //!< Adds a named bitfield to this structure.
  std::vector<SBitfield>::iterator FindBitfield(const std::string& rName);
    //!< Looks up bitfield by given name.
  std::vector<SBitfield>::const_iterator FindBitfield(const std::string& rName) const;
    //!< Looks up bitfield by given name (const).

  std::vector<SBitfield> m_coBitfieldVector;
    //!< Stores all bitfields of this structure.
  unsigned int m_ulTotalBitWidth;
    //!< Total number of bits stored by all bitfields.
};

#endif