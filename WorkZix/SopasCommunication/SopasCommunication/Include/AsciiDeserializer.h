#ifndef __ASCII_DESERIALIZER_H__
#define __ASCII_DESERIALIZER_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "Deserializer.h"

/*======================================================================================*/
/*! \brief A deserializer for COLA-A-protocol

    \ingroup common

    This class implements a deserializer that is used by CColaAProtocol to
    deserialize values of device answers. To decode an answer (needed for 
    CProtocol::ReadVariable() and CProtocol::InvokeMethod()) get the according 
    deserializer of the selected protocol by calling CProtocol::GetDeserializer(). 
    The COLA-A-protocol will return an instance of this class.

    The COLA-A-protocol uses separators (space) between each variable. The length
    of a variable may depend on its value so only the separators are used
    to detect the beginning of the next variable.

    This class supports hex notation ("124A") as well as decimal notation ("+4682").
    See CAsciiParser for limitations.

    \note 64-bit integers are not supported.

    \note All deserializer functions return a boolean value to indicate a successful 
      deserialization. If an error occured during extracting the read pointer of the 
      CBinaryDataStream could have been advanced already. The remaining data cannot be 
      extracted in that case and the result value will be invalid. Also all remaining
      accesses to the data stream will be incorrect. So further processing should be 
      stopped if any call to Deserialize() fails.
 */
/*======================================================================================*/
class CAsciiDeserializer : public CDeserializer
{
public:
  CAsciiDeserializer(void);
    //!< No initialization necessary.

  using CDeserializer::Deserialize; // Un-hide base class implementation

  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_Int8& rResult);
     //!< Deserialize a #SOPAS_Int8.
  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_Int16& rResult);  
    //!< Deserialize a #SOPAS_Int16.
  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_Int32& rResult);
    //!< Deserialize a #SOPAS_Int32.

  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_UInt8& rResult);
    //!< Deserialize a #SOPAS_UInt8.
  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_UInt16& rResult);  
    //!< Deserialize a #SOPAS_UInt16.
  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_UInt32& rResult);
    //!< Deserialize a #SOPAS_UInt32.

  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_Float& rResult);
    //!< Deserialize a #SOPAS_Float.
  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_Double& rResult);
    //!< Deserialize a #SOPAS_Double.
  
  virtual bool Deserialize(CBinaryDataStream& rInputStream, SOPAS_FixString& rResult);
    //!< Deserialize a #SOPAS_FixString.
private:
  CAsciiDeserializer(const CAsciiDeserializer&);
    //!< Do not allow copying
  CAsciiDeserializer& operator=(const CAsciiDeserializer&);
    //!< Do not allow copying

  template<class T>
    bool GetSimpleType(CBinaryDataStream& rInputStream, T& rResult);
    //!< Helper funcion that actually deserializes all simple types.
  bool ConvertFromHexString(CBinaryDataStream& rInputStream, int endIndex, 
    unsigned char* pBinaryData, int binaryLength);
    //!< Helper function that converts a hex-string to binary.
  bool GetNibble(unsigned char data, unsigned char& rNibble);
    //!< Helper function to decode hex values.
};



#endif