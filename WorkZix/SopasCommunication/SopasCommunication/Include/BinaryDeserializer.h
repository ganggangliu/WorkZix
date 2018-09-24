#ifndef __BINARY_DESERIALIZER_H__
#define __BINARY_DESERIALIZER_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "Deserializer.h"

/*======================================================================================*/
/*! \brief A deserializer for COLA-B-protocol

    \ingroup common

    This class implements a deserializer that is used by CColaBProtocol to
    deserialize values of device answers. To decode an answer (needed for 
    CProtocol::ReadVariable() and CProtocol::InvokeMethod()) get the according 
    deserializer of the selected protocol by calling CProtocol::GetDeserializer(). 
    The COLA-B-protocol will return an instance of this class.

    The COLA-B-protocol serializes data with fixed lengths for each type. COLA-B 
    transmits in Big Endian byte order.

    \note 64-bit integers are not supported.

    \note All deserializer functions return a boolean value to indicate a successful 
      deserialization. If an error occured during extracting the read pointer of the 
      CBinaryDataStream could have been advanced already. The remaining data cannot be 
      extracted in that case and the result value will be invalid. Also all remaining
      accesses to the data stream will be incorrect. So further processing should be 
      stopped if any call to Deserialize() fails.
 */
/*======================================================================================*/
class CBinaryDeserializer : public CDeserializer
{
public:
  CBinaryDeserializer(void);
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
  CBinaryDeserializer(const CBinaryDeserializer&);
    //!< Do not allow copying
  CBinaryDeserializer& operator=(const CBinaryDeserializer&);
    //!< Do not allow copying

  template<class T>
    bool GetSimpleType(CBinaryDataStream& rInputStream, T& rResult);
    //!< Helper funcion that actually deserializes all simple types.
};



#endif