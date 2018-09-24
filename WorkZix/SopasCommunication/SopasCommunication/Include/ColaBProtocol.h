#ifndef __COLA_B_PROTOCOL_H__
#define __COLA_B_PROTOCOL_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "Protocol.h"
#include "SopasBasicTypes.h"
#include <string>
#include "BinarySerializer.h"
#include "BinaryDeserializer.h"

/*======================================================================================*/
/*! \brief Implements the COLA-B-protocol

    \ingroup common
    
    This class implements the COLA-B-protocol. It must be used in conjunction with 
    binary framing (CSopasBinaryFramer), ASCII-framing won't work correctly.

    This class implements some specifics for the binary protocol, e.g. it uses
    the CBinarySerializer and CBinaryDeserializer. For more information see \link
    CProtocol base class documentation\endlink.

    The COLA-B protocol allows an optional \e space to be sent after the command (e.g.
    sRN<em>\<space\></em>DeviceIdent). This is not done in this implementation.
 */
/*======================================================================================*/
class CColaBProtocol : public CProtocol
{
public:
  CColaBProtocol(CCommunication& rCommunication, CFramer& rFramer);
    //!< Constructor defines according communication channel and framer to use.
  virtual ~CColaBProtocol(void);
    //!< Destructor deletes communication channel and framer.

  virtual CSerializer& GetSerializer(void);
    //!< Gets data serializer used by this protocol.
  virtual CDeserializer& GetDeserializer(void);
    //!< Gets data deserializer used by this protocol.

protected:
  virtual bool DeserializeCommName(CBinaryDataStream& rInputStream, std::string& rCommName);
    //!< Deserializes the communication name within answer. 
  virtual size_t GetSerializedCommNameSize(const SOPAS_FixString& rCommName);
    //!< Gets the length of the communication name to be serialized to the request.
  virtual bool SerializeCommName(CBinaryDataStream& rOutputStream, const SOPAS_FixString& rCommName);
    //!< Serializes communication name for "by name" request.

private:
  CColaBProtocol(const CColaBProtocol&);
    //!< Do not allow copying
  CColaBProtocol& operator=(const CColaBProtocol&);
    //!< Do not allow copying

  CBinarySerializer m_Serializer;
    //!< Serializer used by this protocol.
  CBinaryDeserializer m_Deserializer; 
    //!< Deserializer used by this protocol.
};

#endif