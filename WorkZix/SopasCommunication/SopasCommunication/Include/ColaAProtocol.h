#ifndef __COLA_A_PROTOCOL_H__
#define __COLA_A_PROTOCOL_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "Protocol.h"
#include "SopasBasicTypes.h"
#include <string>
#include "AsciiSerializer.h"
#include "AsciiDeserializer.h"

/*======================================================================================*/
/*! \brief Implements the COLA-A-protocol

    \ingroup common
    
    This class implements the ASCII-based COLA-A-protocol. This is normally used
    in conjunction with ASCII framing (CSopasAsciiFramer) but may also be used
    with binary framing.

    This class implements some specifics for the ASCII protocol, e.g. it uses
    the CAsciiSerializer and CAsciiDeserializer. For more information see \link
    CProtocol base class documentation\endlink.
 */
/*======================================================================================*/
class CColaAProtocol : public CProtocol
{
public:
  CColaAProtocol(CCommunication& rCommunication, CFramer& rFramer);
    //!< Constructor defines according communication channel and framer to use.
  virtual ~CColaAProtocol(void);
    //!< Destructor deletes communication channel and framer.

  virtual CSerializer& GetSerializer(void);
    //!< Gets data serializer used by this protocol.
  virtual CDeserializer& GetDeserializer(void);
    //!< Gets data deserializer used by this protocol.

protected:
  virtual bool DeserializeCommName(CBinaryDataStream& rInputStream, std::string& rCommName);
    //!< Deserializes the communication name within answer. Protocol dependend.
  virtual size_t GetSerializedCommNameSize(const SOPAS_FixString& rCommName);
    //!< Gets the length of the communication name to be serialized to the request.
  virtual bool SerializeCommName(CBinaryDataStream& rOutputStream, const SOPAS_FixString& rCommName);
    //!< Serializes communication name for "by name" request.

private:
  CColaAProtocol(const CColaAProtocol&);
    //!< Do not allow copying
  CColaAProtocol& operator=(const CColaAProtocol&);
    //!< Do not allow copying

  CAsciiSerializer m_Serializer;
    //!< Serializer used by this protocol.
  CAsciiDeserializer m_Deserializer; 
    //!< Deserializer used by this protocol.
};

#endif