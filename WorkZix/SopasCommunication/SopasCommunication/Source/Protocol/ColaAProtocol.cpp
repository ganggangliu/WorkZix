/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "ColaAProtocol.h"
#include "BinaryDataStream.h"

/*======================================================================================*/
/*! \copydetails CProtocol::CProtocol(CCommunication&,CFramer&)                         */
/*======================================================================================*/
CColaAProtocol::CColaAProtocol(CCommunication& rCommunication, CFramer& rFramer) : 
  CProtocol(rCommunication, rFramer)  
{
}

/*======================================================================================*/
/*! \copydetails CProtocol::~CProtocol()                                                */
/*======================================================================================*/
CColaAProtocol::~CColaAProtocol(void)
{
}

/*======================================================================================*/
/*! This class uses the CAsciiSerializer to serialize values.

    \return The serializer to use for parameters when calling InvokeMethod() or 
      WriteVariable().
 */
/*======================================================================================*/
CSerializer& CColaAProtocol::GetSerializer(void)
{  
  return m_Serializer;
}

/*======================================================================================*/
/*! This class uses the CAsciiDeserializer to deserialize values.

    \return The deserializer to use for answers when calling InvokeMethod() or 
      ReadVariable().
 */
/*======================================================================================*/
CDeserializer& CColaAProtocol::GetDeserializer(void)
{
  return m_Deserializer;
}

/*======================================================================================*/
/*! The communication name is separated from the command by a space. The next
    parameter in the datastream is separated by another space but it is not removed
    from the stream because it belongs to the next parameter.

    \param rInputStream The datastream to extract the communication name from. The
      read pointer must point right behind the SOPAS command.
    \param[out] rCommName The extracted communication name.

    \return \e true if a name could be extracted from the stream. \e false otherwise.
 */
/*======================================================================================*/
bool CColaAProtocol::DeserializeCommName(CBinaryDataStream& rInputStream, std::string& rCommName)
{
  // After command there must be a separator (even for COLA-B)
  bool  bSuccess = (rInputStream.ReadChar() == ' ');

  // Find next separator (end of string)
  // If no separator has been found comparison is done until end of answer
  int i = rInputStream.FindIndexOfNext(' ');

  // Extract string until separator (name may not include spaces)
  rCommName = rInputStream.ReadString(i - rInputStream.GetReadIndex());
  bSuccess &= (rCommName.empty() == false);

  return bSuccess;
}

/*======================================================================================*/
/*! The COLA-A protocol just writes the name to the stream.

    \param rCommName The communication name to be serialized.   

    \return The number of bytes needed to serialize the given name.
 */
/*======================================================================================*/
size_t CColaAProtocol::GetSerializedCommNameSize(const SOPAS_FixString& rCommName)
{
  return m_Serializer.GetSerializedSize(rCommName);
}

/*======================================================================================*/
/*! The COLA-A protocol just writes the name to the stream.

    \param rOutputStream The stream to serialize the communication name to.
    \param rCommName The communication name to be serialized.   

    \return \e true if the name could be written to the stream. \e false otherwise.
 */
/*======================================================================================*/
bool CColaAProtocol::SerializeCommName(CBinaryDataStream& rOutputStream, const SOPAS_FixString& rCommName)
{
  return m_Serializer.Serialize(rOutputStream, rCommName);
}
