#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include <string>
#include "SopasBasicTypes.h"
#include "SopasString.h"
#include "CommunicationHandler.h"
#include <memory>

// Forward declarations
class CSerializer;
class CDeserializer;
class CBinaryDataStream;
class CCommunication;
class CFramer;

/*======================================================================================*/
/*! \brief Defines interface for SOPAS protocols.

    \ingroup common

    Inherit from this class to create a protocol to communicate with SOPAS devices.
    There are currently two SOPAS protocols: COLA-A and COLA-B. The commands are
    common to both protocol variants and are implemented in this base class.

    There are some protocol specifics that are implemented in the derived 
    protocols (e.g. the CSerializer and CDeserializer to use and handling of communication
    names). Everything else is implemented in this base class.

    This class is tightly coupled to CCommunicationHandler which handles
    a lower level of communication: Sending and receiving, apply framing 
    and communication flow for synchronous and asynchronous answers. 
    
    This class offers the following SOPAS device functions:
    - reading a device variable
    - writing to a device variable
    - invoking a device method (synchronous and asynchronous)
    - subscribing and unsubscribing device events

    The device answer can be deserialized by using the selected protocol's 
    GetDeserializer() function.

    Error handling works as follows: Any protocol function checks
    for a matching answer to your request and extracts an error code on failure.
    The error code can be retrieved by GetSopasErrorCode(). So if any protocol
    function (ReadVariable(), ...) returns <em>true</em> there has been no error
    and an answer without error code has been received. If a protocol function returns
    <em>false</em> and the SOPAS error code is not 0 then the device reported an error.
    Otherwise there was a communication error i.e. the data couldn't be sent, no answer
    was received in time or the received answer couldn't be decoded.
 */
/*======================================================================================*/
class CProtocol
{
  friend class CCommunicationHandler;

public:
  CProtocol(CCommunication& rCommunication, CFramer& rFramer);
    //!< Constructor defines according communication channel and framer to use.
  virtual ~CProtocol(void);
    //!< Destructor deletes communication channel and framer.
  
  SOPAS_UInt16 GetSopasErrorCode(void) const;
    //!< Gets SOPAS error code of last received answer.
  virtual CSerializer& GetSerializer(void) = 0;
    //!< Gets data serializer used by this protocol.
  virtual CDeserializer& GetDeserializer(void) = 0;
    //!< Gets data deserializer used by this protocol.

  CBinaryDataStream* ReadVariable(SOPAS_UInt16 uiIndex);
    //!< Reads device variable by index.
  CBinaryDataStream* ReadVariable(const std::string &rName);
    //!< Reads device variable by name.
  bool WriteVariable(SOPAS_UInt16 uiIndex, 
    const CBinaryDataStream* pSerializedVariable);
    //!< Writes device variable by index.
  bool WriteVariable(const std::string &rName, 
    const CBinaryDataStream* pSerializedVariable);
    //!< Writes device variable by name.
  CBinaryDataStream* InvokeMethod(SOPAS_UInt16 uiIndex, 
    const CBinaryDataStream* pSerializedParams);
    //!< Calls device method by index.
  CBinaryDataStream* InvokeMethod(const std::string &rName, 
    const CBinaryDataStream* pSerializedParams);
    //!< Calls device method by name.
  bool SubscribeEvent(SOPAS_UInt16 uiIndex, bool bSubscribe);
    //!< Subscribes/unsubscribes device event by index.
  bool SubscribeEvent(const std::string &rName, bool bSubscribe);
    //!< Subscribes/unsubscribes device event by name.
  bool InvokeAsyncMethod(SOPAS_UInt16 uiIndex, 
    const CBinaryDataStream* pSerializedParams);
    //!< Subscribes/unsubscribes device event by index.
  bool InvokeAsyncMethod(const std::string &rName, 
    const CBinaryDataStream* pSerializedParams);
    //!< Subscribes/unsubscribes device event by name.  

  void PollAsyncAnswers(int iPollTimeMs, CCommunicationHandler::SopasAsyncCallback pCallback = 0, void* pUser = 0);
    //!< Poll for asynchronous events or device method answers.
  void SetAsyncCallback(CCommunicationHandler::SopasAsyncCallback pCallback, void* pUser = 0);
    //!< \brief Sets callback function for asynchronous events or 
    //!< device method answers (function pointer).
  void SetAsyncCallback(CCommunicationHandler::IAsyncCallback* pCallback);
    //!< \brief Sets callback function for asynchronous events or 
    //!< device method answers (implementing class).

private:
  //! \brief State for ExtractCommand().
  /*! Determines what needs to be extracted next
      to interpret the SOPAS command within the answer. */
  enum EExtractType
  {
    EXTRACT_INDEX,      //!< command is followed by a SOPAS index
    EXTRACT_NAME,       //!< command is followed by a SOPAS communication name
    EXTRACT_ERROR_CODE, //!< command is followed by a SOPAS error code
    EXTRACT_UNKNOWN,    //!< name or index not encoded in command
    EXTRACT_INVALID     //!< no valid SOPAS command
  };

  CProtocol(const CProtocol&);
    //!< Do not allow copying
  CProtocol& operator=(const CProtocol&);
    //!< Do not allow copying

  // Helper functions     
  CBinaryDataStream* BuildCommandWithParams(const std::string& rCmd, SDecodedAnswer::EAnswerType eExpectedAnswer,
    const SOPAS_FixString& rName, const CBinaryDataStream* pSerializedParams);
    //!< Builds command string and appends all parameters (if any).
  CBinaryDataStream* BuildCommandWithParams(const std::string& rCmd, SDecodedAnswer::EAnswerType eExpectedAnswer,
    SOPAS_UInt16 uiIndex, const CBinaryDataStream* pSerializedParams);
    //!< Builds command string and appends all parameters (if any).
  
  SDecodedAnswer ExtractAnswer(CBinaryDataStream& rInputStream, bool bLastRequestByIndex);
    //!< Extracts all necessary parts of an answer to determine further processing.
  EExtractType ExtractCommand(CBinaryDataStream& rInputStream, SDecodedAnswer& rDeterminedAnswer);
    //!< Extracts the command within an answer to determine further processing of an answer.

  // Different protocol behaviour for call by name
  virtual bool DeserializeCommName(CBinaryDataStream& rInputStream, std::string& rCommName) = 0;
    //!< Deserializes the communication name within answer. Protocol dependend.
  virtual size_t GetSerializedCommNameSize(const SOPAS_FixString& rCommName) = 0;
    //!< Gets the length of the communication name to be serialized to the request. Protocol dependend.
  virtual bool SerializeCommName(CBinaryDataStream& rOutputStream, const SOPAS_FixString& rCommName) = 0;
    //!< Serializes communication name for "by name" request. Protocol dependend.

  CCommunicationHandler* m_pCommHandler;
    //!< Lower level handling of communication: Send and receive, apply framing, handle 
    //!< communication flow (synchronous and asynchronous answers).
};


/*======================================================================================*/
/*! \fn virtual CSerializer& CProtocol::GetSerializer(void) = 0;

    Gets the CSerializer of the protocol. A serializer is used to serialize the SOPAS
    datatypes to a datastream. It is used internally but also needed by the user if 
    InvokeMethod() or WriteVariable() is called. In that case the datatypes are not 
    known to this framework and each data member has to be serialized to a stream.

    \return The serializer used by this protocol.
 */
/*======================================================================================*/

/*======================================================================================*/
/*! \fn virtual CDeserializer& CProtocol::GetDeserializer(void) = 0;

    Gets the CDeserializer of the protocol. A deserializer is used to deserialize the SOPAS
    datatypes from a datastream. It is used internally but also needed by the user  
    to deserialize return values from InvokeMethod() or ReadVariable(). In that case the 
    datatypes are not known to this framework and each data member has to be deserialized 
    from the stream.

    \return The deserializer used by this protocol.
 */
/*======================================================================================*/


#endif