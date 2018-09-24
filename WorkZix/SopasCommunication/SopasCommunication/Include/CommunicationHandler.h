#ifndef __COMMUNICATION_HANDLER_H__
#define __COMMUNICATION_HANDLER_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "SopasBasicTypes.h"
#include "SopasString.h"
#include "DecodedAnswer.h"
#include <memory>
#include <vector>

// Forward declarations
class CBinaryDataStream;
class CFramer;
class CCommunication;
class CProtocol;
class CDeserializer;

/*======================================================================================*/
/*! \brief Internally handles asynchronous and synchronous communication with the device.

    \ingroup common
   
    This class handles the device communication for the SOPAS protocol. It waits
    for the according answer (synchronous requests) and also decodes asynchronous
    answers (events, methods) that were requested before. SOPAS error codes are also 
    extracted.

    \note This class is not intended to be used directly. It is tightly coupled to CProtocol.
      Use the higher-level CSopasInterface or CProtocol class to make device requests. 
      The higher level classes offer according wrappers which call the according functions 
      of this class.  

    The class uses the underlying CCommunication instance to actually send and receive data.
    To keep the communication handling simple currently no additional threads are involved.
    Synchronous and asynchronous handling is done by polling until the desired timeout
    has expired. The polling uses busy waiting including a Sleep(0) to reduce processor 
    load, if the \e WIN32 define is set.
    
    \section ComSync Synchronous requests
    If a synchronous request is made (e.g. read variable) the framework uses busy waiting 
    and blocks the caller until:
    - A matching answer
    - A SOPAS error code
    - A timeout (15s)

    occurs. 
    
    If the function returns \e true a matching answer has been received. Otherwise
    use GetSopasErrorCode() to retrieve the device error code that has been received. If
    no error code has been received GetSopasErrorCode() will return 0. In that case no 
    answer or invalid answers have been received until the timeout had expired. 
    
    The error code is reset every time a new request is made. If an asynchronous answer is
    received while waiting for a synchronous answer the asynchronous callback will be 
    called. See asynchronous handling below.

    \section ComAsync Asynchronous requests
    A device will send an answer to an asynchronous requests when it's available. 
    The request itself is synchronous and will be acknowledged at once. To receive the answer
    the request must be followed by calls to PollAsyncAnswers() otherwise the answers would
    be queued in the TCP/IP-stack and not processed. So if an asynchronous method is called
    poll until the answer has been received. If a device event is subscribed poll until
    the event is unsubscribed.

    If a registered event or method answer is received during polling a callback function is
    called. To define the callback function use SetAsyncCallback(SopasAsyncCallback) or 
    use SetAsyncCallback(IAsyncCallback*) and derive a class from IAsyncCallback and 
    implement the callback member. You can also use a temporary callback, see 
    PollAsyncAnswers() for details.

    The callback function gets the name/index and type of answer so it's possible to handle
    different events and method answers in one callback function. Processing should be kept
    short because SOPAS protocol operations are blocked then.

    If answers are received that are either not registered or invalid they are discarded.
    The registered callback is also called if an asynchronous answer is received while
    executing a synchronous request.

    See example \ref Async on Mainpage.

    \note On older device (e.g. LMS400) some standard methods were implemented as 
      asynchronous (e.g. Login). The methods can be called as synchronous by this framework.
      In that case an unexpected part of the answer is discarded.
    \note If you set the define \e COMMUNICATION_DEBUG then invalid or unexpected answers 
      will cause an according message on the console.
 */
/*======================================================================================*/
class CCommunicationHandler
{
public:
  typedef void (*SopasAsyncCallback)(const SDecodedAnswer&, CBinaryDataStream&, CDeserializer&, void*);
    //!< \brief Typedef for asynchronous callback function. See 
    //!< CCommunicationHandler::SetAsyncCallback() for parameter description of the callback.

  /*======================================================================================*/
  /*! \brief Interface for asynchronous callbacks (events or methods).
      
      Derive from this class to implement an asynchronous callback within a class.
      You need to implement the AsyncCallback() function and register the callback
      by calling CSopasInterface::SetAsyncCallback(CCommunicationHandler::IAsyncCallback*).
   */
  /*======================================================================================*/
  class IAsyncCallback
  {
  public:
    virtual void AsyncCallback(const SDecodedAnswer&, CBinaryDataStream&, CDeserializer&) = 0;
      //!< \brief Implement this function for callbacks. See 
      //!< CCommunicationHandler::SetAsyncCallback() for parameter description of the callback.
    virtual ~IAsyncCallback(void);
  };

  CCommunicationHandler(CCommunication& rCommunication, CFramer& rFramer,
    CProtocol& rProtocol);
    //!< Constructor defines according communication channel and framer to use.
  ~CCommunicationHandler(void);
    //!< Destructor deletes communication channel and framer.

  void PollAsyncAnswers(int iPollTimeMs, SopasAsyncCallback pCallback, void* pUser = 0);
    //!< Poll for asynchronous events or device method answers.
  void SetAsyncCallback(SopasAsyncCallback pCallback, void* pUser = 0);
    //!< \brief Sets callback function for asynchronous events or 
    //!< device method answers (function pointer).
  void SetAsyncCallback(IAsyncCallback* pCallback);
    //!< \brief Sets callback function for asynchronous events or 
    //!< device method answers (implementing class).
  SOPAS_UInt16 GetSopasErrorCode(void) const;
    //!< Get error code of last operation.

  void RegisterEvent(const std::string& rName);
    //!< Registers asynchronous event by name.
  void RegisterEvent(SOPAS_UInt16 uiIndex);
    //!< Registers asynchronous event by index.
  void UnregisterEvent(const std::string& rName);
    //!< Unregisters asynchronous event by name.
  void UnregisterEvent(SOPAS_UInt16 uiIndex);
    //!< Unregisters asynchronous event by index.
  void RegisterAsyncMethod(const std::string& rName);
    //!< Registers asynchronous method by name.
  void RegisterAsyncMethod(SOPAS_UInt16 uiIndex);
    //!< Registers asynchronous method by index.
  void UnregisterAsyncMethod(const std::string &rName);
    //!< Unregisters asynchronous method by name.
  void UnregisterAsyncMethod(SOPAS_UInt16 uiIndex);
    //!< Unregisters asynchronous method by index.
  
  template<class T>
    CBinaryDataStream* ProcessSyncRequest(const T& rNameOrIndex, const CBinaryDataStream& rDataToSend, 
    SDecodedAnswer::EAnswerType eExpectedAnswerType);
    //!< Processes a synchronous request and waits for the answer.
  
private:
  CCommunicationHandler(const CCommunicationHandler&);
    //!< Copying is forbidden.
  CCommunicationHandler& operator=(const CCommunicationHandler&);
    //!< Copying is forbidden.

  bool HandleSyncAnswer(std::auto_ptr<CBinaryDataStream>& rAnswer, std::auto_ptr<CBinaryDataStream>& rAnswerPayload);
    //!< Checks if answer belongs to a synchronous request and processes it.
  void HandleAsyncAnswer(std::auto_ptr<CBinaryDataStream>& rAnswer, SopasAsyncCallback pTemporaryCallback, void* pUser = 0);
    //!< Checks if answer belongs to an asynchronous request and processes it.
  void RegisterSyncRequest(const SOPAS_FixString& rName, SDecodedAnswer::EAnswerType eAnswerType);
    //!< Remembers type of request to check for matching answer (by name).
  void RegisterSyncRequest(SOPAS_UInt16 uiIndex, SDecodedAnswer::EAnswerType eAnswerType);
    //!< Remembers type of request to check for matching answer (by index).
  CBinaryDataStream* SendAndReceive(const CBinaryDataStream& rDataToSend);
    //!< Frames and sends request to device and waits and extracts answer.

  CCommunication& m_Communication; 
    //!< Communication channel to use.
  CFramer& m_Framer; 
    //!< Framing for data.
  CProtocol& m_Protocol;
    //!< Protocol to use.

  bool m_bLastRequestByIndex;
    //!< Flag indicates last synchronous request was by index.
  SOPAS_UInt16 m_LastSopasErrorCode;
    //!< SOPAS error code of last received answer.
  SDecodedAnswer m_LastSyncRequest;
    //!< Contains last open synchronous request.
  SopasAsyncCallback m_pFctCallback;
  void* m_pUser;
    //!< Asynchronous callback (function pointer).
  IAsyncCallback* m_pClassCallback;
    //!< Asynchronous callback (implementing class).

  std::vector<SDecodedAnswer> m_AsyncMethodRequests;
    //!< Contains all open asynchronous method requests.
  std::vector<SDecodedAnswer> m_SubscribedEvents;
    //!< Contains all registered events.

};

/*======================================================================================*/
/*! The request is framed and sent using the selected CFramer and CCommunication.
    The function returns when an answer has been received or a timeout occurred. 
    The answer must match the request or be an error code otherwise this function
    keeps on waiting. During waiting asynchronous answers are also processed and
    the registered callback is called accordingly (see SetAsyncCallback()).

    The timeout is set to 15s.

    \param rNameOrIndex The name (SOPAS_FixString) or index (#SOPAS_UInt16) of
      the request.
    \param rDataToSend The data to be sent. The complete protocol string as set-up
      by the according CProtocol instance.
    \param eExpectedAnswerType The expected answer. Needed to check whether synchronous
      answers match the request.

    \return The received answer with the read pointer already pointing to the answer's 
      contents. Or \e null if any error occurred.
 */
/*======================================================================================*/
template<class T>
CBinaryDataStream* CCommunicationHandler::ProcessSyncRequest(const T& rNameOrIndex, const CBinaryDataStream& rDataToSend, 
  SDecodedAnswer::EAnswerType eExpectedAnswerType)
{
  // Remember request to compare to answer (by index or by name) and clear SOPAS error code
  RegisterSyncRequest(rNameOrIndex, eExpectedAnswerType);

  // Send and frame data, receive and unframe answer
  return SendAndReceive(rDataToSend);
}

#endif