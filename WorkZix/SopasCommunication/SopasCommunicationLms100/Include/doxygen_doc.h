#ifndef __DOXYGEN_DOC_H__
#define __DOXYGEN_DOC_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

// This headerfile is only included to extract the main documentation page for a doxygen run!

/*======================================================================================*/
/*! \mainpage

    SopasCommunication is a simple framework to demonstrate the communication with
    SICK devices using the SOPAS communication architecture. The framework offers 
    basic functionality to invoke device methods, subscribe to device events, read variables
    from the device and write variables to the device.

    Some predefined SOPAS methods like "login" are available and can be used as an example to
    query information from the device or execute methods on the device. The framework
    supports the selection of communication channel, SOPAS protocol and SOPAS framing.
    
    The communication part is kept simple (see limitations) and should only be used as 
    an working example. Multithreading is not used here. A synchronous request is sent to the 
    device and the framework waits for an answer. This is done by busy waiting but includes a 
    sleep-command to reduce processor load. If no answer has been received after 15 seconds 
    the framework will return. 

    See the following sections for more detailed information.

    \section version Version
    This framework is divided into two parts: A device specific part (in this case LMS100)
    and a common part that's used by all devices. This release includes the following versions:

    \version \e Common: v1.02
    \version \e LMS100: v1.02

    To get the current version number call CSopasInterface::GetFrameworkVersion() or
    CLms100SopasInterface::GetDeviceFrameworkVersion() respectively.

    \section Limitations Limitations
    In this release there are some limitations that might be eliminated
    in future releases:
    - To keep the communication structure simple busy waiting is used during polling.
      A Sleep-command is issued to reduce processor load on WIN32 systems. This is not portable.
    - The login passwords supplied with this framework may not work for older devices 
    (e.g. LMS400). See CSopasLoginLevel on how to change passwords used by the framework.
    - 64 bit integers are not supported
    - Ascii framing is limited to a frame length of 100000 bytes. See CSopasAsciiFramer.

    \section Compilation Compiling the project
    The project is written in C++/CLI and can be compiled with Microsoft Visual Studio 2005
    and above. The .NET-framework V2.0 is necessary to execute the program.

    Only the data transfer classes (CTcpCommunication and CSerialCommunication) and
    the example main function are actually using the .NET-framework. Everything else 
    is written in plain C++. There is an additional project called 
    <em>SopasCommunicationLms100_cpp.vcproj</em> (including the according solution file) 
    which is a plain C++ project for VS2005. The project is compileable but you need to 
    implement the data transfer functions yourself. This part of the framework should compile 
    with any recent C++ compiler.
    
    \note Right now there is a Sleep-command within CCommunicationHandler::SendAndReceive() 
      and CCommunicationHandler::PollAsyncAnswers() to reduce processor load during busy
      waiting. This is not portable and is only compiled if the WIN32 define is set. 
      Other platforms will use busy waiting at full processor load.

    \section Example Example (main)
    The main function (C++/CLI-project only) contains an example for the LMS100. The
    example connects to a LMS100 via TCP/IP using the default address 192.168.0.1. 
    You can also set the IP-address by supplying it as a single command line argument. 
    The example sets the scan config of a connected LMS100 to 50Hz frequency, 0.5 degree
    angular resolution and according start- end endangles. Then the measurement mode is 
    started and the program waits until the LMS100 reaches measurement mode. The
    program then gets a scan and displays some status information.
    
    The program then polls 1000 scans by repeatedly reading the according device variable. 
    As an example it calculates the average of the 90 degree beam. Because the program might 
    actually poll scans faster than they are updated the telegram counter is used to detect 
    new scans.

    As a second example the same procedure is done by subscribing the scandata event and
    processing the scandata in a callback function.
    
    \note The scandata format must be configured to contain distance data in the 
    first 16-bit channel for this example to work properly. This is not checked. 
      
    \section Usage Usage
    \subsection Connection Setting up the connection
    The connection to the device is done by creating a communication channel
    and setting up the connection (TCP/IP and serial connection supported):
    \code
    CTcpCommunication ipComm;
    if(ipComm.Connect(IPAddress::Parse("192.168.0.1")) == true)
    {
      //...
    }
    \endcode
    \note Only the C++/CLI project supplies classes to use a serial
    connection or a TCP/IP-connection. You can implement your own data transfer
    by deriving from CCommunication and implementing all necessary interface methods.

    \subsection Communicating Communicating with the device
    The communication with the device is done by the CSopasInterface class and its
    children. When you create an instance you also specify the communication channel 
    (see above), the framing (binary/ascii) and the protocol (COLA-A/COLA-B) by
    creating instances of the according classes.

    \code
  // Using TCP/IP connection, ASCII framing and COLA-A-protocol
  CSopasAsciiFramer framer;
  CColaAProtocol protocol(ipComm, framer)
  CLms100SopasInterface sopas(protocol);
    \endcode

    \attention Make sure that the CProtocol, CCommunication and CFramer instances stay in scope 
      longer than the CLms100SopasInterface. 
      It is best practise to encapsulate own functions that directly use the protocol into 
      a class that derives from CSopasInterface or its descendants.

    \subsection Data Using predefined methods to get data
    The CLms100SopasInterface offers some functions to get data from the LMS100. See the following
    example:

    \code
  // Get version information
  void SopasExample(CLms100SopasInterface& rSopas)
  {
    std::string version;
    std::string name;
    if (rSopas.GetVersionString(name, version) == true)  
    {
      printf("Name: %s, Version: %s\n", name.c_str(), version.c_str());    
    }

    // Login with according SOPAS userlevel
    if(rSopas.Login(SopasLoginLevel::AUTHORIZEDCLIENT) == true)
    {
      // ...
    }

    // Start measuring (SOPAS method returns 0 if everything is OK)
    CLms100SopasInterface::EMeasureError retVal;
    if(rSopas.StartMeasure(&retVal) == true)
    {
      // ...
    }
  }
    \endcode
    \note If a call to a method of CSopasInterface fails you should call 
      CSopasInterface::GetSopasErrorCode(). There are two possibilities:
    - The device reported a SOPAS error code (e.g. invalid method name).
    - There was another error (e.g. timeout, not enough data received, ...).
      In that case \link CSopasInterface::GetSopasErrorCode() GetSopasErrorCode \endlink
      reports 0.

    \subsection Overview Overview of predefined functions
    Right now following functions are available (CSopasInterface):
    - \link CSopasInterface::Login() Login \endlink
    - \link CSopasInterface::Logout() Logout \endlink
    - \link CSopasInterface::GetVersionString() GetVersionString \endlink    

    For the LMS100 the following functions are available (CLms100SopasInterface):
    - \link CLms100SopasInterface::GetScanData() GetScanData \endlink
    - \link CLms100SopasInterface::GetState() GetState \endlink
    - \link CLms100SopasInterface::StartMeasure() StartMeasure \endlink
    - \link CLms100SopasInterface::StopMeasure() StopMeasure \endlink
    - \link CLms100SopasInterface::SetScanConfig() SetScanConfig \endlink
    - \link CLms100SopasInterface::GetScanDataConfig() GetScanDataConfig \endlink
    - \link CLms100SopasInterface::SubscribeScanDataEvent() SubscribeScanDataEvent \endlink

    \subsection usingdatatypes Data types
    To serialize and deserialize data from the stream correctly the according data 
    types have to be known. Typedefs have been defined for basic data types and 
    some complex and compound types have also been defined. They should be use to
    simplify serialization and deserialization. See \ref datatypes "data types" for details.

    \subsection datastream Data streams
    Data that is to be serialized or deserialized is stored in binary data streams 
    (see CBinaryDataStream). The stream helps to simplify data handling by keeping a write 
    position and a read position that is incremented accordingly. There are some helper functions 
    to write data to the stream or read data from it. The stream allows easy consecutive reading
    and writing, see example below.

    \subsection VariableRead Building your own function to read a variable
    If you want to read a variable from the device you have to create the function
    as follows:
    \code
bool ReadMyVariable(CProtocol& rProtocol)
{
  bool bRetVal = false;

  // Build request, send it and wait for answer
  CBinaryDataStream* pReceivedData = rProtocol.ReadVariable("MyVarName");

  // Function blocks until data has been received. The data is returned or
  // null pointer is returned on error, e.g. timeout, wrong answer, ...
  if (pReceivedData != 0)
  {
    // Deserialize the answer. Let's assume the following data type:
    // struct
    // {
    //   SOPAS_Float f;
    //   SOPAS_UInt8 c;
    //   SOPAS_UInt32 l;
    // };
    
    // Get access to deserializer of according protocol
    CDeserializer& rDeserializer = rProtocol.GetDeserializer();
    // Deserialize variables in the same order as defined in the struct above.
    SOPAS_Float f;
    SOPAS_UInt8 c;
    SOPAS_UInt32 l;

    bRetVal = rDeserializer.Deserialize(*pReceivedData, f);
    bRetVal &= rDeserializer.Deserialize(*pReceivedData, c);
    bRetVal &= rDeserializer.Deserialize(*pReceivedData, l);

    // Additional check: All data of the answer should have been processed.
    // Otherwise the data type structure might have changed.
    bRetVal &= pReceivedData->IsReadPosAtEnd();

    // Note: The struct values are ignored in this example. 
    // bRetVal shows success of deserialization.
  }

  // All data processed, delete it
  delete pReceivedData;

  return bRetVal;
}
    \endcode
    The variable may be a simple type or an aggregated type. In that case all
    elements are serialized for communication one after the other. You need to know
    the type of the variable and call the according deserialize functions to extract
    the values from the answer. There is a typedef for most SOPAS simple types that
    should be used.

    \note The binary data is encapsulated within a CBinaryDataStream which keeps a read and
    write pointer to the current position. This makes it easier to deserialize
    all successive elements from an answer.

    \subsection MethodInvocation Building your own function to invoke a method
    If you want to invoke a method on the device you have to serialize all parameters
    and deserialize the answer yourself. As for variables the according data types
    may be simple or a complex struct. The framework cannot know the structure of the
    parameters. 
    
    The serialization basically is done the same way: You write simple data types
    to a stream by using the typedefs for SOPAS data types. You must create a data
    stream in advance. There are two possibilites:
    - Query the needed size for each data type you are going to serialize.
    - Create a stream that is big enough in any case by estimating its needed size.
      If the serialization fails the stream was not big enough.

    \code
bool InvokeMyMethod(CProtocol& rProtocol)
{
  // Assume a device method that takes the following parameter:
  // struct
  // {
  //   SOPAS_Int8 c;
  //   SOPAS_Float f;
  //   SOPAS_Bool b;
  // };

  bool bRetVal = false;

  // Set parameters accordingly (might be done outside of this function)
  SOPAS_Int8 c = 23;
  SOPAS_Float f = 23.5f;
  SOPAS_Bool b = 0;

  // Create stream to store serialized parameters
  // Using exact stream size.
  // Alternatively just use 100 as stream size which should be big enough...
  CSerializer& rSerializer = rProtocol.GetSerializer();  
  size_t minStreamSize = rSerializer.GetSerializedSize(c) + rSerializer.GetSerializedSize(f)
    + rSerializer.GetSerializedSize(b);

  // Create the stream as lokal variable so it's automatically deleted afterwards
  CBinaryDataStream serializedParamStream(static_cast<unsigned int>(minStreamSize));

  // Serialize parameters into stream
  bool bSerialized = true;
  bSerialized &= rSerializer.Serialize(serializedParamStream, c);
  bSerialized &= rSerializer.Serialize(serializedParamStream, f);
  bSerialized &= rSerializer.Serialize(serializedParamStream, b);
  
  if (bSerialized == true)
  {    
    // Build request    
    CBinaryDataStream* pReceivedData = rProtocol.InvokeMethod("MyMethodName", &serializedParamStream);
    // Deserializing the result is done the same way as for reading variables
    // ...
    // bRetVal might be set to "true" here...
  }
  return bRetVal;
}
    \endcode

    \subsection complex Deserializing complex data types
    Complex datatypes (e.g. structs) simply have to be deserialized element by element. 
    There are some types though that can directly be deserialized.

    \subsubsection strings Strings
    Strings are deserialized by calling the according function. SOPAS distinguishes between
    flex length and fixed length strings. This difference is important for the deserialization itself.
    Therefore string types have been added. Use #SOPAS_FlexString and SOPAS_FixString accordingly. The
    length of the FixString cannot be changed. The contents can be longer or shorter but the string
    will always be transmitted using the intitial length (zero-padding or clipping is used). The FlexString
    is just a typedef for std::string.

    \code
  // Send request, receive data
  // ...
  SOPAS_FlexString flexString; // Length will be extracted from stream
  bSuccess = rDeserializer.Deserialize(*pReceivedData, flexString);
  printf("%s", flexString.c_str());

  SOPAS_FixString fixString(7); // For example this string has a fixed length of 7.
  bSuccess = rDeserializer.Deserialize(*pReceivedData, fixString);
  printf("%s", fixString.data.c_str());
    \endcode

    \subsection arrays Arrays of simple types
    There is also a helper function to deserialize arrays of simple types. Basically
    all elements are deserialized on after the other. The array length must be known.

    For FlexArrays a simple wrapper is implemented. Their length may vary and is taken
    from the data stream. Only the used data is transmitted.
    \code
  // Send request, receive data
  // ...
  SOPAS_UInt16 myArray[10]; // Same size as defined on SOPAS-device
  bSuccess = rDeserializer.DeserializeArray(*pReceivedData, 10, &myArray[0]);

  SOPAS_FlexArray<SOPAS_Double, 10> myFlexArray; // 10 is the maximum size
  bSuccess = rDeserializer.DeserializeArray(*pReceivedData, myFlexArray);

    \endcode
    Check the occupied length of the FlexArray afterwards (SOPAS_FlexArray::uiFlexArrayLength).
    The contained data is only valid up to the according index.

    \subsection XBytes XBytes (bitfields)
    The SOPAS type XByte is used to represent bitfields. Because access to bitfields within
    structs is compiler dependent the XByte type was created. A XByte is like a structure
    of signed and unsigned bitfields. This structure has to be built by the user. Each
    element is accessed by name.

    \code
  // Assume the following bitfield
  // {
  //   int x: 3;
  //   unsigned int y: 9;
  //   unsigned int b: 1;
  // }
  //
  
  // Construct the bitfield:
  SOPAS_XByte myBitfield;
  bool bSuccess = myBitfield.AddSignedBitfield("x", 3);
  bSuccess &= myBitfield.AddUnsignedBitfield("y", 9);
  bSuccess &= myBitfield.AddUnsignedBitfield("b", 1);

  if(bSuccess == true)
  {
    // Send request, receive data
    // ...

    // Use deserializer:
    bSuccess = rDeserializer.Deserialize(*pReceivedData, myBitfield);

    if(bSuccess == true)
    {
      // Access results
      // Signed/unsigned must match for the request. This is checked.
      int x;
      unsigned int y, b;
      bSuccess = myBitfield.GetSignedValue("x", x); 
      bSuccess &= myBitfield.GetUnsignedValue("y", y);
      bSuccess &= myBitfield.GetUnsignedValue("b", b);
      if(bSuccess == true)
      {
        printf("x: %d, y: %u, b: %u\n", x, y, b);
        // Example output might be: "-1, 257, 0"
      }
    }
  }

    \endcode

    \subsection Async Asynchronous events and methods
    Asynchronous device answers can come at any time after a request has been made. 
    They need to be polled after the request otherwise the TCP/IP-queue could get 
    jammed. Note that events will arrive until they are unsubscribed so 
    the program needs to poll in the main loop. Asynchronous method answers only
    need to be polled until they have been received once. The time depends on the 
    called method. 
    
    For more details see CCommunicationHandler.

    \note If you set the define \e COMMUNICATION_DEBUG then invalid or unexpected answers 
      will cause an according message on the console.

    \code
void ProcessEvent(CSopasInterface& rSopas)
{
  // Register callback for event processing
  rSopas.SetAsyncCallback(&MyCallback);

  // Subscribe event
  CProtocol& rProtocol = rSopas.GetProtocol();
  rProtocol.SubscribeEvent("my_evt", true);

  // Process events for 10 seconds
  rSopas.PollAsyncAnswers(10000);

  // Unsubscribe event
  rProtocol.SubscribeEvent("my_evt", false);

  // Poll some more to remove any event that might have been sent while unsubscribing
  rProtocol.PollAsyncAnswers(500);
}
    \endcode

    During polling for every incoming event (that has been registered) the callback will 
    be called. In the callback you can handle all registered events or asynchronous
    method answers you expect:

    \code
void MyCallback(const SDecodedAnswer& rAnswer, CBinaryDataStream& rStream, CDeserializer& rDeserializer)
{
  if(rAnswer.eAnswerType == SDecodedAnswer::EVENT_RESULT)
  {
    // Process subscribed events
    if(rAnswer.coName.compare("my_evt") == 0)
    {
      // Deserialize event data (assuming just one SOPAS_UInt16 here)
      SOPAS_UInt16 data;
      if(rDeserializer.Deserialize(rStream, data) == true)
      {
        // Initialize average and display detailed scan information on first run
        printf("Event data received: %u\n", data);
      }
      else
      {
        printf("Error during scandata deserialization (event based)\n");
      }
    }
  }
}
    \endcode

    Another way is to implement a callback function within a class by deriving from
    CCommunicationHandler::IAsyncCallback.
    
    \code
class MyCallback : public CCommunicationHandler::IAsyncCallback
{
  virtual void AsyncCallback(const SDecodedAnswer&, CBinaryDataStream&, CDeserializer&)
  {
    // See example above for a real callback implementation.
    printf("Callback called");
  }
};

void ProcessEvent(CSopasInterface& rSopas)
{
  // Register callback for event processing
  MyCallback callback;
  rSopas.SetAsyncCallback(&callback);

  // Subscribe event
  CProtocol& rProtocol = rSopas.GetProtocol();
  rProtocol.SubscribeEvent("my_evt", true);

  // ... see example above

  // Make sure that callback class stays in scope longer, or disable callback here:
  rSopas.SetAsyncCallback(static_cast<CCommunicationHandler::IAsyncCallback*>(0));
}
    \endcode
 */
/*======================================================================================*/


#endif