/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "Lms100SopasInterface.h"
#include "SopasAsciiFramer.h"
#include "ColaAProtocol.h"
#include "Communication.h"

class CYourCommunication : public CCommunication
{
public:
  bool Connect(const char* pParams)
  {
    // Dummy function
    return false;
  }

  void Disconnect(void)
  {
    // Dummy function
  }
  virtual CBinaryDataStream* ReadBytes(void)
  {
    // Dummy function
    return 0;
  }

  virtual bool WriteBytes(const CBinaryDataStream& rData)
  {
    // Dummy function
    return false;
  }
};

void main(void)
{
  // Derive your communication channel from class CCommunication and use it here:
  // Note: This example will not work but is compileable.

  // Connect to SOPAS using the communication channel
  CYourCommunication comm;
  if (comm.Connect("ConnectionParameters") == true)
  {
    CSopasAsciiFramer framer;
    CColaAProtocol protocol(comm, framer);
    CLms100SopasInterface sopas(protocol);
    sopas.Login(SopasLoginLevel::AUTHORIZEDCLIENT); // Login to LMS100

    comm.Disconnect();
  } 
}