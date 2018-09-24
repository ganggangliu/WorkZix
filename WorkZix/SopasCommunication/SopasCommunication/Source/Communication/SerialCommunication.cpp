/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "SerialCommunication.h"
#include "BinaryDataStream.h"

CSerialCommunication::CSerialCommunication()
{
  port = nullptr;
}

/*======================================================================================*/
/*! \param portName The name of the port to open, like "COM1".
    \param baudRate Use a supported baudrate for the port.
    \param parity Use the enumeration values (i.e. <em>Even, Mark, None, Odd, Space</em>)
      for parity settings.
    \param dataBits Use a supported value for data bits.
    \param stopBits Use the enumeration values (i.e. <em>None, One, OnePointFive, Two</em>) for 
      stop bit settings.

    \return <em>true</em> if the port could be opened using the settings. <em>false</em>
      otherwise.
 */
/*======================================================================================*/
bool CSerialCommunication::Connect(String^ portName, int baudRate, Parity parity, int dataBits, StopBits stopBits)
{
  bool retVal = false;
  try
  {
    port = gcnew SerialPort(portName, baudRate, parity, dataBits, stopBits);
    port->Open();
    retVal = true;
  }
  catch (Exception^)
  {
    // Ignore, just return false
  }
  return retVal;
}

/*======================================================================================*/
/*! The following defaults are used:
    - Parity: None
    - DataBits: 8
    - StopBits: 1

    See CSerialCommunication(String^, int, Parity, int, StopBits) for more information.

    \param portName The name of the port to open, like "COM1".
    \param baudRate Use a supported baudrate for the port.

    \return <em>true</em> if the port could be opened using the settings. <em>false</em>
      otherwise.
 */
/*======================================================================================*/
bool CSerialCommunication::Connect(String^ portName, int baudRate)
{
  // Using defaults 8N1
  return Connect(portName, baudRate, Parity::None, 8, StopBits::One);
}

void CSerialCommunication::Disconnect(void)
{
  if(static_cast<SerialPort^>(port) != nullptr)
  {
    port->Close();
    port = nullptr;
  }
}

/*======================================================================================*/
/*! The implementation does not block but checks if there are bytes to read. If
    bytes are available for reading at most #READ_BUFFER_SIZE bytes are read and the
    function will return afterwards. If no bytes are available for reading the function
    will return at once.

    See CCommunication::ReadBytes() for more information.

    \return A binary datastream containing all bytes read. <em>Null</em> if no data
      has been read.
 */
/*======================================================================================*/
CBinaryDataStream* CSerialCommunication::ReadBytes(void)
{
  CBinaryDataStream* pData = 0;

  try
  {
    array<Byte>^ mgdData = gcnew array<Byte>(READ_BUFFER_SIZE);
    int readBytes = 0;    
    if (static_cast<SerialPort^>(port) != nullptr)
    {
      if (port->BytesToRead > 0)
      {
        readBytes = port->Read(mgdData, 0, mgdData->Length);
      }      
    }

    // Copy managed Byte-array to unmanaged char array
    if(readBytes > 0)
    {
      pData = new CBinaryDataStream(readBytes);
      for(int i = 0; i < readBytes; ++i)
      {
        (void)pData->WriteToStream(mgdData[i]);
      }
    }
  }
  catch(Exception^)
  {
    // Ignore, just return 0
  }

  return pData;
}

/*======================================================================================*/
/*! The storage size of the CBinaryDataStream is not important here. Only the bytes that
    were written to the stream are transmitted to the serial port here.

    See CCommunication::WriteBytes() for more information.

    \param rData The data to write.
    \return <em>true</em> if data was written successfully. <em>false</em> otherwise.
    
 */
/*======================================================================================*/
bool CSerialCommunication::WriteBytes(const CBinaryDataStream& rData)
{
  bool retVal = false;
  try
  {
    if (static_cast<SerialPort^>(port) != nullptr)
    {
      // Copy unmanaged data to managed data
      int length = rData.GetUsedLength();
      array<Byte>^ mgdData = gcnew array<Byte>(length);
      const unsigned char* pRawData = rData.GetData();
      for(int i = 0; i < length; ++i)
      {
        mgdData[i] = pRawData[i];
      }

      port->Write(mgdData, 0, length);
      retVal = true;
    }
  }
  catch(Exception^)
  {
    // Ignore, just return false
  }
  return retVal;
}
