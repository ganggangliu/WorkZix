/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "Doxygen_doc.h"
#include <stdio.h>

#include "TcpCommunication.h"
#include "SerialCommunication.h"
#include "SopasInterface.h"
#include "SopasAsciiFramer.h"
#include "ColaAProtocol.h"
#include "Lms100SopasInterface.h"
#include "Lms100State.h"
#include "Lms100Scan.h"
#include "OpenCVInc.h"
#include "SickLms511DataTransfer.h"

static void InitLms100(CLms100SopasInterface& rSopas);
static void PollScans(CLms100SopasInterface& rSopas);
static void WaitForMeasureState(CLms100SopasInterface& rSopas);
static void SubscribeScans(CLms100SopasInterface& rSopas);
static void SopasAsyncCallback(const SDecodedAnswer& rAnswer, CBinaryDataStream& rStream, CDeserializer& rDeserializer, void* pUser);
static unsigned int ProcessScan(const SLms100Scan& soScan, bool bInit);

CSickLms511DataTransfer g_SickLms511DataTransfer(-5.0,0.25);
long g_Cont = 0;
char g_szChannelName[256] = {0};

// Note: Ip-Address and TCP/IP are C++/CLI-code. The rest is plain C++.
int main(int argc, char* argv[])
{
  // Display version information
  printf("Sopas Communication framework (common: v%s, lms100: v%s)\n", 
    CSopasInterface::GetFrameworkVersion().c_str(), 
    CLms100SopasInterface::GetDeviceFrameworkVersion().c_str());

  // Default IP-address
  IPAddress^ coAdr = IPAddress::Parse("192.168.0.1");
  strcpy(g_szChannelName, "LCM_IBEO_CLOUD_POINTS");

  // simple command line parsing: If one parameter is given, use it as IP-address
  if(argc >= 2)
  {
    try
    {
      coAdr = IPAddress::Parse(gcnew System::String(argv[1]));
    }
    catch(Exception^)
    {
      printf("Invalid IP-address as command line parameter given. Using default.\n");
    }
  }
  if (argc >= 3)
  {
	  strcpy(g_szChannelName, argv[2]);
  }
  
  // Connect to LMS100 by using TCP/IP
  Console::WriteLine("Connecting to {0}", coAdr->ToString()); // C++/CLI to display IPAddress
  CTcpCommunication ipComm;  
  if(ipComm.Connect(coAdr) == true)
  {
    CSopasAsciiFramer framer;
    CColaAProtocol protocol(ipComm, framer);
    CLms100SopasInterface sopas(protocol);

    // Activate measure mode, set scanconfig and wait for measure state
    InitLms100(sopas);
    WaitForMeasureState(sopas);

    // Polled scandata processing
//    PollScans(sopas);

    // Event based scandata processing
    sopas.SetAsyncCallback(&SopasAsyncCallback);
    SubscribeScans(sopas);

    ipComm.Disconnect();
  } 

  return 0;
}

void InitLms100(CLms100SopasInterface& rSopas)
{
  printf("Started\n");
  std::string version;
  std::string name;
  if (rSopas.GetVersionString(name, version) == true)  
  {
    printf("Name: %s, Version: %s\n", name.c_str(), version.c_str());    
  }
  else
  {
    printf("Sopas error code: %04x\n", rSopas.GetSopasErrorCode());
  }

  if(rSopas.Login(SopasLoginLevel::AUTHORIZEDCLIENT) == true)
  {
    printf("Login successful\n");
  }
  else
  {
    printf("Sopas error code: %04x\n", rSopas.GetSopasErrorCode());    
  }

  // Set scan config to 50Hz, 0.5 deg resolution and angle from -45 deg to 225 deg.
  CLms100SopasInterface::EScanConfigError eError;
  if(rSopas.SetScanConfig(1000/*5000*/, 1000/*5000*/, 0/*-450000*/, 1800000/*2250000*/, &eError) == true)
  {
    // Communication was OK, there might be an error code though
    printf("SetScanConfig: %d (0 = OK)\n", eError);
  }
  else
  {
    printf("Sopas error code: %04x\n", rSopas.GetSopasErrorCode());    
  }

  CLms100SopasInterface::EMeasureError eRetVal;
  if(rSopas.StartMeasure(&eRetVal) == true)
  {
    // Communication was OK, there might be an error code though
    printf("StartMeasure: %d (0 = OK)\n", eRetVal);
  }
  else
  {
    printf("Sopas error code: %04x\n", rSopas.GetSopasErrorCode());
  }  

  // Logout to make parameters active (must be logged in for StartMeasure)
  if(rSopas.Logout() == true)
  {
    printf("Logout successful\n");
  }
  else
  {
    printf("Sopas error code: %04x\n", rSopas.GetSopasErrorCode());    
  }

  // NOTE: It might take some time until the LMS100 is in the MEASURE-state.
  // PollScans waits until measure state is reached
  
}

void WaitForMeasureState(CLms100SopasInterface& rSopas)
{
  // --- Wait for measure state --- //
  printf("Wait for MEASURE state (7)\n");
  SLms100State soState;
  soState.Clear();  
  while((rSopas.GetState(soState) == true) && (soState.eFSMState != SLms100State::LMSSTATE_MEASURE))
  {
    printf("\rstate: %u", soState.eFSMState);
    System::Threading::Thread::CurrentThread->Sleep(1000); // C++/CLI to wait some time
  }
  if(soState.eFSMState != SLms100State::LMSSTATE_MEASURE)
  {
    // Meas-state not reached but while loop was exited => GetState returned false
    printf("\nSopas error code: %04x\n", rSopas.GetSopasErrorCode());
  }
  else
  {
    printf("\nLMS100 is in measure state\n");
  }
}

unsigned int ProcessScan(const SLms100Scan& soScan, bool bInit)
{
  static unsigned int ulDeg90_index = 0; // Calculated during init phase
  static double dScanBeamAvg = 0.0f;
  static unsigned int ulAvgCounter = 0;
  static SOPAS_UInt16 uiLastTgm = 0;
  static unsigned int ulProcessedScanCounter = 0;

  if(bInit == true)
  {
    // Initialize variables for next step
    ulDeg90_index = 0;
    dScanBeamAvg = 0.0f;
    ulAvgCounter = 0;
    uiLastTgm = soScan.StatusBlock.uiTelegramCount;
    ulProcessedScanCounter = 0;    

    // --- Display some status information ---
    printf("\nScandata info:\n--------------\n");
    printf("Version: %u\n", soScan.uiVersionNo);

    // Device info
    printf("Serial-no: %u, ident: %u\n",
      soScan.DeviceBlock.udiSerialNo, soScan.DeviceBlock.uiIdent);
    // Get device state. Some of the fields are not present in the old scandata format.
    // They will not be displayed in that case
    unsigned int uiDevError, uiContWarn, uiContError;
    bool bSuccess = soScan.DeviceBlock.xbState.GetUnsignedValue("bDeviceError", uiDevError);
    bSuccess &= soScan.DeviceBlock.xbState.GetUnsignedValue("bContaminationWarning", uiContWarn);
    bSuccess &= soScan.DeviceBlock.xbState.GetUnsignedValue("bContaminationError", uiContError);
    if(bSuccess == true)
    {
      printf("Device error: %u, cm_warning: %u, cm_error: %u\n", 
        uiDevError, uiContWarn, uiContError);
    }

    // Device name (optional)
    if(soScan.aDeviceName.uiFlexArrayLength == 1)
    {
      printf("Devicename: %s\n", soScan.aDeviceName.aFlexArrayData[0].c_str());
    }

    // Time and date block (optional)
    if(soScan.aTimeBlock.uiFlexArrayLength == 1)
    {
      const SDateTime &rDateTime = soScan.aTimeBlock.aFlexArrayData[0];
#ifndef LMS100_OLD_SCANDATA
      printf("Date: %02u.%02u.%04u %02u:%02u:%02u.%03u\n", 
        rDateTime.usiDay, rDateTime.usiMonth, rDateTime.uiYear,
        rDateTime.usiHour, rDateTime.usiMinute, rDateTime.usiSec, 
        rDateTime.udiUSec / 1000);
#else
      printf("Date: %02u.%02u.%04u %02u:%02u:%02u.%03u timezone: %d\n", 
        rDateTime.usiDay, rDateTime.usiMonth, rDateTime.uiYear,
        rDateTime.usiHour, rDateTime.usiMinute, rDateTime.usiSec, 
        rDateTime.uiMSec, rDateTime.siTimeZone);      
#endif
    }

    // Measurement parameters
    double dScanFreq = static_cast<double>(soScan.MeasurementParam1Block.udiScanFreq) / 100.0f;
    double dMeasFreq = static_cast<double>(soScan.MeasurementParam1Block.udiMeasFreq) / 100.0f;
    printf("Scan frequency: %3.2f Hz, measurement frequency: %3.2f Hz\n", 
       dScanFreq, dMeasFreq);

    // First 16-bit data channel
    // Also calculate index of 90?beam for scan polling below    
    if(soScan.aDataChannel16.uiFlexArrayLength > 0)
    {
      const SDataChannelHdr &rDataChannelHdr = 
        soScan.aDataChannel16.aFlexArrayData[0].DataChannelHdr;
      // Convert units accordingly
      double dAngleResDeg = static_cast<double>(rDataChannelHdr.uiAngleRes) / 10000.0f;
      double dStartAngleDeg = static_cast<double>(rDataChannelHdr.diStartAngle) / 10000.0f;

      printf("16-bit channel 1:\n  type: %s, angle res: %1.4f deg\n",
        rDataChannelHdr.aContentType.data.c_str(), dAngleResDeg);
      printf("  start angle: %3.2f deg, scale: %3.2f, scale offset: %3.2f\n",
        dStartAngleDeg, rDataChannelHdr.dScaleFactor, rDataChannelHdr.dScaleOffset);

      // Calculate index of 90?beam            
      ulDeg90_index = static_cast<unsigned int>(((90.0f - dStartAngleDeg) / dAngleResDeg) + 0.5f);
    }
  }
  else
  {
    // Only display and process new scan telegrams
    if(soScan.StatusBlock.uiTelegramCount != uiLastTgm)
    {
      ++ulProcessedScanCounter;
      uiLastTgm = soScan.StatusBlock.uiTelegramCount;

      // Display encoder information (optional data)
      if(soScan.aEncoderBlock.uiFlexArrayLength == 1)
      {
        printf("encoder: %010u, speed: %+6d, ", 
          soScan.aEncoderBlock.aFlexArrayData[0].udiEncoderPos, 
          soScan.aEncoderBlock.aFlexArrayData[0].iEncoderSpeed);
      }
      // Display status information
      printf("scan count: %05u, tgm count: %05u, ",
        soScan.StatusBlock.uiScanCount, soScan.StatusBlock.uiTelegramCount);

      // Average 90?angle (if 16-bit channel 1 is active)
      // Note: It is not checked that channel 1 contains distance information here...
      if(soScan.aDataChannel16.uiFlexArrayLength > 0)
      {
        if(soScan.aDataChannel16.aFlexArrayData[0].aData.uiFlexArrayLength > ulDeg90_index)
        {
          dScanBeamAvg += soScan.aDataChannel16.aFlexArrayData[0].aData.aFlexArrayData[ulDeg90_index];
          ++ulAvgCounter;

          if(ulAvgCounter == 100)
          {
            dScanBeamAvg /= 100.0f;
            printf("avg: %5.2f", dScanBeamAvg);
            ulAvgCounter = 0;
            dScanBeamAvg = 0.0f;
          }
        }
      }
      // Carriage return to overwrite last output
      printf("\r");
    }
  }

  return ulProcessedScanCounter;
}

void PollScans(CLms100SopasInterface& rSopas)
{  
  printf("\n=> Polled scandata processing\n");
  // --- Get one scan and display detailed info, initialize moving average --- //
  SLms100Scan soScan; // Object is re-used
  if(rSopas.GetScanData(soScan) == true)
  {
    (void)ProcessScan(soScan, true);

    // --- Scandata polling --- //
    // The following loop queries scandata device variable.
    // The loop may be faster than scans are updated, limit processing to new scans
    // by looking at telegram counter.    
    printf("\nPolling scandata\n");

    unsigned int ulProcessedScanCounter = 0;
    while((rSopas.GetScanData(soScan) == true) && (ulProcessedScanCounter < 1000))
    {
      ulProcessedScanCounter = ProcessScan(soScan, false);
    }
  }
  else
  {
    printf("Sopas error code: %04x\n", rSopas.GetSopasErrorCode());
  }
  printf("\n");
}

void SubscribeScans(CLms100SopasInterface& rSopas)
{
  printf("\n=> Event based scandata processing\n");
  if(rSopas.SubscribeScanDataEvent(true) == true)
  {
    printf("Polling\n");
    rSopas.PollAsyncAnswers(10000); // Using default callback set by SetAsyncCallback    

    printf("\nUnsubscribing:");
    if(rSopas.SubscribeScanDataEvent(false) == true)
    {
      printf("\ndone");
    }
    else
    {
      printf("\nSopas error code: %04x", rSopas.GetSopasErrorCode());
    }
    // There might be still an event in the queue 
    // if sent during or shortly after unsubscribing.
    // Usually processed by call to SubscribeEvent itself...
    printf("\nClearing command queue"); 
    rSopas.PollAsyncAnswers(1000);
  }
  else
  {
    printf("Sopas error code: %04x\n", rSopas.GetSopasErrorCode());
  }
}

void SopasAsyncCallback(const SDecodedAnswer& rAnswer, CBinaryDataStream& rStream, CDeserializer& rDeserializer, void* pUser)
{
  static bool bFirstCall = true;

  g_Cont++;

  if(rAnswer.eAnswerType == SDecodedAnswer::EVENT_RESULT)
  {
    // Process subscribed events
    if(rAnswer.coName.compare("LMDscandata") == 0)
    {
      // Process scandata event
      SLms100Scan soScan;
      if(soScan.Deserialize(rDeserializer, rStream) == true)
      {
        // Initialize average and display detailed scan information on first run
//        (void)ProcessScan(soScan, bFirstCall);
//       bFirstCall = false;
		 if (g_Cont%2 == 0)
		 {
			return;
		 }

		int nGridFront = 50000;
		int nGridSide = 20000;
		int nGridBack = 20000;
		int nGridSize = 100;

		int nDataLen = soScan.aDataChannel16.aFlexArrayData[0].aData.uiFlexArrayLength;
		double dAngRes = soScan.aDataChannel16.aFlexArrayData[0].DataChannelHdr.uiAngleRes/10000.0;
		SOPAS_UInt16* pDist = soScan.aDataChannel16.aFlexArrayData[0].aData.aFlexArrayData;
		double dStartAng = soScan.aDataChannel16.aFlexArrayData[0].DataChannelHdr.diStartAngle/10000.0;
		
		cv::Mat img = cv::Mat::zeros((nGridFront+nGridBack)/nGridSize+1,nGridSide*2/nGridSize+1,CV_8UC3);
		img.col((nGridSide/nGridSize)+1) = cvScalar(255,255,255);
		img.row((nGridFront/nGridSize)+1) = cvScalar(255,255,255);
		cv::Vec<uchar,3> color_(255,255,255);

		for (int i = 0; i < nDataLen; i++)
		{
			double dAng = dStartAng + i*dAngRes;
			double dDist = (double)pDist[i]*2.0;
			double X = cos(dAng/180.f*CV_PI)*dDist;
			double Y = sin(dAng/180.f*CV_PI)*dDist;
			if (X>=-1*nGridSide && X<=nGridSide && Y<=nGridFront && Y>=-1.f*nGridBack)
			{
				X = nGridSide + X;
				Y = nGridFront - Y;
				double XX = floor(X/nGridSize);
				double YY = floor(Y/nGridSize);
				if (YY>=0 && YY < img.rows && XX>=0 && XX < img.cols)
				{
					img.at<cv::Vec<uchar,3>>(YY,XX) = color_;
				}
			}
		}

		cvNamedWindow("img",1);
		imshow("img",img);
		cv::waitKey(1);

		g_SickLms511DataTransfer.SendData(g_szChannelName, pDist, nDataLen);
		printf("Sick lms511 data send:<%s> %d\n", g_szChannelName, g_Cont);
      }
      else
      {
        printf("Error during scandata deserialization (event based)\n");
      }
    }
  }
}
