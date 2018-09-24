#ifndef VIRTUALBUS_H
#define VIRTUALBUS_H

#define VIRTUALBUSDLL

#include <windows.h>

#include "lcm/lcm-cpp.hpp"
#include "lcm/lcm.h"
#include "lcm/lcm_coretypes.h"

#include "VirtualBusLog.h"

/*
** messages used in internal system
*/
#define LCM_MSG_INTERNAL_SYS_MSG_CHANNEL "LCM_MSG_INTERNAL_SYS_MSG_CHANNEL"

typedef struct 
{
	int msgId;
}INTERNAL_SYS_MSG;

enum
{
	INTERNAL_SYS_MSG_ID_QUIT
};

/*
** Class VirtualSwitchBus
*/
class VIRTUALBUSDLL VirtualBus
{
public:
	VirtualBus();
	VirtualBus(const std::string& url);

	~VirtualBus();

	void startReceive();
	void stopReceive();
	
	//通过lcm发送消息(方式1)        
	template<class MessageType>
    bool publish(const std::string& channel, const MessageType* msg);
	
	//通过lcm发送消息(方式2)  
    bool publish(const std::string& channel, void *data, unsigned int datalen);
	template <typename MessageHandlerClass>
	bool subscribe(const std::string& channel,
    void (MessageHandlerClass::*handlerMethod)(const lcm::ReceiveBuffer* rbuf, const std::string& channel),
		  MessageHandlerClass* handler);

	template <typename MessageType, typename MessageHandlerClass>
	bool subscribe(const std::string& channel,
	void (MessageHandlerClass::*handlerMethod)(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const MessageType* msg),
         MessageHandlerClass* handler);

private:
	lcm::LCM m_lcm;
	char m_internalChannel[128];
	volatile bool m_receiveFlag; 
	void receivedInternalSysMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan);  //主要用于LCM停止接收消息
	
private:
	HANDLE m_hThread;
	static DWORD WINAPI threadFun(LPVOID wparam);
	void run();
};

//通过lcm发送消息(实现方式1)
	template<class MessageType>
    bool VirtualBus::publish(const std::string& channel, const MessageType* msg)
	{
		VIRTUALBUS_INFO_LOG(1,("[INFO] %s, enter publish(2).\r\n",__FUNCTION__));
		int hResult = 0;
		if(!m_lcm.good())
		{
			VIRTUALBUS_ERR_LOG("[ERR] %s, calling good() in LCM err.\r\n",__FUNCTION__);
			return (false);
		}
		hResult = m_lcm.publish(channel, msg);
		if( -1 == hResult)
		{
			VIRTUALBUS_ERR_LOG("[ERR] %s, calling publish() in LCM err.\r\n",__FUNCTION__);
			return (false);
		}
		VIRTUALBUS_INFO_LOG(1,("[INFO] %s, leave publish(2).\r\n",__FUNCTION__));
		return (true);
	}

	template <typename MessageHandlerClass>
	bool VirtualBus::subscribe(const std::string& channel,
    void (MessageHandlerClass::*handlerMethod)(const lcm::ReceiveBuffer* rbuf, const std::string& channel),
		  MessageHandlerClass* handler)
	{
		VIRTUALBUS_INFO_LOG(1,("[INFO] %s, enter subscribe(2).\r\n",__FUNCTION__));
		if(!m_lcm.good())
		{
			VIRTUALBUS_ERR_LOG("[ERR] %s, calling good() in LCM err.\r\n",__FUNCTION__);
			return (false);
		}
		if(channel == std::string(m_internalChannel))
		{
			VIRTUALBUS_ERR_LOG("[ERR] %s, channel name is not good.because it has been used for internal message,please select another channel name.\r\n",__FUNCTION__);			
			return (false);
		}
		m_lcm.subscribe(channel, handlerMethod, handler);
		VIRTUALBUS_INFO_LOG(1,("[INFO] %s, leave subscribe(2).\r\n",__FUNCTION__));
		return (true);
	}

   template <typename MessageType, typename MessageHandlerClass>
   bool VirtualBus::subscribe(const std::string& channel,
   void (MessageHandlerClass::*handlerMethod)(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const MessageType* msg),
         MessageHandlerClass* handler)
   {
#if 1
	   	VIRTUALBUS_INFO_LOG(1,("[INFO] %s, enter subscribe(3).\r\n",__FUNCTION__));
		if(!m_lcm.good())
		{
			VIRTUALBUS_ERR_LOG("[ERR] %s, calling good() in LCM err.\r\n",__FUNCTION__);
			return (false);
		}
		if(channel == std::string(m_internalChannel))
		{
			VIRTUALBUS_ERR_LOG("[ERR] %s, channel name is not good.because it has been used for internal message,please select another channel name.\r\n",__FUNCTION__);			
			return (false);
		}
		m_lcm.subscribe(channel, handlerMethod, handler);
		VIRTUALBUS_INFO_LOG(1,("[INFO] %s, leave subscribe(3).\r\n",__FUNCTION__));
#endif
		return (true);
   }
#endif
