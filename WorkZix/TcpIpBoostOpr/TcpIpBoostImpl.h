#include "TcpIpBoostOpr.h"
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/function.hpp>

using namespace boost;
using namespace boost::asio;

class CClientBoostImpl
{
public:
	CClientBoostImpl();
	~CClientBoostImpl();

	void SetCallBack(function<void(char*,int,void*)> TcpIpDataCallBack, void* pUser);
	int Connect(int nPort,char* pAddress,char* pLocalAddress = 0);
	int Start();
	int ReceiveMsg(char* msg,int len);
	int SendMsg(char* msg,int len);
	int DisConnect();
	int Close();

//friend void ThreadTcpIpDataReadLoop(CClientBoostImpl* pClient);

private:
	void ThreadTcpIpDataReadLoop();
	void handler(const boost::system::error_code& error, std::size_t bytes_transferred);
	io_service m_iosev;
	ip::tcp::socket m_Socket;
	ip::tcp::endpoint m_RemoteEndPoint;
	ip::tcp::endpoint m_LocalEndPoint;
	boost::thread* m_pThread;
	char* m_pBuff;
	unsigned int m_nBufLen;
	unsigned int m_bytes_transferred;
//	lpTcpIpDataRecFunc m_pCallBack;
	function<void(char*,int,void*)> m_pCallBack;
	void* m_pUser;
};