#ifndef TCPIP_BOOST_IMPL_H
#define TCPIP_BOOST_IMPL_H

#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/function.hpp>

typedef boost::function<void(unsigned char*,int,void*)> FunctionTcpIpCallBack;

class CClientBoostImpl
{
public:
	CClientBoostImpl();
	~CClientBoostImpl();

	void SetCallBack(FunctionTcpIpCallBack TcpIpDataCallBack, void* pUser);
	int Connect(int nPort,char* pAddress,char* pLocalAddress = 0);
	int Start();
	int ReceiveMsg(unsigned char* msg,int len);
	int SendMsg(unsigned char* msg,int len);
//	int DisConnect();
	int Close();

private:
	void ThreadTcpIpDataReadLoop();
	void handler(const boost::system::error_code& error, std::size_t bytes_transferred);
	boost::asio::io_service m_iosev;
	boost::asio::ip::tcp::socket m_Socket;
	boost::asio::ip::tcp::endpoint m_RemoteEndPoint;
	boost::asio::ip::tcp::endpoint m_LocalEndPoint;
	boost::thread* m_pThread;
	unsigned char* m_pBuff;
	unsigned int m_nBufLen;
	unsigned int m_bytes_transferred;
	FunctionTcpIpCallBack m_pCallBack;
	void* m_pUser;
};

#endif