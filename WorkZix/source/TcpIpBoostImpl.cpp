#include "TcpIpBoostImpl.h"
using namespace boost;
using namespace boost::asio;

void CClientBoostImpl::ThreadTcpIpDataReadLoop()
{
	if (m_pCallBack.empty())
	{
		return;
	}
	try  
	{  
		while( true )  
		{  
			boost::this_thread::interruption_point();
			std::size_t nRt = m_Socket.read_some(boost::asio::buffer(m_pBuff, m_nBufLen));
			if (nRt == 0)
			{
				boost::this_thread::sleep(boost::posix_time::millisec(1));
				continue;
			}
			if (!m_pCallBack.empty())
			{
				((m_pCallBack))(m_pBuff, nRt, m_pUser);
			}
		}   
	}  
	catch(...)  
	{   
		std::cout << "Thread end!\n" << std::endl;   
	}   
}

CClientBoostImpl::CClientBoostImpl():
m_Socket(m_iosev)
{
	m_pThread = 0;
	m_bytes_transferred = 0;
	m_pUser = 0;
	m_nBufLen = 10000;
	m_pBuff = new unsigned char[m_nBufLen];
	return;
}

CClientBoostImpl::~CClientBoostImpl()
{
	if (m_pBuff)
	{
		delete [] m_pBuff;
		m_pBuff = 0;
	}
	if (m_pThread)
	{
		delete m_pThread;
		m_pThread = 0;
	}
}

void CClientBoostImpl::SetCallBack(FunctionTcpIpCallBack TcpIpDataCallBack, void* pUser)
{
	m_pCallBack = TcpIpDataCallBack;
	m_pUser = pUser;
}
int CClientBoostImpl::Connect(int nPort,char* pAddress,char* pLocalAddress)
{
	m_Socket.open(boost::asio::ip::tcp::v4());
	system::error_code ec;
	ip::tcp::endpoint RemoteEndPoint(ip::address_v4::from_string(pAddress), nPort);
	m_RemoteEndPoint = RemoteEndPoint;
	if (pLocalAddress != 0)
	{
		ip::tcp::endpoint LocalEndPoint(ip::address_v4::from_string(pLocalAddress), 0);
		m_Socket.bind(LocalEndPoint,ec);
		if(ec)
		{
			std::cout << boost::system::system_error(ec).what() << std::endl;
			return 0;
		}
	}
	m_Socket.connect(m_RemoteEndPoint, ec);
	if(ec)
	{
		std::cout << boost::system::system_error(ec).what() << std::endl;
		return 0;
	}

	return 1;
}
int CClientBoostImpl::Start()
{
	if (m_pThread)
	{
		delete m_pThread;
		m_pThread = 0;
	}

	m_pThread = new boost::thread(boost::bind(&CClientBoostImpl::ThreadTcpIpDataReadLoop, this));

	return 1;
}
int CClientBoostImpl::ReceiveMsg(unsigned char* msg,int len)
{
	if (m_pCallBack)
	{
		return 0;
	}

	system::error_code ec;
	std::size_t nRt = m_Socket.read_some(boost::asio::buffer(msg, len), ec);

	return nRt;
// 	m_iosev.reset();
// 	io_service ServTimer;
// 	boost::asio::deadline_timer timer(ServTimer);
// 	if (nTimeOutMilis <= 0)
// 		timer.expires_from_now(boost::posix_time::hours(24));
// 	else
// 		timer.expires_from_now(boost::posix_time::millisec(nTimeOutMilis));
// 	timer.async_wait(boost::bind(&io_service::stop,  &m_iosev));
// 	boost::thread thread(boost::bind(&boost::asio::io_service::run, &ServTimer));
// 	m_Socket.async_read_some(
// 		boost::asio::buffer(msg, len),
// 		boost::bind(
// 			&CClientBoostImpl::handler, 
// 			this,
// 			boost::asio::placeholders::error,       
// 			boost::asio::placeholders::bytes_transferred
// 		)
// 	);
// 	std::size_t nRt = m_iosev.run();
// 	m_iosev.stop();
// 	ServTimer.stop();
// 
// 	return (nRt==0?0:m_bytes_transferred);
}
int CClientBoostImpl::SendMsg(unsigned char* msg,int len)
{
	system::error_code ec;
	std::size_t nRt = m_Socket.write_some(boost::asio::buffer(msg, len), ec);

	return nRt;
// 	m_iosev.reset();
// 	io_service ServTimer;
// 	boost::asio::deadline_timer timer(ServTimer);
// 	if (nTimeOutMilis <= 0)
// 		timer.expires_from_now(boost::posix_time::hours(24));
// 	else
// 		timer.expires_from_now(boost::posix_time::millisec(nTimeOutMilis));
// 	timer.async_wait(boost::bind(&io_service::stop,  &m_iosev));
// 	boost::thread thread(boost::bind(&boost::asio::io_service::run, &ServTimer));
// 	m_Socket.async_send(
// 		boost::asio::buffer(msg, len),
// 		boost::bind(
// 			&CClientBoostImpl::handler, 
// 			this,
// 			boost::asio::placeholders::error,       
// 			boost::asio::placeholders::bytes_transferred
// 		)
// 	);
// 	std::size_t nRt = m_iosev.run();
// 	ServTimer.stop();
// 
// 	return (nRt==0?0:m_bytes_transferred);


}
// int CClientBoostImpl::DisConnect()
// {
// 	m_Socket.close();
// 
// 	return 1;
// }
int CClientBoostImpl::Close()
{
	m_Socket.close();
	m_pThread->interrupt();
	boost::this_thread::sleep(boost::posix_time::millisec(500));

	return 1;
}

void CClientBoostImpl::handler(const boost::system::error_code& error, std::size_t bytes_transferred)
{
	m_bytes_transferred = bytes_transferred;
}