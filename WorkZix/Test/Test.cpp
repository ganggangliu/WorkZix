#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>

using namespace boost;
using namespace boost::asio;

//////////////////////////////////////////////////////////////////////////
//boost TcpIp operator
//////////////////////////////////////////////////////////////////////////
int main1(int argc, char* argv[])
{
	// 所有asio类都需要io_service对象
	io_service iosev;
	// socket对象
	ip::tcp::socket socket(iosev);
	// 连接端点，这里使用了本机连接，可以修改IP地址测试远程连接
	ip::tcp::endpoint ep(ip::address_v4::from_string("127.0.0.1"), 1000);
	// 连接服务器
	boost::system::error_code ec;
	ip::tcp::endpoint LocalEndPt(ip::address_v4::from_string("127.0.0.1"), 1000);
	socket.bind(LocalEndPt,ec);
	socket.connect(ep,ec);
	// 如果出错，打印出错信息
	if(ec)
	{
		std::cout << boost::system::system_error(ec).what() << std::endl;
		return -1;
	}
	// 接收数据
	char buf[100];
	size_t len=socket.read_some(boost::asio::buffer(buf), ec);
	std::cout.write(buf, len);

	return 0;
}

//////////////////////////////////////////////////////////////////////////
//boost TcpIp operator
//////////////////////////////////////////////////////////////////////////
long g_nCont = 0;
void handler_read1(const boost::system::error_code& e, std::size_t bytes_transferred)
{
//	if (bytes_transferred != 0)
	{
		std::cout << "CallBack:" << g_nCont++ << std::endl;
	}
	
}

class CTest
{
public:
	CTest():
	  m_socket(m_iosev)
	  {

	  }
	  int Start()
	  {
		  system::error_code ec;
		  ip::tcp::endpoint RemoteEndPoint(ip::address_v4::from_string("172.29.150.4"), 55555);
		  m_socket.connect(RemoteEndPoint, ec);
		  /*m_socket.async_read_some(boost::asio::buffer(m_Buff),
			  boost::bind(&CTest::handler_read,
			  this,
			  boost::asio::placeholders::error,
			  boost::asio::placeholders::bytes_transferred));*/
// 		  m_socket.async_read_some(boost::asio::buffer(m_Buff),
// 			  handler_read1);
// 		  m_iosev.run();

		  m_socket.async_read_some(boost::asio::buffer(m_Buff),
			  handler_read1);

		  boost::thread thread(boost::bind(&boost::asio::io_service::run, &m_iosev));
		  
		  long nCont = 0;
		  while(1)
		  {
			  Sleep(1);
//			  std::cout << "Req:" << nCont++ << std::endl;
			  m_socket.async_read_some(boost::asio::buffer(m_Buff),
				  handler_read1);
		  }
		  Sleep(INFINITE);

		  return 1;
	  }

private:
	void handler_read(const boost::system::error_code& e, std::size_t bytes_transferred)
	{
		std::cout << bytes_transferred << std::endl;
	}
	void ServerLoop()
	{
		while(1)
		{
			m_socket.async_read_some(boost::asio::buffer(m_Buff),
				handler_read1);
			m_iosev.run();
		}
	}
	io_service m_iosev;
	ip::tcp::socket m_socket;
	char m_Buff[10];
	std::string m_Data;
};

void handle_wait(const boost::system::error_code& error,  
                     boost::asio::deadline_timer& t)
{
	int nnn = 0;
}

void print(const boost::system::error_code&)  
    {  
        std::cout << ("Hello, world!") << std::endl;  
    } 

int main(int argc, char* argv[])
{
	boost::asio::io_service io;  
  
    boost::asio::deadline_timer t(io, boost::posix_time::seconds(3));  
    t.async_wait(print);  
    io.run();

	return 0;
// 	boost::asio::io_service io;  
// 	boost::asio::deadline_timer t(io);  
// 	size_t a = t.expires_from_now(boost::posix_time::seconds(1));  
// 	int count = 0;  
// 	t.async_wait(boost::bind(&handle_wait,
// 		boost::asio::placeholders::error,  
// 		boost::ref(t)));
// 	io.run();      



	CTest aaa;

	aaa.Start();

	Sleep(INFINITE);

	return 0;
}


//////////////////////////////////////////////////////////////////////////
//boost multi-thread and lock
//////////////////////////////////////////////////////////////////////////
boost::shared_ptr<boost::thread> g_pThread1,g_pThread2;
boost::mutex io_mutex;

int g_Cont = 0;
void ThreadBody(int id)  
{  
	try  
	{  
		while( true )  
		{  
//			boost::mutex::scoped_lock lock(io_mutex);
// 			lock.lock();
// 			lock.unlock();
			io_mutex.lock();
			io_mutex.unlock();
			printf("%d:%d\n",id ,g_Cont++);
			boost::this_thread::interruption_point();  
//			std::cout << "Processing..." << std::endl;
			boost::posix_time::hours h(2);
			boost:this_thread::sleep(boost::posix_time::millisec(1000));  
		}   
	}  
	catch(...)  
	{   
		std::cout << "Interrupt exception was thrown." << std::endl;   
	}   

	std::cout << "Leave Thread." << std::endl;  
}  

void main1()  
{  

	g_pThread1 = shared_ptr<thread>(new thread(bind(&ThreadBody,1)));
	g_pThread2 = shared_ptr<thread>(new thread(bind(&ThreadBody,2)));

	while(1)
	{
		char key = getchar();
		if (key == 's')
		{
			g_pThread1->interrupt();
			g_pThread2->interrupt();
			g_pThread1 = shared_ptr<thread>(new thread(bind(&ThreadBody,1)));
			g_pThread2 = shared_ptr<thread>(new thread(bind(&ThreadBody,2)));
		}
		if (key == 'e')
		{
			g_pThread1->interrupt();
			g_pThread2->interrupt();
		}
	}
}
// 
// 
// using boost::asio::local::stream_protocol;
// 
// boost::asio::io_service io;
// 
// stream_protocol::socket s(io);
// s.connect(stream_protocol::endpoint(address));
// 
// s.async_read_some(aBuffer, aCallback);  // start async_read
// 
// boost::thread thread(boost::bind(&boost::asio::io_service::run, &io));
// 
// usleep(1000000); // do some stuff    
// 
// boost::asio::read(bBuffer);  // request a blocking read