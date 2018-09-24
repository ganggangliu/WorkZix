#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>

using namespace boost;
using namespace boost::asio;

int main_test_tcpip_sync(int argc, char* argv[]);
int main_test_tcpip_async(int argc, char* argv[]);
void main_test_multi_thread();
int main_test_serial_port();

int main(int argc, char* argv[])
{
//	main_test_serial_port();
	main_test_tcpip_async(argc, argv);

	return 1;
}

//////////////////////////////////////////////////////////////////////////
//boost TcpIp operator sync
//////////////////////////////////////////////////////////////////////////
int main_test_tcpip_sync(int argc, char* argv[])
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
//boost TcpIp operator async
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
		  ip::tcp::endpoint RemoteEndPoint(ip::address_v4::from_string("192.168.1.10"), 55555);
		  m_socket.connect(RemoteEndPoint, ec);
		  //m_socket.async_read_some(boost::asio::buffer(m_Buff),
			 // boost::bind(&CTest::handler_read,
			 // this,
			 // boost::asio::placeholders::error,
			 // boost::asio::placeholders::bytes_transferred));
// 		  m_socket.async_read_some(boost::asio::buffer(m_Buff),
// 			  handler_read1);
// 		  m_iosev.run();

		  io_service aaa;
		  boost::asio::deadline_timer timer(aaa);
		  timer.expires_from_now(boost::posix_time::seconds(5));
		  timer.async_wait(boost::bind(&io_service::stop,  &m_iosev));
		  boost::thread thread(boost::bind(&boost::asio::io_service::run, &aaa));
		
		  m_socket.async_read_some(boost::asio::buffer(m_Buff),
			  handler_read1);

		  std:size_t nRt = m_iosev.run();
		  if (nRt == 0)
		  {
			  printf("Over Time!\n");
		  }
		  else
		  {
			  timer.cancel();
			  printf("data get!\n");
		  }

//		  boost::thread thread(boost::bind(&boost::asio::io_service::run, &m_iosev));
		  Sleep(INFINITE);

		  long nCont = 0;
		  while(1)
		  {
			  Sleep(1);
//			  std::cout << "Req:" << nCont++ << std::endl;
			  m_socket.async_read_some(boost::asio::buffer(m_Buff),
				  handler_read1);
			  m_iosev.run();
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

int main_test_tcpip_async(int argc, char* argv[])
{
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

void main_test_multi_thread()  
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

//////////////////////////////////////////////////////////////////////////
//boost serial_port 
//////////////////////////////////////////////////////////////////////////
int main_test_serial_port()
{
	try  
	{  
		boost::asio::io_service io;  
		boost::asio::serial_port sp(io, "COM1");  

		sp.set_option(boost::asio::serial_port::baud_rate(38400));  
		sp.set_option(boost::asio::serial_port::flow_control());  
		sp.set_option(boost::asio::serial_port::parity());  
		sp.set_option(boost::asio::serial_port::stop_bits());  
		sp.set_option(boost::asio::serial_port::character_size(8));  

		boost::asio::write(sp, boost::asio::buffer("\n", 1));  

		char buf[101];  
		boost::system::error_code err;  
		while (true)  
		{  
			size_t ret = sp.read_some(boost::asio::buffer(buf, 100), err);  
			if (err)  
			{  
				std::cout << "read_some Error: " << err.message() << std::endl;  
				break;  
			}  
			else  
			{  
				buf[ret] = '\0';  
				std::cout << buf;  
			}  
		}  

		io.run();  
	}  
	catch (exception& err)  
	{  
//		std::cout << "Exception Error: " << err.what() << std::endl;  
	}  


	getchar();  
	return 0; 

// 	boost::asio::deadline_timer timer(io);  
// 	timer.expires_from_now(boost::posix_time::millisec(60000));  
// 	timer.async_wait(boost::bind(&boost::asio::serial_port::cancel,  &sp));

	Sleep(INFINITE);
	return 0;
}

#include <boost/function/function0.hpp>  
#include <boost/thread/thread.hpp>  
#include <iostream>  

class HelloWorld  
{  
public:  
	void hello()  
	{  
		std::cout<<"Hello world, I'm a thread!"<<std::endl;  
	}  
	void start()  
	{  
		boost::function0<void> f = boost::bind(&HelloWorld::hello, this);  
		boost::thread thrd(f);  
		thrd.join();  
	}  
};  

int main11()  
{  
	HelloWorld hello;  
	hello.start();  

	std::system("pause");  
	return 0;  
}  

//////////////////////////////////////////////////////////////////////////
//CallBack function
//////////////////////////////////////////////////////////////////////////
#include <boost/function/function1.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

class DemoA
{
public:
	typedef function<void(int)> func_t;
	func_t func;
	int n;

	DemoA(int i):n(i){}
	template<typename T>
	void accept(T f){func = f;}
	void run(){func(n);}
	void callbackFunc1(int i){}
};

void callbackFunc(int i)
{

}

void test4()
{
	DemoA de(10);
	DemoA::func_t f = boost::bind(&DemoA::callbackFunc1, &de,_1);  
	de.accept(f);
	de.run();
}