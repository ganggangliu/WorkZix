#ifndef LCM_RECEIVER_4_L
#define LCM_RECEIVER_4_L

#include "linux.h"
#include <lcm/lcm-cpp.hpp>

typedef void(*lpLcmRecFunc)(void*, void*);

//template <class T>
//void* threadReadData(void* pUser);

template <class T>
class CLcmReceive
{
public:
    CLcmReceive(){}
    CLcmReceive(std::string& szURL,std::string& szChanNmae)
    {
        Init(szURL,szChanNmae);
    }

    CLcmReceive(std::string& szChanNmae)
    {
        std::string szURL = "udpm://239.255.76.67:7667?ttl=1";
        Init(szURL,szChanNmae);
    }

    ~CLcmReceive()
    {
        pthread_mutex_destroy(&m_mutex);
        if (m_plcmIpc)
        {
            delete m_plcmIpc;
            m_plcmIpc = NULL;
        }
    }

    void Init(std::string& szURL,std::string& szChanNmae)
    {
        m_plcmIpc = new lcm::LCM(szURL);
        m_plcmIpc->subscribe(szChanNmae, &CLcmReceive::handleMessage, this);
        pthread_mutex_init (&m_mutex,NULL);
        m_RepeatGetCont = 0;
        m_hLcmCallBack = 0;
        m_hThread = 0;
        m_pUser = NULL;
    }

    long Start()
    {
        int ret;
        ret=pthread_create(&m_id,NULL,threadReadData,this);
        if(ret!=0)
        {
            return 1;
        }
        else
        {
            return 0;
        }
        return 1;
    }

    long BeginThread()
    {
        while(0 == m_plcmIpc->handle());
        return 0;
    }


    void handleMessage(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const T* msg)
    {
        pthread_mutex_lock (&m_mutex);
        m_Msg = *msg;
        m_RepeatGetCont = 0;
        pthread_mutex_unlock(&m_mutex);
        if (m_hLcmCallBack)
        {
            (*m_hLcmCallBack)(&m_Msg, m_pUser);
        }
    }

    long GetData(T& Msg)
    {
        pthread_mutex_lock (&m_mutex);
        Msg = m_Msg;
        m_RepeatGetCont++;
        pthread_mutex_unlock(&m_mutex);
        return m_RepeatGetCont;
    }

    void SetCallBack(lpLcmRecFunc hLcmCallBack, void* pUser = NULL)
    {
        m_hLcmCallBack = hLcmCallBack;
        m_pUser = pUser;
    }

    long Send(std::string szChannle, T& Msg)
    {
        return m_plcmIpc->publish(szChannle, &Msg);
    }

    void* threadReadData(void* pUser)
    {
        CLcmReceive<T>* pLcmRec = (CLcmReceive<T>*) pUser;
        pLcmRec->BeginThread();
        return 1;
    }

public:
    lcm::LCM* m_plcmIpc;
    T m_Msg;
    void* m_pUser;
    pthread_mutex_t m_mutex;
    long m_RepeatGetCont;
    lpLcmRecFunc m_hLcmCallBack;
    pthread_t m_hThread;
    long nTest;
    pthread_t m_id;
};

//template <class T>
//void* threadReadData(void* pUser)
//{
//    CLcmReceive<T>* pLcmRec = (CLcmReceive<T>*) pUser;
//    pLcmRec->BeginThread();

//    return 0;
//}

#endif
