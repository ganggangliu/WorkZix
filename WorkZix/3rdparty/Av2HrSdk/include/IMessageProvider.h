#pragma once

#include "Av2HR_define.h"
#include "IMessageObserver.h"

#include <list>
#include <algorithm>

class IMessageProvider
{
public:
    IMessageProvider(void);
    virtual ~IMessageProvider(void);

public:
    int Attach(IMessageObserver* pObserver)
    {
        std::list<IMessageObserver*>::iterator itr = std::find(m_lstMessageObserver.begin(), m_lstMessageObserver.end(), pObserver);
        if (itr != m_lstMessageObserver.end())
        {
            return -1;
        }
        else
        {
            m_lstMessageObserver.push_back(pObserver);
            return 0;
        }
    }

    int Detach(IMessageObserver* pObserver)
    {
        std::list<IMessageObserver*>::iterator itr = std::find(m_lstMessageObserver.begin(), m_lstMessageObserver.end(), pObserver);
        if (itr != m_lstMessageObserver.end())
        {
            m_lstMessageObserver.erase(itr);

            return 0;
        }
        else
        {
            return -1;
        }
    }

    int Notify(const av2hr_position_t* p)
    {
        for (std::list<IMessageObserver*>::iterator itr = m_lstMessageObserver.begin();
            itr != m_lstMessageObserver.end();
            ++itr)
        {
            IMessageObserver* pObserver = *itr;
            if (NULL == pObserver)
            {
                continue;
            }

            pObserver->Av2HR_onNewPosition(p);
        }

        return 0;
    }

    int Notify(const av2hr_segment_t* p)
    {
        for (std::list<IMessageObserver*>::iterator itr = m_lstMessageObserver.begin();
            itr != m_lstMessageObserver.end();
            ++itr)
        {
            IMessageObserver* pObserver = *itr;
            if (NULL == pObserver)
            {
                continue;
            }

            pObserver->Av2HR_onNewSegment(p);
        }

        return 0;
    }

    int Notify(const av2hr_stub_t* p)
    {
        for (std::list<IMessageObserver*>::iterator itr = m_lstMessageObserver.begin();
            itr != m_lstMessageObserver.end();
            ++itr)
        {
            IMessageObserver* pObserver = *itr;
            if (NULL == pObserver)
            {
                continue;
            }

            pObserver->Av2HR_onNewStub(p);
        }

        return 0;
    }

    int Notify(const av2hr_profile_t* p)
    {
        for (std::list<IMessageObserver*>::iterator itr = m_lstMessageObserver.begin();
            itr != m_lstMessageObserver.end();
            ++itr)
        {
            IMessageObserver* pObserver = *itr;
            if (NULL == pObserver)
            {
                continue;
            }

            pObserver->Av2HR_onNewProfile(p);
        }

        return 0;
    }

    int Notify(const av2hr_metadata_t* p)
    {
        for (std::list<IMessageObserver*>::iterator itr = m_lstMessageObserver.begin();
            itr != m_lstMessageObserver.end();
            ++itr)
        {
            IMessageObserver* pObserver = *itr;
            if (NULL == pObserver)
            {
                continue;
            }

            pObserver->Av2HR_onNewMetaData(p);
        }

        return 0;
    }

    int Notify(const av2hr_msgtype_e msgType)
    {
        for (std::list<IMessageObserver*>::iterator itr = m_lstMessageObserver.begin();
            itr != m_lstMessageObserver.end();
            ++itr)
        {
            IMessageObserver* pObserver = *itr;
            if (NULL == pObserver)
            {
                continue;
            }

            pObserver->Av2HR_onMissingMessage(msgType);
        }

        return 0;
    }

private:
    std::list<IMessageObserver*> m_lstMessageObserver;
};

extern "C" IMessageProvider* GetMessageCodecProviderSingleton();
