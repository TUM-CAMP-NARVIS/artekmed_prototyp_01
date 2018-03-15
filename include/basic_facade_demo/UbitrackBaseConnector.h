//
// Created by Ulrich Eck on 15.03.18.
//

#ifndef BASIC_FACADE_DEMO_UBITRACKBASECONNECTOR_H
#define BASIC_FACADE_DEMO_UBITRACKBASECONNECTOR_H

#include <memory>
#include <vector>
#include <condition_variable>
#include <mutex>



// include Ubitrack BasicFacade
#include <utFacade/BasicFacadeTypes.h>
#include <utFacade/BasicFacade.h>

typedef unsigned long long int TimestampT;

class UbitrackBaseConnector {
public:
    explicit UbitrackBaseConnector(const std::string& _components_path);
    virtual ~UbitrackBaseConnector() = default;

    /*
     * waits for an image to be pushed (left-eye) and returns its timestamp
     */
    TimestampT wait_for_frame();


    /*
     * livecycle management for the utconnector
     * not thread-safe
     */
    virtual bool initialize(const std::string& _utql_filename);
    virtual bool teardown();

    virtual bool start();
    virtual bool stop();

    inline TimestampT now() {
        return m_utFacade->now();
    }

protected:

    void set_new_frame(TimestampT ts);

    std::unique_ptr<Ubitrack::Facade::BasicFacade> m_utFacade;
    bool m_haveNewFrame;
    TimestampT m_lastTimestamp;

    bool m_dataflowLoaded;
    bool m_dataflowRunning;

private:
    std::mutex m_waitMutex;
    std::condition_variable m_waitCondition;
};


#endif //BASIC_FACADE_DEMO_UBITRACKBASECONNECTOR_H
