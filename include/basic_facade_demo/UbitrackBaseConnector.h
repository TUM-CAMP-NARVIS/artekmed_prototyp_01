//
// Created by Ulrich Eck on 15.03.18.
//

#ifndef BASIC_FACADE_DEMO_UBITRACKBASECONNECTOR_H
#define BASIC_FACADE_DEMO_UBITRACKBASECONNECTOR_H

#include <memory>
#include <vector>
#include <condition_variable>
#include <mutex>
#include <chrono>


// include Ubitrack BasicFacade
#include <utFacade/BasicFacadeTypes.h>
#include <utFacade/BasicFacade.h>

typedef unsigned long long int TimestampT;

class UbitrackBaseConnector {
public:
    explicit UbitrackBaseConnector(const std::string& _components_path);
    virtual ~UbitrackBaseConnector() = default;

    /*
     * waits for an image to be pushed and returns its timestamp
     */
    TimestampT wait_for_frame();
    // returns 0 on success, 1 if no frame was received, 2 on timeout
    unsigned int wait_for_frame_timeout(unsigned int timeout_ms, TimestampT& ts);

    /*
     * livecycle management for the utconnector
     * not thread-safe
     */
    virtual bool initialize(const std::string& _utql_filename);
    virtual bool teardown();

    virtual bool start();
    virtual bool stop();

    TimestampT now();

    bool isRunning();
    bool isLoaded();

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
