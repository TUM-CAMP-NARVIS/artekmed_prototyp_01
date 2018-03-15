//
// Created by Ulrich Eck on 15.03.18.
//

#include "basic_facade_demo/UbitrackBaseConnector.h"

#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("BasicFacadeExample.UbitrackBaseConnector"));

// implementation of UbitrackBaseConnector

UbitrackBaseConnector::UbitrackBaseConnector(const std::string& _components_path)
        : m_utFacade(new Ubitrack::Facade::BasicFacade(_components_path.c_str()) )
        , m_haveNewFrame( false )
        , m_lastTimestamp( 0 )
        , m_dataflowLoaded( false )
        , m_dataflowRunning( false )
{
}


bool UbitrackBaseConnector::initialize(const std::string& _utql_filename) {
    // needs to catch exceptions ..
    m_utFacade->loadDataflow( _utql_filename.c_str(), true );
    if (m_utFacade->getLastError() != 0) {
        LOG4CPP_ERROR(logger, "Error while loading dataflow: " << m_utFacade->getLastError());
        return false;
    };
    m_dataflowLoaded = true;
    return true;
}

bool UbitrackBaseConnector::teardown() {
    // needs to catch exceptions ..
    if (m_dataflowRunning) {
        if (!stop()){
            return false;
        }
    }

    if (m_dataflowLoaded) {
        m_utFacade->clearDataflow();
        if (m_utFacade->getLastError() != 0) {
            LOG4CPP_ERROR(logger, "Error while clearing dataflow: " << m_utFacade->getLastError());
            return false;
        };
        m_dataflowLoaded = false;
    }
    return true;
}

bool UbitrackBaseConnector::start() {
    // needs to catch exceptions ..
    if (m_dataflowLoaded) {
        m_utFacade->startDataflow();
        if (m_utFacade->getLastError() != 0) {
            LOG4CPP_ERROR(logger, "Error while starting dataflow: " << m_utFacade->getLastError());
            return false;
        };
        m_dataflowRunning = true;
    }
    return true;
}

bool UbitrackBaseConnector::stop() {
    // needs to catch exceptions ..
    if (m_dataflowRunning) {
        m_utFacade->stopDataflow();
        if (m_utFacade->getLastError() != 0) {
            LOG4CPP_ERROR(logger, "Error while stopping dataflow: " << m_utFacade->getLastError());
            return false;
        };
        m_dataflowRunning = false;
    }
    return true;
}

void UbitrackBaseConnector::set_new_frame(unsigned long long int ts) {
    std::unique_lock<std::mutex> ul( m_waitMutex );
    m_lastTimestamp = ts;
    m_haveNewFrame = true;
    m_waitCondition.notify_one();
}


unsigned long long int UbitrackBaseConnector::wait_for_frame() {
    // find a way to exit here in case we want to stop
    // alternatively, we could only query the m_haveNewFrame variable (polling)
    unsigned long long int ts(0);
    while (!m_haveNewFrame) {
        std::unique_lock<std::mutex> ul( m_waitMutex );
        m_waitCondition.wait( ul );
        ts = m_lastTimestamp;
    }

    // reset haveNewFrame immediately to prepare for the next frame
    // maybe this should be done in a seperate method ??
    {
        std::unique_lock<std::mutex> ul( m_waitMutex );
        m_haveNewFrame = false;
    }
    return ts;
}