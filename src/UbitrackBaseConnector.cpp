//
// Created by Ulrich Eck on 15.03.18.
//

#include "artekmed/UbitrackBaseConnector.h"

#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("ArtekmedP1.UbitrackBaseConnector"));

// implementation of UbitrackBaseConnector

UbitrackBaseConnector::UbitrackBaseConnector(const std::string& _components_path)
        : m_utFacade(new Ubitrack::Facade::AdvancedFacade(_components_path.c_str()) )
        , m_haveNewFrame( false )
        , m_lastTimestamp( 0 )
        , m_dataflowLoaded( false )
        , m_dataflowRunning( false )
{
}

Ubitrack::Measurement::Timestamp UbitrackBaseConnector::now() {
    return Ubitrack::Measurement::now();
}

bool UbitrackBaseConnector::isRunning() {
    return m_dataflowRunning;
}

bool UbitrackBaseConnector::isLoaded() {
    return m_dataflowLoaded;
}



bool UbitrackBaseConnector::initialize(const std::string& _utql_filename) {
    try{
        m_utFacade->loadDataflow( _utql_filename.c_str(), true );
    } catch (Ubitrack::Util::Exception &e) {
        LOG4CPP_ERROR(logger, "Error while loading dataflow: " << e.what());
        return false;
    }
    m_dataflowLoaded = true;
    return true;
}

bool UbitrackBaseConnector::teardown() {
    if (m_dataflowRunning) {
        if (!stop()){
            return false;
        }
    }

    if (m_dataflowLoaded) {
        try{
            m_utFacade->clearDataflow();
        } catch (Ubitrack::Util::Exception &e) {
            LOG4CPP_ERROR(logger, "Error while clearing dataflow: " << e.what());
            return false;
        }
        m_dataflowLoaded = false;
    }
    return true;
}

bool UbitrackBaseConnector::start() {
    if (m_dataflowLoaded) {
        try{
            m_utFacade->startDataflow();
        } catch (Ubitrack::Util::Exception &e) {
            LOG4CPP_ERROR(logger, "Error while starting dataflow: " << e.what());
            return false;
        }
        m_dataflowRunning = true;
    }
    return true;
}

bool UbitrackBaseConnector::stop() {
    // needs to catch exceptions ..
    if (m_dataflowRunning) {
        try{
            m_utFacade->stopDataflow();
        } catch (Ubitrack::Util::Exception &e) {
            LOG4CPP_ERROR(logger, "Error while stopping dataflow: " << e.what());
            return false;
        }
        m_dataflowRunning = false;
    }
    return true;
}

void UbitrackBaseConnector::set_new_frame(Ubitrack::Measurement::Timestamp ts) {
    std::unique_lock<std::mutex> ul( m_waitMutex );
    m_lastTimestamp = ts;
    m_haveNewFrame = true;
    m_waitCondition.notify_one();
}


Ubitrack::Measurement::Timestamp UbitrackBaseConnector::wait_for_frame() {
    // find a way to exit here in case we want to stop
    // alternatively, we could only query the m_haveNewFrame variable (polling)
    Ubitrack::Measurement::Timestamp ts(0);
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

unsigned int UbitrackBaseConnector::wait_for_frame_timeout(unsigned int timeout_ms, Ubitrack::Measurement::Timestamp& ts) {
    auto timeout = std::chrono::milliseconds(timeout_ms);

    std::unique_lock<std::mutex> ul( m_waitMutex );
    if(m_waitCondition.wait_for( ul , timeout) == std::cv_status::no_timeout) {
        if (m_haveNewFrame) {
            ts = m_lastTimestamp;
            m_haveNewFrame = false;
            return 0;
        } else {
            // no new frame
            ts = 0;
            return 1;
        }
    } else {
        // timeout
        ts = 0;
        return 2;
    }
}
