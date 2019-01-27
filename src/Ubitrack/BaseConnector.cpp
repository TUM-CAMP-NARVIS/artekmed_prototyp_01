//
// Created by Ulrich Eck on 15.03.18.
//

#include "artekmed/Ubitrack/BaseConnector.h"

#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("ArtekmedP1.UbitrackBaseConnector"));

// implementation of UbitrackBaseConnector

UbitrackBaseConnector::UbitrackBaseConnector(const std::string& _components_path)
        : m_utFacade(new Ubitrack::Facade::AdvancedFacade(_components_path.c_str()) )
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

    m_cameras.clear();
    add_cameras();

    for (auto&& cam : m_cameras) {
        cam->initialize(m_utFacade.get());
    }

    m_dataflowLoaded = true;
    return true;
}

bool UbitrackBaseConnector::teardown() {
    for (auto&& cam : m_cameras) {
        cam->teardown(m_utFacade.get());
    }

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


unsigned int UbitrackBaseConnector::wait_for_frame_timeout(unsigned int timeout_ms, std::vector<Ubitrack::Measurement::Timestamp>& tsv) {

    int idx = 0;
    unsigned int to = timeout_ms;

    bool have_new_frame = false;
    bool have_timeout = false;

    tsv.resize(m_cameras.size());

    for (auto&& cam: m_cameras) {
        Ubitrack::Measurement::Timestamp t = 0;

        // first camera is sync master where we wait with timeout.
        if (idx == 0) {
            auto ret = cam->wait_for_frame_timeout(to, t);
            switch (ret) {
                case 0:
                    have_new_frame = true;
                    break;
                case 2:
                    have_timeout = true;
                    break;
                default:
                    break;
            }
        } else if (have_new_frame){
            // pull all other cameras with 0 timeout.
            auto ret = cam->wait_for_frame_timeout(0, t);
            switch (ret) {
                case 0:
                    have_new_frame = true;
                    break;
                default:
                    break;
            }
        }
        tsv.push_back(t);
        idx++;
    }
    if (have_new_frame) {
        return 0;
    }
    if (have_timeout) {
        return 2;
    }
    // no new frame
    return 1;
}
