//
// Created by Ulrich Eck on 15.03.18.
//

#ifndef ARTEKMED_UBITRACKBASECONNECTOR_H
#define ARTEKMED_UBITRACKBASECONNECTOR_H

#include <memory>
#include <vector>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <tuple>


// include Ubitrack BasicFacade
#include <utFacade/AdvancedFacade.h>
#include <utFacade/DataflowObserver.h>
#include <utMeasurement/Measurement.h>

#include "artekmed/Ubitrack/CameraConnector.h"

/**
* A Basic data flow observer
*/
class UbitrackDataflowObserver : public Ubitrack::Facade::DataflowObserver
{
public:
    /** constructor */
    UbitrackDataflowObserver()
    : Ubitrack::Facade::DataflowObserver()
    , m_registrations(0)
    {

    }

    /** called when a component is added */
    virtual void notifyAddComponent( const char* sPatternName, const char* sComponentName ) throw() {
        auto element = std::make_tuple( std::string(sPatternName), std::string(sComponentName) );
        m_registrations.emplace_back(element);
    };

    /** called when a component is removed */
    virtual void notifyDeleteComponent( const char* sPatternName, const char* sComponentName ) throw() {

    };

    /** virtual destructor */
    virtual ~UbitrackDataflowObserver()
    {}

    /** simple accessor */
    const std::vector< std::tuple< std::string, std::string > >& getRegistrations() {
        return m_registrations;
    }
protected:
    std::vector< std::tuple< std::string, std::string > > m_registrations;
};


class UbitrackBaseConnector {
public:
    explicit UbitrackBaseConnector(const std::string& _components_path);
    virtual ~UbitrackBaseConnector() = default;


    /*
     * livecycle management for the utconnector
     * not thread-safe
     */
    virtual bool initialize(const std::string& _utql_filename);
    virtual bool teardown();


    unsigned int wait_for_frame_timeout(unsigned int timeout_ms, std::vector<Ubitrack::Measurement::Timestamp>& ts);

    virtual void add_cameras() {};

    std::vector<std::shared_ptr<artekmed::RGBDCameraConnector>>& cameras() {
        return m_cameras;
    }

    virtual bool start();
    virtual bool stop();

    Ubitrack::Measurement::Timestamp now();

    bool isRunning();
    bool isLoaded();

protected:

    std::unique_ptr<Ubitrack::Facade::AdvancedFacade> m_utFacade;

    std::vector<std::shared_ptr<artekmed::RGBDCameraConnector>> m_cameras;

    bool m_dataflowLoaded;
    bool m_dataflowRunning;

private:
};


#endif //ARTEKMED_UBITRACKBASECONNECTOR_H
