#include "simple_ar_demo/utconnector.h"

// implementation of UTBaseConnector

UTBaseConnector::UTBaseConnector(const std::string& _components_path)
	: m_utFacade( _components_path.c_str() )
	, m_haveNewFrame( false )
	, m_lastTimestamp( 0 )
	, m_dataflowLoaded( false )
	, m_dataflowRunning( false )
{

}

UTBaseConnector::~UTBaseConnector() {
	// what needs to be done for teardown ??

}

bool UTBaseConnector::initialize(const std::string& _utql_filename) {
	// needs to catch exceptions ..
	m_utFacade.loadDataflow( _utql_filename.c_str() );
	m_dataflowLoaded = true;
	return true;
}

bool UTBaseConnector::teardown() {
	// needs to catch exceptions ..
	if (m_dataflowRunning) {
		stop();
	}

	if (m_dataflowLoaded) {
		m_utFacade.clearDataflow();
		m_dataflowLoaded = false;
	}
	return true;
}

bool UTBaseConnector::start() {
	// needs to catch exceptions ..
	if (m_dataflowLoaded) {
		m_utFacade.startDataflow();
		m_dataflowRunning = true;
	}
	return true;
}

bool UTBaseConnector::stop() {
	// needs to catch exceptions ..
	if (m_dataflowRunning) {
		m_utFacade.stopDataflow();
		m_dataflowRunning = false;
	}
	return true;
}

void UTBaseConnector::set_new_frame(unsigned long long int ts) {
	std::unique_lock<tbb::mutex> ul( m_waitMutex );
	m_lastTimestamp = ts;
	m_haveNewFrame = true;
	m_waitCondition.notify_one();
}


unsigned long long int UTBaseConnector::wait_for_frame() {
	// find a way to exit here in case we want to stop
	// alternatively, we could only query the m_haveNewFrame variable (polling)
	unsigned long long int ts(0);
	while (!m_haveNewFrame) {
		std::unique_lock<tbb::mutex> ul( m_waitMutex );
		m_waitCondition.wait( ul );
		ts = m_lastTimestamp;
	}

	// reset haveNewFrame immediately to prepare for the next frame
	// maybe this should be done in a seperate method ??
	{
		std::unique_lock<tbb::mutex> ul( m_waitMutex );
		m_haveNewFrame = false;
	}
	return ts;
}

// implementation of UTSimpleARConnector

UTSimpleARConnector::UTSimpleARConnector(const std::string& _components_path)
	: UTBaseConnector(_components_path)
	// initialize all sinks and local variables here
{}

UTSimpleARConnector::~UTSimpleARConnector()
{}

bool UTSimpleARConnector::camera_left_get_intrinsics(const TimestampT ts, glm::mat3& intrinsics, glm::ivec2& resolution) 
{
	return true;
}

bool UTSimpleARConnector::camera_left_update_texture(const TimestampT ts, GLuint textureid) 
{
	return true;
}

bool UTSimpleARConnector::camera_left_get_pose(const TimestampT ts, glm::mat4& pose) 
{
	return true;
}

bool UTSimpleARConnector::target1_get_pose(const TimestampT ts, glm::mat4& pose) 
{
	return true;
}

bool UTSimpleARConnector::initialize(const std::string& _utql_filename) 
{
	bool ret = UTBaseConnector::initialize(_utql_filename);
	return ret;
}

bool UTSimpleARConnector::teardown() 
{
	bool ret = UTBaseConnector::teardown();
	return ret;
}

void UTSimpleARConnector::receive_camera_left_image(Facade::BasicImageMeasurement& pose) 
{
}

