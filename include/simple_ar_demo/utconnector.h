#ifndef __SIMPLE_AR_DEMO_UTCONNECTOR_HH__
#define __SIMPLE_AR_DEMO_UTCONNECTOR_HH__

#ifdef _WIN32
# pragma warning (disable : 4231)
#endif


#include <string>

#include <tbb/mutex.h>
#include "tbb/compat/condition_variable"

#include <utFacade/BasicFacadeTypes.h>
#include <utFacade/BasicFacade.h>

using namespace Ubitrack;

class UTBaseConnector {
public:
	UTBaseConnector(const std::string& _components_path);
	~UTBaseConnector();

	/*
	 * waits for an image to be pushed (left-eye) and returns its timestamp
	 */
	unsigned long long int wait_for_frame();


	/*
	 * livecycle management for the utconnector
	 * not thread-safe
	 */
	virtual bool initialize(const std::string& _utql_filename);
	virtual bool teardown();

	virtual bool start();
	virtual bool stop();

protected:

	void set_new_frame(unsigned long long int ts);


	Facade::BasicFacade m_utFacade;
	bool m_haveNewFrame;
	unsigned long long int m_lastTimestamp;

	bool m_dataflowLoaded;
	bool m_dataflowRunning;

private:
	tbb::mutex m_waitMutex;
	std::condition_variable m_waitCondition;
};




#endif // __SIMPLE_AR_DEMO_UTCONNECTOR_HH__