/*
 * UbiTrack - Basic Facade Example
 * by Ulrich Eck <ueck@net-labs.de>
 *
 */

// platform independent Sleep
#ifdef _WINDOWS
#pragma warning (disable : 4231)
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(x) usleep((x)*1000)
#endif

#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <functional>
#include <vector>

#include <GL/glew.h>
#include <GLFW/glfw3.h>



#ifdef _WIN32
 #include <conio.h>
 #pragma warning (disable : 4231)
#endif

#include <utFacade/BasicFacade.h>

#include "simple_ar_demo/utconnector.h"
#include "simple_ar_demo/glfwwindow.h"
#include "simple_ar_demo/renderer.h"

// can be replaced with boost::*
#include "simple_ar_demo/optionparser.h"
#include  "simple_ar_demo/easylogging++.h"
// only once in main!!
_INITIALIZE_EASYLOGGINGPP

using namespace Ubitrack;

/*
 * Commandline arguments handling
 */
struct Arg: public option::Arg
{
  static option::ArgStatus Required(const option::Option& option, bool)
  {
    return option.arg == 0 ? option::ARG_ILLEGAL : option::ARG_OK;
  }
};


 enum  optionIndex { UNKNOWN, HELP, UTQL, COMPONENTSPATH };
 const option::Descriptor usage[] =
 {
  {UNKNOWN,           0,"" , ""                 ,Arg::None,     "USAGE: simple_ar_demo [options]\n\n"
                                                                "Options:" },
  {HELP,              0,"" , "help"             ,Arg::None,     "  --help  \tPrint usage and exit." },
  {UTQL,              0,"u", "utql"             ,Arg::Required, "  --utql, -u  \tUTQL File to load." },
  {COMPONENTSPATH,    0,"c", "components_path"  ,Arg::Required, "  --components_path, -c  \tThe Ubitrack Components Path." },
  {0,0,0,0,0,0}
 };

/*
 * Utils
 */

void print_vector(std::vector<double>& v) {
	std::cout << "[";
	for (unsigned int i=0; i < v.size(); ++i) {
		std::cout << v.at(i) << ", ";
	}
	std::cout << "]";
}

int main(int argc, const char* argv[]) {
	_START_EASYLOGGINGPP(argc, argv);

	LINFO << "Starting SimpleAR demo";
	// program options
	std::string sUtqlFile;
	std::string sComponentsPath;
	std::string sLogConfig("log4cpp.conf");

    try
    {
		argc-=(argc>0); argv+=(argc>0); // skip program name argv[0] if present
		option::Stats  stats(usage, argc, argv);
		option::Option* options = new option::Option[stats.options_max];
		option::Option* buffer  = new option::Option[stats.buffer_max];
		option::Parser parse(usage, argc, argv, options, buffer);

	   if (parse.error())
	     return 1;

	   if (options[HELP] || argc == 0) {
	     option::printUsage(std::cout, usage);
	     return 0;
	   }

		// use cmdline arguments
		sUtqlFile = options[UTQL].arg;
		sComponentsPath = options[COMPONENTSPATH].arg;


	    // initialize Ubitrack logging
		Facade::initUbitrackLogging(sLogConfig.c_str());

		// configure ubitrack
        LINFO << "Initialize Connector.";
		UTSimpleARConnector connector( sComponentsPath.c_str() );

		LINFO << "Instantiating dataflow network from " << sUtqlFile << "...";
		if (!connector.initialize( sUtqlFile.c_str() )) {
			LERROR << "Unable to load dataflow.";
			return 1;
		};

		// initialize GLFW
		LINFO << "Initialize GLFW";
		glfwInit();

		LINFO << "Create OpenGL Window.";
		Window* window = new Window(800, 600, "Simple AR Demo");

		// initialize GLEW
		LINFO << "Initialize GLEW";
		glewExperimental = GL_TRUE;
		glewInit();

		// create renderer class
		Renderer* renderer = new Renderer();


		Sleep(100);

		LINFO << "Starting dataflow";
		connector.start();

		// load ipsi scene here


		while(!window->windowShouldClose())
		{

			LDEBUG << "Wait for frame.";
			unsigned long long timestamp = connector.wait_for_frame();
			LDEBUG << "Got frame: " << timestamp;

			// receive tracking data

			glm::mat4 camera_pose;
			connector.camera_left_get_pose(timestamp, camera_pose);

			// update model based on tracking data

			// think about stereo-rendering here ..

			//integrate IPSI

			// initialize rendering
			renderer->pre_render(window);

			// all processing per frame goes here.
			renderer->render(window, timestamp);

			// finalize rendering
			renderer->post_render(window);

			// trigger event processing
			glfwPollEvents();

		}
		
        LINFO << "Stopping dataflow...";
        connector.stop();


		// this should be executed also if execptions happen above .. restructure try/catch block
        LINFO << "Finished, cleaning up...";
		connector.teardown();
	}
	catch( std::exception& e )
	{
	        LERROR << "exception occurred: " << e.what();
	}
	catch(...) {
		LERROR << "unkown error occured";
	}

	LINFO << "simple_ar_demo terminating.";

	glfwTerminate();

	
	
}