/*
 * UbiTrack - Basic Facade Example
 * by Ulrich Eck <ueck@net-labs.de>
 *
 */

#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <functional>

#include <vector>

#ifdef _WIN32
 #include <conio.h>
 #pragma warning (disable : 4231)
#endif

#include <SFML/System.hpp>
#include <SFML/Graphics.hpp>


#include <utFacade/BasicFacadeTypes.h>
#include <utFacade/BasicFacade.h>

#include "simple_ar_demo/optionparser.h"
using namespace Ubitrack;


bool bStop = false;


void ctrlC ( int i )
{
        bStop = true;
}

void print_vector(std::vector<double>& v) {
	std::cout << "[";
	for (unsigned int i=0; i < v.size(); ++i) {
		std::cout << v.at(i) << ", ";
	}
	std::cout << "]";
}

class FacadeHandler {
public:

	void receivePose(Facade::BasicPoseMeasurement& pose) {
		std::vector<double> v(7, 0.);
		pose.get(v);
		std::cout << "Example: received pushed pose: " << pose.time() << " ";
		print_vector(v);
		std::cout << std::endl;

	}

};

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



int main(int argc, const char* argv[]) {
	
	signal ( SIGINT, &ctrlC );

	// program options
	std::string sUtqlFile;
	std::string sComponentsPath;
	std::string sLogConfig("log4cpp.conf");

    try
    {
	    // initialize logging
		Facade::initUbitrackLogging(sLogConfig.c_str());

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


        // configure ubitrack
        std::cout << "Loading components..." << std::endl << std::flush;
        Facade::BasicFacade utFacade( sComponentsPath.c_str() );
		FacadeHandler handler;

        std::cout << "Instantiating dataflow network from " << sUtqlFile << "..." << std::endl << std::flush;
		if (!utFacade.loadDataflow( sUtqlFile.c_str() )) {
			std::cout << "Unable to load dataflow." << std::endl;
			return 1;
		};

		std::string pushsink_name("PushSinkPose");
		Ubitrack::Facade::BasicPushSink< Facade::BasicPoseMeasurement >* pushsink = utFacade.getPushSink<Facade::BasicPoseMeasurement>(pushsink_name.c_str());
		if (pushsink == NULL) {
			std::cout << "Error getting PushSinkPose." << std::endl;
			std::cout << (utFacade.getLastError() == NULL ? "Unkown Error" : utFacade.getLastError()) << std::endl;
			exit( 1 );
		} else {
			pushsink->registerCallback(std::bind(&FacadeHandler::receivePose, handler, std::placeholders::_1));
		}

		std::string pullsink_name("PullSinkPose");
		Ubitrack::Facade::BasicPullSink< Facade::BasicPoseMeasurement >* pullsink = utFacade.getPullSink<Facade::BasicPoseMeasurement>(pullsink_name.c_str());
		if (pullsink == NULL) {
			std::cout << "Error getting PullSinkPose." << std::endl;
			std::cout << (utFacade.getLastError() == NULL ? "Unkown Error" : utFacade.getLastError()) << std::endl;
			exit( 1 );
		}

		std::string pushsource_name("PushSourcePose");
		Ubitrack::Facade::BasicPushSource< Facade::BasicPoseMeasurement >* pushsource = utFacade.getPushSource<Facade::BasicPoseMeasurement>(pushsource_name.c_str());
		if (pushsource == NULL) {
			std::cout << "Error getting PushSourcePose." << std::endl;
			std::cout << (utFacade.getLastError() == NULL ? "Unkown Error" : utFacade.getLastError()) << std::endl;
			exit( 1 );
		}



        std::cout << "Starting dataflow" << std::endl;
        utFacade.startDataflow();

		sf::Time dura = sf::milliseconds(100);

		while( !bStop )
        {
			unsigned long long timestamp = utFacade.now();
			
			// push a pose to ubitrack
			std::vector<double> push_pose(7, 0.);
			push_pose.at(6) = 1.;
			Facade::BasicPoseMeasurement bm(timestamp, push_pose);
			std::cout << "Example: send pose to ubitrack: ";
			print_vector(push_pose);
			std::cout << std::endl;

			pushsource->send(bm);

			sf::sleep(dura);

			// pull a pose from ubitrack
			std::shared_ptr<Facade::BasicPoseMeasurement> pull_bm = pullsink->get(timestamp);
			if ((pull_bm) && (pull_bm->isValid())) {
				std::vector<double> pull_pose_vec(pull_bm->size());
				pull_bm->get(pull_pose_vec);
				std::cout << "Example: sucessfully pulled pose: ";
				print_vector(pull_pose_vec);
				std::cout << std::endl;
				// cleanup
			}


			sf::sleep(dura);

            #ifdef _WIN32
            if(kbhit())
            {
                    char c = getch();               
                    if(c == 'q') bStop = true;
            }
            #endif
        }

        std::cout << "Stopping dataflow..." << std::endl << std::flush;
        utFacade.stopDataflow();

		// disconnecting a pushsinkcallback
		pushsink->unregisterCallback();

        std::cout << "Finished, cleaning up..." << std::endl << std::flush;
	}
	catch( std::exception& e )
	{
	        std::cout << "exception occurred" << std::endl << std::flush;
	        std::cerr << e.what() << std::endl;
	}

	std::cout << "basic_facade_example terminated." << std::endl << std::flush;

	
	
	
}