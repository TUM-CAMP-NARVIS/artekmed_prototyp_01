/*
 * UbiTrack - Basic Facade Example
 * by Ulrich Eck <ueck@net-labs.de>
 *
 */

#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <functional>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <conio.h>
#endif

#include <utFacade/BasicFacadeTypes.h>
#include <utFacade/BasicFacade.h>

#include "argparser.hpp"
using namespace Ubitrack;


bool bStop = false;


void ctrlC ( int i )
{
        bStop = true;
}


class FacadeHandler {
public:

	void receivePose(const Facade::BasicPoseMeasurement& pose) {
	std::cout << "Example: received pushed pose: tbd"  << std::endl;
//	std::cout << "Example: received pushed pose: " << pose << std::endl;

	}

};



int main(int ac, const char* av[]) {
	
	signal ( SIGINT, &ctrlC );

    try
    {
	    // initialize logging
		Facade::initUbitrackLogging("log4cpp.conf");
//	    Util::initLogging();

	    // program options
	    std::string sUtqlFile;
	    std::string sComponentsPath;
	    bool bNoExit;


		argparser::parser parser( "basic_facade_example", "Demonstrates the usage of the new BasicFacade." );
		const auto& arg_show_help = parser.add< bool >( "help", "show this information", 'h' );
		const auto& arg_components_path = parser.add< std::string >( "components_path", "The Ubitrack Components Path.", 'c', argparser::Optional, "" );
		const auto& arg_utql = parser.add< std::string >( "utql", "The UTQL File.", 'u', argparser::Optional, "" );
		const auto& arg_noexit = parser.add< bool >( "noexit", "do not exit on return", 'n' );

	    try
	    {
			// Example command line:
			parser.parse( ac, av );


	        bNoExit = arg_noexit.value();
			sUtqlFile = arg_utql.value();
			sComponentsPath = arg_components_path.value();

			if (sUtqlFile.empty()) {
				parser.each_unlabeled_argument([&]( const std::string& arg ) {
					if (sUtqlFile.empty()) {
						sUtqlFile = arg;
					}
				});
			}

	        // print help message if nothing specified
	        if ( arg_show_help.value() || sUtqlFile.empty() )
	        {
	            std::cout << "Syntax: basic_facade_example [options] [--utql] <UTQL file>" << std::endl << std::endl;
				parser.show_usage( std::cout );
	            return 1;
	        }
		}
		catch( std::exception& e )
		{
	        std::cerr << "Error parsing command line parameters : " << e.what() << std::endl;
			parser.show_usage( std::cerr );
	        exit( 1 );
		}

        // configure ubitrack
        std::cout << "Loading components..." << std::endl << std::flush;
        Facade::BasicFacade utFacade( sComponentsPath );
		FacadeHandler handler;

        std::cout << "Instantiating dataflow network from " << sUtqlFile << "..." << std::endl << std::flush;
        utFacade.loadDataflow( sUtqlFile );

		std::string pushsink_name("PushSinkPose");
		auto pushsink = utFacade.getPushSink<Facade::BasicPoseMeasurement>(pushsink_name);
		if (pushsink == NULL) {
			std::cout << "Error getting PushSinkPose." << std::endl;
			std::cout << utFacade.getLastError() << std::endl;
			exit( 1 );
		} else {
			pushsink->registerCallback(std::bind(&FacadeHandler::receivePose, handler, std::placeholders::_1));
		}

		std::string pullsink_name("PullSinkPose");
		auto pullsink = utFacade.getPullSink<Facade::BasicPoseMeasurement>(pullsink_name);
		if (pullsink == NULL) {
			std::cout << "Error getting PullSinkPose." << std::endl;
			std::cout << utFacade.getLastError() << std::endl;
			exit( 1 );
		}

		std::string pushsource_name("PushSourcePose");
		auto pushsource = utFacade.getPushSource<Facade::BasicPoseMeasurement>(pushsource_name);
		if (pushsource == NULL) {
			std::cout << "Error getting PushSourcePose." << std::endl;
			std::cout << utFacade.getLastError() << std::endl;
			exit( 1 );
		}



        std::cout << "Starting dataflow" << std::endl;
        utFacade.startDataflow();
      	
		while( !bStop )
        {
			unsigned long long timestamp = utFacade.now();
			
			// push a pose to ubitrack
			std::vector<double> push_pose {0., 0., 0., 0., 0., 0., 1.};
			Facade::BasicPoseMeasurement bm(timestamp, push_pose);
			std::cout << "Example: send pose to ubitrack: tbd " << std::endl;
			pushsource->send(bm);

			// pull a pose from ubitrack
			auto pull_bm = pullsink->get(timestamp);
			if (pull_bm->is_valid()) {
				std::vector<double> pull_pose_vec(pull_bm->size());
				pull_bm->get(pull_pose_vec);
				std::cout << "Example: sucessfully pulled pose: tbd" << std::endl;
			}

			std::chrono::milliseconds dura( 100 );
			std::this_thread::sleep_for(dura);
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