/*
* UbiTrack - Basic Facade Example
* by Ulrich Eck <ueck@net-labs.de>
*
*/

// platform independent Sleep
#ifdef _WINDOWS

// target XP
#ifndef WINVER
#define WINVER 0x0501
#endif

#ifndef _WIN32_WINNT
#define _WIN32_WINNT WINVER
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#define UBITRACK_UNDEF_WIN32_LEAN_AND_MEAN
#endif

#ifndef NOMINMAX
#define NOMINMAX
#define UBITRACK_UNDEF_NOMINMNAX
#endif

#include <Windows.h>

#ifdef UBITRACK_UNDEF_NOMINMNAX
#undef NOMINMAX
#endif

#ifdef UBITRACK_UNDEF_WIN32_LEAN_AND_MEAN
#undef WIN32_LEAN_AND_MEAN
#endif

#pragma warning (disable : 4231)
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(x) usleep((x)*1000)
#endif

#include <cstdlib>
#include <csignal>
#include <iostream>
#include <functional>
#include <vector>

#include <GL/glew.h>

// open3d includes
#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>

#ifdef _WIN32
#include <conio.h>
#pragma warning (disable : 4231)
#endif

#include <utFacade/BasicFacade.h>

#include "artekmed/UbitrackPointCloudConnector.h"
#include "artekmed/UbitrackPointCloudVisualizer.h"
#include "artekmed/UbitrackImage.h"

// logging
#include <log4cpp/OstreamAppender.hh>
#include <log4cpp/PatternLayout.hh>
#include <log4cpp/Category.hh>
#include <log4cpp/PropertyConfigurator.hh>

#include <boost/program_options.hpp>




using namespace Ubitrack;

static log4cpp::Category& logger( log4cpp::Category::getInstance( "ArtekmedP1.Main" ) );

int main(int ac, char** av) {

	// program options
	std::string sUtqlFile;
	std::string sExtraUtqlFile;
	std::string sComponentsPath;

	try
	{
		// describe program options
		namespace po = boost::program_options;
		po::options_description poDesc( "Allowed options", 80 );
		poDesc.add_options()
			( "help", "print this help message" )
			( "components_path", po::value< std::string >( &sComponentsPath ), "Directory from which to load components" )
			( "utql", po::value< std::string >( &sUtqlFile ), "UTQL request or response file, depending on whether a server is specified. "
				"Without specifying this option, the UTQL file can also be given directly on the command line." )
			( "extra-dataflow", po::value< std::string >( &sExtraUtqlFile ), "Additional UTQL response file to be loaded directly without using the server" )
			( "path", "path to ubitrack bin directory" )
		;
			
		// specify default options
		po::positional_options_description inputOptions;
		inputOptions.add( "utql", 1 );		
			
		// parse options from command line and environment
		po::variables_map poOptions;
		po::store( po::command_line_parser( ac, av ).options( poDesc ).positional( inputOptions ).run(), poOptions );
		po::store( po::parse_environment( poDesc, "UBITRACK_" ), poOptions );
		po::notify( poOptions );

		// print help message if nothing specified
		if ( poOptions.count( "help" ) || sUtqlFile.empty() )
		{
			std::cout << "Syntax: basicfacade_example [options] [--utql] <UTQL file>" << std::endl << std::endl;
			std::cout << poDesc << std::endl;
			return 1;
		}
			
			
	}
	catch( std::exception& e )
	{
		std::cerr << "Error parsing command line parameters : " << e.what() << std::endl;
		std::cerr << "Try basicfacade_example --help for help" << std::endl;
		return 1;
	}		

	// add a stderr appender with some nice layout and set priority to INFO and event priority to notice
	log4cpp::Appender* app = new log4cpp::OstreamAppender( "stderr", &std::cerr );
	log4cpp::PatternLayout* layout = new log4cpp::PatternLayout();
	layout->setConversionPattern( "%d{%H:%M:%S.%l} %6p %20f:%-3l %m   (%c)%n" );
	app->setLayout( layout );

	log4cpp::Category::getRoot().setAdditivity( false );
	log4cpp::Category::getRoot().addAppender( app );
	log4cpp::Category::getRoot().setPriority( log4cpp::Priority::INFO ); // default: INFO
	log4cpp::Category::getInstance( "Ubitrack.Events" ).setPriority( log4cpp::Priority::NOTICE ); // default: NOTICE

    open3d::UbitrackPointCloudVisualizer visualizer;


	LOG4CPP_INFO( logger, "Starting Basic Facade Demo demo" );
	try
	{
		// initialize Ubitrack logging
		Facade::initUbitrackLogging("log4cpp.conf");
		// initialize Open3D Verbosity Level
		open3d::SetVerbosityLevel(open3d::VerbosityLevel::VerboseAlways);


        std::string window_name = "ARTEKMED: Pointcloud Viewer";
        int width = 1024;
        int height = 768;
        int left = 50;
        int top = 50;

		// configure ubitrack
		LOG4CPP_INFO( logger, "Initialize Connector." );
		std::shared_ptr<UbitrackPointCloudConnector> connector = std::make_shared<UbitrackPointCloudConnector>( sComponentsPath );

		LOG4CPP_INFO( logger, "Instantiating dataflow network from " << sUtqlFile << "..." );
		if (!connector->initialize( sUtqlFile )) {
			LOG4CPP_ERROR( logger, "Unable to load dataflow." );
			return 1;
		};

        LOG4CPP_INFO(logger, "Setup Connector for Visualizer.");
        // must be done after window is created to ensure opengl context
        visualizer.SetUbitrackConnector(connector);

        if (!visualizer.CreateVisualizerWindow(window_name, width, height, left, top)) {
            open3d::PrintWarning("[UbitrackVisualizer] Failed creating OpenGL window.\n");
            return 1;
        }


        if (connector->have_camera01()) {
			// create ubitrack camera imageand add to visualizer
			auto point_cloud1 = std::make_shared<open3d::PointCloud>();
			visualizer.setPointCloud1(point_cloud1);
        }

		if (connector->have_camera02()) {
			auto point_cloud2 = std::make_shared<open3d::PointCloud>();
			visualizer.setPointCloud2(point_cloud2);
		}

		if (connector->have_camera03()) {
			auto point_cloud3 = std::make_shared<open3d::PointCloud>();
			visualizer.setPointCloud3(point_cloud3);
		}

		auto origin = open3d::CreateMeshCoordinateFrame(0.1);
        visualizer.AddGeometry(origin);

        visualizer.Run();
        visualizer.DestroyVisualizerWindow();


		// this should be executed also if execptions happen above .. restructure try/catch block
		LOG4CPP_INFO( logger, "Finished, cleaning up..." );
		connector->teardown();
	}
	catch( std::exception& e )
	{
		LOG4CPP_ERROR( logger, "exception occurred: " << e.what() );
	}
	catch(...) {
		LOG4CPP_ERROR( logger, "unkown error occured" );
	}

	LOG4CPP_INFO( logger, "artekmed terminating." );



}