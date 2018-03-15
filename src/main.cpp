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

#include <cstdlib>
#include <csignal>
#include <iostream>
#include <functional>
#include <vector>


// open3d includes
#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>

#ifdef _WIN32
#include <conio.h>
#pragma warning (disable : 4231)
#endif

#include <utFacade/BasicFacade.h>

#include "basic_facade_demo/UbitrackSingleCameraConnector.h"
#include "basic_facade_demo/UbitrackVisualizer.h"

// logging
#include <log4cpp/OstreamAppender.hh>
#include <log4cpp/PatternLayout.hh>
#include <log4cpp/Category.hh>
#include <log4cpp/PropertyConfigurator.hh>

#include <boost/program_options.hpp>




using namespace Ubitrack;

static log4cpp::Category& logger( log4cpp::Category::getInstance( "BasicFacadeExample.Main" ) );

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

    three::UbitrackVisualizer visualizer;

	LOG4CPP_INFO( logger, "Starting SimpleAR demo" );
	try
	{
		// initialize Ubitrack logging
		Facade::initUbitrackLogging("log4cpp.conf");
		// initialize Open3D Verbosity Level
		three::SetVerbosityLevel(three::VerbosityLevel::VerboseAlways);


		// configure ubitrack
		LOG4CPP_INFO( logger, "Initialize Connector." );
		UbitrackSingleCameraConnector connector( sComponentsPath );

		LOG4CPP_INFO( logger, "Instantiating dataflow network from " << sUtqlFile << "..." );
		if (!connector.initialize( sUtqlFile )) {
			LOG4CPP_ERROR( logger, "Unable to load dataflow." );
			return 1;
		};

        std::string window_name = "Basic Facade Demo";
        int width = 640;
        int height = 480;
        int left = 50;
        int top = 50;

        if (!visualizer.CreateWindow(window_name, width, height, left, top)) {
            three::PrintWarning("[UbitrackVisualizer] Failed creating OpenGL window.\n");
            return 1;
        }

		LOG4CPP_INFO( logger, "Starting dataflow" );
		connector.start();

		// load initial renderer (static) configuration
		TimestampT ts = connector.now();

		// retrieve camera left intrinsics information
		Eigen::Matrix3d intrinsics_left;
		Eigen::Vector2i resolution_left;
		connector.camera_left_get_intrinsics(ts, intrinsics_left, resolution_left);

		// store camera left intrinsics information
//		renderer->set_intrinsics_left(intrinsics_left, resolution_left);

		// some "global" variables to use during the rendering loop
		std::shared_ptr<Facade::BasicImageMeasurement > cam_img_left;

        //testing open3d
        auto mesh = three::CreateMeshSphere(0.05);
        visualizer.AddGeometry(mesh);


//		// begin opengl initialization
//		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
//
//		// Enable depth test
//		glEnable(GL_DEPTH_TEST);
//		// Accept fragment if it closer to the camera than the former one
//		glDepthFunc(GL_LESS);
//
//		//// GL: disable backface culling
//		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
//		glDisable( GL_CULL_FACE );
//		glPixelStorei( GL_PACK_ALIGNMENT,   1 );
//		glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
//
//		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
//		//glEnable( GL_BLEND );
//
//		// GL: misc stuff
//		glShadeModel( GL_SMOOTH );
//		glEnable( GL_NORMALIZE );
//
//		// //GL: light parameters
//		GLfloat light_pos[] = { 1.0f, 1.0f, 1.0f, 0.0f };
//		GLfloat light_amb[] = { 0.2f, 0.2f, 0.2f, 1.0f };
//		GLfloat light_dif[] = { 0.9f, 0.9f, 0.9f, 1.0f };
//
//		//GL: enable lighting
//		glLightfv( GL_LIGHT0, GL_POSITION, light_pos );
//		glLightfv( GL_LIGHT0, GL_AMBIENT,  light_amb );
//		glLightfv( GL_LIGHT0, GL_DIFFUSE,  light_dif );
//		glEnable( GL_LIGHTING );
//		glEnable( GL_LIGHT0 );

        visualizer.Run();
        visualizer.DestroyWindow();


//		while(!window->windowShouldClose())
//		{
//
//			ts = connector.wait_for_frame();
//
//			// transfer camera_left_image to renderer (only reference, not copied)
//			connector.camera_left_get_current_image(cam_img_left);
//			renderer->set_camera_left_image(cam_img_left);
//
//			// receive camera pose
//			glm::mat4 cam_pose_left;
//
//			connector.camera_left_get_pose(ts, cam_pose_left);
//			renderer->set_camera_left_pose(cam_pose_left);
//
//			// update model based on tracking data
//
//			// think about stereo-rendering here ..
//
//			//integrate IPSI
//
//			// initialize rendering
//			renderer->pre_render(window);
//
//			// all processing per frame goes here.
//			renderer->render(window, ts);
//
//			// finalize rendering
//			renderer->post_render(window);
//
//			// trigger event processing
//			glfwPollEvents();
//
//		}

		LOG4CPP_INFO( logger, "Stopping dataflow..." );
		connector.stop();


		// this should be executed also if execptions happen above .. restructure try/catch block
		LOG4CPP_INFO( logger, "Finished, cleaning up..." );
		connector.teardown();
	}
	catch( std::exception& e )
	{
		LOG4CPP_ERROR( logger, "exception occurred: " << e.what() );
	}
	catch(...) {
		LOG4CPP_ERROR( logger, "unkown error occured" );
	}

	LOG4CPP_INFO( logger, "basic_facade_demo terminating." );



}