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
void APIENTRY DebugOutputCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, GLvoid* userParam){

	printf("OpenGL Debug Output message : ");

	if(source == GL_DEBUG_SOURCE_API_ARB)					printf("Source : API; ");
	else if(source == GL_DEBUG_SOURCE_WINDOW_SYSTEM_ARB)	printf("Source : WINDOW_SYSTEM; ");
	else if(source == GL_DEBUG_SOURCE_SHADER_COMPILER_ARB)	printf("Source : SHADER_COMPILER; ");
	else if(source == GL_DEBUG_SOURCE_THIRD_PARTY_ARB)		printf("Source : THIRD_PARTY; ");
	else if(source == GL_DEBUG_SOURCE_APPLICATION_ARB)		printf("Source : APPLICATION; ");
	else if(source == GL_DEBUG_SOURCE_OTHER_ARB)			printf("Source : OTHER; ");

	if(type == GL_DEBUG_TYPE_ERROR_ARB)						printf("Type : ERROR; ");
	else if(type == GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR_ARB)	printf("Type : DEPRECATED_BEHAVIOR; ");
	else if(type == GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR_ARB)	printf("Type : UNDEFINED_BEHAVIOR; ");
	else if(type == GL_DEBUG_TYPE_PORTABILITY_ARB)			printf("Type : PORTABILITY; ");
	else if(type == GL_DEBUG_TYPE_PERFORMANCE_ARB)			printf("Type : PERFORMANCE; ");
	else if(type == GL_DEBUG_TYPE_OTHER_ARB)				printf("Type : OTHER; ");

	if(severity == GL_DEBUG_SEVERITY_HIGH_ARB)				printf("Severity : HIGH; ");
	else if(severity == GL_DEBUG_SEVERITY_MEDIUM_ARB)		printf("Severity : MEDIUM; ");
	else if(severity == GL_DEBUG_SEVERITY_LOW_ARB)			printf("Severity : LOW; ");

	// You can set a breakpoint here ! Your debugger will stop the program,
	// and the callstack will immediately show you the offending call.
	printf("Message : %s\n", message);
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
		//const char * test="H\:\\develop\\simple_ipsi_demo\\config\\simple_demo\\main_calib.dfg" ;
		////std::string test("H\:\\develop\\simple_ipsi_demo\\config\\simple_demo\\main_calib.dfg");
		//UTSimpleARConnector connector_2(test );
		//connector_2.initialize(test);
/*		if (!connector.initialize( sUtqlFile.c_str() )) {
			LERROR << "Unable to load dataflow.";
			return 1;
		};*/

		// initialize GLFW
		LINFO << "Initialize GLFW";
		glfwInit();

		LINFO << "Create OpenGL Window.";
		Window* window = new Window( 1280, 480, "Simple AR Demo");

		// initialize GLEW
		LINFO << "Initialize GLEW";
		glewExperimental = GL_TRUE;
		glewInit();

		if ( GLEW_ARB_debug_output ){
			printf("The OpenGL implementation provides debug output. Let's use it !\n");
			glDebugMessageCallbackARB(&DebugOutputCallback, NULL);
			glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS_ARB); 
		}else{
			printf("ARB_debug_output unavailable. You have to use glGetError() and/or gDebugger to catch mistakes.\n");
		}
		// create renderer class
		Renderer* renderer = new Renderer();

		Sleep(100);

		LINFO << "Starting dataflow";
		connector.start();

		// load ipsi scene here


		// load initial renderer (static) configuration
		TimestampT ts = connector.now();

		// retrieve camera left intrinsics information
		glm::mat3 intrinsics_left;
		glm::ivec2 resolution_left;
		glm::mat3 intrinsics_right;
		glm::ivec2 resolution_right;
		glm::mat4 left2right;
		connector.camera_left_get_intrinsics(ts, intrinsics_left, resolution_left);
		connector.camera_right_get_intrinsics(ts, intrinsics_right, resolution_right);
		connector.left2right_get_pose(ts, left2right);


		// store camera left intrinsics information
		renderer->set_intrinsics_left(intrinsics_left, resolution_left);
		renderer->set_intrinsics_right(intrinsics_right, resolution_right);
		renderer->set_left2right(left2right);


		// some "global" variables to use during the rendering loop
		std::shared_ptr<Facade::BasicImageMeasurement > cam_img_left;
		std::shared_ptr<Facade::BasicImageMeasurement > cam_img_right;
		std::shared_ptr<Facade::BasicImageMeasurement> depth_img_left;
		std::shared_ptr<Facade::BasicImageMeasurement> depth_img_right;

		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

		// Enable depth test
		glEnable(GL_DEPTH_TEST);
		// Accept fragment if it closer to the camera than the former one
		glDepthFunc(GL_LESS); 

		//// GL: disable backface culling
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		glDisable( GL_CULL_FACE );
		glPixelStorei( GL_PACK_ALIGNMENT,   1 );
		glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );

		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
		//glEnable( GL_BLEND );

		// GL: misc stuff
		glShadeModel( GL_SMOOTH );
		glEnable( GL_NORMALIZE );
		// OpenGL setup
		// GL: enable and set colors
		//glEnable( GL_COLOR_MATERIAL );
		//glClearColor( 0.0, 0.0, 0.0, 1.0 ); // TODO: make this configurable (but black is best for optical see-through ar!)

		//// GL: enable and set depth parameters
		//glEnable( GL_DEPTH_TEST );
		//glClearDepth( 1.0 );

		//// GL: disable backface culling
		//glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		//glDisable( GL_CULL_FACE );

		// //GL: light parameters
		GLfloat light_pos[] = { 1.0f, 1.0f, 1.0f, 0.0f };
		GLfloat light_amb[] = { 0.2f, 0.2f, 0.2f, 1.0f };
		GLfloat light_dif[] = { 0.9f, 0.9f, 0.9f, 1.0f };

		//GL: enable lighting
		glLightfv( GL_LIGHT0, GL_POSITION, light_pos );
		glLightfv( GL_LIGHT0, GL_AMBIENT,  light_amb );
		glLightfv( GL_LIGHT0, GL_DIFFUSE,  light_dif );
		glEnable( GL_LIGHTING );
		glEnable( GL_LIGHT0 );

		while(!window->windowShouldClose())
		{

			//LDEBUG << "Wait for frame.";
			ts = connector.wait_for_frame();
			//LDEBUG << "Got frame: " << timestamp;

			// transfer camera_left_image to renderer (only reference, not copied)
			connector.camera_left_get_current_image(cam_img_left);
			renderer->set_camera_left_image(cam_img_left);

			// receive camera pose
			glm::mat4 cam_pose_left;

			connector.camera_left_get_pose(ts, cam_pose_left);
			renderer->set_camera_left_pose(cam_pose_left);
			connector.camera_depth_get_current_image_left(ts, depth_img_left);
			renderer->set_camera_depth_image_left(depth_img_left);

			connector.camera_depth_get_current_image_right(ts, depth_img_right);
			renderer->set_camera_depth_image_right(depth_img_right);

			connector.camera_get_current_image_right(ts, cam_img_right);
			renderer->set_camera_right_image(cam_img_right);

			// update model based on tracking data

			// think about stereo-rendering here ..

			//integrate IPSI

			// initialize rendering
			renderer->pre_render(window);

			// all processing per frame goes here.
			renderer->render(window, ts);

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