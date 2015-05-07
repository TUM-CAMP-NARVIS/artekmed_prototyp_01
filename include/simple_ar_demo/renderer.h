#ifndef __SIMPLE_AR_DEMO_RENDERER_HH__
#define __SIMPLE_AR_DEMO_RENDERER_HH__

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <utFacade/BasicFacadeTypes.h>
#include <utFacade/BasicFacade.h>


/*
 * The following things need to be considered:
 * - Stereo (side-by-side) rendering
 * - Synchronization of rendering with video
 *
 */

class Renderer {

	Renderer();
	~Renderer();

	unsigned long long int wait_for_image();

	// glfw window events
	void on_window_close(GLFWwindow* window);
	void on_window_size(GLFWwindow* window, int width, int height);


	void pre_render(GLFWwindow* window);
	void render(GLFWwindow* window, unsigned long long int ts);
	void post_render(GLFWwindow* window);

};




#endif // __SIMPLE_AR_DEMO_RENDERER_HH__