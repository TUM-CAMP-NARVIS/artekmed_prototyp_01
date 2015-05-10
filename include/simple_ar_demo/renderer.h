#ifndef __SIMPLE_AR_DEMO_RENDERER_HH__
#define __SIMPLE_AR_DEMO_RENDERER_HH__

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "simple_ar_demo/glfwwindow.h"
/*
 * The following things need to be considered:
 * - Stereo (side-by-side) rendering
 * - Synchronization of rendering with video
 *
 */

class Renderer {
public:
	// probably need some parameters here ..
	Renderer();
	~Renderer();

	// think about stereo-rendering before further extension !!
	virtual void pre_render(Window* window);
	virtual void render(Window* window, unsigned long long int ts);
	virtual void post_render(Window* window);

};




#endif // __SIMPLE_AR_DEMO_RENDERER_HH__