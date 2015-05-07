#include "simple_ar_demo/renderer.h"



/*
 * on_window_close callback is called before window closes
 */
void Renderer::on_window_close(GLFWwindow* window) {
	bool time_to_close = true;

	// check state of system and decide if window can close

	if (!time_to_close)
        glfwSetWindowShouldClose(window, GL_FALSE);
}


/*
 * on_window_size callback receives size of window in screen coordinates
 */
void Renderer::on_window_size(GLFWwindow* window, int width, int height) {

	int fb_width, fb_height;
	glfwGetFramebufferSize(window, &fb_width, &fb_height);
	glViewport(0, 0, fb_width, fb_height);

}



void Renderer::pre_render(GLFWwindow* window) {
	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT);
}

void Renderer::post_render(GLFWwindow* window) {
	glfwSwapBuffers(window);
	glfwPollEvents();
}