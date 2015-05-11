#include "simple_ar_demo/renderer.h"
#include  "simple_ar_demo/easylogging++.h"

Renderer::Renderer() {

}

Renderer::~Renderer() {

}

void Renderer::pre_render(Window* window) {
	//LDEBUG << "Renderer::pre_render";
	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT);
}

void Renderer::post_render(Window* window) {
	//LDEBUG << "Renderer::post_render";
	window->swap();
}

void Renderer::render(Window* window, unsigned long long int ts) {
	// do render all scene elements here
}