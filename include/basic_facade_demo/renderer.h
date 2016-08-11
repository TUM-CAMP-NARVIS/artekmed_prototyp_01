#ifndef __SIMPLE_AR_DEMO_RENDERER_HH__
#define __SIMPLE_AR_DEMO_RENDERER_HH__

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "basic_facade_demo/glfwwindow.h"

// include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_access.hpp> 
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/quaternion.hpp>


#include <utFacade/BasicFacadeTypes.h>
#include "basic_facade_demo/shader.hpp"
#include  "basic_facade_demo/texture.hpp"
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

	bool update_background_left();

	// some helper functions for transferring data
	inline void set_camera_left_image(std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement >&img) {
		m_camera_left_image = img;
	}

	inline void set_camera_left_pose(glm::mat4& pose) {
		m_camera_left_pose = pose;
	}

	inline void set_intrinsics_left(glm::mat3& intrinsics, glm::ivec2& resolution) {
		m_intrinsics_left = intrinsics;
		m_resolution_left = resolution;
	}

protected:

	glm::mat4 compute_projection_matrix(glm::mat3& intrinsics, glm::ivec2& resolution, float n, float f);
	// needs better structure .. especially for stereo rendering, to avoid duplicating code.
	bool camera_left_update_texture();
	bool render_video_background();
	bool drawObject_Left();



	glm::mat3 m_intrinsics_left;
	glm::ivec2 m_resolution_left;

	glm::mat4 m_camera_left_pose;
	glm::mat4 m_projection_left;

	void setup_shader();
	std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement > m_camera_left_image;
	bool m_bTextureLeftInitialized;
	unsigned int m_pow2WidthLeft;
	unsigned int m_pow2HeightLeft;
	GLuint m_texture_left;

	GLuint background_MatrixID;
	GLuint background_textureID;
	GLuint background_depthID;
	GLuint vertexbuffer;
	GLuint uvbuffer;
	GLuint Texture;
	GLuint backgroundID;
	GLuint object_programID;
	GLuint object_matrixID;
	GLint object_modelMatrixID;
	GLint object_viewMatrixID;
	GLint object_projectMatrixID;
	GLint object_depthTextureID;
	GLint object_isRightID;
	GLuint object_textureID;
	GLuint object_vertexbuffer;
	GLuint object_uvbuffer;
	GLuint VertexArrayID[2];
	float randomA;
	float randomB;
};




#endif // __SIMPLE_AR_DEMO_RENDERER_HH__