#ifndef __SIMPLE_AR_DEMO_RENDERER_HH__
#define __SIMPLE_AR_DEMO_RENDERER_HH__

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "simple_ar_demo/glfwwindow.h"

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
#include "simple_ar_demo/shader.hpp"
#include  "simple_ar_demo/texture.hpp"
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

	bool update_background_depth();

	// some helper functions for transferring data
	inline void set_camera_left_image(std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement >&img) {
		m_camera_left_image = img;
	}
	inline void set_camera_depth_image_left(std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement >&img) {
		m_camera_depth_left = img;
	}
	inline void set_camera_right_image(std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement >&img) {
		m_camera_right_image = img;
	}
	inline void set_camera_depth_image_right(std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement >&img) {
		m_camera_depth_right = img;
	}

	inline void set_camera_left_pose(glm::mat4& pose) {
		m_camera_left_pose = pose;
	}

	inline void set_intrinsics_left(glm::mat3& intrinsics, glm::ivec2& resolution) {
		m_intrinsics_left = intrinsics;
		m_resolution_left = resolution;
	}
	inline void set_intrinsics_right(glm::mat3& intrinsics, glm::ivec2& resolution) {
		m_intrinsics_right = intrinsics;
		m_resolution_right = resolution;
	}

protected:

	glm::mat4 compute_projection_matrix(glm::mat3& intrinsics, glm::ivec2& resolution, float n, float f);
	// needs better structure .. especially for stereo rendering, to avoid duplicating code.
	bool camera_left_update_texture();
	bool camera_right_update_texture();
	bool camera_depth_update_buffer_left();
	bool camera_depth_update_buffer_right();
	bool render_video_background();
	bool drawObject();



	glm::mat3 m_intrinsics_left;
	glm::ivec2 m_resolution_left;
	glm::mat3 m_intrinsics_right;
	glm::ivec2 m_resolution_right;

	glm::mat4 m_camera_left_pose;

	void setup_shader();
	std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement > m_camera_left_image;
	std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement > m_camera_depth_left;
	std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement > m_camera_right_image;
	std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement > m_camera_depth_right;
	bool m_bTextureLeftInitialized;
	bool m_bTextureRightInitialized;
	bool m_bDepthInitialized;
	unsigned int m_pow2WidthLeft;
	unsigned int m_pow2HeightLeft;
	unsigned int m_pow2WidthRight;
	unsigned int m_pow2HeightRight;
	GLuint m_texture_left;
	GLuint m_texture_right;
	GLuint m_texture_depth_left;
	GLuint m_texture_depth_right;

	GLuint MatrixID;
	GLuint background_textureID;
	GLuint background_depthID;
	GLuint vertexbuffer;
	GLuint uvbuffer;
	GLuint Texture;
	GLuint backgroundID;
	GLuint program_2;
	GLuint object_matrixID;
	GLint object_modelMatrixID;
	GLint object_viewMatrixID;
	GLint object_projectMatrixID;
	GLint object_depthTextureID;
	GLuint object_textureID;
	GLuint object_vertexbuffer;
	GLuint object_uvbuffer;
	GLuint VertexArrayID[2];
	float randomA;
	float randomB;
};




#endif // __SIMPLE_AR_DEMO_RENDERER_HH__