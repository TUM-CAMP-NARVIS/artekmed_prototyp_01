#include "basic_facade_demo/renderer.h"

#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("BasicFacadeExample.renderer"));

Renderer::Renderer()
: m_bTextureLeftInitialized(false) 
, m_pow2WidthLeft(0)
, m_pow2HeightLeft(0)
, m_texture_left(0)
{
	randomA = 0.00001f;
	randomB = 0.51234f;
	setup_shader();
}

Renderer::~Renderer() {

}
void Renderer::setup_shader()
{	
	glGenVertexArrays(2, VertexArrayID);
	backgroundID = LoadShaders("camera_texture.vert", "camera_texture.frag");
	object_programID = LoadShaders("cube.vert", "cube.frag");

	background_MatrixID = glGetUniformLocation(backgroundID, "MVP");
	background_textureID  = glGetUniformLocation(backgroundID, "myTextureSampler");
	
	object_matrixID=glGetUniformLocation(object_programID, "MVP");
	object_modelMatrixID=glGetUniformLocation(object_programID, "M");
	object_viewMatrixID=glGetUniformLocation(object_programID, "V");
	object_projectMatrixID=glGetUniformLocation(object_programID, "P");
	object_textureID=glGetUniformLocation(object_programID, "myTextureSampler");
	Texture = loadBMP_custom("uvtemplate.bmp");

	static const GLfloat g_vertex_buffer_data_2[] = { 
	-1.0f,-1.0f,-1.0f,
	-1.0f,-1.0f, 1.0f,
	-1.0f, 1.0f, 1.0f,
	1.0f, 1.0f,-1.0f,
	-1.0f,-1.0f,-1.0f,
	-1.0f, 1.0f,-1.0f,
	1.0f,-1.0f, 1.0f,
	-1.0f,-1.0f,-1.0f,
	1.0f,-1.0f,-1.0f,
	1.0f, 1.0f,-1.0f,
	1.0f,-1.0f,-1.0f,
	-1.0f,-1.0f,-1.0f,
	-1.0f,-1.0f,-1.0f,
	-1.0f, 1.0f, 1.0f,
	-1.0f, 1.0f,-1.0f,
	1.0f,-1.0f, 1.0f,
	-1.0f,-1.0f, 1.0f,
	-1.0f,-1.0f,-1.0f,
	-1.0f, 1.0f, 1.0f,
	-1.0f,-1.0f, 1.0f,
	1.0f,-1.0f, 1.0f,
	1.0f, 1.0f, 1.0f,
	1.0f,-1.0f,-1.0f,
	1.0f, 1.0f,-1.0f,
	1.0f,-1.0f,-1.0f,
	1.0f, 1.0f, 1.0f,
	1.0f,-1.0f, 1.0f,
	1.0f, 1.0f, 1.0f,
	1.0f, 1.0f,-1.0f,
	-1.0f, 1.0f,-1.0f,
	1.0f, 1.0f, 1.0f,
	-1.0f, 1.0f,-1.0f,
	-1.0f, 1.0f, 1.0f,
	1.0f, 1.0f, 1.0f,
	-1.0f, 1.0f, 1.0f,
	1.0f,-1.0f, 1.0f
	};
	static const GLfloat g_vertex_buffer_data[]={
		-1,1,0,
		1,1,0,
		1,-1,0,
		-1,1,0,
		1,-1,0,
		-1,-1,0
	};
	// Two UV coordinatesfor each vertex. They were created withe Blender.
	static const GLfloat g_uv_buffer_data_2[] = { 
		0.000059f, 1.0f-0.000004f, 
		0.000103f, 1.0f-0.336048f, 
		0.335973f, 1.0f-0.335903f, 
		1.000023f, 1.0f-0.000013f, 
		0.667979f, 1.0f-0.335851f, 
		0.999958f, 1.0f-0.336064f, 
		0.667979f, 1.0f-0.335851f, 
		0.336024f, 1.0f-0.671877f, 
		0.667969f, 1.0f-0.671889f, 
		1.000023f, 1.0f-0.000013f, 
		0.668104f, 1.0f-0.000013f, 
		0.667979f, 1.0f-0.335851f, 
		0.000059f, 1.0f-0.000004f, 
		0.335973f, 1.0f-0.335903f, 
		0.336098f, 1.0f-0.000071f, 
		0.667979f, 1.0f-0.335851f, 
		0.335973f, 1.0f-0.335903f, 
		0.336024f, 1.0f-0.671877f, 
		1.000004f, 1.0f-0.671847f, 
		0.999958f, 1.0f-0.336064f, 
		0.667979f, 1.0f-0.335851f, 
		0.668104f, 1.0f-0.000013f, 
		0.335973f, 1.0f-0.335903f, 
		0.667979f, 1.0f-0.335851f, 
		0.335973f, 1.0f-0.335903f, 
		0.668104f, 1.0f-0.000013f, 
		0.336098f, 1.0f-0.000071f, 
		0.000103f, 1.0f-0.336048f, 
		0.000004f, 1.0f-0.671870f, 
		0.336024f, 1.0f-0.671877f, 
		0.000103f, 1.0f-0.336048f, 
		0.336024f, 1.0f-0.671877f, 
		0.335973f, 1.0f-0.335903f, 
		0.667969f, 1.0f-0.671889f, 
		1.000004f, 1.0f-0.671847f, 
		0.667979f, 1.0f-0.335851f
	};

	static const GLfloat g_uv_buffer_data[] = { 
		0,1,
		1,1,
		1,0,
		0,1,
		1,0,
		0,0
	};


	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

	glGenBuffers(1, &uvbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data), g_uv_buffer_data, GL_STATIC_DRAW);

	glGenBuffers(1, &object_vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, object_vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data_2), g_vertex_buffer_data_2, GL_STATIC_DRAW);

	glGenBuffers(1, &object_uvbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, object_uvbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data_2), g_uv_buffer_data_2, GL_STATIC_DRAW);
}
bool Renderer::update_background_left()
{
	//glDisable(GL_BLEND);
	glViewport(0, 0, 640, 480);
	glDisable(GL_DEPTH_TEST);
	glUseProgram(backgroundID);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, m_texture_left);
	glUniform1i(background_textureID, 0);
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glVertexAttribPointer(
		0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
		);

	// 2nd attribute buffer : UVs
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	glVertexAttribPointer(
		1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
		2,                                // size : U+V => 2
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
		);
	// Draw the triangle !
	glDrawArrays(GL_TRIANGLES, 0, 2*3); // 12*3 indices starting at 0 -> 12 triangles
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	//glEnable(GL_BLEND);
	return true;
}

bool Renderer::drawObject_Left()
{ 
	float scale_object = 0.3f;
	//LINFO<<"Render object";
	glViewport(0, 0, 640, 480);
	glEnable(GL_DEPTH_TEST);
	//glClear(GL_DEPTH_BUFFER_BIT );
	glm::mat4 Projection = glm::perspective(45.0f, 640.f/480.f, 0.01f, 100.0f);
	// Camera matrix
	glm::mat4 View       = glm::lookAt(
		glm::vec3(4,3,3), // Camera is at (4,3,3), in World Space
		glm::vec3(0,0,0), // and looks at the origin
		glm::vec3(0,1,0)  // Head is up (set to 0,-1,0 to look upside-down)
		);
	//glm::vec3 viewShift = glm::vec3(1.0f, 0.0f, 3.9f);
	glm::vec3 viewShift = glm::vec3(0.0f, 0.0f, 4.0f);
	glm::mat4 Model( 
		scale_object, 0.0, 0.0, 0.0, 
		0.0, scale_object, 0.0, 0.0,
		0.0, 0.0, scale_object, 0.0,
		0.0,0.0,0.0,1.0);
	
	//Model = glm::translate(
	//	Model,
	//	viewShift
	//	);
	//glm::mat4 modelView= View*Model;
	// Model matrix : an identity matrix (model will be at the origin)
	// Our ModelViewProjection : multiplication of our 3 matrices
	//glm::mat4 MVP        = Projection * modelView; // Remember, matrix multiplication is the other way around
	glUseProgram(object_programID);
	glm::mat4 left_view=  m_camera_left_pose;
	glUniformMatrix4fv(object_modelMatrixID, 1, GL_FALSE, &Model[0][0]);
	glUniformMatrix4fv(object_viewMatrixID, 1, GL_FALSE, &left_view[0][0]);
	glUniformMatrix4fv(object_projectMatrixID, 1, GL_FALSE, &m_projection_left[0][0]);

	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, Texture);
	glUniform1i(object_textureID, 2);

	// 1rst attribute buffer : vertices
	glEnableVertexAttribArray(2);
	glBindBuffer(GL_ARRAY_BUFFER, object_vertexbuffer);
	glVertexAttribPointer(
		2,                  // attribute. No particular reason for 0, but must match the layout in the shader.
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
		);

	// 2nd attribute buffer : UVs
	glEnableVertexAttribArray(3);
	glBindBuffer(GL_ARRAY_BUFFER, object_uvbuffer);
	glVertexAttribPointer(
		3,                                // attribute. No particular reason for 1, but must match the layout in the shader.
		2,                                // size : U+V => 2
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
		);
	glEnableVertexAttribArray(4);
	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	glVertexAttribPointer(
		4,                                // attribute. No particular reason for 1, but must match the layout in the shader.
		2,                                // size : U+V => 2
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
		);
	// Draw the triangle !
	glDrawArrays(GL_TRIANGLES, 0, 12*3); // 12*3 indices starting at 0 -> 12 triangles
	glDisableVertexAttribArray(2);
	glDisableVertexAttribArray(3);
	glDisableVertexAttribArray(4);
	return true;
}

void Renderer::pre_render(Window* window) {
	//LDEBUG << "Renderer::pre_render";
	// clear buffers
	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	m_projection_left = compute_projection_matrix(m_intrinsics_left, m_resolution_left, 0.01f, 100);

	if(m_camera_left_image)
	{
		camera_left_update_texture();
		update_background_left();
	}
}

void Renderer::post_render(Window* window) {
	//LDEBUG << "Renderer::post_render";
	window->swap();
}

void Renderer::render(Window* window, unsigned long long int ts) {
	drawObject_Left();
}


bool Renderer::render_video_background() {
	// Disable transparency for background image. The Transparency
	// module might have enabled global transparency for the virtual
	// scene.  We have to restore this state below.
	glDisable( GL_BLEND );
	// check if we have an image to display as background
	if (!m_camera_left_image->isValid()) return false;
	int m_width  = m_resolution_left.x;
	int m_height = m_resolution_left.y;

	// store the projection matrix
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();

	// create a 2D-orthogonal projection matrix
	glLoadIdentity();
	gluOrtho2D( 0.0, m_width, 0.0, m_height );

	// prepare fullscreen bitmap without fancy extras
	GLboolean bLightingEnabled = glIsEnabled( GL_LIGHTING );
	glDisable( GL_LIGHTING );
	glDisable( GL_DEPTH_TEST );

	glEnable( GL_TEXTURE_2D );

	glBindTexture( GL_TEXTURE_2D, m_texture_left );

	// display textured rectangle

	// needs improvement of BasicImageMeasurement.
	bool flip_image = false;
	double y0 = flip_image ? 0 : m_height;
	double y1 = m_height - y0;
	double tx = double( m_camera_left_image->getDimX() ) / m_pow2WidthLeft;
	double ty = double( m_camera_left_image->getDimY() ) / m_pow2HeightLeft;

	//LINFO << "render_texture: " << m_texture_left << "," << m_width << "," << m_height << ","<< y0 << "," << y1 << "," << tx << "," << ty;

	// draw two triangles
	glBegin( GL_TRIANGLE_STRIP );
	glTexCoord2d(  0, ty ); glVertex2d(       0, y1 );
	glTexCoord2d(  0,  0 ); glVertex2d(       0, y0 );
	glTexCoord2d( tx, ty ); glVertex2d( m_width, y1 );
	glTexCoord2d( tx,  0 ); glVertex2d( m_width, y0 );
	glEnd();
		
	glDisable( GL_TEXTURE_2D );

	// restore opengl state
	glEnable( GL_BLEND );
	glEnable( GL_DEPTH_TEST );
	if ( bLightingEnabled )
		glEnable( GL_LIGHTING );

	glPopMatrix();
	glMatrixMode( GL_MODELVIEW );

	return true;
}

bool Renderer::camera_left_update_texture() 
{

	GLenum imgFormat = GL_LUMINANCE;
	switch(m_camera_left_image->getPixelFormat())
	{
	case Ubitrack::Facade::BasicImageMeasurement::LUMINANCE:
		break;
	case Ubitrack::Facade::BasicImageMeasurement::RGB:
		imgFormat = GL_RGB;
		break;
	case Ubitrack::Facade::BasicImageMeasurement::BGR:
		imgFormat = GL_BGR_EXT;
		break;
	default:
		LOG4CPP_ERROR(logger, "received incompatible pixelformat: " << m_camera_left_image->getPixelFormat());
		return false;
	}

	unsigned int width = m_camera_left_image->getDimX();
	unsigned int height = m_camera_left_image->getDimY();
	unsigned int nchannels = m_camera_left_image->getDimZ();
	unsigned int pixelSize = m_camera_left_image->getPixelSize();


	if ((nchannels == 3) && (pixelSize == 1)) {
		// all good BGR/RGB uchar image type .. only supported for now
		glEnable( GL_TEXTURE_2D );

		if ( !m_bTextureLeftInitialized )
		{
			m_bTextureLeftInitialized = true;
			
			// generate power-of-two sizes
			m_pow2WidthLeft = 1;
			while ( m_pow2WidthLeft < width )
				m_pow2WidthLeft <<= 1;
				
			m_pow2HeightLeft = 1;
			while ( m_pow2HeightLeft < height )
				m_pow2HeightLeft <<= 1;
			
			// create new empty texture
			glGenTextures( 1, &m_texture_left );
			glBindTexture( GL_TEXTURE_2D, m_texture_left );

			// define texture parameters
			glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
			glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
			glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
			glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
			glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL );
			// load empty texture image (defines texture size)
			glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, m_pow2WidthLeft, m_pow2HeightLeft, 0, imgFormat, GL_UNSIGNED_BYTE, 0 );
			//LINFO << "glTexImage2D( width=" << m_pow2WidthLeft << ", height=" << m_pow2HeightLeft << " ): " << glGetError();
		}
		
		// load image into texture
		glBindTexture( GL_TEXTURE_2D, m_texture_left );
		glTexImage2D( GL_TEXTURE_2D, 0, 3, width, height, 0, imgFormat, GL_UNSIGNED_BYTE, m_camera_left_image->getDataPtr());
		//glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, width, height, imgFormat, GL_UNSIGNED_BYTE, m_camera_left_image->getDataPtr() );
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); 
		glDisable( GL_TEXTURE_2D );

	} else {
		LOG4CPP_ERROR(logger, "Received incompatible  left image, format: " << m_camera_left_image->getPixelFormat() << " nchannels " << nchannels << " pixelsize " << pixelSize);
		return false;
	}

	return true;
}

glm::mat4 Renderer::compute_projection_matrix(glm::mat3& intrinsics, glm::ivec2& resolution, float n, float f) {

	float l = 0;
	float r = (float)resolution.x;
	float b = 0;
	float t = (float)resolution.y;

	// mat4 should be all zeros, but glm initializes mat4 as identity
	glm::mat4 m2(intrinsics);
	m2[3].w = 0.0;

	float norm = glm::sqrt( m2[0].z*m2[0].z + m2[1].z*m2[1].z + m2[2].z*m2[2].z );
	float nf = ( -f - n );
	float add = f * n * norm;

	// copy 3rd row to 4th row
	m2[0].w = m2[0].z;
	m2[1].w = m2[1].z;
	m2[2].w = m2[2].z;
	m2[3].w = m2[3].z;

	m2[0].z *= nf;
	m2[1].z *= nf;
	m2[2].z *= nf;
	m2[3].z *= nf;

	m2[3].z += add;

	glm::mat4 ortho;

	ortho[0].x = static_cast< float >( 2.0 ) / ( r - l );
	ortho[1].x = static_cast< float >( 0.0 );
	ortho[2].x = static_cast< float >( 0.0 );
	ortho[3].x = ( r + l ) / ( l - r );
	ortho[0].y = static_cast< float >( 0.0 );
	ortho[1].y = static_cast< float >( 2.0 ) / ( t - b );
	ortho[2].y = static_cast< float >( 0.0 );
	ortho[3].y = ( t + b ) / ( b - t );
	ortho[0].z = static_cast< float >( 0.0 );
	ortho[1].z = static_cast< float >( 0.0 );
	ortho[2].z = static_cast< float >( 2.0 ) / ( n - f );
	ortho[3].z = ( f + n ) / ( n - f );
	ortho[0].w = static_cast< float >( 0.0 );
	ortho[1].w = static_cast< float >( 0.0 );
	ortho[2].w = static_cast< float >( 0.0 );
	ortho[3].w = static_cast< float >( 1.0 );

	glm::mat4 proj_mat = ortho * m2;
	return proj_mat;

}