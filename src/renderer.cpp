#include "simple_ar_demo/renderer.h"
#include  "simple_ar_demo/easylogging++.h"


Renderer::Renderer()
: m_bTextureLeftInitialized(false) 
, m_pow2WidthLeft(0)
, m_pow2HeightLeft(0)
, m_texture_left(0)
{

}

Renderer::~Renderer() {

}

void Renderer::pre_render(Window* window) {
	//LDEBUG << "Renderer::pre_render";
	// clear buffers
	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	// create a perspective projection matrix
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();

	// load camera projection matrix
	//gluPerspective( 45., ((double)m_resolution_left.x/(double)m_resolution_left.y), 0.001, 100. );

	/*
	working projection matrix:

	(1.810660, 0.000000, 0.000000, 0.000000), 
	(0.000000, 2.414214, 0.000000, 0.000000), 
	(0.000000, 0.000000, -1.000020, -1.000000), 
	(0.000000, 0.000000, -0.002000, 0.000000)
	*/

	// compute projection matrix
	//glMatrixMode( GL_PROJECTION );
	//glLoadIdentity();
	glm::mat4 proj_matrix = compute_projection_matrix(m_intrinsics_left, m_resolution_left, 0.01, 100.);
	glMultMatrixf( glm::value_ptr(proj_matrix) );

	/*
	not working projection matrix:

	(1.719300, 0.000000, 0.000000, 0.000000), 
	(0.000000, 2.307032, 0.000000, 0.000000), 
	(-0.113703, -0.114905, -1.000020, -1.000000), 
	(0.000000, 0.000000, 0.000000, 0.000000)
	*/
	//GLfloat model[16]; 
	//glGetFloatv(GL_PROJECTION_MATRIX, model);

	//glm::mat4 dbg_proj_mat = glm::make_mat4(model);
	//LINFO << "proj matr: " << glm::to_string(dbg_proj_mat);


	// clear model-view transformation
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	
	// update textures from current camera image
	// if no strict synchronization between camera and renderer is desirable, 
	// then the timestamp of the image-measurement should be checked for updates
	camera_left_update_texture();

	// render the AR video background
	render_video_background();
}

void Renderer::post_render(Window* window) {
	//LDEBUG << "Renderer::post_render";
	window->swap();
}

void Renderer::render(Window* window, unsigned long long int ts) {


	// do render all scene elements here
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixf(glm::value_ptr(m_camera_left_pose));

	//glTranslatef(0.0f, 0.0f, -7.0f);
	glBegin(GL_POLYGON);
	glColor3f(   1.0,  1.0, 0.0 );
	glVertex3f(  0.05, -0.05, 0.05 );
	glVertex3f(  0.05,  0.05, 0.05 );
	glVertex3f( -0.05,  0.05, 0.05 );
	glVertex3f( -0.05, -0.05, 0.05 );
	glEnd();

	// Purple side - RIGHT
	glBegin(GL_POLYGON);
	glColor3f(  1.0,  0.0,  1.0 );
	glVertex3f( 0.05, -0.05, -0.05 );
	glVertex3f( 0.05,  0.05, -0.05 );
	glVertex3f( 0.05,  0.05,  0.05 );
	glVertex3f( 0.05, -0.05,  0.05 );
	glEnd();
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
		LERROR << "received incompatible pixelformat: " << m_camera_left_image->getPixelFormat();
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
		    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL );
			
			// load empty texture image (defines texture size)
			glTexImage2D( GL_TEXTURE_2D, 0, 3, m_pow2WidthLeft, m_pow2HeightLeft, 0, imgFormat, GL_UNSIGNED_BYTE, 0 );
			LINFO << "glTexImage2D( width=" << m_pow2WidthLeft << ", height=" << m_pow2HeightLeft << " ): " << glGetError();
		}
		
		// load image into texture
		glBindTexture( GL_TEXTURE_2D, m_texture_left );
		glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, width, height, imgFormat, GL_UNSIGNED_BYTE, m_camera_left_image->getDataPtr() );
		//LINFO << "update texture: " << width << "," << height << "," << imgFormat;
		
		glDisable( GL_TEXTURE_2D );

	} else {
		LERROR << "Received incompatible image, format: " << m_camera_left_image->getPixelFormat() << " nchannels " << nchannels << " pixelsize " << pixelSize;
		return false;
	}

	return true;
}


glm::mat4 Renderer::compute_projection_matrix(glm::mat3& intrinsics, glm::ivec2& resolution, float n, float f) {

	float l = 0;
	float r = resolution.x;
	float b = 0;
	float t = resolution.y;

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