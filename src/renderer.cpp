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
	//gluPerspective( m_moduleKey.m_fov, ((double)m_width/(double)m_height), m_moduleKey.m_near, m_moduleKey.m_far );
	// compute projection matrix
	glm::mat4 proj_matrix = compute_projection_matrix(m_intrinsics_left, m_resolution_left, 0.01, 100.);
	//LDEBUG << "projection matrix: " << glm::to_string(proj_matrix);
	glMultMatrixf( glm::value_ptr(proj_matrix) );


	// clear model-view transformation
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	// compute viewpoint matrix

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
		glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, width, height, imgFormat, GL_UNSIGNED_BYTE,m_camera_left_image->getDataPtr() );
		
		glDisable( GL_TEXTURE_2D );

	} else {
		LERROR << "Received incompatible image, format: " << m_camera_left_image->getPixelFormat() << " nchannels " << nchannels << " pixelsize " << pixelSize;
		return false;
	}

	return true;
}


glm::mat4 Renderer::compute_projection_matrix(glm::mat3& intrinsics, glm::ivec2& resolution, float n, float f) {

/*
	// XXX Adapted from Ubitrack::Algorithm::Projection

	Math::Matrix< T, 4, 4 > m2;
	ublas::subrange( m2, 0, 3, 0, 4 ) = m;

	T norm  = sqrt ( m2( 2, 0 )*m2( 2, 0 ) + m2( 2, 1 )*m2( 2, 1 ) + m2( 2, 2 )*m2( 2, 2 ) );

	// copy 3rd row to 4th row
	ublas::row( m2, 3 ) = ublas::row( m2, 2 );
	ublas::row( m2, 2 ) *= ( -f - n );

	//factor for normalization
	T add =  f * n * norm;

	m2( 2, 3 ) += add;

	//compute ortho matrix
	Math::Matrix< T, 4, 4 > ortho;

	ortho( 0, 0 ) = static_cast< T >( 2.0 ) / ( r - l );
	ortho( 0, 1 ) = static_cast< T >( 0.0 );
	ortho( 0, 2 ) = static_cast< T >( 0.0 );
	ortho( 0, 3 ) = ( r + l ) / ( l - r );
	ortho( 1, 0 ) = static_cast< T >( 0.0 );
	ortho( 1, 1 ) = static_cast< T >( 2.0 ) / ( t - b );
	ortho( 1, 2 ) = static_cast< T >( 0.0 );
	ortho( 1, 3 ) = ( t + b ) / ( b - t );
	ortho( 2, 0 ) = static_cast< T >( 0.0 );
	ortho( 2, 1 ) = static_cast< T >( 0.0 );
	ortho( 2, 2 ) = static_cast< T >( 2.0 ) / ( n - f );
	ortho( 2, 3 ) = ( f + n ) / ( n - f );
	ortho( 3, 0 ) = static_cast< T >( 0.0 );
	ortho( 3, 1 ) = static_cast< T >( 0.0 );
	ortho( 3, 2 ) = static_cast< T >( 0.0 );
	ortho( 3, 3 ) = static_cast< T >( 1.0 );

	return ublas::prod( ortho, m2 );
*/

	// no guarantee for correctness !!!!

	float l = 0;
	float r = resolution.x;
	float b = 0;
	float t = resolution.y;


	glm::mat4 m2(intrinsics);

	// XXX check column vs. row major !!!

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

	return ortho * m2;

/* 
  typical result
  1024x768
[ 1.70912 0 -0.109582 0 ]
[ 0 2.28297 -0.126095 0 ]
[ 0 0 -1.0002 -0.020002 ]
[ 0 0 -1 0 ]
*/
}