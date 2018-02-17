/*
 * BackgroundImage.cpp
 *
 *  Created on: 25/11/2013
 *      Author: jack
 */
/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

#include "ubitrack_python/BackgroundImage.h"

namespace Ubitrack { namespace Python {



bool BackgroundImage::getImageFormat(const Vision::Image::ImageFormatProperties& fmtSrc,
        Vision::Image::ImageFormatProperties& fmtDst,
        bool use_gpu, int& umatConvertCode,
        GLenum& glFormat, GLenum& glDatatype)
{
    bool ret = true;

    // determine datatype
    switch(fmtSrc.depth) {
    case CV_8U:
        glDatatype = GL_UNSIGNED_BYTE;
        break;
    case CV_16U:
        glDatatype = GL_UNSIGNED_SHORT;
        break;
    case CV_32F:
        glDatatype = GL_FLOAT;
        break;
    case CV_64F:
        glDatatype = GL_DOUBLE;
        break;
    default:
        // assume unsigned byte
        glDatatype = GL_UNSIGNED_BYTE;
        // Log Error ?
        ret = false;
        break;
    }

    // determine image properties
    switch (fmtSrc.imageFormat) {
    case Vision::Image::LUMINANCE:
        glFormat = GL_LUMINANCE;
        fmtDst.channels = 1;
        break;
    case Vision::Image::RGB:
        glFormat = use_gpu ? GL_RGBA : GL_RGB;
        fmtDst.channels = use_gpu ? 4 : 3;
        fmtDst.imageFormat = use_gpu ? Vision::Image::RGBA : Vision::Image::RGB;
        umatConvertCode = cv::COLOR_RGB2RGBA;
        break;
#ifndef GL_BGR_EXT
    case Vision::Image::BGR:
        fmtDst.channels = use_gpu ? 4 : 3;
        glFormat = image_isOnGPU ? GL_RGBA : GL_RGB;
        fmtDst.imageFormat = use_gpu ? Vision::Image::RGBA : Vision::Image::BGR;
        umatConvertCode = cv::COLOR_BGR2RGBA;
        break;
    case Vision::Image::BGRA:
        fmt.channels = 4;
        glFormat = use_gpu ? GL_RGBA : GL_BGRA;
        fmtDst.imageFormat = use_gpu ? Vision::Image::RGBA : Vision::Image::BGRA;
        umatConvertCode = cv::COLOR_BGRA2RGBA;
        break;
#else
    case Vision::Image::BGR:
        fmtDst.channels = use_gpu ? 4 : 3;
        glFormat = use_gpu ? GL_RGBA : GL_BGR_EXT;
        fmtDst.imageFormat = use_gpu ? Vision::Image::RGBA : Vision::Image::BGR;
        umatConvertCode = cv::COLOR_BGR2RGBA;
        break;
    case Vision::Image::BGRA:
        fmtDst.channels = 4;
        glFormat = use_gpu ? GL_RGBA : GL_BGRA_EXT;
        fmtDst.imageFormat = use_gpu ? Vision::Image::RGBA : Vision::Image::BGRA;
        umatConvertCode = cv::COLOR_BGRA2RGBA;
        break;
#endif
    case Vision::Image::RGBA:
        fmtDst.channels = 4;
        glFormat = GL_RGBA;
        fmtDst.imageFormat = Vision::Image::RGBA;
        break;
    default:
        // Log Error ?
        ret = false;
        break;
    }

    // update dependent parameters
    fmtDst.bitsPerPixel = fmtSrc.bitsPerPixel / fmtSrc.channels * fmtDst.channels;
    fmtDst.matType = CV_MAKE_TYPE(fmtDst.depth, fmtDst.channels);

    return ret;
}

BackgroundImage::BackgroundImage( )
	: m_bTextureInitialized( false )
	, m_lastImageTimestamp(0)
{
}


BackgroundImage::~BackgroundImage()
{
}


/** deletes OpenGL state */
void BackgroundImage::glCleanup()
{
	//LOG4CPP_DEBUG( logger, "glCleanup() called" );

	if ( m_bTextureInitialized ) {
 		glBindTexture( GL_TEXTURE_2D, 0 );
 		glDisable( GL_TEXTURE_2D );
 		glDeleteTextures( 1, &m_texture );
 	}
}


/** render the object */
void BackgroundImage::draw( int m_width, int m_height )
{
	// Disable transparency for background image. The Transparency
	// module might have enabled global transparency for the virtual
	// scene.  We have to restore this state below.
	glDisable( GL_BLEND );

	// check if we have an image to display as background
	if ( m_image.get() == 0 ) return;

	// store the projection matrix
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();

	// create a 2D-orthogonal projection matrix
	glLoadIdentity();
	gluOrtho2D( 0.0, m_width, 0.0, m_height );


	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();

	// prepare fullscreen bitmap without fancy extras
	GLboolean bLightingEnabled = glIsEnabled( GL_LIGHTING );
	glDisable( GL_LIGHTING );
	glDisable( GL_DEPTH_TEST );

	// lock it to avoid random crashes
	boost::mutex::scoped_lock l( m_imageLock );

	// find out texture format
    // find out texture format
    int umatConvertCode = -1;
    GLenum glFormat = GL_LUMINANCE;
    GLenum glDatatype = GL_UNSIGNED_BYTE;
    int numOfChannels = 1;
    Vision::Image::ImageFormatProperties fmtSrc, fmtDst;
    m_image->getFormatProperties(fmtSrc);
    m_image->getFormatProperties(fmtDst);

    getImageFormat(fmtSrc, fmtDst, false, umatConvertCode, glFormat, glDatatype);

	// texture version
	glEnable( GL_TEXTURE_2D );

	if ( !m_bTextureInitialized )
	{
		m_bTextureInitialized = true;

		// generate power-of-two sizes
		m_pow2Width = 1;
		while ( m_pow2Width < (unsigned)m_image->width() )
			m_pow2Width <<= 1;

		m_pow2Height = 1;
		while ( m_pow2Height < (unsigned)m_image->height() )
			m_pow2Height <<= 1;

		// create new empty texture
		glGenTextures( 1, &m_texture );
		glBindTexture( GL_TEXTURE_2D, m_texture );

		// define texture parameters
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
		glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL );

		// load empty texture image (defines texture size)
        glTexImage2D( GL_TEXTURE_2D, 0, 3, m_pow2Width, m_pow2Height, 0, glFormat, glDatatype, 0 );
		//LOG4CPP_DEBUG( logger, "glTexImage2D( width=" << m_pow2Width << ", height=" << m_pow2Height << " ): " << glGetError() );
	}

	// load image into texture
	glBindTexture( GL_TEXTURE_2D, m_texture );

	// only update texture if lastTS < image.time()
	if (m_lastImageTimestamp < m_image.time()) {
		glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, m_image->width(), m_image->height(),
			glFormat, glDatatype, m_image->Mat().data);
	}

	// display textured rectangle
	double y0 = m_image->origin() ? 0 : m_height;
	double y1 = m_height - y0;
	double tx = double( m_image->width() ) / m_pow2Width;
	double ty = double( m_image->height() ) / m_pow2Height;

	// draw two triangles
	glBegin( GL_TRIANGLE_STRIP );
	glTexCoord2d(  0, ty ); glVertex2d(       0, y1 );
	glTexCoord2d(  0,  0 ); glVertex2d(       0, y0 );
	glTexCoord2d( tx, ty ); glVertex2d( m_width, y1 );
	glTexCoord2d( tx,  0 ); glVertex2d( m_width, y0 );
	glEnd();

	glDisable( GL_TEXTURE_2D );

	// change timestamp to image time
	m_lastImageTimestamp = m_image.time();

	// restore opengl state
	glEnable( GL_BLEND );
	glEnable( GL_DEPTH_TEST );
	if ( bLightingEnabled )
		glEnable( GL_LIGHTING );


	glPopMatrix();

	glMatrixMode( GL_PROJECTION );
	glPopMatrix();

	glMatrixMode( GL_MODELVIEW );

}

/**
 * callback from Image port
 * passes image to the parent module
 * @param img an image Measurement
 */
void BackgroundImage::imageIn( const Ubitrack::Measurement::ImageMeasurement& img )
{
	//LOG4CPP_DEBUG( logger, "received background image with timestamp " << img.time() );
	boost::mutex::scoped_lock l( m_imageLock );
	m_image = img;
}

} } // namespace Ubitrack::Python





