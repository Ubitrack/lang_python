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



BackgroundImage::BackgroundImage( )
	: m_lastImageTimestamp(0)
{
}


BackgroundImage::~BackgroundImage()
{
}


/** deletes OpenGL state */
void BackgroundImage::glCleanup()
{
	//LOG4CPP_DEBUG( logger, "glCleanup() called" );
    m_textureUpdate.cleanupTexture();
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

	glEnable( GL_TEXTURE_2D );

    if ( !m_textureUpdate.isInitialized() )
    {
        m_textureUpdate.initializeTexture(m_image);
    }

    if (m_lastImageTimestamp < m_image.time()) {
        m_textureUpdate.updateTexture(m_image);
    }

	// load image into texture
	glBindTexture( GL_TEXTURE_2D, m_textureUpdate.textureId() );

	// display textured rectangle
	double y0 = m_image->origin() ? 0 : m_height;
	double y1 = m_height - y0;
	double tx = double( m_image->width() ) / m_textureUpdate.pow2width();
	double ty = double( m_image->height() ) / m_textureUpdate.pow2height();

	// draw two triangles
	glBegin( GL_TRIANGLE_STRIP );
	glTexCoord2d(  0, ty ); glVertex2d(       0, y1 );
	glTexCoord2d(  0,  0 ); glVertex2d(       0, y0 );
	glTexCoord2d( tx, ty ); glVertex2d( m_width, y1 );
	glTexCoord2d( tx,  0 ); glVertex2d( m_width, y0 );
	glEnd();

    glBindTexture(GL_TEXTURE_2D, 0);

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





