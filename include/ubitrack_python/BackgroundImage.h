/*
 * BackgroundImage.h
 *
 *  Created on: 25/11/2013
 *      Author: jack
 */

#ifndef BACKGROUNDIMAGE_H_
#define BACKGROUNDIMAGE_H_

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#ifdef _WIN32
  #include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>


#include <utMeasurement/Measurement.h>
#include <utVision/Image.h>
#include <utVision/TextureUpdate.h>
#include <utUtil/Logging.h>

namespace Ubitrack { namespace Python {

class BackgroundImage
{
public:

	/**
	 * Constructor
	 */
	BackgroundImage();
	~BackgroundImage();

	/** render the object */
	virtual void draw( int m_width, int m_height );

	/**
	 * callback from Image port
	 * passes image to the parent module
	 * @param img an image Measurement
	 */
	void imageIn( const Measurement::ImageMeasurement& img );

	/** deletes OpenGL state */
    virtual void glCleanup();


protected:

	Measurement::ImageMeasurement m_image;
	boost::mutex m_imageLock;
	Measurement::Timestamp m_lastImageTimestamp;
	Vision::TextureUpdate m_textureUpdate;

};


}} // end ns Ubitrack::Python


#endif /* BACKGROUNDIMAGE_H_ */
