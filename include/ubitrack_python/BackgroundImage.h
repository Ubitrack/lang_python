/*
 * BackgroundImage.h
 *
 *  Created on: 25/11/2013
 *      Author: jack
 */

#ifndef BACKGROUNDIMAGE_H_
#define BACKGROUNDIMAGE_H_

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>


#include <utMeasurement/Measurement.h>
#include <utVision/Image.h>
#include <utVision/TextureUpdate.h>
#include <utUtil/Logging.h>


#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif


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
