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

	bool getImageFormat(const Vision::Image::ImageFormatProperties& fmtSrc,
            Vision::Image::ImageFormatProperties& fmtDst,
			bool use_gpu, int& umatConvertCode,
			GLenum& glFormat, GLenum& glDatatype);


	Measurement::ImageMeasurement m_image;
	boost::mutex m_imageLock;

	// variables for textured drawing
	Measurement::Timestamp m_lastImageTimestamp;
	bool m_bTextureInitialized;
	GLuint m_texture;
	unsigned m_pow2Width;
	unsigned m_pow2Height;

};


}} // end ns Ubitrack::Python


#endif /* BACKGROUNDIMAGE_H_ */
