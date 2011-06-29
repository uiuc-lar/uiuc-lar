/*
 *  colorTransform.h
 *
 * 	Logan Niehaus
 * 	6/28/11
 * 	auxiliary function that normalizes the RGB color space according to:
 *
 *		(rn, gn, bn) = 255*(r/max(g,b), g/max(r,b), b/max(r,g))
 *
 *	which is then saturated between 0 and 255. also produces a color saliency map
 *	with the equation:
 *
 *		s = 255 - min(rn, gn, bn)
 *
 *
 */


#include <yarp/sig/Image.h>

//namespaces
using namespace yarp::sig;

//apply color normalization and invert to get color based salience
void normalizeColor(ImageOf<PixelRgb> &src, ImageOf<PixelRgb> &dst, ImageOf<PixelFloat> &sal) {


	dst.resize(src);
	sal.resize(src);

	PixelRgb px;
	float r,g,b;
	float rn,gn,bn,l,c;

	for (int i = 0; i < src.width(); i++) {
		for (int j = 0; j < src.height(); j++) {

			px = src.pixel(i,j);
			r = px.r;
			g = px.g;
			b = px.b;

			rn = 255.0*r/b;
			gn = 255.0*g/r;
			bn = 255.0*b/g;

			if (g > b) rn = 255.0*r/g;
			if (b > r) gn = 255.0*g/b;
			if (r > g) bn = 255.0*b/r;

			if (rn > 255.0) rn = 255.0;
			if (gn > 255.0) gn = 255.0;
			if (bn > 255.0) bn = 255.0;

			dst.pixel(i,j) = PixelRgb((unsigned char)rn,
										(unsigned char)gn,
										(unsigned char)bn);

			c = rn;
			if (gn < c) c = gn;
			if (bn < c) c = bn;

			sal.pixel(i,j) = 255 - c;

		}
	}

}
