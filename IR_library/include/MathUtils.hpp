#ifndef MATHUTILS
#define MATHUTILS
#include <cmath>
namespace IRlibrary {

	/** Returns true if value close to zero **/

	/** Wraps angle between 0 to 2 pi **/


	/** Wraps angle between -pi to pi **/


	bool nearZero(double, double eps = 1e-12);

    int fixInaccuracies(double);

    double wrapTo2PI (double);

    double wrapToPI(double);

} /* IRlibrary */
#endif /* ifndef MATHUTILS */
