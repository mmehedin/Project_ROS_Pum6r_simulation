#ifndef MATHUTILS
#define MATHUTILS

#include <cmath>

namespace IRlibrary {

	/** Returns true if value close to zero **/
	bool nearZero (double val, double eps = 1e-10) {
		return true;
	}

	/** Wraps angle (in radians) between 0 to 2 pi **/
	double wrapTo2PI (double val) {
	}

	/** Wraps angle (in radians) between -pi to pi **/
	double wrapToPI (double val) {
		return val;
	}

	/** Converts angle from degree to radians **/
	double deg2rad (double val) {
		return val;
	}
	/** Converts angle from radians to degree **/
	double rad2deg (double val) {
		return val;
	}


} /* IRlibrary */
#endif /* ifndef MATHUTILS */
