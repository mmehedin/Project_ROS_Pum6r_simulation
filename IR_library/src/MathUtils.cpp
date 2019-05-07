
#include <MathUtils.hpp>
#include <iostream>

namespace IRlibrary {

	/** Returns true if value close to zero **/
	bool nearZero (double val, double eps) {//=1e-10
	//if ((val <eps & val > -1 * eps) || (val< -1* eps & val > eps)){
	//std::cout << "val= " << fabs(val)<<"\n";
	//std::cout << "eps= " << eps<<"\n";
    if(fabs(val) <= eps){
		return true;
	}else
		return false;
}


	/** Wraps angle (in radians) between 0 to 2 pi **/
	double wrapTo2PI (double val) {
	    val = fmod(val,2*M_PI);
    if (val < 0)
        val += 2*M_PI;
    return val;
	}

	/** Wraps angle (in radians) between -pi to pi **/
	double wrapToPI (double val) {
        double temp = wrapTo2PI(val);
        if (temp>=M_PI)
            return temp-2*M_PI;

        return temp;

	}

	/** Converts angle from degree to radians **/
	double deg2rad (double val) {
		return val;
	}

	/** Converts angle from radians to degree **/
	double rad2deg (double val) {
		return val;
	}

	int fixInaccuracies(double val){
    if (nearZero(1-val))
        return 1;
    return val;
}



} /* IRlibrary */
