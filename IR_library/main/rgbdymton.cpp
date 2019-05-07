#include "RigidBodyMotion.hpp"
//#include <libeigen3>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <Eigen>
using namespace Eigen;
namespace IRlibrary {

/*
/** Returns true if a 3x3 matrix is skew-symmetric **/
	bool isso3(so3Mat mat){
		return true;
	}

	/** Returns true if a 4x4 matrix is SE(3) **/
	//bool isSE3(SE3Mat mat){
	//	return true;
	//}

	/** Returns true if a 4x4 matrix is se(3) **/
	bool isse3(se3Mat mat){
		return true;
	}

	/** Checks if the vector is unit **/
	bool isUnit(Vec2 vec){
		return true;
	}
	bool isUnit(Vec3 vec){
		return true;
	}
	bool isUnit(Vec4 vec){
		return true;
	}

	/** Computes the inverse of rotation matrix **/
	SO3Mat RotInv(SO3Mat mat){
		return mat;
	}

	/** Computes the 3X3 skew-symmetric matrix corresponding to 3x1 omega vector **/
	so3Mat VecToso3(Vec3 omega){
		so3Mat mat;
		mat << 1, 0, 0,
				0, 1, 0,
				0, 0, 1;
		return mat;
	}

	/** Computes the 3-vector corresponding to 3x3 skew-symmetric matrix **/
	Vec3 so3ToVec(so3Mat mat){
		Vec3 omega;
		return omega;
	}
}
