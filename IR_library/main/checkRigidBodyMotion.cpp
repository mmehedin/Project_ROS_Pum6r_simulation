

#include <cmath>
#include <iostream>
#include "TypeDefs.hpp"
#include "RigidBodyMotion.hpp"
#include "ForwardKinematics.hpp"
#include <MathUtils.hpp>

//#include <libeigen3>
#include <eigen3/Eigen/Dense>
//#include <Eigen3>

using namespace IRlibrary;
using namespace Eigen;

//bool isSO3(SO3Mat mat);
inline bool equalQ(double val1, double val2, double eps = 1e-12) { return fabs(val1 - val2) <= eps; }

int main(int argc, char **argv)
{

    std::cout << "Near Zero: " << nearZero(0)<<"\n";

std::cout << "Near Zero: " << 	nearZero(0.0)<<"\n";
	std::cout << "Near Zero: " << nearZero(1e-2)<<"\n";
std::cout << "Near Zero: " << 	nearZero(1e-12)<<"\n";
	std::cout << "Near Zero: " << nearZero(1.0)<<"\n";
	std::cout << "Near Zero: " << nearZero(1e10)<<"\n";
	std::cout << "Near Zero: " << nearZero(1e-2, 0.0)<<"\n";
	std::cout << "Near Zero: " << nearZero(1e-2, 1e-1)<<"\n";


    std::cout << "WrapTo2PI: " << (equalQ(wrapTo2PI(0), 0, 0) || equalQ(wrapTo2PI(0), 2 * M_PI))<<"\n";
	std::cout << "WrapTo2PI: " << equalQ(wrapTo2PI(M_PI), M_PI)<<"\n";
	std::cout << "WrapTo2PI: " << (equalQ(wrapTo2PI(2 * M_PI), 0.0) || equalQ(wrapTo2PI(2 * M_PI), 2 * M_PI))<<"\n";
	std::cout << "WrapTo2PI: " << equalQ(wrapTo2PI(3 * M_PI), M_PI)<<"\n";
	std::cout << "WrapTo2PI: " << (equalQ(wrapTo2PI(4 * M_PI), 0.0) || equalQ(wrapTo2PI(4 * M_PI), 2 * M_PI))<<"\n";
	std::cout << "WrapTo2PI: " << equalQ(wrapTo2PI(9 * M_PI), M_PI)<<"\n";
	std::cout << "WrapTo2PI: " << equalQ(wrapTo2PI(M_PI/3), M_PI/3)<<"\n";
	std::cout << "WrapTo2PI: " << equalQ(wrapTo2PI(13 * M_PI/3.), M_PI/3.)<<"\n";

	std::cout << "WrapToPI: " << (equalQ(wrapToPI(0), 0, 0))<<"\n";
std::cout << "WrapToPI: " << (equalQ(wrapToPI(M_PI), -M_PI) || equalQ(wrapToPI(M_PI), M_PI))<<"\n";
	std::cout << "WrapToPI: " << (equalQ(wrapToPI(2 * M_PI), 0.0))<<"\n";
	std::cout << "WrapToPI: " << (equalQ(wrapToPI(3 * M_PI), -M_PI) || equalQ(wrapToPI(3 * M_PI), M_PI))<<"\n";
	std::cout << "WrapToPI: " << (equalQ(wrapToPI(4 * M_PI), 0.0))<<"\n";
	std::cout << "WrapToPI: " << (equalQ(wrapToPI(9 * M_PI), -M_PI) || equalQ(wrapToPI(9 * M_PI), M_PI))<<"\n";
	std::cout << "WrapToPI: " << (equalQ(wrapToPI(M_PI/3), M_PI/3))<<"\n";
	std::cout << "WrapToPI: " << (equalQ(wrapToPI(13 * M_PI/3.), M_PI/3.))<<"\n";


	SO3Mat mat3_2;
	double tht2 = M_PI/3;
	mat3_2 << cos(tht2), -sin(tht2), 0, sin(tht2), cos(tht2), 0, 0, 0, 1;
	so3Mat mat_so3_2;
	mat_so3_2 << 0, -tht2, 0, tht2, 0, 0, 0, 0, 0;
	std::cout << "matrixlog: " << (mat_so3_2.isApprox(MatrixLog3(mat3_2)))<<"\n";

	SO3Mat mat5;
	double tht = M_PI/3;
	mat5 << cos(tht), -sin(tht), 0, sin(tht), cos(tht), 0, 0, 0, 1;
	so3Mat mat5_so3;
	mat5_so3 << 0, -tht, 0, tht, 0, 0, 0, 0, 0;
	std::cout << "MatrixExp3.all: " <<(mat5.isApprox(MatrixExp3(mat5_so3)));

	mat5 = SO3Mat::Identity();
	std::cout << "MatrixExp3.all: " <<(mat5.isApprox(MatrixExp3(so3Mat::Zero())));

	AxisAngle aa;
	aa = AxisAng3(Vec3{0, 0, 0});
	std::cout << "AxisAngle: " << (equalQ(aa.omega.norm(), 1))<<"\n";
	aa = AxisAng3(Vec3{0, 0, 1});
	std::cout << "AxisAngle: " << (equalQ(aa.omega.norm(), 1))<<"\n";
	std::cout << "AxisAngle: " << (equalQ(aa.theta, 1))<<"\n";
	double theta3 = M_PI/3;
	aa = AxisAng3(10 * Vec3{cos(theta3), sin(theta3), 0});
	std::cout << "AxisAngle: " << (equalQ(aa.omega.norm(), 1))<<"\n";
	std::cout << "AxisAngle: " << (equalQ(aa.theta, 10))<<"\n";


    std::cout << "isse3: " << isse3(se3Mat::Identity())<<"\n";
	so3Mat mat2;
	double x1 = rand();
	double x2 = rand();
	double x3 = rand();
	mat2 << 0, -x3, x2, x3, 0, -x1, -x2, x1, 0;
	se3Mat matV;
	matV.bottomRows<1>() = Vec4::Zero();
	matV.topLeftCorner<3,3>() = mat2;
	auto matV1 = matV;
	std::cout << "isse3: " << (isse3(matV1))<<"\n";
	std::cout << "isse3: " << (isse3(se3Mat::Zero()))<<"\n";
	std::cout << "bottom rows: " << matV.bottomRows<1>();
	matV.bottomRows<1>() = Vec4{0, 0, 0, 1};
	auto matV2 = matV;
	std::cout << "isse3: " << (isse3(matV2))<<"\n";

	double theta = M_PI/3;
	SO3Mat rotMat;
	rotMat << cos(theta) , -sin(theta), 0,
				 sin(theta), cos(theta), 0,
				 0, 0, 1;
    SE3Mat rotMat2;
    rotMat2 << cos(theta) , -sin(theta), 0, 1,
				 sin(theta), cos(theta), 0, 2,
				 0,0,1,0,
				 0, 0, 0, 1;
				 Vec4 vec;
				 vec<<1,0,0,0;
    Vec3 vec_3;
				 vec_3<<0.99999999999,0,0;
    //rotMat2 << 0, 0, 1;
	std::cout << rotMat << std::endl;
	//std::cout << rotMat2 << std::endl;
	std::cout << "row number " << rotMat.rows() << std::endl;
	VectorXd x = rotMat.row(rotMat.rows()-1);
	Vector3d y;
	y << 0,0,1;
	std::cout << y;// << std::endl();
	std::cout << x;// << std::endl();
	std::cout << ((x - y).norm() == 0);
	//std::cout << x.isApprox(y);
	std::cout << "row 2 is: " << rotMat.row(rotMat.rows()-1) << std::endl;
	//std::cout << RotInv(rotMat) << std::endl;
	std::cout << rotMat.inverse() << std::endl;
	std::cout << rotMat.transpose() << std::endl;


	std::cout << "is SO3 " << isSO3(rotMat) << std::endl;
	std::cout << "is so3 " << isso3(rotMat) << std::endl;
	std::cout << "is SE3 " << isSE3(rotMat2) << std::endl;
	std::cout << "is se3 " << isse3(rotMat2) << std::endl;
	std::cout << "is unit " << isUnit(vec) << std::endl;
	std::cout << "is unit3 " << isUnit(vec_3) << std::endl;
	std::cout << "inverse rotation matrix " << RotInv(rotMat) << std::endl;
	Vec3 om;
	om<<1,2,3;

	std::cout << "skew symmetric matrix " << VecToso3(om) << std::endl;
	so3Mat m = VecToso3(om);
    std::cout << "omega from skew " << so3ToVec(m) << std::endl;
	std::cout << "omega from omegatheta " << AxisAng3(om).omega << std::endl;
	std::cout << "theta from omegatheta  " << AxisAng3(om).theta  << std::endl;

    SO3Mat q =  MatrixExp3(m) ;
	std::cout << "R in SO3 from so3 " <<q<< std::endl;
	std::cout << "omegatheta in so3 from R in SO3 " << MatrixLog3(q) << std::endl;
	SE3Mat t =RpToTrans(rotMat,om);
	std::cout << "T from R and p " << t << std::endl;

    SO3Mat R_12; Vec3 p_12;
	TransToRp(t, R_12, p_12);
	std::cout << "R " << R_12 << std::endl;
	std::cout << "p " << p_12 << std::endl;


	std::cout << "inverse transformatin matrix " << TransInv(rotMat2)*rotMat2 << std::endl;
	std::cout << "inverse transformatin matrix " << TransInv(rotMat2) << std::endl;
    Twist V;
    V << 1,2,3,4,5,6;
    se3Mat se333;
    se333 = VecTose3(V);
	std::cout << "twist to se3 " << se333;

	std::cout << "se3 to twist  " << se3ToVec(se333)<< std::endl;

	std::cout << "adjoint from SE3  " << Adjoint(rotMat2)<< std::endl;
	Vec3 s,que;
	double h;
	s<<1,0,0;
	que<<1,0,0;
	h=2;
	std::cout << "screw to axis  " << ScrewToAxis(que,s,h)<< std::endl;
	Twist STheta;
	STheta <<2,0,0,2,3,0;
	ScrewAxis Se;
	double th;
    AxisAng(STheta, Se, th);
	std::cout << "Stheta to S and theta  " <<Se << th << std::endl;
    se3Mat inmat;
    inmat << cos(theta) , -sin(theta), 0, 1,
				 sin(theta), cos(theta), 0, 2,
				 0,0,1,0,
				 0, 0, 0, 0;
	//MatrixExp6(inmat);
	std::cout << "Homogenouse matrix transfomration from se3 to SE3  " <<std::endl << MatrixExp6(inmat) << std::endl;
	std::cout.flush();
	//MatrixLog6(t);
	std::cout << "Transformation in SE3 to log in se3 " <<MatrixLog6(t)<<std::endl;


	SE3Mat M;
	M << 1,0,0,3,
	0,1,0,0,
	0,0,1,0,
	0,0,0,1;
	std::vector<ScrewAxis> Blist;

    ScrewAxis S1, S2;
    S1<<0,0,1,0,-2,0;
    S2 << 0,0,1,0,0,0;
	Blist.push_back(S1);
	Blist.push_back(S2);
    std::vector<double> thetalist = {30,60};

	std::cout << "Forward Kinematics Bodyframe " << FKinBody(M, Blist, thetalist)<<"\n";
    std::cout << "Forward Kinematics Spaceframe " << FKinSpace(M, Blist, thetalist)<<"\n";


	//MatrixExp6(se333);
	//std::cout << "row  " << ((rotMat.block(2,0,1,3)-rotMat2).norm()==0)<< std::endl;
	//std::cout << type_info(rotMat.block(2,0,1,3));
	//std::cout << "row  " << (rotMat.block(2,0,1,3)).isApprox(rotMat2)<< std::endl;
	//MatrixXd mat1(rotMat.rows(), rotMat.cols());
	//MatrixXd mat1(3,3);
	//mat1 = MatrixXd::Identity(rotMat.rows(), rotMat.cols());
	 //std:: cout << mat1;

	return 0;
}




//Rigid body motions
//bool isSO3(SO3Mat mat){

//}
/*
bool isSO3(SO3Mat mat){
//Eigen::MatrixBase::Identity<Derived> x = new Eigen::MatrixBase::Identity(mat.rows(), mat.cols());
    std::cout << "determinant " << mat.determinant() <<std:: endl;
    //std::cout << "identity " << x <<std:: endl;
    MatrixXd id1(mat.rows(), mat.cols());
	//MatrixXd mat1(3,3);
	id1 = MatrixXd::Identity(mat.rows(), mat.cols());
        if (mat.determinant() !=1 || mat.transpose()*mat!=id1)//Eigen::MatrixBase::Identity(mat.rows(), mat.cols()))
            return false;
        else
            return true;
}
*/
/*
template<typename Derived>
void printFirstRow(const Eigen::MatrixBase<Derived>& x)
{
  cout << x.row(0) << endl;
}
*/
