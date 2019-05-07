#include <RigidBodyMotion.hpp>
#include <MathUtils.hpp>

namespace IRlibrary {
    bool nearZeroVal(double val, double eps){
	//if ((val <eps & val > -1 * eps) || (val< -1* eps & val > eps)){
    if(abs(val)<=eps){
    //std::cout <<"true";
		return true;
	}else
		return false;
}

double fixInaccuraciesVal(double val, double eps){
    if (nearZeroVal(1.0-val, eps))
        return 1.0;
    return val;
}


	/** Returns true if a 3X3 matrix satisfies properties of SO(3) **/
	bool isSO3(SO3Mat mat){
//Eigen::MatrixBase::Identity<Derived> x = new Eigen::MatrixBase::Identity(mat.rows(), mat.cols());
    //std::cout << "determinant " << mat.determinant() <<std:: endl;
    //std::cout << "identity " << x <<std:: endl;
    MatrixXd id1(mat.rows(), mat.cols());
	//MatrixXd mat1(3,3);
	id1 = MatrixXd::Identity(mat.rows(), mat.cols());
        if (!nearZero(mat.determinant()-1.0) || !(id1.isApprox ( mat.transpose()*mat)))//Eigen::MatrixBase::Identity(mat.rows(), mat.cols()))
            return false;
        else
            return true;
}

	/** Returns true if a 3x3 matrix is skew-symmetric **/
	bool isso3(so3Mat mat){
	if (mat.rows()!= 3 || mat.cols()!=3){
        return false;
	}
	if (mat!=(-1) * mat.transpose())
	if (mat!=(-1) * mat.transpose())
        return false;
    return true;
	}

	/** Returns true if a 4x4 matrix is SE(3) **/
	bool isSE3(SE3Mat mat){

        //float row_number = mat.rows()-1;
        //float row_number = 2;
        //std::cout << "row number " << row_number << std::endl;
        //std::cout << mat.block(0,0,3,3);
        MatrixXd rotMat = mat.block(0,0,3,3);
        if (!isSO3(rotMat))
            return false;
        //std::cout << std::endl();
        if (mat.rows()!=4 || mat.cols()!=4)
            return false;
        VectorXd x = mat.row(mat.rows()-1);
        Vector4d y;
        y << 0,0,0,1;
        if ((x - y).norm() != 0)
            return false;
        //std::cout << "row4 " << mat[row_number] <<std:: endl;
        //if (mat[:, 4])
		return true;
	}


	/** Returns true if a 4x4 matrix is se(3) **/
	bool isse3(se3Mat mat){
        //MatrixXd x;
        //x = (MatrixXd) mat;
        //std::cout << x.exp();
        //SE3Mat x;
        //x = mat.exp();
        if (se3Mat::Identity().isApprox(mat))
            return false;
        if (mat.rows()!=4 ||mat.cols()!=4)
            return false;
        //if (!(mat.block<1,4>(3,0)).isApprox( Vec4{0, 0, 0, 0}))
        auto bot = mat.bottomRows<1>();
        //std::cout << "bot: " << bot<<"\n";
        if (!bot.isApprox((se3Mat::Zero()).bottomRows<1>()))//Vec4::Zero()
            return false;
        //if (!isSE3(mat.array().exp()))
        //    return false;
		return true;
	}


	/** Checks if the vector is unit **/
	bool isUnit(Vec2 vec){
	double val = fixInaccuraciesVal(sqrt(pow(vec(0), 2) + pow(vec(1), 2)));
    if ( val == 1)
            return true;
        return false;

		return true;
	}
	bool isUnit(Vec3 vec){
	double val ;
	val= fixInaccuraciesVal(sqrt(pow(vec(0), 2) + pow(vec(1), 2) + pow(vec(2), 2)));
        if ( val == 1.0){
            //std::cout<<"unit ? :" <<val;
            return true;
        }
        return false;
	}
	bool isUnit(Vec4 vec){
	double val = fixInaccuraciesVal(sqrt(pow(vec(0), 2) + pow(vec(1), 2) + pow(vec(2), 2)+ pow(vec(3), 2)));
        if (val==1)
            return true;
		return false;
	}

	/** Computes the inverse of rotation matrix **/
	SO3Mat RotInv(SO3Mat mat){
		return mat.transpose();
	}


	/** Computes the 3X3 skew-symmetric matrix corresponding to 3x1 omega vector **/
	so3Mat VecToso3(Vec3 omega){
        if (omega.isApprox(Vec3::Zero()))
            return so3Mat::Identity();
		so3Mat mat;
		mat << 0, -omega(2), omega(1),
				omega(2), 0, -omega(0),
				-omega(1), omega(0), 0;

        for (int i=0; i<3;i++){
            for (int j = 0; j<3;j++){
                if (nearZero(mat(i,j))){
                    mat(i,j)=0;
                }
            }
		}
		return mat;
	}

	/** Computes the 3-vector corresponding to 3x3 skew-symmetric matrix **/
	Vec3 so3ToVec(so3Mat mat){
		Vec3 omega;
		omega <<  mat(2,1), mat(0,2), mat(1,0);
		return omega;
	}

	/** Extracts the rotation axis, omega_hat, and the rotation amount, theta, from the 3-vector omega_hat theta of exponential coordinates for rotation, expc3 **/
	AxisAngle AxisAng3 (Vec3 expc3){
		AxisAngle axisAngle;
		axisAngle.theta = expc3.norm();//vector lenght= theta
		axisAngle.omega = expc3.normalized();//unit vector
		if (axisAngle.theta==0)
            axisAngle.omega= Vec3{1,0,0};
		return axisAngle;
	}

	/** Computes the rotation matrix R \in SO(3) corresponding to the matrix exponential of so3Mat **/
	SO3Mat MatrixExp3 (so3Mat in_mat){
		SO3Mat mat;
		if(in_mat.isApprox(so3Mat::Zero()))
            return SO3Mat::Identity();
		Vec3 omega_hat_theta;
		omega_hat_theta = so3ToVec(in_mat);
		double s1, c1, theta;
		AxisAngle om_h_theta;
		om_h_theta = AxisAng3(omega_hat_theta);
		theta = om_h_theta.theta;
		so3Mat unit_omega_skew = VecToso3(om_h_theta.omega);

		s1 = sin(theta);
		c1 = cos(theta);

		mat = SO3Mat::Identity() + s1*unit_omega_skew+ (1-c1)*unit_omega_skew*unit_omega_skew;

		//mat << 1, 0, 0,
		//			 0, 1, 0,
		//			 0, 0, 1;
        //std::cout <<  "rot mat R in SO3: " << mat ;

		//return in_mat.array().exp();
		for (int i=0; i<3;i++){
            for (int j = 0; j<3;j++){
                if (nearZero(mat(i,j))){
                    mat(i,j)=0;
                }
            }
		}

		return mat;
	}


	/** Computes the matrix logarithm so3mat \in so(3) of the rotation matrix R \in SO(3) **/
	so3Mat MatrixLog3(SO3Mat in_mat){
		so3Mat mat;
        double theta_2;
		if (in_mat.isApprox(SO3Mat::Identity())){
            theta_2 = 0;
            mat = so3Mat::Identity();
		}
		else{
            theta_2 = acos((0.5*(in_mat(0,0)+in_mat(1,1)+in_mat(2,2)-1)));
            mat =(1/(2*sin(theta_2)))*(in_mat - in_mat.transpose());


            for (int i=0; i<3;i++){
            for (int j = 0; j<3;j++){
                if (nearZero(mat(i,j))){
                    mat(i,j)=0;
                }
            }
		}
}
		//std::cout << "matrix_log theta= "<< theta_2<<"\n";


		//std::cout << "matrix_log R= "<< mat<<"\n";
		//mat << 1, 0, 0,
		//		0, 1, 0,
		//		0, 0, 1;
		//return in_mat.array().log();
		return mat*theta_2;

	}


	/** Compute the 4x4 transformation matrix **/
	SE3Mat RpToTrans(SO3Mat R, Vec3 p){
		SE3Mat T;
		T << R(0, 0), R(0,1), R(0,2), p(0),
			R(1,0), R(1,1), R(1,2), p(1),
			R(2,0), R(2,1), R(2,2), p(2),
			0, 0, 0, 1;
		return T;
	}

	/** Extracts the rotation matrix and position vector from homogeneous transformation matrix **/
	void TransToRp(SE3Mat mat, SO3Mat &R, Vec3 &p){
		R = mat.block<3, 3>(0, 0);
		p << mat(0,3), mat(1,3), mat(2,3);
		std::cout << "transtoRP R from T" << "\n"<< R << "\n";
		std::cout << "transtoRP p from T" << "\n" <<p << "\n";
	}

	/** Inverse of transformation matrix **/
	SE3Mat TransInv(SE3Mat mat){
	SE3Mat Ti;
	SO3Mat R, Rt;
	Vec3 p, mRtp;
	TransToRp(mat, R, p);
	Rt = RotInv(R);
	mRtp = (-1)*Rt*p;
	Ti <<  Rt(0, 0), Rt(0,1), Rt(0,2), mRtp(0),
			Rt(1,0), Rt(1,1), Rt(1,2), mRtp(1),
			Rt(2,0), Rt(2,1), Rt(2,2), mRtp(2),
			0,      0,         0,    1;

		return Ti;
	}

	/** Return the se(3) matrix corresponding to a 6-vector twist V **/
	se3Mat VecTose3(Twist V){
		se3Mat mat;
		so3Mat omegaskew;
		Vec3 omega ;
		omega << V(0), V(1), V(2);
		omegaskew = VecToso3(omega);
		mat <<  omegaskew(0, 0), omegaskew(0,1), omegaskew(0,2), V(3),
			omegaskew(1,0), omegaskew(1,1), omegaskew(1,2), V(4),
			omegaskew(2,0), omegaskew(2,1), omegaskew(2,2), V(5),
			0, 0, 0, 0;
		return mat;
	}

	/** Returns Twist from se(3) matrix **/
	Twist se3ToVec(se3Mat mat){
		Twist V;
		so3Mat omegaskew;
		omegaskew = mat.block<3,3>(0,0);
		Vec3 omega;
		omega =so3ToVec(omegaskew);

		V << omega(0), omega(1), omega(2), mat(0,3), mat(1,3), mat(2,3);

		return V;
	}

	/** Compute 6x6 adjoint matrix from T **/
	AdjMat Adjoint(SE3Mat mat) {
		AdjMat adj_mat;
        so3Mat pskew;
        SO3Mat R, pR;
        Vec3 p;

        TransToRp(mat, R, p);
        if (p.isApprox(Vec3::Zero()))
            pskew = so3Mat::Zero();
        else
            pskew = VecToso3(p);
        std::cout<< "adjoint pskew is " << "\n"<< pskew << "\n";
        pR = pskew*R;
        std::cout<< "adjoint pskewR is " << "\n"<< pR << "\n";
        adj_mat <<   R(0, 0), R(0,1), R(0,2),      0 ,    0,    0,
                     R(1,0), R(1,1), R(1,2),     0 ,    0,    0,
                    R(2,0), R(2,1), R(2,2),      0 ,    0,    0,
                    pR(0, 0), pR(0,1), pR(0,2),      R(0, 0), R(0,1), R(0,2),
                     pR(1,0), pR(1,1), pR(1,2),      R(1,0), R(1,1), R(1,2),
                    pR(2,0), pR(2,1), pR(2,2),     R(2,0), R(2,1), R(2,2);

		return adj_mat;//mat.adjoint();
	}


	/** Returns a normalized screw axis representation **/
	ScrewAxis ScrewToAxis(Vec3 q, Vec3 s, double h)  {
	//std::cout <<s(0)+s(1)+s(2)<<std::endl;
	Vec3 temp;
	temp =(-1)*s.cross(q)+h*s;
        if (isUnit(s)){
        //std::cout <<"yes"<<s(0)+s(1)+s(2)<<std::endl;

        }else if ((isUnit(temp)) && (s.norm()==0.0)){
            std::cout << "condition 2 satisfied " <<std::endl;
        }else{
             try
                {
                    throw 20;
                    }
                catch (int e)
                {
                        std::cout << "An exception occurred s not a unit vector and s not zero with v not unit. Exception Nr. " << e << '\n';
                }
                throw std::invalid_argument( "received conditions not satisifed for screw values" );
        }
		ScrewAxis S;
		Vec3 sxq, hs, sxqhs;
        sxq = (-1)*s.cross(q);
        hs = h*s;
        sxqhs = sxq + hs;
		S << s(0), s(1), s(2), sxqhs(0), sxqhs(1), sxqhs(2);
		return S;
	}


	/** Extracts the normalized screw axis S and the distance traveled along the screw theta from the 6-vector of exponential coordinates Stheta **/
	void AxisAng(Twist STheta, ScrewAxis &S, double &theta){
        Vec3 omega, omeganorm, v, vnorm;
        omega << STheta(0), STheta(1),STheta(2);
        v << STheta(3), STheta(4),STheta(5);;
        if (omega.norm()==0.0){
            theta = v.norm();
            vnorm = v.normalized();
            S << 0,0,0, vnorm(0), vnorm(1), vnorm(2);
        } else if (omega.norm()!=0){
            theta = omega.norm();
            omeganorm = omega.normalized();
            vnorm = v/theta;
            S << omeganorm(0), omeganorm(1), omeganorm(2), vnorm(0), vnorm(1), vnorm(2);
        }
        else{}
		//S << 0, 0, 0, 0, 0, 1;
		//theta = 0.0;
	}

	/** Computes the homogeneous transformation matrix T \in SE(3) corresponding to matrix exponential of se3mat \in se(3) **/
	SE3Mat MatrixExp6(se3Mat in_mat){

		SE3Mat mat;
				//Matrix3d I, expOmTh;
        SO3Mat  expOmTh;
        so3Mat I, omegaSkew;
        //se3Mat  omegaSkew;
        //Matrix3d omegaTheta;
        so3Mat omegaTheta;
        Vec3 omega, vTheta, v, gtheta;
        //omegaTheta = in_mat.block<3,3>(0,0);//<< in_mat(2,1), in_mat(0,2), in_mat(1,0)

        if (in_mat.isApprox(se3Mat::Zero()))
            return SE3Mat::Identity();


        omegaTheta = in_mat.topLeftCorner<3,3>();
        //std::cout <<"in_mat " << in_mat;
        //std::cout <<"omegaTheta " << omegaTheta;

		//expOmTh = omegaTheta.array().exp();
		expOmTh = MatrixExp3(omegaTheta);
		//std::cout << "expOmTh" << expOmTh;
		omega<< in_mat(2,1), in_mat(0,2), in_mat(1,0);
		//omegaSkew = omegaTheta.normalized();//wrong

		//omega<< omegaSkew(2,1), omegaSkew(0,2), omegaSkew(1,0);
		double theta;
		vTheta << in_mat(0,3), in_mat(1,3), in_mat(2,3);
		if (omega.isApprox(Vec3::Zero())){
                theta = vTheta.norm();
                gtheta = vTheta;
                for (int i = 0; i<3;i++){
                if (nearZero(v(i)))
                    v(i)=0;
            }
		}else{

            theta = omega.norm();
            omegaSkew = omegaTheta*(1/theta);
            //std::cout <<"theta " << theta<<"\n";
            //std::cout <<"vTheta " << vTheta<<"\n";
            v = vTheta*(1/theta);
            for (int i = 0; i<3;i++){
                if (nearZero(v(i)))
                    v(i)=0;
            }
            //std::cout <<"v " << v<<"\n";
            //std::cout <<"omegaskew " << omegaSkew<<"\n";
            //I =  Matrix<double, 3, 3>::Identity();
            I = so3Mat::Identity();

            gtheta = (I*theta +(1-cos(theta))*omegaSkew+(theta- sin(theta))*omegaSkew* omegaSkew)*v;//works with no v
        }

        //gtheta <<theta,theta,theta;
        mat << expOmTh(0,0),expOmTh(0,1), expOmTh(0,2),gtheta(0),
                expOmTh(1,0), expOmTh(1,1), expOmTh(1,2),gtheta(1),
                expOmTh(2,0), expOmTh(2,1), expOmTh(2,2),gtheta(2),
                0,0,0,1;
        //std::cout << "transf from MatrixExp6: " << mat << "\n";
        //std::cout << "theta from MatrixExp6: " << theta << "\n";
        //std::cout << "v from MatrixExp6: " << v << "\n";
        //std::cout << "gtheta from MatrixExp6: " << gtheta << "\n";
    for (int i=0; i<4;i++){
            for (int j = 0; j<4;j++){
                if (nearZero(mat(i,j))){
                    mat(i,j)=0;
                }
            }
		}


		return mat;
		//return in_mat.array().exp();
	}

	/** Computes the matrix logarithm se3mat \in se(3) of the homogeneous transformation matrix T \in SE(3) **/
	se3Mat MatrixLog6(SE3Mat T){
		se3Mat mat1;
		SO3Mat R, omega, ginvOfTheta;
		so3Mat omegatheta;
        Vec3 p, normp, temp, v, vtheta;
        double theta;
        TransToRp(T, R, p);

        //std::cout << R << "\n";
        //std::cout << p << "\n";

        SO3Mat I = SO3Mat::Identity();
        //R=I;
        double eps = 1e-10;
        if (R.isIdentity(eps)){
            std::cout << "R is identity" << std::endl;

            normp = temp.normalized();//v
            theta = p.norm();
            omega = SO3Mat::Zero();
            //std::cout << "theta is " << theta << "\n";
            //std::cout << "omega is " << omega << "\n";
            return se3Mat::Zero();

        }else if (R(0,0)+R(1,1)+R(2,2)==-1) {
            theta = -M_PI;
            Vec3 x, o;
            x << R(1,3), R(2,3), 1+R(3,3);
            o = (1/(sqrt(2*(1+R(3,3)))))*x;
            omega = VecToso3(o);
        }else{
            //omegatheta= R.array().log();
            theta = acos((1/2)*(R(0,0)+R(1,1)+R(2,2)-1));
            //std::cout << "Matrixlog6 theta is " << theta<< "\n";
            omega =(1/(2*sin(theta)))*(R-R.transpose());
            //std::cout << "Matrixlog6 omega is " << omega<< "\n";
        }
        omegatheta = omega*theta;
        ginvOfTheta = (1/theta)* I -(0.5)*omega + (1/theta - (0.5)*(1/(tan(theta/2))))*omega*omega;
       // std::cout << "Matrixlog6 ginvOfTheta is " << ginvOfTheta<< "\n";
        v = ginvOfTheta*p;
        vtheta = v*theta;

            mat1 << omegatheta(0,0), omegatheta(0,1), omegatheta(0,2), vtheta(0),
                  omegatheta(1,0), omegatheta(1,1), omegatheta(1,2), vtheta(1),
                  omegatheta(2,0), omegatheta(2,1), omegatheta(2,2), vtheta(2),
                  0,0,0,0;
        for (int i=0; i<4;i++){
            for (int j = 0; j<4;j++){
                if (nearZero(mat1(i,j))){
                    mat1(i,j)=0;
                }
            }
		}

        //std::cout << "omega theta is " << omegatheta<< "\n";
        //std::cout << "vtheta is " << vtheta<< "\n";
        //std::cout << "Matrixlog6 omegatheta is " << mat1<< "\n";

		return mat1;
	}

} /* IRlibrary */


