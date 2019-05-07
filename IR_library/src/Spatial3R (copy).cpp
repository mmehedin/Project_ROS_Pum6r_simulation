#include "Puma6r.hpp"

namespace IRlibrary
{
    void Puma6r::zeroForwardKinematics() {
        SE3Mat T = FKinSpace(M, Slist, q);
        Vec3 p;
        SO3Mat R;
        TransToRp(T, R, p);
        //setM();
        //setSlist();
        //x=base;
		x[0] = base[0] + cos(q[0])*(l[1]*cos(q[1])+l[2]*cos(q[1]+q[2]));
		x[1] = base[1] + sin(q[0])*( l[1]*cos(q[1])+l[2]*cos(q[1]+q[2]));
		x[2] = base[2] + l[0] + l[1]*sin[q[1]]+l[2]*sin(q[1]+q[2]);
		//x[3] = q[1]+q[2]+M_PI;//q[0] + q[1] + q[2];

        so3Mat omega_skew = MatrixLog3(R);
        axisAngle.omega = so3ToVec(omega_skew);
        if (isUnit(naxisAngle.omega)){
            axisAngle.theta = 1;
        } else {
            axisAngle.theta = axisAngle.omega.norm();
            axisAngle.omega = axisAngle.omega.normalized();
        }
		//x=base;
		//axisAngle.omega = {0,0,0};
		//axisAngle.theta = q[1]+q[2];//q[1]+q[2]+M_PI;
		//std::out << "" << "\n\n";
		std::out << "x= " << x << "\n\n";
		std::out << "" << << "\n\n";
		std::out << "" << << "\n\n";
		std::out << "" << << "\n\n";
		std::out << "" << << "\n\n";
	}

    bool Puma6r::inverseKinematics_numerical(Eigen::VectorXd const &initial_thetaList, Vec3 const & x_in){
        SE3Mat T;
        double ctht = cos(x_in[2]);
        double stht = sin(x_in[2]);
        T << ctht, -stht, 0,x_in[0],
        stht, ctht, 0, x_in[1],
        0,0,1,0,
        0,0,0,1;
        Eigen::VectorXd thetaList;
        if(IKinSpace(Slist, M,T, initial_thetaList, thetaList)){
            std::cout << "Planar3R::inverseKinematics_numerical [Error] failed to converge or at singularity\n";

            return 1;
        }
        x=x_in;
        q<<thetaList;
        return 0;
	}

	bool Puma6r::inverseKinematics(Vec2 const & x_in){
        SE3Mat T;
        double x = x_in[0]; double y = x_in[1];

        auto xyp = x_in - base;
        double l1 = l[0];double l2 = l[1], l3=l[2];
        double pSqr = xyp.squaredNorm();
        if (pSqr > (l1+l2+l3)*(l1+l2+l3)|| pSqr < (l1-l2-l3)*(l1-l2-l3)){
            std::cout << "Planar3R::inverseKinematics [Error] Point outside workspace\n";
            return 1;
        }
        if (pSqr > 0.99 * (l1+l2+l3)*(l1+l2+l3)|| pSqr < 1.01 * (l1-l2-l3)*(l1-l2-l3)){
            std::cout << "Planar3R::inverseKinematics [Warning] Point close to singularity\n";
            return 1;
        }
        //Eigen::VectorXd Slist thetaList;
        q[2] = atan2(y,x);//since the system is redundant set theta3=phi
        double x3 = x - l[2] * cos(q[2]); //x for joint 3
        double y3 = y - l[2] * sin(q[2]);//y for joint 3
        Vec2 x3_in;
        x3_in << x3 ,y3;
        //IRlibrary::Planar2R obj = Planar2R(l[0],l[1]);
//        Vec2 xy_i = {};
        planar2r.setLinks(l[0],l[1]);
        planar2r.setXY(x3_in);
        auto q1 = planar2r.getConfig();
        //std::cout << q[0] << " " << q[1] << std::endl;
        //std::cout << q1[0] << " " << q1[1] << std::endl;
        q[0] = q1[0];
        q[1] = q1[1];


        return 0;
	}

	bool Puma6r::checkInDexWs(Vec3 x_in){
        double pDist = (x_in.head(2)-base).norm();

        if(pDist>wsRad[2] or pDist < wsRad[1]){
            return false;
        }else
        return true;
	}

} /* IRlibrary */


