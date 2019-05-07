#ifndef IRLIBRARY_SPATIAL3R_H
#define IRLIBRARY_SPATIAL3R_H


#include <array>
#include <cmath>
#include <TypeDefs.hpp>
#include <Planar2R.hpp>
#include <ForwardKinematics.hpp>
#include "InverseKinematics.hpp"
#include <vector>

namespace IRlibrary {
	class Spatial3R {
		protected:
			Vec3 l;
			Vec3 q; // Configuration space (angle1, angle2, angle3)
			Vec3 x; // Task space Cartesian cordinates and end effector angle (x, y, z)
			Vec4 quat; // Quaternion for orientation
			AxisAngle axisAngle; // Axis angle representation for orientation (w_x, w_y, w_z, theta)
			Vec3 base; // Cordinates of the base in 3D
            SE3Mat M;
			std::vector <ScrewAxis> Slist;
			std::vector <double> wsRad;
			Planar2R planar2r;
		public:
			// Constructors
			Spatial3R() : l({0, 0, 0}), q({0, 0, 0}), base({0, 0, 0}) { setM(); setSlist();wsCalc(); zeroForwardKinematics(); };
			Spatial3R(double l1_i, double l2_i, double l3_i) : l({l1_i, l2_i, l3_i}), q({0, 0, 0}), base({0, 0, 0}) {setM(); setSlist();wsCalc(); zeroForwardKinematics(); };

			void setConfig(const Vec3 &q_i) { q = q_i; zeroForwardKinematics(); }
			void setX(const Vec3 &x_i) { x = x_i; }
			void setOrigin(const Vec3 &o_i) { base = o_i; }
			void setLinks(double l1_i, double l2_i, double l3_i) {l[0] = l1_i; l[1] = l2_i; l[2] = l3_i; setM(); setSlist();wsCalc(); zeroForwardKinematics();}

			Vec3 getConfig() { return q; }
			Vec3 getX() { return x; }
			Vec4 getQuaternion () { quat = {1, 0, 0, 0}; return quat; }
			AxisAngle getAxisAngle () { return axisAngle; }


			inline void setM() {
                M << 1,0,0,l[1]+l[2],
                     0,0,1,l[0],
                     0,-1,0,0,
                     0,0,0,1;
			}//{M = SE3Mat::Identity(); M(1,3) = l[1]+l[2];}
			inline void setSlist(){
                ScrewAxis S;
                S << 0,1,0,0,0,0;
                Slist.push_back(S);
                S << 0,0,-1, l[0],0,0;
                Slist.push_back(S);
                S << 0,0,-1, l[0],-l[1],0;
                Slist.push_back(S);
               S << 0,0,-1,l[0], -l[1]-l[2],0;
            Slist.push_back(S);

			}
            virtual void zeroForwardKinematics();

            bool inverseKinematics(Vec3 const &);

			bool inverseKinematics_numerical(Eigen::VectorXd const &, Vec3 const &);

			inline void wsCalc() {
                wsRad.push_back(l[0]+l[1]+l[2]);
                wsRad.push_back(std::fabs(-l[0] + l[1] + l[2]));
                wsRad.push_back(std::fabs(l[0]-l[1]+l[2]));
                wsRad.push_back(std::fabs(l[0]+l[1]-l[2]));
                std::sort(wsRad.begin(), wsRad.end());
			}

			bool checkInDexWs(Vec3 x_in);

	};
} /* IRlibrary */

#endif /* ifndef IRLIBRARY_PLANAR3R_H */
