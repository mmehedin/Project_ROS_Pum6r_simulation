#include <Planar3R.hpp>
#include <iostream>
int main(int argc, char ** argv) {
	IRlibrary::Planar3R obj(2, 5, 2);
	//  IRlibrary::Vec2 q = {3.14, 0.3};
    //IRlibrary::Vec3 q = {M_PI/3., M_PI/6.,M_PI/4.};
	//obj.setConfig(q);
	//auto xy1 = obj.getX();
	//IRlibrary::Vec3 initial_thetaList =q;
	//initial_thetaList[0]-=M_PI/180.;
	//initial_thetaList[1]-=M_PI/180.;
	//initial_thetaList[2]-=M_PI/180.;
	//obj.inverseKinematics_numerical(initial_thetaList, xy1);
	IRlibrary::Vec2 xy = {3.14, 2.3};
	obj.inverseKinematics(xy);


	auto q1 = obj.getConfig();
	//std::cout << q[0] << " " << q[1] << " " << q[2] <<std::endl;
	std::cout << q1[0] << " " << q1[1] <<" " << q1[2] <<std::endl;

	//IRlibrary::Vec2 xy = obj.getXY();

			//std::cout <<"l1 " << obj.l[0]<<"\n";
        //std::cout <<"l2 " << obj.l[1]<<"\n";
        //std::cout << "q1 " << q[0]<<"\n";
	//std::cout << xy[0] << " " << xy[1] << std::endl;
	//std::cout << obj.getAngle()<<"\n";
/*
	IRlibrary::Planar2R obj(2, 3);
	IRlibrary::Vec2 q = {0, 1.57};
	obj.setConfig(q);
	IRlibrary::Vec2 xy = obj.getXY();
			//std::cout <<"l1 " << obj.l[0]<<"\n";
        //std::cout <<"l2 " << obj.l[1]<<"\n";
        //std::cout << "q1 " << q[0]<<"\n";
	std::cout << xy[0] << " " << xy[1] << std::endl;
	std::cout << obj.getAngle()<<"\n";

*/


	return 0;
}


