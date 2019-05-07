#include <Planar2R.hpp>
#include <iostream>
int main(int argc, char ** argv) {
	IRlibrary::Planar2R obj(2, 3);
	//  IRlibrary::Vec2 q = {3.14, 0.3};
	IRlibrary::Vec2 q = {0,0};//{0, M_PI/2.};
	obj.setConfig(q);
	auto xy1 = obj.getXY();
	obj.setXY(xy1);
	auto q1 = obj.getConfig();
	std::cout << q[0] << " " << q[1] << std::endl;
	std::cout << q1[0] << " " << q1[1] << std::endl;

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

