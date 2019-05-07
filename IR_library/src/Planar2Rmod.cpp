#include "Planar2Rmod.hpp"

namespace IRlibrary
{
	void Planar2Rmod::zeroForwardKinematics() {
		//Complete the lines below
		xy[0] = base[0] +(l[1]/3)*cos(q[1])+(5*l[1]/12)*cos(3/5+q[1])+l[0]*cos(q[0]);
		xy[1] = base[1] +(l[1]/3)*sin(q[1])+(5*l[1]/12)*sin(3/5+q[1])+l[0]*sin(q[0]);
	}

} /* IRlibrary */


