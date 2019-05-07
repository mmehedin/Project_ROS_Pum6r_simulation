#ifndef JACOBIAN_HPP
#define JACOBIAN_HPP
#include <algorithm>    // std::reverse
#include <vector>
//#include "TypeDefs.hpp"
#include "RigidBodyMotion.hpp"
#include "ForwardKinematics.hpp"
#include "TypeDefs.hpp"


namespace IRlibrary
{
        std::vector<ScrewAxis> reverseNegativeScrewAxis (std::vector<ScrewAxis>);
        JacobianMat JacobianBody (std::vector<ScrewAxis>,std::vector<double>);
        JacobianMat JacobianSpace (std::vector<ScrewAxis>,std::vector<double>);
}/* IRlibrary */
#endif
