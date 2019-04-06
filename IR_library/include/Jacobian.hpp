#ifndef JACOBIAN_HPP
#define JACOBIAN_HPP
namespace IRlibrary {
	JacMat JacobianBody(std::vector <ScrewAxis>, std::vector <double> );
	JacMat JacobianSpace(std::vector <ScrewAxis>, std::vector <double> );

}
#endif /* ifndef JACOBIAN_HPP */
