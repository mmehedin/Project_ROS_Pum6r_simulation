#include <ForwardKinematics.hpp>
#include <MathUtils.hpp>

namespace IRlibrary
{
	/** Computes the end-effector frame given the zero position of the end-effector M, list of joint screws Blist expressed in the end-effector frame, and the list of joint values thetalist **/
	SE3Mat FKinBody (SE3Mat M, std::vector <ScrewAxis> Blist, std::vector <double> thetaList){
		SE3Mat T_endEffector, temp=SE3Mat::Identity();//temp=M
		//std::cout << "FkinBody M " << M << "\n";
		//for (auto v : Blist)
            //std::cout << "FkinBody Blist " << v << "\n";
		//std::cout << "FkinBody thetalist " <<thetaList << "\n";

        //std::reverse(Blist.begin(),Blist.end());
        //std::reverse(thetaList.begin(),thetaList.end());
        ScrewAxis S_current;
        double theta;
		while(!Blist.empty()){
            S_current = Blist.back();
            //std::cout << "FKinBody S_current " << S_current <<"\n";
            Blist.pop_back();
            theta = thetaList.back();
            //std::cout << "FKinBody theta " << theta <<"\n";
            thetaList.pop_back();
            se3Mat sthetaskew = VecTose3 (S_current*theta);
            //std::cout << "FKinBody sthetaskew " << sthetaskew <<"\n";
            SE3Mat T;
            T = MatrixExp6(sthetaskew);//pass the se3
            //std::cout << "FKinBody T current " << T <<"\n";
            temp=T*temp;
		}
		T_endEffector = M*temp;//temp*M;
		//T_endEffector = temp;
		for (int i=0; i<4;i++){
            for (int j = 0; j<4;j++){
                if (nearZero(T_endEffector(i,j))){
                    T_endEffector(i,j)=0;
                }
            }
		}
		std::cout << "FKinBody T_endeffector " << T_endEffector <<"\n";
		return T_endEffector;


	}

	/** Computes the end-effector frame given the zero position of the end-effector M, list of joint screws Slist expressed in the space frame, and the list of joint values thetalist **/
	SE3Mat FKinSpace (SE3Mat M, std::vector <ScrewAxis> Slist, std::vector <double> thetaList){ //){
		SE3Mat T_endEffector, temp=SE3Mat::Identity();
		//std::cout << "FKinSpace M " << M << "\n";
        //std::reverse(Slist.begin(),Slist.end());
        //std::reverse(thetaList.begin(),thetaList.end());
        ScrewAxis S_current;
        double theta;
		while(!Slist.empty()){
            S_current = Slist.back();
            //std::cout << "FKinSpace S_current " << S_current <<"\n";
            Slist.pop_back();
            theta = thetaList.back();
            //std::cout << "FKinSpace theta " << theta <<"\n";
            thetaList.pop_back();
            se3Mat sthetaskew = VecTose3 (S_current*theta);
            //std::cout << "FKinSpace sthetaskew " << sthetaskew <<"\n";
            SE3Mat T;
            T = MatrixExp6(sthetaskew);
            //std::cout << "FKinSpace T current " << T <<"\n";
            temp=T*temp;
		}
		T_endEffector = temp*M;

		for (int i=0; i<4;i++){
            for (int j = 0; j<4;j++){
                if (nearZero(T_endEffector(i,j))){
                    T_endEffector(i,j)=0;
                }
            }
		}
        std::cout << "FKinSpace T_endeffector " << T_endEffector <<"\n";
		return T_endEffector;
	}
} /* IRlibrary */
