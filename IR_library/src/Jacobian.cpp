//Jacobian functions
#include <Jacobian.hpp>
#include <vector>
#include "RigidBodyMotion.hpp"
#include "ForwardKinematics.hpp"
#include "TypeDefs.hpp"
#include <algorithm>

namespace IRlibrary{

      std::vector<ScrewAxis> reverseNegativeScrewAxis (std::vector<ScrewAxis> Blist){

            std::reverse(Blist.begin(), Blist.end());

            for(int j=0;j<Blist.size();j++){
                //std::cout << current_saxis_list[j] <<"\n\n";
                Blist[j]=Blist[j]*(-1);
            }

            return Blist;
      }

    /** Computes the body Jacobian Jb(theta) \in R 6xn given a list of joint screws B_i
    expressed in the body frame and a list of joint angles**/

     JacobianMat JacobianBody (std::vector<ScrewAxis> Blist,std::vector<double> thetaList){
        size_t n= Blist.size();
        JacobianMat Jb(6,n);// = JacobianMat::Identity();
        Jb.col(n-1) = Blist[n-1];

        SE3Mat T = SE3Mat::Identity();
        for (int i = n-2; i>=0;i--){
            ScrewAxis current_B = Blist[i];
            double theta_current = thetaList[i];
            SE3Mat M = SE3Mat::Identity();

            //select current subset of screw axis and theta
            std::vector<ScrewAxis>::const_iterator first = Blist.begin()+ i+1;
            std::vector<ScrewAxis>::const_iterator last = Blist.end();
            std::vector<ScrewAxis> current_saxis_list(first, last);
            //std::reverse(current_saxis_list.begin(), current_saxis_list.end());
            std::cout << "JacobianSpace current saxis size "<< current_saxis_list.size() <<"\n";
            for(int j=0;j<current_saxis_list.size();j++){
                std::cout << current_saxis_list[j] <<"\n\n";
            }

            current_saxis_list = reverseNegativeScrewAxis(current_saxis_list);
            std::vector<double>::const_iterator first_ta = thetaList.begin() + i+1;
            std::vector<double>::const_iterator last_ta = thetaList.end();
            std::vector<double> current_theta_list(first_ta, last_ta);
            std::reverse(current_theta_list.begin(), current_theta_list.end());
            //calculate current T without M as a product of exponential matrices
            T = FKinSpace(M,current_saxis_list, current_theta_list);///-1*
            std::cout << "current T is " << "\n" << T << "\n";
            AdjMat ad_T = Adjoint(T);
            std::cout << "adjoint T is " << "\n" << ad_T << "\n";
            Jb.col(i) = ad_T*current_B;

        }
            std::cout << "JacobianB Matrix is "<<"\n"<<Jb <<"\n\n";
        return Jb;
     }



  /** Computes the space Jacobian Js(theta) \in R 6xn given a list of joint screws S_i
    expressed in the space frame and a list of joint angles**/

    JacobianMat JacobianSpace (std::vector<ScrewAxis> Slist,std::vector<double> thetaList){
        //JacobianMat Js = JacobianMat::Identity();
        size_t n= Slist.size();
        JacobianMat Js(6,n);
        Js.col(0) = Slist[0];
        SE3Mat T = SE3Mat::Identity();
        for (int i = 1;i<n;i++){
            ScrewAxis current_S = Slist[i];
            double theta_current = thetaList[i];
            SE3Mat M = SE3Mat::Identity();

            //select current subset of screw axis and theta
            std::vector<ScrewAxis>::const_iterator first = Slist.begin();
            std::vector<ScrewAxis>::const_iterator last = Slist.begin()+i;
            std::vector<ScrewAxis> current_saxis_list(first, last);
            std::cout << "JacobianSpace current saxis size "<< current_saxis_list.size() <<"\n";
            for(int j=0;j<current_saxis_list.size();j++){
                std::cout << current_saxis_list[j] <<"\n\n";
            }


            std::vector<double>::const_iterator first_ta = thetaList.begin();
            std::vector<double>::const_iterator last_ta = thetaList.begin() + i;
            std::vector<double> current_theta_list(first_ta, last_ta);

            //calculate current T without M as a product of exponential matrices
            T = FKinSpace(M,current_saxis_list, current_theta_list);
            std::cout << "current T is " << "\n" << T << "\n";
            AdjMat ad_T = Adjoint(T);
            std::cout << "adjoint T is " << "\n" << ad_T << "\n";
            Js.col(i) = ad_T*current_S;


        }
        //For loop i=1 to <n
        //Multiply screw axis (i-1)with joint angle (i-1)
        //se3 matrix
        //Exponent for the 6 dimensional vector
        //Adjoint
        //Multiply with screw axis
        std::cout << "JacobianS Matrix is "<<"\n"<<Js <<"\n\n";

        return Js;
    }
}
