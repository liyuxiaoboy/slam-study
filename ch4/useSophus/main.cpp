#include <iostream>
#include<cmath>
using namespace std;

#include<Eigen/Core>
#include<Eigen/Geometry>

#include"sophus/so3.h"
#include"sophus/se3.h"
 int main(int argc, char** argv)
 {
   Eigen::Matrix3d R=Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d(0,0,1)).toRotationMatrix();
   cout<<R<<endl;
   
   Sophus::SO3 SO3_R(R);
   Sophus::SO3 SO3_v(0,0,M_PI/2);
   
   Eigen::Quaterniond q(R);
   Sophus::SO3 SO3_q(q);
   cout<<"SO3 from matrix:"<<endl<<SO3_R.matrix()<<endl<<endl;
   cout<<"SO3 from vector:"<<SO3_v<<endl<<endl;
   cout<<"SO3 from Quaternion:"<<SO3_q<<endl<<endl;
   
   //下面有个小问题经过一次转换后再回来矩阵不一样了可能是精确度问题
   Eigen::Vector3d so3=SO3_R.log();
   cout<<"so3="<<so3.transpose()<<endl<<endl;
   Sophus::SO3 tex=Sophus::SO3::exp(so3);
   cout<<"text="<<tex.matrix()<<endl<<endl;
   
   Eigen::Matrix3d so3_hat=Sophus::SO3::hat(so3);
   cout<<"so3 hat=\n"<<so3_hat<<endl<<endl;
   cout<<"so3 hat vee=\n"<<Sophus::SO3::vee(so3_hat).transpose()<<endl<<endl;
   
   Eigen::Vector3d update_so3(1e-4,0,0);
   cout<<Sophus::SO3::exp(update_so3)<<endl<<endl;
   Sophus::SO3 SO3_update=Sophus::SO3::exp(update_so3)*SO3_R;//
   cout<<"SO3 update="<<endl<<SO3_update.matrix()<<endl<<endl<<SO3_update<<endl<<endl;
   
   cout<<"**********下面是变换矩阵******************"<<endl<<endl;
   
   Eigen::Vector3d t(1,0,0);
   Sophus::SE3 SE3_Rt(R,t);
   Sophus::SE3 SE3_qt(q,t);
   cout<<"SE3 from R t="<<endl<<endl<<SE3_Rt<<endl<<endl;
   cout<<"SE3 from q t="<<endl<<endl<<SE3_qt<<endl<<endl;
   
   typedef Eigen::Matrix<double,6,1> Vector6d;
   Vector6d se3= SE3_Rt.log();
   cout<<"se3 ="<<se3.transpose()<<endl;
   
   cout<<"se3 hat="<<endl<<Sophus::SE3::hat(se3)<<endl<<endl;
   cout<<"se3 hat vee"<<endl<<Sophus::SE3::vee(Sophus::SE3::hat(se3)).transpose()<<endl<<endl;
   
   Vector6d updated_se3;
   updated_se3.setZero();
   updated_se3(0,0)=1e-4d;
   Sophus::SE3 SE3_updated =Sophus::SE3::exp(updated_se3)*SE3_Rt;
   cout<<"SE3 updated =\n"<<endl<<SE3_updated.matrix()<<endl;
   
   
 }