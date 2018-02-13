#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>

#include <Eigen/Geometry>

int main(int argc, char **argv) {
    Eigen::Matrix3d rotation_matrix=Eigen::Matrix3d::Identity();
    Eigen::AngleAxisd rotation_vector (M_PI/4, Eigen::Vector3d(0,0,1));
    cout.precision(4);
    cout<<"rotation matrix = \n"<<rotation_vector.matrix()<<endl<<endl;
    
    rotation_matrix = rotation_vector.toRotationMatrix();
    cout<<rotation_matrix<<endl;
    
    Eigen::Vector3d v (1,0,0);
    Eigen::Vector3d v_rotated = rotation_vector*v;
    cout<<"v after rotation_vector="<<v_rotated.transpose()<<endl<<endl;
    
    v_rotated=rotation_matrix*v;
    cout<<v_rotated.transpose()<<endl<<endl;
    
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0);
    cout<<"yaw pitch roll ="<<euler_angles.transpose()<<endl;
    
   /* Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
    cout<<T.matrix()<<endl;
    T.rotate(rotation_matrix);
    T.pretranslate(Eigen::Vector3d(1,3,4));
    cout<<"transform matrix =\n" <<T.matrix()<<endl<<endl;
    
    cout<<"v tranformed ="<<(T*v).transpose()<<endl<<endl;*/
    
    Eigen::Quaterniond q;
    q=Eigen::Quaterniond(rotation_vector);
    cout<<"the Quaterniond is:\n"<<q.coeffs()<<endl;
    
    v_rotated=q*v;
    cout<<v_rotated.transpose()<<endl;
    
        
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
    cout<<T.matrix()<<endl;
    T.rotate(q);
    T.pretranslate(Eigen::Vector3d(1,3,4));
    cout<<"transform matrix =\n" <<T.matrix()<<endl<<endl;
    
    cout<<"v tranformed ="<<(T*v).transpose()<<endl<<endl;
    
    return 0;
    
}
