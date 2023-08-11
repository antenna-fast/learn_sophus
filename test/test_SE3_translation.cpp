#include <iostream>
#include <Eigen/Core>
#include <sophus/so3.h>
#include <sophus/se3.h>

using namespace std;


int main(int argc, char **argv) {
    cout << "you are running " << argv[0] << endl;

    // not init SE3 (default constructor)
    Sophus::SE3 SE3_test;
    cout << "SE3_test: \n" << SE3_test << endl;

    vector<Eigen::Vector3d> vPoints;
    
    // test pose translation
    Eigen::Vector3d t_cw_1(1, 0, 0);
    Eigen::Vector3d t_cw_2(4, 0, 0);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    Sophus::SE3 T_cw_1(R, t_cw_1); 
    Sophus::SE3 T_cw_2(R, t_cw_2); 

    // cout << "T_cw_1: " << T_cw_1.translation().x() << ", " << T_cw_1.translation().y() << ", " << T_cw_1.translation().z() << endl;
    cout << "T_cw_1: \n" << T_cw_1 << endl;
    cout << "T_cw_2: \n" << T_cw_2 << endl;

    Sophus::SE3 T_21 = T_cw_2 * T_cw_1.inverse();
    cout << "T_21: \n" << T_21 << endl;

    return 0;

}