#include <iostream>
#include <Eigen/Core>
#include <sophus/so3.h>
#include <sophus/se3.h>

using namespace std;
//using namespace Eigen;
//using namespace Sophus;


int main(int argc, char **argv) {
    // test SE3
    Eigen::AngleAxisd A2(M_PI/2, Eigen::Vector3d(0,0,1));
    Eigen::Matrix3d R2=A2.matrix();
    Eigen::Quaterniond Q2(A2);
    Sophus::SO3 SO3_2(R2);

    //一、初始化李代数的几种方式
    Eigen::Vector3d t(1,0,0);

    //1. 使用旋转矩阵和平移向量来初始化SE3
    Sophus::SE3 SE_Rt(R2,t);
    cout<<"SE3 SE_Rt from  Rotation_Matrix and Transform=\n"<<SE_Rt<<endl<<endl;//注意尽管SE(3)是对应一个4*4的矩阵,但是输出SE(3)时是以一个六维向量输出的,其中前前三位为对应的so3,后3维度为实际的平移量t，而不是se3中的平移分量
    //2. 使用四元数和平移向量来初始化SE3
    Sophus::SE3 SE_Qt(Q2,t);
    cout<<"SE3 SE_Qt from  Quaterion and Transform=\n"<<SE_Qt<<endl<<endl;
    //3. 使用SO3和平移向量来初始化SE3
    Sophus::SE3 SE_St(SO3_2,t);
    cout<<"SE3 SE_St from  SO3 and Transform=\n"<<SE_St<<endl<<endl;

    //二、SE(3)与se(3)的相互转换，以及se3对应的hat和vee操作
    Sophus::Vector6d se3_Rt=SE_Rt.log();//se(3)在Sophus中用Vector6d表示,使用对数映射获得李群对应的李代数
    cout<<"se(3) se3_Rt from SE3_Rt\n"<<se3_Rt<<endl<<endl;//se3输出的是一个六维度向量,其中前3维是平移分量,后3维度是旋转分量

    Sophus::SE3 SE3_Rt2=Sophus::SE3::exp(se3_Rt);//使用指数映射将李代数转化为李群
    cout<<"SE(3) SO3_Rt2 from se3_Rt"<<SE3_Rt2<<endl<<endl;

    Sophus::Matrix4d M_se3_Rt=Sophus::SE3::hat(se3_Rt);
    cout<<"se(3) hat=\n"<<M_se3_Rt<<endl<<endl;

    Sophus::Vector6d V_M_se3=Sophus::SE3::vee(M_se3_Rt);
    cout<<"se(3) vee=\n"<<V_M_se3<<endl<<endl;


    //三、增量扰动模型
    Sophus::Vector6d update_se3=Sophus::Vector6d::Zero();
    update_se3(0)=1e-4d;

    cout<<"update_se3\n"<<update_se3.transpose()<<endl<<endl;

    Eigen::Matrix4d update_matrix2=Sophus::SE3::exp(update_se3).matrix();//将李群转换为旋转矩阵
    cout<<"update matrix=\n"<<update_matrix2<<endl<<endl;

    Sophus::SE3 SE3_updated=Sophus::SE3::exp(update_se3)*SE3_Rt2;
    cout<<"SE3 updated=\n"<<SE3_updated<<endl<<endl;

    Eigen::Matrix4d SE3_updated_matrix=SE3_updated.matrix();//将李群转换为旋转矩阵
    cout<<"SE3 updated Matrix=\n"<<SE3_updated_matrix<<endl<<endl;

    return 0;

}