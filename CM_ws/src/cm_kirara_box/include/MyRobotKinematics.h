 























 

#ifndef MYROBOTKINEMATICS_H
#define MYROBOTKINEMATICS_H

#include <Eigen/Dense>

Eigen::Matrix4d DH_mat(double theta, double d, double alpha, double a);
Eigen::Matrix4d tran(const Eigen::VectorXd& theta, const Eigen::VectorXd& d, const Eigen::VectorXd& alpha, const Eigen::VectorXd& a);
Eigen::Matrix4d tran_Arbitary(const Eigen::VectorXd& theta, const Eigen::VectorXd& d, const Eigen::VectorXd& alpha, const Eigen::VectorXd& a,const Eigen::Matrix4d& T_base); 
void tran_all(const Eigen::VectorXd& theta, const Eigen::VectorXd& d, const Eigen::VectorXd& alpha, const Eigen::VectorXd& a,
              Eigen::Matrix4d& T1, Eigen::Matrix4d& T2, Eigen::Matrix4d& T3,
              Eigen::Matrix4d& T4, Eigen::Matrix4d& T5, Eigen::Matrix4d& T6);
void tran_AA(const Eigen::VectorXd& theta, const Eigen::VectorXd& d, const Eigen::VectorXd& alpha, const Eigen::VectorXd& a,
             const Eigen::Matrix4d& T_base,Eigen::Matrix4d& T1, Eigen::Matrix4d& T2, Eigen::Matrix4d& T3,
              Eigen::Matrix4d& T4, Eigen::Matrix4d& T5, Eigen::Matrix4d& T6);
#endif  


 

 
#include <cmath>   

Eigen::Matrix4d DH_mat(double theta, double d, double alpha, double a) {
    Eigen::Matrix4d result;

    result << std::cos(theta), -std::sin(theta) * std::cos(alpha), std::sin(theta) * std::sin(alpha), a * std::cos(theta),
              std::sin(theta), std::cos(theta) * std::cos(alpha), -std::cos(theta) * std::sin(alpha), a * std::sin(theta),
              0, std::sin(alpha), std::cos(alpha), d,
              0, 0, 0, 1;

    return result;
}

Eigen::Matrix4d tran(const Eigen::VectorXd& theta, const Eigen::VectorXd& d, const Eigen::VectorXd& alpha, const Eigen::VectorXd& a) {
    Eigen::Matrix4d T1, T2, T3, T4, T5, T6;

    T1 = DH_mat(theta(0), d(0), alpha(0), a(0));
    T2 = T1 * DH_mat(theta(1), d(1), alpha(1), a(1));
    T3 = T2 * DH_mat(theta(2), d(2), alpha(2), a(2));
    T4 = T3 * DH_mat(theta(3), d(3), alpha(3), a(3));
    T5 = T4 * DH_mat(theta(4), d(4), alpha(4), a(4));
    T6 = T5 * DH_mat(theta(5), d(5), alpha(5), a(5));

    return T6;
}

Eigen::Matrix4d tran_Arbitary(const Eigen::VectorXd& theta, const Eigen::VectorXd& d, const Eigen::VectorXd& alpha, const Eigen::VectorXd& a,const Eigen::Matrix4d& T_base) 
{
    Eigen::Matrix4d T1, T2, T3, T4, T5, T6;
    T1 = T_base *DH_mat(theta(0), d(0), alpha(0), a(0));
    T2 = T1 * DH_mat(theta(1), d(1), alpha(1), a(1));
    T3 = T2 * DH_mat(theta(2), d(2), alpha(2), a(2));
    T4 = T3 * DH_mat(theta(3), d(3), alpha(3), a(3));
    T5 = T4 * DH_mat(theta(4), d(4), alpha(4), a(4));
    T6 = T5 * DH_mat(theta(5), d(5), alpha(5), a(5));

    return T6;
}

void tran_all(const Eigen::VectorXd& theta, const Eigen::VectorXd& d, const Eigen::VectorXd& alpha, const Eigen::VectorXd& a,
              Eigen::Matrix4d& T1, Eigen::Matrix4d& T2, Eigen::Matrix4d& T3,
              Eigen::Matrix4d& T4, Eigen::Matrix4d& T5, Eigen::Matrix4d& T6) {
    T1 = DH_mat(theta(0), d(0), alpha(0), a(0));
    T2 = T1 * DH_mat(theta(1), d(1), alpha(1), a(1));
    T3 = T2 * DH_mat(theta(2), d(2), alpha(2), a(2));
    T4 = T3 * DH_mat(theta(3), d(3), alpha(3), a(3));
    T5 = T4 * DH_mat(theta(4), d(4), alpha(4), a(4));
    T6 = T5 * DH_mat(theta(5), d(5), alpha(5), a(5));
}
 
void tran_AA(const Eigen::VectorXd& theta, const Eigen::VectorXd& d, const Eigen::VectorXd& alpha, const Eigen::VectorXd& a,
              const Eigen::Matrix4d& T_base,Eigen::Matrix4d& T1, Eigen::Matrix4d& T2, Eigen::Matrix4d& T3,
              Eigen::Matrix4d& T4, Eigen::Matrix4d& T5, Eigen::Matrix4d& T6) {
    T1 =T_base* DH_mat(theta(0), d(0), alpha(0), a(0));
    T2 = T1 * DH_mat(theta(1), d(1), alpha(1), a(1));
    T3 = T2 * DH_mat(theta(2), d(2), alpha(2), a(2));
    T4 = T3 * DH_mat(theta(3), d(3), alpha(3), a(3));
    T5 = T4 * DH_mat(theta(4), d(4), alpha(4), a(4));
    T6 = T5 * DH_mat(theta(5), d(5), alpha(5), a(5));
}