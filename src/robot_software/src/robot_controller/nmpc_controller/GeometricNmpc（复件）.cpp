// #define DEBUG_NMPC 0  // 设置为1开启调试输出,0关闭

// #if DEBUG_NMPC
// #define DEBUG_PRINT(x) std::cout << x << std::endl
// #define DEBUG_MATRIX(name, mat) std::cout << #name ": \n" << mat << std::endl
// #define DEBUG_DIMS(name, mat) std::cout << #name ": " << mat.rows() << "x" << mat.cols() << std::endl
// #else
// #define DEBUG_PRINT(x)
// #define DEBUG_MATRIX(name, mat)
// #define DEBUG_DIMS(name, mat)
// #endif

// #include "robot_software/robot_controller/nmpc_controller/GeometricNmpc.h"

// #include <string.h>

// #include <Eigen/Core>
// #include <chrono>
// #include <cstring>
// #include <iomanip>
// #include <proxsuite/helpers/optional.hpp>  // for c++14
// #include <proxsuite/proxqp/dense/dense.hpp>
// #include <unsupported/Eigen/MatrixFunctions>

// #include "robot_software/robot_utils/UtilFunc.h"

// using namespace proxsuite::proxqp;
// using std::nullopt;  // c++17 simply use std::nullopt
// using namespace std;
// using namespace Eigen;
// namespace Galileo
// {
// GeometricNmpc::GeometricNmpc()
//     : dataCenter_(DataCenter::getInstance())
// {
//     // for (int i = 0; i < Horizon; i++)
//     // {
//     //     vector_Q.block<Kdim, 1>(Kdim * i, 0) = vec_Q;
//     // }
//     // weight_Q = vector_Q.asDiagonal();

//     mass = dataCenter_.read<robot_constants>()->mass;
//     auto inertiaVector = dataCenter_.read<robot_constants>()->inertiaVector;
//     Jb.setZero();
//     Jb(0, 0) = inertiaVector[0];
//     Jb(1, 1) = inertiaVector[1];
//     Jb(2, 2) = inertiaVector[2];
//     Jb(3, 3) = mass;
//     Jb(4, 4) = mass;
//     Jb(5, 5) = mass;
//     fd.setZero();
//     for (int i = 0; i < Horizon; i++)
//     {
//         fd(6 * i + 5) = mass * 9.8;
//     }
//     f_qp.setZero(3 * stanceLegNum * Horizon, 1);
// }

// Matrix3d GeometricNmpc::skew(const Vector3d& vec)
// {
//     Matrix3d m;
//     m << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
//     return m;
// }

// Vector3d GeometricNmpc::invskew(const Matrix3d& Mat)
// {
//     Vector3d v;
//     v(0) = Mat(2, 1);
//     v(1) = Mat(0, 2);
//     v(2) = Mat(1, 0);
//     return v;
// }

// Matrix<double, 6, 6> GeometricNmpc::Adjoint(const Matrix<double, 4, 4>& M)
// {
//     Matrix<double, 6, 6> Ad;
//     Matrix<double, 3, 3> R = M.block(0, 0, 3, 3);
//     Vector3d p = M.block(0, 3, 3, 1);

//     Ad.setZero();
//     Ad.block(0, 0, 3, 3) = R;
//     Ad.block(3, 0, 3, 3) = skew(p) * R;
//     Ad.block(3, 3, 3, 3) = R;
//     return Ad;
// }

// Matrix<double, 6, 6> GeometricNmpc::adjoint(const Matrix<double, 6, 1>& Twist)
// {
//     Matrix<double, 6, 6> ad;
//     Vector3d w = Twist.block(0, 0, 3, 1);
//     Vector3d v = Twist.block(3, 0, 3, 1);

//     ad.setZero();
//     ad.block(0, 0, 3, 3) = skew(w);
//     ad.block(3, 0, 3, 3) = skew(v);
//     ad.block(3, 3, 3, 3) = skew(w);
//     return ad;
// }

// void GeometricNmpc::getState(const Matrix<double, 12, 1>& state, const Matrix<double, 12, 1>& state_d)
// {
//     const AngleAxisd roll(state(0), Vector3d::UnitX());
//     const AngleAxisd pitch(state(1), Vector3d::UnitY());
//     const AngleAxisd yaw(state(2), Vector3d::UnitZ());

//     RotationMatrix = yaw * pitch * roll;
//     geoM.block(0, 0, 3, 3) = RotationMatrix;
//     geoM.block(0, 3, 3, 1) = state.block(3, 0, 3, 1);
//     geoM(3, 3) = 1;
//     geoTwist << state.block(6, 0, 6, 1);

//     const AngleAxisd roll_d(state_d(0), Vector3d::UnitX());
//     const AngleAxisd pitch_d(state_d(1), Vector3d::UnitY());
//     const AngleAxisd yaw_d(state_d(2), Vector3d::UnitZ());

//     RotationMatrix_d = yaw_d * pitch_d * roll_d;
//     geoM_d.block(0, 0, 3, 3) = RotationMatrix_d;
//     geoM_d.block(0, 3, 3, 1) = state_d.block(3, 0, 3, 1);
//     geoM_d(3, 3) = 1;
//     geoTwist_d << state_d.block(6, 0, 6, 1);
// }

// Matrix<double, 4, 4> GeometricNmpc::getErrorM(const Matrix<double, 4, 4>& M, const Matrix<double, 4, 4>& M_d)
// {
//     Matrix<double, 4, 4> eM;
//     eM = M_d.inverse() * M;
//     eM.block(0, 3, 3, 1) = M.block(0, 3, 3, 1) - M_d.block(0, 3, 3, 1);
//     //    Matrix<double, 4, 4> invMd;
//     //    invMd.setIdentity();
//     //    invMd.block(0, 0, 3, 3) = M_d.block(0, 0, 3, 3).transpose();
//     //    invMd.block(0, 3, 3, 1) = -M_d.block(0, 0, 3, 3).transpose() * M_d.block(0, 3, 3, 1);
//     //    cout << (M.block(0, 3, 3, 1) - M_d.block(0, 3, 3, 1)) << "invMd" << endl;
//     return eM;
// }

// Matrix<double, 6, 1> GeometricNmpc::getErrorTwist(const Matrix<double, 6, 1>& Twist,
//                                                   const Matrix<double, 6, 1>& Twist_d,
//                                                   const Matrix<double, 4, 4>& eM)
// {
//     Matrix<double, 6, 1> eTwist;
//     eTwist = Twist - Adjoint(eM).inverse() * Twist_d;
//     return eTwist;
// }

// Matrix<double, 13, 1> GeometricNmpc::getDeltaX_so3(const Matrix<double, 12, 1>& state,
//                                                    const Matrix<double, 12, 1>& state_d)
// {
//     Matrix<double, 13, 1> dX;
//     Matrix<double, 3, 1> dR;
//     Matrix<double, 3, 1> dP;
//     Matrix<double, 3, 1> dW;
//     Matrix<double, 3, 1> dV;
//     dR << invskew(RotationMatrix_d.inverse() * RotationMatrix);
//     dP << state.block(3, 0, 3, 1) - state_d.block(3, 0, 3, 1);
//     dW << state.block(6, 0, 3, 1) - (RotationMatrix.transpose() * RotationMatrix_d) * state_d.block(6, 0, 3, 1);
//     dV << state.block(9, 0, 3, 1) - state_d.block(9, 0, 3, 1);

//     dX << dR, dP, dW, dV, 0;
//     return dX;
// }

// Matrix<double, 13, 13> GeometricNmpc::getA_so3(const Matrix<double, 12, 1>& state,
//                                                const Matrix<double, 12, 1>& state_d,
//                                                const Matrix<double, 6, 6>& Jb)
// {
//     Matrix<double, 13, 13> A;
//     Vector3d w = state.block(6, 0, 3, 1);
//     Vector3d v = state.block(9, 0, 3, 1);
//     Matrix3d Ib = Jb.block(0, 0, 3, 3);
//     A.setZero();
//     A.block(0, 3, 3, 3).setIdentity();
//     A.block(6, 6, 3, 3) = -skew(w);
//     A.block(6, 9, 3, 3).setIdentity();
//     A.block(9, 9, 3, 3) = Ib.inverse() * (skew(Ib * w) - skew(w) * Ib);

//     //    cout << Jb << "Jb" << endl;
//     //    cout << G << "G" << endl;
//     //    cout << adjoint(Twist).transpose() * Jb << "adjoint(Twist).transpose() * Jb" << endl;

//     return A;
// }

// Matrix<double, 13, Eigen::Dynamic> GeometricNmpc::getB_so3(const Matrix<double, 6, 6>& Jb,
//                                                            const Matrix<double, 6, Eigen::Dynamic>& T)
// {
//     Matrix<double, Kdim, Eigen::Dynamic> B;
//     int dim = T.cols();
//     B.resize(Kdim, dim);
//     B.setZero();
//     B.block(6, 0, 6, dim) = Jb.inverse() * T;
//     return B;
// }

// Matrix<double, 13, 1> GeometricNmpc::getDeltaX(const Matrix<double, 4, 4>& eM, const Matrix<double, 6, 1>& eTwist)
// {
//     Matrix<double, 13, 1> dX;
//     dX << invskew(eM.log().block(0, 0, 3, 3)), eM.log().block(0, 3, 3, 1), eTwist, 0;
//     return dX;
// }

// Matrix<double, 13, 13> GeometricNmpc::getA(const Matrix<double, 6, 1>& Twist,
//                                            const Matrix<double, 6, 6>& Jb,
//                                            const Matrix<double, 4, 4>& M)
// {
//     Matrix<double, 13, 13> A;
//     Matrix<double, 6, 6> G;
//     Vector3d w = Twist.block(0, 0, 3, 1);
//     Vector3d v = Twist.block(3, 0, 3, 1);
//     A.setZero();
//     A.block(0, 0, 6, 6) = -adjoint(Twist);
//     A.block(0, 6, 6, 6).setIdentity();
//     A.block(6, 0, 6, 6).setZero();

//     G.block(0, 0, 3, 3) = skew(Jb.block(0, 0, 3, 3) * w);
//     G.block(0, 3, 3, 3) = mass * skew(v);
//     G.block(3, 0, 3, 3) = mass * skew(v);
//     G.block(3, 3, 3, 3).setZero();

//     A.block(6, 6, 6, 6) = Jb.inverse() * (G + adjoint(Twist).transpose() * Jb);
//     A.block(9, 10, 3, 3) = M.block(0, 0, 3, 3).transpose();
//     //    cout << Jb << "Jb" << endl;
//     //    cout << G << "G" << endl;
//     //    cout << adjoint(Twist).transpose() * Jb << "adjoint(Twist).transpose() * Jb" << endl;

//     return A;
// }

// Matrix<double, 6, Eigen::Dynamic> GeometricNmpc::getT(const Matrix<double, 4, 3> footpos,
//                                                       const Matrix<int, 4, 1>& Leg_Contact_State)
// {
//     Matrix<double, 6, Eigen::Dynamic> T;
//     T.resize(6, 3 * Leg_Contact_State.sum());
//     int k = -1;
//     for (int i = 0; i < 4; i++)
//     {
//         if (Leg_Contact_State(i, 0) == 1)
//         {
//             k++;
//             T.block<3, 3>(3, k * 3).setIdentity();
//             T.block<3, 3>(0, k * 3) = skew(footpos.row(i));
//         }
//     }
//     return T;
// }

// Matrix<double, 13, Eigen::Dynamic> GeometricNmpc::getB(const Matrix<double, 6, 6>& Jb,
//                                                        const Matrix<double, 6, Eigen::Dynamic>& T)
// {
//     Matrix<double, Kdim, Eigen::Dynamic> B;
//     int dim = T.cols();
//     B.resize(Kdim, dim);
//     B.setZero();
//     B.block(6, 0, 6, dim) = Jb.inverse() * T;
//     return B;
// }

// void GeometricNmpc::discreteAB(const Matrix<double, Kdim, Kdim>& A, const Matrix<double, Kdim, Eigen::Dynamic>& B)
// {
//     A_dt = Matrix<double, Kdim, Kdim>::Identity() + A * dt_MPC;
//     B_dt = B * dt_MPC;  // MatrixXd::Identity(12, B.cols()) +
// }

// void GeometricNmpc::setABqp(const Matrix<double, Kdim, 1>& dX,
//                             const Matrix<double, Kdim, Kdim>& A,
//                             const Matrix<double, Kdim, Eigen::Dynamic>& B,
//                             const Matrix<int, 4, 1>& Leg_Contact_State,
//                             const DiagonalMatrix<double, Kdim * Horizon>& weight_Q,
//                             const DiagonalMatrix<double, Eigen::Dynamic>& weight_R)
// {
//     int num = Leg_Contact_State.sum();
//     A_qp.block<Kdim, Kdim>(0, 0) = A;
//     B_qp.block(0, 0, Kdim, 3 * num) = B;

//     for (int i = 1; i < Horizon; i++)
//     {
//         A_qp.block<Kdim, Kdim>(i * Kdim, 0) = A_qp.block<Kdim, Kdim>((i - 1) * Kdim, 0) * A;
//     }

//     for (int i = 0; i < Horizon; i++)
//     {
//         for (int j = 0; j < Horizon; j++)
//         {
//             if (i == j)
//             {
//                 B_qp.block(i * Kdim, j * 3 * num, Kdim, 3 * num) = B;
//             }
//             else if (i > j)
//             {
//                 B_qp.block(i * Kdim, j * 3 * num, Kdim, 3 * num) = A_qp.block<Kdim, Kdim>((i - j - 1) * Kdim, 0) * B;
//                 //                B_qp.block(i * Kdim, j * 3 * num, 13, 3 * num) = A_qp.block<13, 13>((i - j - 1) *
//                 //                Kdim, 0) * B_mat_exp;
//             }
//         }
//     }

//     H_qp.triangularView<Upper>() = B_qp.transpose() * weight_Q * B_qp;  // 正定矩阵只计算上三角 更快
//     H_qp.triangularView<Lower>() = H_qp.triangularView<Upper>().transpose();
//     H_qp.diagonal() += weight_R.diagonal();
//     H_qp = H_qp.eval();

//     g_qp = B_qp.transpose() * weight_Q * (A_qp * dX) + weight_R * (f_qp - fdd);
//     // cout << fdd << endl
//     //      << endl;
//     //    cout << weight_Q.diagonal() << endl;
//     //    cout << A_qp << endl << endl;
// }

// void GeometricNmpc::setABqp(const Matrix<double, Kdim, 1>& dX,
//                             const Matrix<double, Kdim, Kdim>& A,
//                             const Matrix<double, Kdim, Eigen::Dynamic>& B,
//                             const Matrix<int, 4, 1>& Leg_Contact_State,
//                             const DiagonalMatrix<double, Kdim * Horizon>& weight_Q,
//                             const DiagonalMatrix<double, 6 * Horizon>& weight_P,
//                             const Matrix<double, 6, Eigen::Dynamic>& T,
//                             const Matrix<double, Eigen::Dynamic, 1>& f)
// {
//     int num = Leg_Contact_State.sum();
//     A_qp.block<Kdim, Kdim>(0, 0) = A;
//     B_qp.block(0, 0, Kdim, 3 * num) = B;

//     for (int i = 1; i < Horizon; i++)
//     {
//         A_qp.block<Kdim, Kdim>(i * Kdim, 0) = A_qp.block<Kdim, Kdim>((i - 1) * Kdim, 0) * A;
//     }

//     for (int i = 0; i < Horizon; i++)
//     {
//         for (int j = 0; j < Horizon; j++)
//         {
//             if (i == j)
//             {
//                 B_qp.block(i * Kdim, j * 3 * num, Kdim, 3 * num) = B;
//                 T_qp.block(i * 6, j * 3 * num, 6, 3 * num) = T;
//             }
//             else if (i > j)
//             {
//                 B_qp.block(i * Kdim, j * 3 * num, Kdim, 3 * num) = A_qp.block<Kdim, Kdim>((i - j - 1) * Kdim, 0) * B;
//                 //                B_qp.block(i * Kdim, j * 3 * num, 13, 3 * num) = A_qp.block<13, 13>((i - j - 1) *
//                 //                Kdim, 0) * B_mat_exp;
//             }
//         }
//     }

//     //    cout << T_qp << endl << "T_qp" << endl;

//     H_qp.triangularView<Upper>() = B_qp.transpose() * weight_Q * B_qp;  // 正定矩阵只计算上三角 更快
//     H_qp.triangularView<Lower>() = H_qp.triangularView<Upper>().transpose();
//     H_qp += T_qp.transpose() * weight_P * T_qp;
//     H_qp = H_qp.eval();
//     //    cout << f << endl << "f" << endl;

//     g_qp = B_qp.transpose() * weight_Q * (A_qp * dX) + T_qp.transpose() * weight_P * (T_qp * f - fd);

//     //    cout << H_qp.block(0, 0, 12, 12) << endl << endl;
//     //    cout << g_qp << endl << endl;

//     //    cout << B_qp.transpose() * weight_Q * (A_qp * dX) << "  B_qp.transpose() * weight_Q * (A_qp * dX)" << endl
//     //    << endl; cout << T_qp.transpose() * weight_P * (T_qp * f - fd) << "   T_qp.transpose() * weight_P * (T_qp
//     //    * f - fd)" << endl; cout << (T_qp * f - fd) << "   (T_qp * f - fd)" << endl; cout << (T_qp * f) << " (T_qp
//     //    * f)"
//     //    << endl; cout << (fd) << "   (fd)" << endl;
// }

// void GeometricNmpc::setConstrains(const Matrix<int, 4, 1>& Leg_Contact_State,
//                                   const Matrix<double, Eigen::Dynamic, 1>& f,
//                                   double fmax,
//                                   double fmin,
//                                   double u)
// {
//     Matrix<double, 6, 3> C1;
//     Matrix<double, 6, 1> d1;

//     Matrix<double, 6, 1> dd1;
//     double fz = 0;
//     int num = Leg_Contact_State.sum();

//     C1 << 0, 0, 1, 0, 0, -1, 1, 0, -u, 0, 1, -u, -1, 0, -u, 0, -1, -u;
//     d1 << fmax, fmin, 0, 0, 0, 0;

//     for (int i = 0; i < num; i++) fz += f(2 + 3 * i);

//     for (int i = 0; i < num; i++)
//     {
//         dd1 << -f(2 + 3 * i), f(2 + 3 * i), u * f(2 + 3 * i) - f(3 * i), u * f(2 + 3 * i) - f(1 + 3 * i),
//             u * f(2 + 3 * i) + f(3 * i), u * f(2 + 3 * i) + f(1 + 3 * i);

//         for (int j = 0; j < Horizon; j++)
//         {
//             C_qp.block<6, 3>(i * 6 + j * (num * 6 + 1), i * 3 + j * num * 3) = C1;
//             C_qp.block<1, 3>(num * 6 + j * (num * 6 + 1), i * 3 + j * num * 3) << 0, 0, -1;

//             d_qp.block<6, 1>(i * 6 + j * (num * 6 + 1), 0) = d1 + dd1;
//             d_qp(num * 6 + j * (num * 6 + 1)) = -mass * 9.8 + fz;
//         }
//     }
// }

// void GeometricNmpc::initMatrix(const Matrix<int, 4, 1>& Leg_Contact_State)
// {
//     stanceLegNum = Leg_Contact_State.sum();

//     B_qp.setZero(Horizon * Kdim, Horizon * 3 * stanceLegNum);
//     T_qp.setZero(Horizon * 6, Horizon * 3 * stanceLegNum);
//     H_qp.setZero(Horizon * 3 * stanceLegNum, Horizon * 3 * stanceLegNum);
//     g_qp.setZero(Horizon * 3 * stanceLegNum, 1);

//     C_qp.setZero((6 * stanceLegNum + 1) * Horizon, 3 * stanceLegNum * Horizon);
//     d_qp.setZero((6 * stanceLegNum + 1) * Horizon, 1);

//     fdd.setZero(Horizon * 3 * stanceLegNum, 1);
//     if (stanceLegNumlast != stanceLegNum)
//     {
//         f_qp.setZero(Horizon * 3 * stanceLegNum, 1);
//         fGround.setZero(3 * stanceLegNum, 1);
//     }

//     vector_R.setZero(3 * stanceLegNum * Horizon);
//     weight_R.setZero(3 * stanceLegNum * Horizon);
//     vector_Q.setZero();
//     vector_P.setZero();
//     for (int i = 0; i < Horizon; i++)
//     {
//         vector_Q.block<Kdim, 1>(Kdim * i, 0) = vec_Q;
//         vector_P.block<6, 1>(6 * i, 0) = vec_P;
//     }
//     for (int i = 0; i < stanceLegNum * Horizon; i++)
//     {
//         vector_R.block<3, 1>(3 * i, 0) = vec_R;
//         fdd(i * 3 + 2) = mass * 9.8 / stanceLegNum;
//     }

//     weight_Q = vector_Q.asDiagonal();
//     weight_P = vector_P.asDiagonal();
//     weight_R = vector_R.asDiagonal();

//     stanceLegNumlast = stanceLegNum;

//     std::cout << "weight_Q: " << weight_Q.diagonal() << std::endl;
//     std::cout << "weight_P: " << weight_P.diagonal() << std::endl;
//     std::cout << "weight_R: " << weight_R.diagonal() << std::endl;

//     std::cout << "  vec_Q: " << vec_Q << std::endl;
//     std::cout << "  vec_P: " << vec_P << std::endl;
//     std::cout << "  vec_R: " << vec_R << std::endl;
// }

// void GeometricNmpc::initMPC_proxsuite(const Matrix<double, Eigen::Dynamic, Eigen::Dynamic, RowMajor>& H_qp,
//                                       const Matrix<double, Eigen::Dynamic, 1>& g_qp,
//                                       const Matrix<double, Eigen::Dynamic, Eigen::Dynamic, RowMajor>& C_qp,
//                                       const Matrix<double, Eigen::Dynamic, Eigen::Dynamic, RowMajor>& d_qp,
//                                       const Matrix<int, 4, 1>& Leg_Contact_State)
// {
//     // std::cout
//     //     << "Solve a simple example with inequality constraints using dense ProxQP"
//     //     << std::endl;

//     // define the problem
//     double eps_abs = 1e-9;
//     dense::isize dim = 3 * stanceLegNum * Horizon, n_eq = 0, n_in = (6 * stanceLegNum + 1) * Horizon;

//     // create qp object and pass some settings
//     dense::QP<double> qp(dim, n_eq, n_in);

//     qp.settings.eps_abs = eps_abs;
//     qp.settings.initial_guess = InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
//     qp.settings.verbose = false;

//     // initialize qp with matrices describing the problem
//     // note: it is also possible to use update here
//     qp.init(H_qp, g_qp, nullopt, nullopt, C_qp, nullopt, d_qp);

//     qp.solve();

//     for (int i = 0; i < 3 * stanceLegNum; i++)
//     {
//         fGround(i) = fGround(i) + qp.results.x(i);  // youwenti
//         //        cout << xOpt[i + 12] << endl;
//     }
//     int k = 0;
//     for (int i = 0; i < 4; i++)
//     {
//         if (Leg_Contact_State(i) == 1)
//         {
//             fLeg.col(i) << -fGround(k), -fGround(k + 1), -fGround(k + 2);

//             k += 3;
//         }
//         else
//         {
//             fLeg.col(i).setZero();
//         }
//     }
//     //
//     f_qp.setZero(3 * stanceLegNum * Horizon, 1);
//     for (int i = 0; i < Horizon; i++)
//     {
//         f_qp.block(3 * stanceLegNum * i, 0, 3 * stanceLegNum, 1) = fGround;
//     }
//     // std::cout << "wrench: " << T * fGround << std::endl;
//     robot_controller::GeoController geo_controller;
//     geo_controller.f = fLeg;
//     dataCenter_.write(geo_controller);
// }

// // void GeometricNmpc::resizef();
// void GeometricNmpc::run()
// {
//     try
//     {
//         DEBUG_PRINT("Step 1: Starting run function...");

//         // 读取状态数据
//         DEBUG_PRINT("Step 2: Reading state data...");
//         Matrix<double, 12, 1> state;
//         state << dataCenter_.read<robot_state::BaseState>()->eulerAngles,
//             dataCenter_.read<robot_state::BaseState>()->position,
//             dataCenter_.read<robot_state::BaseState>()->angularVelocity,
//             dataCenter_.read<robot_state::BaseState>()->linearVelocity;

//         DEBUG_PRINT("Step 3: Reading target state data...");
//         Matrix<double, 12, 1> state_d;
//         state_d << dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetEulerAngles,
//             dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetPosition,
//             dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetAngularVelocity,
//             dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetLinearVelocity;

//         DEBUG_PRINT("Step 4: Reading leg contact state...");
//         Matrix<int, 4, 1> Leg_Contact_State = dataCenter_.read<robot_FSM::legState>()->legPhase;
//         DEBUG_MATRIX("Leg_Contact_State", Leg_Contact_State);

//         DEBUG_PRINT("Step 5: Reading foot positions...");
//         Matrix<double, 4, 3> footpos = dataCenter_.read<robot_state::LegState>()->legPosBaseInWorld.transpose();

//         auto utc_start = std::chrono::high_resolution_clock::now();

//         DEBUG_PRINT("Step 6: Initializing matrices...");
//         initMatrix(Leg_Contact_State);
//         auto utc_end1 = std::chrono::high_resolution_clock::now();

//         DEBUG_PRINT("Step 7: Getting state...");
//         getState(state, state_d);

//         DEBUG_PRINT("Step 8: Computing error matrix...");
//         ErrorM = getErrorM(geoM, geoM_d);

//         DEBUG_PRINT("Step 9: Computing error twist...");
//         ErrorTwist = getErrorTwist(geoTwist, geoTwist_d, ErrorM);

//         DEBUG_PRINT("Step 10: Computing DeltaX...");
//         DeltaX = getDeltaX(ErrorM, ErrorTwist);

//         DEBUG_PRINT("Step 11: Computing matrix A...");
//         auto A = getA(geoTwist, Jb, geoM);

//         DEBUG_PRINT("Step 12: Computing matrix T...");
//         T = getT(footpos, Leg_Contact_State);

//         DEBUG_PRINT("Step 13: Computing matrix B...");
//         auto B = getB(Jb, T);

//         auto utc_end2 = std::chrono::high_resolution_clock::now();

//         DEBUG_PRINT("Step 14: Discretizing AB...");
//         discreteAB(A, B);

//         DEBUG_PRINT("Step 15: Setting up ABqp...");
//         DEBUG_DIMS("DeltaX", DeltaX);
//         DEBUG_DIMS("A_dt", A_dt);
//         DEBUG_DIMS("B_dt", B_dt);
//         DEBUG_DIMS("weight_Q", weight_Q);
//         DEBUG_DIMS("weight_R", weight_R);

//         setABqp(DeltaX, A_dt, B_dt, Leg_Contact_State, weight_Q, weight_R);

//         DEBUG_PRINT("Step 16: Setting constraints...");
//         DEBUG_DIMS("fGround", fGround);
//         setConstrains(Leg_Contact_State, fGround, fmax, fmin, u);

//         auto utc_end3 = std::chrono::high_resolution_clock::now();

//         DEBUG_PRINT("Step 17: Initializing MPC...");
//         DEBUG_DIMS("H_qp", H_qp);
//         DEBUG_DIMS("g_qp", g_qp);
//         DEBUG_DIMS("C_qp", C_qp);
//         DEBUG_DIMS("d_qp", d_qp);

//         initMPC_proxsuite(H_qp, g_qp, C_qp, d_qp, Leg_Contact_State);

//         auto utc_end4 = std::chrono::high_resolution_clock::now();

// #if DEBUG_NMPC
//         DEBUG_PRINT("Step 18: Run completed successfully");
//         std::cout << "Total execution time: " << std::chrono::duration<double, std::milli>(utc_end4 - utc_start).count()
//                   << "ms" << std::endl;
// #endif
//     }
//     catch (const std::exception& e)
//     {
//         std::cerr << "Error in GeometricNmpc::run(): " << e.what() << std::endl;
//     }
//     catch (...)
//     {
//         std::cerr << "Unknown error in GeometricNmpc::run()" << std::endl;
//     }
// }

// void GeometricNmpc::declare_and_get_parameters(rclcpp::Node* node)
// {
//     node->declare_parameter<double>("geometric_nmpc.dt_MPC", 0);
//     node->declare_parameter<int>("geometric_nmpc.n_steps", 0);
//     node->declare_parameter<double>("geometric_nmpc.u", 0);
//     node->declare_parameter<double>("geometric_nmpc.fmax", 0);
//     node->declare_parameter<double>("geometric_nmpc.fmin", 0);
//     node->declare_parameter<std::vector<double>>("geometric_nmpc.vec_Q", std::vector<double>(13, 0));
//     node->declare_parameter<std::vector<double>>("geometric_nmpc.vec_R", std::vector<double>(3, 0));
//     node->declare_parameter<std::vector<double>>("geometric_nmpc.vec_P", std::vector<double>(6, 0));

//     dt_MPC = node->get_parameter("geometric_nmpc.dt_MPC").as_double();
//     n_steps = node->get_parameter("geometric_nmpc.n_steps").as_int();
//     u = node->get_parameter("geometric_nmpc.u").as_double();
//     fmax = node->get_parameter("geometric_nmpc.fmax").as_double();
//     fmin = node->get_parameter("geometric_nmpc.fmin").as_double();

//     vec_Q =
//         stdVectorToEigen<Eigen::Matrix<double, 13, 1>>(node->get_parameter("geometric_nmpc.vec_Q").as_double_array());
//     vec_R =
//         stdVectorToEigen<Eigen::Matrix<double, 3, 1>>(node->get_parameter("geometric_nmpc.vec_R").as_double_array());
//     vec_P =
//         stdVectorToEigen<Eigen::Matrix<double, 6, 1>>(node->get_parameter("geometric_nmpc.vec_P").as_double_array());

//     std::cout << "vec_Q: " << vec_Q.transpose() << std::endl;
//     std::cout << "vec_R: " << vec_R.transpose() << std::endl;
//     std::cout << "vec_P: " << vec_P.transpose() << std::endl;
// }

// void GeometricNmpc::set_parameters(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
// {
//     for (const auto& changed_parameter : event->changed_parameters)
//     {
//         const auto& name = changed_parameter.name;
//         const auto& value = changed_parameter.value;

//         if (name == "geometric_nmpc.dt_MPC")
//             dt_MPC = value.double_value;
//         else if (name == "geometric_nmpc.u")
//             u = value.double_value;
//         else if (name == "geometric_nmpc.fmax")
//             fmax = value.double_value;
//         else if (name == "geometric_nmpc.fmin")
//             fmin = value.double_value;
//         else if (name == "geometric_nmpc.vec_Q")
//         {
//             const auto& array = value.double_array_value;
//             vec_Q = stdVectorToEigen<Eigen::Matrix<double, 13, 1>>(array);
//         }
//         else if (name == "geometric_nmpc.vec_R")
//         {
//             const auto& array = value.double_array_value;
//             vec_R = stdVectorToEigen<Eigen::Matrix<double, 3, 1>>(array);
//         }
//         else if (name == "geometric_nmpc.vec_P")
//         {
//             const auto& array = value.double_array_value;
//             vec_P = stdVectorToEigen<Eigen::Matrix<double, 6, 1>>(array);
//         }
//     }
// }
// }  // namespace Galileo