// #pragma once
// #include <Eigen/Dense>

// #include "rclcpp/rclcpp.hpp"
// #include "robot_software/robot_utils/DataCenter.hpp"
// namespace Galileo
// {
// class GeometricNmpc
// {
// public:
//     GeometricNmpc();
//     ~GeometricNmpc() = default;

//     void run();

//     void declare_and_get_parameters(rclcpp::Node *node);

//     void set_parameters(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

// private:
//     double mass = 50;
//     mat33 inertiaMatrix;
//     double dt_MPC = 0.03;
//     double u = 0.6;
//     double fmax = 1000;
//     double fmin = 20;
//     static constexpr int Horizon = 10;
//     static constexpr int Kdim = 13;
//     static constexpr int Pdim = 12;

//     // double vec_Q[13] = {0};  // 似乎太大的跨度不可取
//     // double vec_R[3] = {0};

//     // double vec_P[6] = {0};

//     Eigen::Matrix<double, 13, 1> vec_Q;
//     Eigen::Matrix<double, 3, 1> vec_R;
//     Eigen::Matrix<double, 6, 1> vec_P;

//     int stanceLegNum = 4;
//     int stanceLegNumlast = 0;

//     ///****Matrices****///
//     Eigen::Matrix<double, 4, 4> geoM;
//     Eigen::Matrix<double, 4, 4> geoM_d;
//     Eigen::Matrix<double, 6, 1> geoTwist;
//     Eigen::Matrix<double, 6, 1> geoTwist_d;

//     Eigen::Matrix<double, 3, 3> RotationMatrix;
//     Eigen::Matrix<double, 3, 3> RotationMatrix_d;
//     Horizon

//         Eigen::Matrix<double, Kdim, Kdim>
//             A_dt;
//     Eigen::Matrix<double, Kdim, Eigen::Dynamic> B_dt;

//     Eigen::Matrix<double, Kdim * Horizon, Kdim> A_qp;
//     Eigen::Matrix<double, Kdim * Horizon, Eigen::Dynamic> B_qp;
//     Eigen::Matrix<double, 6 * Horizon, Eigen::Dynamic> T_qp;

//     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_qp;
//     Eigen::Matrix<double, Eigen::Dynamic, 1> g_qp;
//     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> C_qp;  // 6 * 4 * Horizon, 3 * 4 * Horizon
//     Eigen::Matrix<double, Eigen::Dynamic, 1> d_qp;

//     Eigen::Matrix<double, Kdim * Horizon, 1> vector_Q;
//     Eigen::Matrix<double, Eigen::Dynamic, 1> vector_R;
//     Eigen::Matrix<double, 6 * Horizon, 1> vector_P;

//     Eigen::DiagonalMatrix<double, Kdim * Horizon> weight_Q;  // 12 * Horizon //定义为对角阵会提高速度
//     Eigen::DiagonalMatrix<double, Eigen::Dynamic> weight_R;  // n * Horizon, n * Horizon
//     Eigen::DiagonalMatrix<double, 6 * Horizon> weight_P;     // n * Horizon, n * Horizon

//     Eigen::Matrix<double, 3, 4> fLeg;
//     Eigen::Matrix<double, Eigen::Dynamic, 1> fGround;
//     Eigen::Matrix<double, Eigen::Dynamic, 1> f_qp;
//     Eigen::Matrix<double, 6 * Horizon, 1> fd;
//     Eigen::Matrix<double, Eigen::Dynamic, 1> fdd;

//     ///****functions****///
//     Eigen::Matrix<double, 3, 3> skew(const Eigen::Matrix<double, 3, 1> &w);
//     Eigen::Matrix<double, 3, 1> invskew(const Eigen::Matrix<double, 3, 3> &Mat);

//     Eigen::Matrix<double, 6, 6> Adjoint(const Eigen::Matrix<double, 4, 4> &M);
//     Eigen::Matrix<double, 6, 6> adjoint(const Eigen::Matrix<double, 6, 1> &Twist);
//     //    Matrix<double, 6, 6> coadjoint();

//     void getState(const Eigen::Matrix<double, 12, 1> &state, const Eigen::Matrix<double, 12, 1> &state_d);
//     Eigen::Matrix<double, 4, 4> getErrorM(const Eigen::Matrix<double, 4, 4> &M, const Eigen::Matrix<double, 4, 4>
//     &M_d); Eigen::Matrix<double, 6, 1> getErrorTwist(const Eigen::Matrix<double, 6, 1> &Twist,
//                                               const Eigen::Matrix<double, 6, 1> &Twist_d,
//                                               const Eigen::Matrix<double, 4, 4> &eM);
//     Eigen::Matrix<double, Kdim, 1> getDeltaX(const Eigen::Matrix<double, 4, 4> &eM,
//                                              const Eigen::Matrix<double, 6, 1> &eTwist);
//     Eigen::Matrix<double, Kdim, Kdim> getA(const Eigen::Matrix<double, 6, 1> &Twist,
//                                            const Eigen::Matrix<double, 6, 6> &Jb,
//                                            const Eigen::Matrix<double, 4, 4> &M);
//     Eigen::Matrix<double, 6, Eigen::Dynamic> getT(const Eigen::Matrix<double, 4, 3> footpos,
//                                                   const Eigen::Matrix<int, 4, 1> &Leg_Contact_State);
//     Eigen::Matrix<double, Kdim, Eigen::Dynamic> getB(const Eigen::Matrix<double, 6, 6> &Jb,
//                                                      const Eigen::Matrix<double, 6, Eigen::Dynamic> &T);

//     Eigen::Matrix<double, Kdim, 1> getDeltaX_so3(const Eigen::Matrix<double, 12, 1> &state,
//                                                  const Eigen::Matrix<double, 12, 1> &state_d);
//     Eigen::Matrix<double, Kdim, Kdim> getA_so3(const Eigen::Matrix<double, 12, 1> &state,
//                                                const Eigen::Matrix<double, 12, 1> &state_d,
//                                                const Eigen::Matrix<double, 6, 6> &Jb);
//     Eigen::Matrix<double, Kdim, Eigen::Dynamic> getB_so3(const Eigen::Matrix<double, 6, 6> &Jb,
//                                                          const Eigen::Matrix<double, 6, Eigen::Dynamic> &T);

//     void discreteAB(const Eigen::Matrix<double, Kdim, Kdim> &A, const Eigen::Matrix<double, Kdim, Eigen::Dynamic>
//     &B); void setABqp(const Eigen::Matrix<double, Kdim, 1> &dX,
//                  const Eigen::Matrix<double, Kdim, Kdim> &A,
//                  const Eigen::Matrix<double, Kdim, Eigen::Dynamic> &B,
//                  const Eigen::Matrix<int, 4, 1> &Leg_Contact_State,
//                  const Eigen::DiagonalMatrix<double, Kdim * Horizon> &weight_Q,
//                  const Eigen::DiagonalMatrix<double, Eigen::Dynamic> &weight_R);

//     void setABqp(const Eigen::Matrix<double, Kdim, 1> &dX,
//                  const Eigen::Matrix<double, Kdim, Kdim> &A,
//                  const Eigen::Matrix<double, Kdim, Eigen::Dynamic> &B,
//                  const Eigen::Matrix<int, 4, 1> &Leg_Contact_State,
//                  const Eigen::DiagonalMatrix<double, Kdim * Horizon> &weight_Q,
//                  const Eigen::DiagonalMatrix<double, 6 * Horizon> &weight_P,
//                  const Eigen::Matrix<double, 6, Eigen::Dynamic> &T,
//                  const Eigen::Matrix<double, Eigen::Dynamic, 1> &f);

//     void setConstrains(const Eigen::Matrix<int, 4, 1> &Leg_Contact_State,
//                        const Eigen::Matrix<double, Eigen::Dynamic, 1> &f,
//                        double fmax,
//                        double fmin,
//                        double u);
//     void initMatrix(const Eigen::Matrix<int, 4, 1> &Leg_Contact_State);

//     void initMPC_proxsuite(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &H_qp,
//                            const Eigen::Matrix<double, Eigen::Dynamic, 1> &g_qp,
//                            const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &C_qp,
//                            const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &d_qp,
//                            const Eigen::Matrix<int, 4, 1> &Leg_Contact_State);

//     DataCenter &dataCenter_;
// };
// }  // namespace Galileo