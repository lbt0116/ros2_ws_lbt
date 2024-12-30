#pragma once

#include "robot_software/robot_controller/nmpc_controller/OptProblemBase.h"

namespace Galileo
{
using namespace aligator;
using namespace aligator::context;

using MultibodyPhaseSpace = proxsuite::nlp::MultibodyPhaseSpace<double>;
using KinodynamicsFwdDynamics = dynamics::KinodynamicsFwdDynamicsTpl<double>;
using CentroidalMomentumDerivativeResidual = CentroidalMomentumDerivativeResidualTpl<double>;
using CentroidalMomentumResidual = CentroidalMomentumResidualTpl<double>;
using CentroidalWrenchConeResidual = CentroidalWrenchConeResidualTpl<double>;
using CentroidalFrictionConeResidual = CentroidalFrictionConeResidualTpl<double>;
using FramePlacementResidual = FramePlacementResidualTpl<double>;
using FrameTranslationResidual = FrameTranslationResidualTpl<double>;
using FrameVelocityResidual = FrameVelocityResidualTpl<double>;
using CenterOfMassTranslationResidual = CenterOfMassTranslationResidualTpl<double>;
using IntegratorSemiImplEuler = dynamics::IntegratorSemiImplEulerTpl<double>;

class KinoDynamicProblem : public OptProblemBase
{
public:
    KinoDynamicProblem();
    ~KinoDynamicProblem();

    StageModel create_stage(const std::vector<bool>& land_constraint) override;

    CostStack create_terminal_cost() override;

    void update_terminal_constraint(const Eigen::Vector3d& com_ref) override;

    void create_terminal_constraint(const Eigen::Vector3d& com_ref) override;

    /*
     * @brief: dynamic
     * @param:
     * @return:
     */
    /*
     * 问题结构:
     * ├── problem
     * │   ├── x0 (初始状态)
     * │   ├── stage (阶段)
     * │   │   ├── dynamics (动力学)
     * │   │   ├── cost (代价函数)
     * │   │   └── constraint (约束条件)
     * │   └── terminal_cost (终端代价)
     */

private:
    struct params
    {
        // controller weight
        matxd w_x;
        matxd w_u;
        matxd w_cent;
        matxd w_centder;
        matxd w_force;
        matxd w_force_der;
        matxd w_frame;

        // controller parameters
        vecxd qmax;
        vecxd qmin;

        // physics parameters
        double timestep = 0.01;
        int force_size = 3;
        double mu = 0.5;

        // constraints flag
        bool kinematics_limits = false;
        bool force_cone = false;
    } params_;

    int nq_;
    int nv_;
    int nu_;

    MultibodyPhaseSpace space;
};
}  // namespace Galileo
