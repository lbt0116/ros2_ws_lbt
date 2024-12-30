#include "robot_software/robot_controller/nmpc_controller/KinoDynamicProblem.h"

namespace Galileo
{
KinoDynamicProblem::KinoDynamicProblem()
    : space(MultibodyPhaseSpace(model)),
      nq_(model->nq),
      nv_(model->nv),
      nu_(model->nv - 6 + 4 * 3)
{
    x.resize(nq_ + nv_, 1);
    xd.resize(nq_ + nv_, 1);
    ud.resize(nu_, 1);
    std::vector<std::string> foot_frame_name = {"toe1Link", "toe2Link", "toe3Link", "toe4Link"};

    for (const auto& foot_name : foot_frame_name)
    {
        std::size_t fid = model->getFrameId(foot_name);  // 获取帧ID
        contactIds.push_back(fid);
    }

    // 创建欧拉角到四元数的转换
    Eigen::Vector3d euler_angles = dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetEulerAngles;
    Eigen::Quaterniond quat = Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitZ())
                              * Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY())
                              * Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitX());

    // p q j v w jv
    x << dataCenter_.read<robot_state::BaseState>()->position, dataCenter_.read<robot_state::BaseState>()->quaternion,
        dataCenter_.read<robot_state::JointState>()->jointPosition,
        dataCenter_.read<robot_state::BaseState>()->linearVelocity,
        dataCenter_.read<robot_state::BaseState>()->angularVelocity,
        dataCenter_.read<robot_state::JointState>()->jointVelocity;

    xd << dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetPosition,
        quat.coeffs(),  // 直接使用 coeffs() 方法获取四元数向量 (x,y,z,w)
        dataCenter_.read<robot_target_trajectory::TargetJointTrajectory>()->targetJointPosition,
        dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetLinearVelocity,
        dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetAngularVelocity,
        dataCenter_.read<robot_target_trajectory::TargetJointTrajectory>()->targetJointVelocity;

    contactPose = dataCenter_.read<robot_state::LegState>()->legPosBaseInWorld;
    contactForce = dataCenter_.read<robot_state::LegState>()->legForceInWorld;
    contactPhase = dataCenter_.read<robot_FSM::legState>()->legPhase;
}

KinoDynamicProblem::~KinoDynamicProblem()
{
}

StageModel KinoDynamicProblem::create_stage(const std::vector<bool>& land_constraint)
{
    auto rcost = CostStack(space, nu_);
    std::vector<bool> contact_states;
    for (auto const& x : contactPhase)
    {
        contact_states.push_back(x);
    }

    // computeControlFromForces(contactForce);

    auto cent_mom = CentroidalMomentumResidual(space.ndx(), nu_, model, Eigen::VectorXd::Zero(6));

    auto centder_mom = CentroidalMomentumDerivativeResidual(space.ndx(),
                                                            model,
                                                            dataCenter_.read<robot_constants>()->gravity,
                                                            contact_states,
                                                            contactIds,
                                                            params_.force_size);

    rcost.addCost("state_cost", QuadraticStateCost(space, nu_, xd, params_.w_x));
    rcost.addCost("control_cost", QuadraticControlCost(space, contactForce.reshaped(12, 1), params_.w_u));
    rcost.addCost("centroidal_cost", QuadraticResidualCost(space, cent_mom, params_.w_cent));
    rcost.addCost("centroidal_derivative_cost", QuadraticResidualCost(space, centder_mom, params_.w_centder));

    for (int i = 0; i < 4; i++)
    {
        FrameTranslationResidual frame_residual =
            FrameTranslationResidual(space.ndx(), nu_, model, contactPose.col(i), contactIds[i]);

        rcost.addCost("leg" + std::to_string(i) + "_pose_cost",
                      QuadraticResidualCost(space, frame_residual, params_.w_frame));
    }

    KinodynamicsFwdDynamics ode = KinodynamicsFwdDynamics(
        space, model, dataCenter_.read<robot_constants>()->gravity, contactPhase, contactIds, params_.force_size);

    IntegratorSemiImplEuler dyn_model = IntegratorSemiImplEuler(ode, params_.timestep);

    StageModel stm = StageModel(rcost, dyn_model);

    if (params_.kinematics_limits)
    {
        StateErrorResidual state_fn = StateErrorResidual(space, nu_, space.neutral());
        std::vector<int> state_id;
        for (int i = 6; i < nv_; i++)
        {
            state_id.push_back(i);
        }
        FunctionSliceXpr state_slice = FunctionSliceXpr(state_fn, state_id);
        stm.addConstraint(state_slice, BoxConstraint(-params_.qmax, -params_.qmin));
    }

    pinocchio::Motion v_ref = pinocchio::Motion::Zero();

    for (int i = 0; i < 4; i++)
    {
        if (contactPhase[i])
        {
            FrameVelocityResidual frame_vel =
                FrameVelocityResidual(space.ndx(), nu_, model, v_ref, contactIds[i], pinocchio::LOCAL);

            if (params_.force_cone)
            {
                CentroidalFrictionConeResidual friction_residual =
                    CentroidalFrictionConeResidual(space.ndx(), nu_, i, params_.mu, 1e-4);
                stm.addConstraint(friction_residual, NegativeOrthant());
            }

            std::vector<int> vel_id = {0, 1, 2};

            FunctionSliceXpr vel_slice = FunctionSliceXpr(frame_vel, vel_id);
            stm.addConstraint(vel_slice, EqualityConstraint());

            if (land_constraint[i])
            {
                std::vector<int> frame_id = {2};

                FrameTranslationResidual frame_residual =
                    FrameTranslationResidual(space.ndx(), nu_, model, contactPose.col(i), contactIds[i]);

                FunctionSliceXpr frame_slice = FunctionSliceXpr(frame_residual, frame_id);
                stm.addConstraint(frame_slice, EqualityConstraint());
            }
        }
    }

    return stm;
}

CostStack KinoDynamicProblem::create_terminal_cost()
{
    auto ter_space = MultibodyPhaseSpace(model);
    auto term_cost = CostStack(ter_space, nu_);
    auto cent_mom = CentroidalMomentumResidual(ter_space.ndx(), nu_, model, Eigen::VectorXd::Zero(6));

    term_cost.addCost("state_cost", QuadraticStateCost(ter_space, nu_, xd, params_.w_x));
    term_cost.addCost("centroidal_cost", QuadraticResidualCost(ter_space, cent_mom, params_.w_cent * 10));

    return term_cost;
}

void KinoDynamicProblem::update_terminal_constraint(const Eigen::Vector3d& com_ref)
{
}

void KinoDynamicProblem::create_terminal_constraint(const Eigen::Vector3d& com_ref)
{
    if (!problemInitialized_)
    {
        throw std::runtime_error("Create problem first!");
    }
    CenterOfMassTranslationResidual com_cstr = CenterOfMassTranslationResidual(space.ndx(), nu_, model, com_ref);

    problem_->addTerminalConstraint(com_cstr, EqualityConstraint());
    terminalConstraint_ = true;
}
}  // namespace Galileo
