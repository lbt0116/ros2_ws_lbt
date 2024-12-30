#pragma once
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/costs/quad-state-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/modelling/function-xpr-slice.hpp>
#include <proxsuite-nlp/modelling/constraints/box-constraint.hpp>
#include <proxsuite-nlp/modelling/constraints/negative-orthant.hpp>
#ifndef ALIGATOR_PINOCCHIO_V3
#error "aligator was not compiled with Pinocchio 3 support. simple-mpc requires Pinocchio 3 features in aligator."
#endif

#include <aligator/context.hpp>

#include "robot_software/robot_utils/DataCenter.hpp"

namespace Galileo
{
using namespace aligator;
using namespace aligator::context;
using CostStack = CostStackTpl<double>;
using ControlErrorResidual = ControlErrorResidualTpl<double>;
using QuadraticControlCost = QuadraticControlCostTpl<double>;
using QuadraticStateCost = QuadraticStateCostTpl<double>;
using QuadraticResidualCost = QuadraticResidualCostTpl<double>;
using StateErrorResidual = StateErrorResidualTpl<double>;
using BoxConstraint = proxsuite::nlp::BoxConstraintTpl<double>;
using NegativeOrthant = proxsuite::nlp::NegativeOrthantTpl<double>;
using EqualityConstraint = proxsuite::nlp::EqualityConstraintTpl<double>;
using FunctionSliceXpr = FunctionSliceXprTpl<double>;

class OptProblemBase
{
public:
    std::shared_ptr<pinocchio::Model> model;
    OptProblemBase();
    virtual ~OptProblemBase() = default;

    virtual std::vector<xyz::polymorphic<StageModel>> create_stages(
        const std::vector<std::vector<bool>>& contact_phases,
        const std::vector<std::vector<pinocchio::SE3>>& contact_poses,
        const std::vector<std::vector<Eigen::VectorXd>>& contact_forces);

    void create_problem(const ConstVectorRef& x0,
                        const size_t horizon,
                        const int force_size,
                        const double gravity,
                        const bool terminal_constraint);

    virtual StageModel create_stage(const std::vector<bool>& contact_phases,
                                    const std::vector<pinocchio::SE3>& contact_poses,
                                    const std::vector<Eigen::VectorXd>& contact_forces,
                                    const std::vector<bool>& land_constraint) = 0;

    // Manage terminal cost and constraint
    virtual CostStack create_terminal_cost() = 0;

    virtual void update_terminal_constraint(const Eigen::Vector3d& com_ref) = 0;

    virtual void create_terminal_constraint(const Eigen::Vector3d& com_ref) = 0;

    std::unique_ptr<TrajOptProblem> problem_;

protected:
    // Size of the problem
    int nq_;
    int nv_;
    int ndx_;
    int nu_;
    bool problemInitialized_ = false;
    bool terminalConstraint_ = false;

    DataCenter& dataCenter_;

    matxd x;
    matxd xd;
    matxd ud;
    mat34 contactPose;
    mat34 contactForce;
    vec4i contactPhase;
    std::vector<pinocchio::FrameIndex> contactIds;
};
}  // namespace Galileo
