#include "robot_software/robot_controller/nmpc_controller/OptProblemBase.h"

namespace Galileo
{
OptProblemBase::OptProblemBase()
    : dataCenter_(DataCenter::getInstance())
{
    model = dataCenter_.read<pinocchio::Model>();
}

std::vector<xyz::polymorphic<StageModel>> OptProblemBase::create_stages(
    const std::vector<std::vector<bool>>& contact_phases,
    const std::vector<std::vector<pinocchio::SE3>>& contact_poses,
    const std::vector<std::vector<Eigen::VectorXd>>& contact_forces)
{
    if (contact_phases.size() != contact_poses.size())
    {
        throw std::runtime_error("Contact phases and poses sequences do not have the same size");
    }
    if (contact_phases.size() != contact_forces.size())
    {
        throw std::runtime_error("Contact phases and forces sequences do not have the same size");
    }

    const size_t num_feet = 4;
    std::vector<bool> previous_phases(num_feet, true);
    std::vector<xyz::polymorphic<StageModel>> stage_models;

    for (std::size_t i = 0; i < contact_phases.size(); i++)
    {
        std::vector<bool> land_constraint(num_feet, false);

        for (size_t j = 0; j < num_feet; j++)
        {
            // 计算着地约束
            if (!previous_phases[j] && contact_phases[i][j])
            {
                land_constraint[j] = true;
            }
        }

        StageModel stage = create_stage(contact_phases[i], contact_poses[i], contact_forces[i], land_constraint);
        stage_models.push_back(std::move(stage));
        previous_phases = contact_phases[i];
    }

    return stage_models;
}

void OptProblemBase::create_problem(const ConstVectorRef& x0,
                                    const size_t horizon,
                                    const int force_size,
                                    const double gravity,
                                    const bool terminal_constraint = false)
{
    const size_t num_feet = 4;

    // 初始化力的参考值
    Eigen::VectorXd force_ref(force_size);
    force_ref.setZero();
    force_ref[2] = -dataCenter_.read<robot_constants>()->mass * gravity / num_feet;

    // 创建单个时间步的数据
    std::vector<bool> contact_phase(num_feet, true);
    std::vector<pinocchio::SE3> contact_pose(num_feet, pinocchio::SE3::Identity());
    std::vector<Eigen::VectorXd> contact_force(num_feet, force_ref);

    // 创建整个时域的数据
    std::vector<std::vector<bool>> contact_phases(horizon, contact_phase);
    std::vector<std::vector<pinocchio::SE3>> contact_poses(horizon, contact_pose);
    std::vector<std::vector<Eigen::VectorXd>> contact_forces(horizon, contact_force);

    std::vector<xyz::polymorphic<StageModel>> stage_models =
        create_stages(contact_phases, contact_poses, contact_forces);

    problem_ = std::make_unique<TrajOptProblem>(x0, std::move(stage_models), create_terminal_cost());
    problemInitialized_ = true;

    if (terminal_constraint)
    {
        create_terminal_constraint(x0.head(3));
    }
}
}  // namespace Galileo
