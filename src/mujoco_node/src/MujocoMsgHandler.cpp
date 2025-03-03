#include "MujocoMsgHandler.h"

#include <Eigen/Dense>
#include <algorithm>

#include "sensor_msgs/image_encodings.hpp"
namespace Galileo
{
MujocoMsgHandler::MujocoMsgHandler(mj::Simulate* sim)
    : Node("MujocoMsgHandler", rclcpp::NodeOptions().use_intra_process_comms(true)),
      sim_(sim),
      name_prefix("simulation/")
{
    model_param_name = name_prefix + "model_file";
    this->declare_parameter(model_param_name, "");

    // reset_service_ = this->create_service<communication::srv::SimulationReset>(
    //     name_prefix + "sim_reset",
    //     std::bind(&MujocoMsgHandler::reset_callback, this,
    //               std::placeholders::_1, std::placeholders::_2));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", qos);
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
    mujoco_msg_publisher_ = this->create_publisher<custom_msgs::msg::MujocoMsg>("mujoco_msg", qos);

    timers_.emplace_back(this->create_wall_timer(1ms, std::bind(&MujocoMsgHandler::publish_mujoco_callback, this)));

    // timers_.emplace_back(this->create_wall_timer(
    //     1ms, std::bind(&MujocoMsgHandler::joint_callback, this)));

    // timers_.emplace_back(this->create_wall_timer(
    //     20ms, std::bind(&MujocoMsgHandler::odom_callback, this)));
    //
    // timers_.emplace_back(this->create_wall_timer(
    //     100ms, std::bind(&MujocoMsgHandler::drop_old_message, this)));

    actuator_cmd_subscription_ = this->create_subscription<custom_msgs::msg::ActuatorCmds>(
        "actuators_cmds", qos, std::bind(&MujocoMsgHandler::actuator_cmd_callback, this, std::placeholders::_1));
    sim_msg_subscription_ = this->create_subscription<custom_msgs::msg::ToSimMsg>(
        "simulation", 1, std::bind(&MujocoMsgHandler::sim_msg_callback, this, std::placeholders::_1));

    // param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this
    // );
    // cb_handle_ = param_subscriber_->add_parameter_callback(
    //     model_param_name, std::bind(&MujocoMsgHandler::parameter_callback,
    //                                 this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Start MujocoMsgHandler ...");

    std::string model_file = this->get_parameter(model_param_name).get_parameter_value().get<std::string>();
    mju::strcpy_arr(sim_->filename, model_file.c_str());
    sim_->uiloadrequest.fetch_add(1);
}

MujocoMsgHandler::~MujocoMsgHandler()
{
    RCLCPP_INFO(this->get_logger(), "close node ...");
}

std::shared_ptr<MujocoMsgHandler::ActuatorCmds> MujocoMsgHandler::get_actuator_cmds_ptr()
{
}

void MujocoMsgHandler::publish_mujoco_callback()
{
    if (sim_->d_ != nullptr)
    {
        const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);

        if (sim_->single_step_mode)
        {
            if (sim_->single_step_flag)
            {
                imu_callback();
                joint_callback();
                mujoco_msg_callback();
                sim_->single_step_flag = false;
            }
            else
            {
            }
        }
        else
        {
            imu_callback();
            joint_callback();
            mujoco_msg_callback();
        }
    }
}

void MujocoMsgHandler::imu_callback()
{
    auto message = sensor_msgs::msg::Imu();
    message.header.frame_id = &sim_->m_->names[0];
    message.header.stamp = rclcpp::Clock().now();
    // const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);

    for (int i = 0; i < sim_->m_->nsensor; i++)
    {
        if (sim_->m_->sensor_type[i] == mjtSensor::mjSENS_ACCELEROMETER)
        {
            message.linear_acceleration.x = sim_->d_->sensordata[sim_->m_->sensor_adr[i]];
            message.linear_acceleration.y = sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 1];
            message.linear_acceleration.z = sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 2];
        }
        else if (sim_->m_->sensor_type[i] == mjtSensor::mjSENS_FRAMEQUAT)
        {
            message.orientation.w = sim_->d_->sensordata[sim_->m_->sensor_adr[i]];
            message.orientation.x = sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 1];
            message.orientation.y = sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 2];
            message.orientation.z = sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 3];
        }
        else if (sim_->m_->sensor_type[i] == mjtSensor::mjSENS_GYRO)
        {
            message.angular_velocity.x = sim_->d_->sensordata[sim_->m_->sensor_adr[i]];
            message.angular_velocity.y = sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 1];
            message.angular_velocity.z = sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 2];
        }
    }

    imu_publisher_->publish(message);
    // RCLCPP_INFO(this->get_logger(), "imu node %.2f",
    // message.linear_acceleration.z);
}

void MujocoMsgHandler::mujoco_msg_callback()
{
    auto msg = custom_msgs::msg::MujocoMsg();
    std::vector<std::string> foot_geom_names = {"toe1_contact", "toe2_contact", "toe3_contact", "toe4_contact"};

    std::vector<int> foot_geom_ids;
    for (const auto& foot_name : foot_geom_names)
    {
        int geom_id = mj_name2id(sim_->m_, mjOBJ_GEOM, foot_name.c_str());
        if (geom_id == -1)
        {
            std::cerr << "Geometry " << foot_name << " not found in the model!" << std::endl;
            return;
        }
        foot_geom_ids.push_back(geom_id);
    }
    int ground_geom_id = mj_name2id(sim_->m_, mjOBJ_GEOM, "floor");

    // 初始化接触状态数组，默认值为 0（未接触）
    int contact_state[4] = {0, 0, 0, 0};
    // 初始化12维接触力向量,每条腿3维力
    std::vector<double> contact_forces(12, 0.0);

    for (int i = 0; i < sim_->d_->ncon; ++i)
    {
        const mjContact& contact = sim_->d_->contact[i];

        // 遍历所有足端
        for (size_t foot_idx = 0; foot_idx < foot_geom_ids.size(); ++foot_idx)
        {
            int foot_geom_id = foot_geom_ids[foot_idx];

            // 检查是否是足端与地面的接触
            if ((contact.geom1 == foot_geom_id && contact.geom2 == ground_geom_id)
                || (contact.geom1 == ground_geom_id && contact.geom2 == foot_geom_id))
            {
                // 如果发生接触，标记对应的接触状态为 1
                contact_state[foot_idx] = 1;

                // 提取接触力
                mjtNum force[6];  // 包含法向力和摩擦力的 6D 力
                mj_contactForce(sim_->m_, sim_->d_, i, force);

                // 将接触力存入对应腿的位置(每条腿3维力)
                contact_forces[foot_idx * 3] = force[0];
                contact_forces[foot_idx * 3 + 1] = force[1];
                contact_forces[foot_idx * 3 + 2] = force[2];
            }
        }
    }

    // 将接触力和接触状态填入消息
    msg.ground_reaction_force = contact_forces;
    for (int i = 0; i < 4; i++)
    {
        msg.contact_state[i] = contact_state[i];
    }
    // 读取躯干位置和速度
    // 位置: xyz在qpos的前3个元素，四元数在后4个元素
    // 获取躯干 ID
    int body_index = mj_name2id(sim_->m_, mjOBJ_BODY, "base_link");
    // mjtNum real_vel[6];
    mjtNum* real_vel = sim_->d_->cvel + 6 * body_index;  // w v
    mjtNum* real_p = sim_->d_->subtree_com;              //
    mjtNum* force_ptr = sim_->d_->xfrc_applied + 6 * body_index;
    mjtNum* real_quad = sim_->d_->xquat + body_index * 4;  // w v

    msg.p_real[0] = real_p[0];
    msg.p_real[1] = real_p[1];
    msg.p_real[2] = real_p[2];

    msg.v_real[0] = real_vel[3];
    msg.v_real[1] = real_vel[4];
    msg.v_real[2] = real_vel[5];



    mujoco_msg_publisher_->publish(msg);
}

void MujocoMsgHandler::joint_callback()
{
    if (sim_->d_ != nullptr)
    {
        sensor_msgs::msg::JointState jointState;
        jointState.header.frame_id = &sim_->m_->names[0];
        jointState.header.stamp = rclcpp::Clock().now();
        for (int i = 0; i < sim_->m_->njnt; i++)
        {
            if (sim_->m_->jnt_type[i] == mjtJoint::mjJNT_HINGE)
            {
                std::string jnt_name(mj_id2name(sim_->m_, mjtObj::mjOBJ_JOINT, i));
                jointState.name.emplace_back(jnt_name);
                jointState.position.push_back(sim_->d_->qpos[sim_->m_->jnt_qposadr[i]]);
                jointState.velocity.push_back(sim_->d_->qvel[sim_->m_->jnt_dofadr[i]]);
                jointState.effort.push_back(sim_->d_->qfrc_actuator[sim_->m_->jnt_dofadr[i]]);
            }
        }
        joint_state_publisher_->publish(jointState);
        // RCLCPP_INFO(this->get_logger(), "joint node %.2f",
        // jointState.position.at(0));
    }
}

void MujocoMsgHandler::actuator_cmd_callback(const custom_msgs::msg::ActuatorCmds::SharedPtr msg) const
{
    if (!sim_ || !sim_->d_ || !sim_->m_)
    {
        return;
    }

    for (size_t k = 0; k < msg->actuators_name.size(); k++)
    {
        const std::string& actuator_name = msg->actuators_name[k];
        int actuator_id = mj_name2id(sim_->m_, mjOBJ_ACTUATOR, actuator_name.c_str());
        int joint_id = mj_name2id(sim_->m_, mjOBJ_JOINT, actuator_name.c_str());

        if (actuator_id == -1)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("MuJoCo"), "Actuator '%s' not found in MuJoCo model.", actuator_name.c_str());
            continue;
        }
        // Get the joint position and velocity directly from qpos and qvel
        double joint_position = sim_->d_->qpos[sim_->m_->jnt_qposadr[joint_id]];
        double joint_velocity = sim_->d_->qvel[sim_->m_->jnt_dofadr[joint_id]];

        // Calculate control signal
        double position_error = msg->pos[k] - joint_position;
        double velocity_error = msg->vel[k] - joint_velocity;

        sim_->d_->ctrl[actuator_id] = msg->kp[k] * position_error + msg->kd[k] * velocity_error + msg->torque[k];

        // Apply torque limits dynamically from the message
        double torque_limit = msg->torque_limit[k];
        sim_->d_->ctrl[actuator_id] = std::clamp(sim_->d_->ctrl[actuator_id], -torque_limit, torque_limit);
    }
}

void MujocoMsgHandler::parameter_callback(const rclcpp::Parameter&)
{
}

void MujocoMsgHandler::drop_old_message()
{
}

void MujocoMsgHandler::sim_msg_callback(const custom_msgs::msg::ToSimMsg::SharedPtr msg)
{
    if (msg->start_simulation)
    {
        for (size_t k = 0; k < msg->actuators_name.size(); k++)
        {
            const std::string& actuator_name = msg->actuators_name[k];
            int actuator_id = mj_name2id(sim_->m_, mjOBJ_ACTUATOR, actuator_name.c_str());
            int joint_id = mj_name2id(sim_->m_, mjOBJ_JOINT, actuator_name.c_str());

            if (actuator_id == -1)
            {
                RCLCPP_WARN(
                    rclcpp::get_logger("MuJoCo"), "Actuator '%s' not found in MuJoCo model.", actuator_name.c_str());
                continue;
            }
            // Get the joint position and velocity directly from qpos and qvel
            sim_->d_->qpos[sim_->m_->jnt_qposadr[joint_id]] = msg->initial_joint_positions[k];
        }
    }
}
}  // namespace Galileo
