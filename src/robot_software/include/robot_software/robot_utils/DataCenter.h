#pragma once

#include <atomic>
#include <concepts>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <typeindex>
#include <unordered_map>

namespace robot_utils
{

template <typename T>
concept CopyConstructible = std::is_copy_constructible_v<T>;

class DataCenter
{
public:
    static DataCenter& getInstance()
    {
        static DataCenter instance;
        return instance;
    }

    // 通用的设置数据方法
    template <CopyConstructible T>
    void setData(T&& data)
    {
        auto ptr = std::make_shared<T>(std::forward<T>(data));
        auto void_ptr = std::shared_ptr<void>(ptr, ptr.get());
        data_map_[std::type_index(typeid(T))].store(void_ptr);
    }

    // 通用的获取数据方法
    template <typename T>
    std::shared_ptr<T> getData() const
    {
        auto it = data_map_.find(std::type_index(typeid(T)));
        if (it == data_map_.end())
        {
            return nullptr;
        }

        auto void_ptr = it->second.load();
        if (!void_ptr)
        {
            return nullptr;
        }

        auto result = std::static_pointer_cast<T>(void_ptr);
        if (!result)
        {
            throw std::runtime_error("类型转换失败，请检查数据类型是否匹配");
        }
        return result;
    }

private:
    DataCenter() = default;
    ~DataCenter() = default;
    DataCenter(const DataCenter&) = delete;
    DataCenter& operator=(const DataCenter&) = delete;

    // 使用类型索引作为键存储数据
    mutable std::unordered_map<std::type_index, std::atomic<std::shared_ptr<void>>> data_map_;
};

// 定义各种数据类型
namespace data_types
{

// 状态估计器数据类型
struct StateEstimate
{
    // 状态估计的具体数据成员
};

struct FilterCovariance
{
    // 协方差矩阵数据
};

struct ImuMeasurement
{
    // IMU数据
};

// 控制器数据类型
struct ControlCommand
{
    // 控制指令数据
};

struct GaitSchedule
{
    // 步态数据
};

// 接口数据类型
struct JointState
{
    // 关节状态数据
};

struct ModelState
{
    // 模型状态数据
};

}  // namespace data_types

}  // namespace robot_utils