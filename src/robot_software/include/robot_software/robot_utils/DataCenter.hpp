#pragma once

#include <atomic>
#include <concepts>
#include <memory>
#include "robot_software/robot_utils/DataTypes.hpp"

// 定义可存储数据类型的概念
namespace Galileo
{
template <typename T>
concept StorableData = std::is_default_constructible_v<T> && std::is_copy_constructible_v<T>;

class DataCenter
{
public:
    static DataCenter& getInstance()
    {
        static DataCenter instance;
        return instance;
    }

    // 通用写入接口
    template <StorableData T>
    void write(const T& data)
    {
        auto new_data = std::make_shared<T>(data);
        getData<T>().store(new_data);
    }

    // 通用读取接口
    template <StorableData T>
    std::shared_ptr<T> read() const
    {
        return getData<T>().load();
    }

private:
    DataCenter() = default;
    ~DataCenter() = default;
    DataCenter(const DataCenter&) = delete;
    DataCenter& operator=(const DataCenter&) = delete;

    // 获取对应类型的原子智能指针
    template <StorableData T>
    std::atomic<std::shared_ptr<T>>& getData()
    {
        return data_storage<T>;
    }

    template <StorableData T>
    const std::atomic<std::shared_ptr<T>>& getData() const
    {
        return data_storage<T>;
    }

    // 每个类型对应一个静态原子智能指针
    template <StorableData T>
    static std::atomic<std::shared_ptr<T>> data_storage;
};

// 静态成员定义
template <StorableData T>
std::atomic<std::shared_ptr<T>> DataCenter::data_storage;

}  // namespace Galileo