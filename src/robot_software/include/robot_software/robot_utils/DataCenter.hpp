#pragma once

#include <atomic>
#include <concepts>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <typeindex>
#include <unordered_map>

namespace Galileo
{

template <typename T>
concept CopyConstructible = std::is_copy_constructible_v<std::remove_reference_t<T>>;

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
        using DataType = std::remove_reference_t<T>;
        auto ptr = std::make_shared<DataType>(std::forward<T>(data));
        auto void_ptr = std::shared_ptr<void>(ptr, ptr.get());
        data_map_[std::type_index(typeid(DataType))].store(void_ptr);
    }

    // 通用的获取数据方法
    template <typename T>
    std::shared_ptr<T> getData() const
    {
        using DataType = std::remove_reference_t<T>;
        auto it = data_map_.find(std::type_index(typeid(DataType)));
        if (it == data_map_.end())
        {
            return nullptr;
        }

        auto void_ptr = it->second.load();
        if (!void_ptr)
        {
            return nullptr;
        }

        auto result = std::static_pointer_cast<DataType>(void_ptr);
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

    mutable std::unordered_map<std::type_index, std::atomic<std::shared_ptr<void>>> data_map_;
};

}  // namespace Galileo