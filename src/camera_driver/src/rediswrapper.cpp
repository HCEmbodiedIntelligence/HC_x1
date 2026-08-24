#include "rediswrapper.h"
#include <stdexcept>
#include <iostream>

RedisWrapper::RedisWrapper(const std::string& host, int port,
                           const std::string& password, size_t pool_size)
{
    try
    {
        options_.host = host;
        options_.port = port;
        options_.password = password;
        options_.socket_timeout = std::chrono::milliseconds(2000); // 设置连接超时

        pool_options_.size = pool_size;

        redis_ = std::make_shared<Redis>(options_, pool_options_);
        redis_->ping();
    }
    catch (const Error& e)
    {
        throw std::runtime_error(std::string(e.what()));
    }
    catch (const std::bad_alloc& e)
    {
        throw std::runtime_error("Redis bad_alloc: " + std::string(e.what()));
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Redis initialization error: " + std::string(e.what()));
    }
}

// 添加连接状态检查方法
bool RedisWrapper::isConnected() const
{
    if (!redis_) return false;

    try {
        redis_->ping();
        return true;
    } catch (...) {
        return false;
    }
}

bool RedisWrapper::reconnect()
{
    try
    {
        // 使用相同的参数重新创建Redis对象
        redis_ = std::make_shared<Redis>(options_, pool_options_);
        redis_->ping();
        return true;
    }
    catch (...)
    {
        return false;
    }
}

/* 字符串操作实现 */
bool RedisWrapper::Set(const std::string& key, const std::string& value)
{
    for (int i = 0; i < max_retries_; ++i)
    {
        try
        {
            if (!isConnected() && !reconnect())
                continue;
            return redis_->set(key, value);
        }
        catch (...)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    return false;
}

bool RedisWrapper::SetJson(const std::string& key, const json& value)
{
    return Set(key, value.dump());
}

std::optional<std::string> RedisWrapper::Get(const std::string& key)
{
    for (int i = 0; i < max_retries_; ++i)
    {
        try
        {
            return redis_->get(key);
        }
        catch (const sw::redis::IoError& e)
        {
            // 简单的等待重试
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        catch (const Error& e)
        {
            throw std::runtime_error("Redis GET error: " + std::string(e.what()));
        }
    }
    return std::nullopt;
}

std::optional<json> RedisWrapper::GetJson(const std::string& key)
{
    auto val = Get(key);
    if (!val || val->empty())
        return std::nullopt;

    try
    {
        return json::parse(*val);
    }
    catch (const json::parse_error& e)
    {
        // 处理 JSON 解析错误
        std::cerr << "JSON parse error: " << e.what() << std::endl;
        return std::nullopt;
    }
}

bool RedisWrapper::Delete(const std::string& key)
{
    try {
        return redis_->del(key) > 0;
    } catch (const Error& e) {
        throw std::runtime_error("Redis DEL error: " + std::string(e.what()));
    }
}

/* 列表操作实现 */
long long RedisWrapper::LPush(const std::string& key, const std::vector<std::string>& values)
{
    try {
        return redis_->lpush(key, values.begin(), values.end());
    } catch (const Error& e) {
        throw std::runtime_error("Redis LPUSH error: " + std::string(e.what()));
    }
}

long long RedisWrapper::LPushJson(const std::string& key, const std::vector<json>& values)
{
    std::vector<std::string> str_values;
    str_values.reserve(values.size());
    for (const auto& j : values) {
        str_values.push_back(j.dump());
    }
    return LPush(key, str_values);
}

std::optional<std::string> RedisWrapper::LPop(const std::string& key)
{
    try {
        return redis_->lpop(key);
    } catch (const Error& e) {
        throw std::runtime_error("Redis LPOP error: " + std::string(e.what()));
    }
}

std::optional<json> RedisWrapper::LPopJson(const std::string& key)
{
    auto val = LPop(key);
    if (!val) return std::nullopt;
    return json::parse(*val);
}

std::vector<std::string> RedisWrapper::LRange(const std::string& key, long long start, long long stop)
{
    try {
        std::vector<std::string> result;
        // 使用 back_inserter 作为输出迭代器
        redis_->lrange(key, start, stop, std::back_inserter(result));
        return result;
    } catch (const Error& e) {
        throw std::runtime_error("Redis LRANGE error: " + std::string(e.what()));
    }
}

std::vector<json> RedisWrapper::LRangeJson(const std::string& key, long long start, long long stop)
{
    auto vals = LRange(key, start, stop);
    std::vector<json> result;
    result.reserve(vals.size());
    for (const auto& v : vals)
        result.push_back(json::parse(v));
    return result;
}

long long RedisWrapper::LLen(const std::string& key)
{
    try
    {
        return redis_->llen(key);
    } catch (const Error& e) {
        throw std::runtime_error("Redis LLEN error: " + std::string(e.what()));
    }
}
