#ifndef REDIS_WRAPPER_H
#define REDIS_WRAPPER_H

#include <sw/redis++/redis++.h>
#include <nlohmann/json.hpp>
#include <memory>
#include <string>
#include <vector>
#include <optional>

using json = nlohmann::json;
using namespace sw::redis;

class RedisWrapper {
public:
    // 初始化连接
    explicit RedisWrapper(const std::string& host="127.0.0.1",
                         int port=6379,
                         const std::string& password="",
                         size_t pool_size=1);

    /* 字符串操作 */
    bool Set(const std::string& key, const std::string& value);
    bool SetJson(const std::string& key, const json& value);
    std::optional<std::string> Get(const std::string& key);
    std::optional<json> GetJson(const std::string& key);
    bool Delete(const std::string& key);

    /* 列表操作 */
    long long LPush(const std::string& key, const std::vector<std::string>& values);
    long long LPushJson(const std::string& key, const std::vector<json>& values);
    long long RPush(const std::string& key, const std::vector<std::string>& values);
    long long RPushJson(const std::string& key, const std::vector<json>& values);
    std::optional<std::string> LPop(const std::string& key);
    std::optional<json> LPopJson(const std::string& key);
    std::optional<std::string> RPop(const std::string& key);
    std::optional<json> RPopJson(const std::string& key);
    std::vector<std::string> LRange(const std::string& key, long long start, long long stop);
    std::vector<json> LRangeJson(const std::string& key, long long start, long long stop);
    long long LLen(const std::string& key);

private:
    bool isConnected() const;
    bool reconnect();

    int max_retries_ = 3;
    std::shared_ptr<sw::redis::Redis> redis_;
    ConnectionOptions options_;
    ConnectionPoolOptions pool_options_;
};

#endif // REDIS_WRAPPER_H
