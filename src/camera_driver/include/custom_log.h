#ifndef CUSTOM_LOG_H
#define CUSTOM_LOG_H 

#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/async.h>
#include <filesystem>
#include <mutex>
#include <memory>
#include <map>
#include <vector>
#include <algorithm>

namespace fs = std::filesystem;

class CustomLogger 
{
public:
    struct Config 
    {
        std::string base_path;
        size_t max_file_size;
        int max_backup_files;
        size_t max_logs_size;
        size_t min_free_space;
        bool debug_mode;
        
        // 构造函数提供默认值
        Config() 
            : base_path("./logs"),
              max_file_size(2 * 1024 * 1024),
              max_backup_files(2),
              max_logs_size(100 * 1024 * 1024),
              min_free_space(100 * 1024 * 1024),
              debug_mode(false)
        {}
    };
    
    static CustomLogger& instance();
    void initialize(const Config& config=Config());
  
    CustomLogger(const CustomLogger&) = delete;
    CustomLogger& operator=(const CustomLogger&) = delete;
    
    std::shared_ptr<spdlog::logger> getLogger(const std::string& name, 
        bool is_critical=false);
    void setEnvironment(bool debug_mode=true);
    
private:
    CustomLogger() : initialized_(false) {}
    // 检查磁盘和日志空间（仅在初始化时调用）
    void checkStorageConditions();
    
    // 清理旧日志
    void cleanupOldLogs(int cleanup_type);
    
    // 计算当前日志总大小
    size_t calculateLogsSize() const;
    
    bool initialized_;
    Config config_;
    std::mutex mutex_;
    std::map<std::string, std::shared_ptr<spdlog::logger>> loggers_;
    bool storage_ok_{true};
};

#endif