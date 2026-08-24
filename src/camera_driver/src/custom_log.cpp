#include "custom_log.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <iostream>

CustomLogger& CustomLogger::instance()
{
    static CustomLogger cl;
    return cl;
}

void CustomLogger::initialize(const Config& config)
{
    if (initialized_)
        throw std::runtime_error("CustomLogger already initialized");

    config_ = config;
    // 1. 创建日志目录
    try
    {
        fs::create_directories(config_.base_path);
    }
    catch (...)
    {
        // 如果失败，使用/tmp目录
        config_.base_path = "/tmp/logs";
        fs::create_directories(config_.base_path);
    }

    // 2. 检查存储条件（磁盘空间 + 日志大小）
    checkStorageConditions();

    initialized_ = true;
    // 3. 初始化异步线程池
    spdlog::init_thread_pool(2048, 1);
}

size_t CustomLogger::calculateLogsSize() const
{
    try
    {
        size_t total_size = 0;

        if (fs::exists(config_.base_path))
            for (const auto& entry : fs::recursive_directory_iterator(config_.base_path))
                if (entry.is_regular_file() && entry.path().extension() == ".log")
                {
                    try
                    {
                        total_size += entry.file_size();
                    }
                    catch (...)
                    {
                        // 跳过无法获取大小的文件
                    }
                }
        return total_size;
    }
    catch (...)
    {
        return 0;
    }
}

void CustomLogger::checkStorageConditions()
{
    try
    {
        // 1. 检查磁盘剩余空间
        auto space = fs::space(config_.base_path);
        size_t logs_size = calculateLogsSize();

        std::cout << "=== Storage Check ===" << std::endl;
        std::cout << "Disk total: " << space.capacity / (1024*1024) << "MB" << std::endl;
        std::cout << "Disk free: " << space.available / (1024*1024) << "MB"
                  << " (min required: " << config_.min_free_space / (1024*1024) << "MB)" << std::endl;
        std::cout << "Logs size: " << logs_size / (1024*1024) << "MB"
                  << " (max allowed: " << config_.max_logs_size / (1024*1024) << "MB)" << std::endl;

        // 2. 判断是否需要清理
        int cleanup_type = 0;
        std::string reason;

        if (space.available < config_.min_free_space)
        {
            cleanup_type = 1;
            reason = "Disk free space below minimum requirement";
        }

        if (logs_size > config_.max_logs_size)
        {
            cleanup_type = 2;
            if (!reason.empty()) reason += " and ";
            reason += "Logs size exceeded limit";
        }

        // 3. 如果需要清理，执行清理
        if (cleanup_type)
        {
            std::cout << "Need cleanup: " << reason << std::endl;
            cleanupOldLogs(cleanup_type);

            // 清理后再次检查
            space = fs::space(config_.base_path);
            logs_size = calculateLogsSize();

            bool disk_ok = space.available >= config_.min_free_space;
            bool logs_ok = logs_size <= config_.max_logs_size;

            storage_ok_ = disk_ok && logs_ok;

            std::cout << "After cleanup:" << std::endl;
            std::cout << "  Disk free: " << space.available / (1024*1024) << "MB" << std::endl;
            std::cout << "  Logs size: " << logs_size / (1024*1024) << "MB" << std::endl;
            std::cout << "  Storage OK: " << (storage_ok_ ? "YES" : "NO") << std::endl;
        }
        else
        {
            storage_ok_ = true;
            std::cout << "Storage conditions OK, no cleanup needed." << std::endl;
        }

        std::cout << "===================" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Storage check failed: " << e.what() << std::endl;
        storage_ok_ = false;
    }
}

void CustomLogger::cleanupOldLogs(int cleanup_type)
{
    try
    {
        // 收集所有日志文件信息
        struct LogFileInfo
        {
            fs::path path;
            std::time_t modify_time;
            size_t size;
        };

        std::vector<LogFileInfo> log_files;

        if (!fs::exists(config_.base_path))
            return;

        for (const auto& entry : fs::recursive_directory_iterator(config_.base_path))
        {
            if (entry.is_regular_file() && entry.path().extension() == ".log")
            {
                try
                {
                    // 获取文件信息
                    auto ftime = fs::last_write_time(entry.path());
                    auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
                        ftime - fs::file_time_type::clock::now() + std::chrono::system_clock::now());
                    std::time_t modify_time = std::chrono::system_clock::to_time_t(sctp);

                    log_files.push_back({
                        entry.path(),
                        modify_time,
                        entry.file_size()
                    });
                }
                catch (...)
                {
                    // 跳过无法获取信息的文件
                }
            }
        }

        if (log_files.empty())
            return;

        // 按修改时间排序（最旧的在前面）
        std::sort(log_files.begin(), log_files.end(),
            [](const LogFileInfo& a, const LogFileInfo& b) {
                return a.modify_time < b.modify_time;
            });

        // 删除旧文件直到满足条件
        size_t deleted_count = 0;
        size_t deleted_size = 0;

        for (const auto& file : log_files)
        {
            // 检查当前条件
            auto space = fs::space(config_.base_path);
            size_t logs_size = calculateLogsSize();

            bool logs_ok   = logs_size <= config_.max_logs_size;
            bool logs_ok_t = logs_size <= config_.max_logs_size - 10 * 1024 * 1024;

            if(cleanup_type == 1 && logs_ok_t) break;
            else if(cleanup_type == 2 && logs_ok) break;

            try
            {
                // 删除文件
                fs::remove(file.path);
                deleted_count++;
                deleted_size += file.size;

                // 输出删除信息
                char time_str[100];
                std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S",
                             std::localtime(&file.modify_time));

                std::cout << "  Deleted: " << file.path.filename()
                          << " (" << time_str
                          << ", " << file.size / 1024 << "KB)" << std::endl;
            }
            catch (...)
            {
                // 跳过删除失败的文件
            }
        }

        if (deleted_count > 0)
        {
            std::cout << "Cleaned " << deleted_count << " files, "
                      << deleted_size / (1024*1024) << "MB freed" << std::endl;
        }
        else
        {
            std::cout << "No files needed to be cleaned" << std::endl;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Cleanup error: " << e.what() << std::endl;
    }
}

std::shared_ptr<spdlog::logger> CustomLogger::getLogger(const std::string& name, bool is_critical)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 检查是否已存在
    auto it = loggers_.find(name);
    if (it != loggers_.end())
        return it->second;

    // 获取当前日期
    auto now = std::chrono::system_clock::now();
    time_t time = std::chrono::system_clock::to_time_t(now);
    tm tm = *std::localtime(&time);

    // 创建目录结构
    std::ostringstream year_month_ss, filename_ss;
    year_month_ss << std::put_time(&tm, "%Y-%m");
    filename_ss << std::put_time(&tm, "%Y-%m-%d");

    // 新的目录结构：base_path/name/year_month/
    fs::path log_dir = fs::path(config_.base_path) / name / year_month_ss.str();

    // 创建目录，失败时尝试备用方案
    std::error_code ec;
    fs::create_directories(log_dir, ec);
    if (ec) 
    {
        // 备用方案：尝试在临时目录创建
        log_dir = fs::temp_directory_path() / "logs_fallback" / name / year_month_ss.str();
        fs::create_directories(log_dir, ec);
        if (ec) {
            throw std::runtime_error("Failed to create log directory: " + ec.message());
        }
    }

    // 创建文件名：name/year_month/yyyy-mm-dd.log
    std::string filename = (log_dir / (filename_ss.str() + ".log")).string();

    // 创建旋转文件sink
    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        filename, config_.max_file_size, config_.max_backup_files);

    // 创建控制台sink
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

    // 设置输出模式
    std::string file_pattern = "[%Y-%m-%d %H:%M:%S.%e] [%l]";
    std::string console_pattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n]";
    // std::string console_pattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] [t %t]";

    if (is_critical)
    {
        file_pattern = "[CRITICAL] " + file_pattern;
        console_pattern = "[%^CRITICAL%$] " + console_pattern;
    }

    file_pattern += " %v";
    console_pattern += " %v";

    rotating_sink->set_pattern(file_pattern);
    console_sink->set_pattern(console_pattern);
    rotating_sink->set_level(spdlog::level::info);
    // 根据环境设置日志级别
    if (config_.debug_mode)
    {
        // 调试环境：控制台输出debug及以上
        console_sink->set_level(spdlog::level::debug);
    }
    else
    {
        // 生产环境：控制台输出info及以上
        console_sink->set_level(spdlog::level::info);
    }
    // 创建logger
    std::vector<spdlog::sink_ptr> sinks {rotating_sink, console_sink};

    // 异步模式
    auto logger = std::make_shared<spdlog::async_logger>(
        name,
        sinks.begin(),
        sinks.end(),
        spdlog::thread_pool(),
        spdlog::async_overflow_policy::block);

    // 同步模式
    // auto logger = std::make_shared<spdlog::logger>(name,
    //     sinks.begin(), sinks.end());

    // 根据环境设置logger级别
    if (config_.debug_mode)
    {
        // 调试环境：logger设为debug，所有消息通过
        logger->set_level(spdlog::level::debug);
    }
    else
    {
        // 生产环境：logger设为info，debug消息在logger层面过滤
        logger->set_level(spdlog::level::info);
    }
    logger->flush_on(spdlog::level::info);
    spdlog::flush_every(std::chrono::seconds(5));

    spdlog::register_logger(logger);
    loggers_[name] = logger;

    return logger;
}

void CustomLogger::setEnvironment(bool debug_mode)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (config_.debug_mode == debug_mode)
        return;
    config_.debug_mode = debug_mode;

    spdlog::level::level_enum level = debug_mode
        ? spdlog::level::debug
        : spdlog::level::info;

    for (auto& [name, logger] : loggers_)
        if (logger)
        {
            for (auto& sink : logger->sinks())
            {
                auto console_sink = std::dynamic_pointer_cast<spdlog::sinks::stdout_color_sink_mt>(sink);
                if (console_sink)
                    console_sink->set_level(level);
            }
            // 关键：也更新logger全局级别
            logger->set_level(level);
        }
}
