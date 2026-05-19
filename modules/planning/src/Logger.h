#ifndef CHELLO_LOGGER_H
#define CHELLO_LOGGER_H

#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <ctime>
#include <filesystem>
#include <exception>

// 使用命名空间简化文件系统相关操作
namespace fs = std::__fs::filesystem;

/**
 * @brief 日志类，支持多线程安全的日志记录。
 *
 * 该类采用单例模式设计，支持将日志输出到控制台和文件。日志按照日期进行分类保存，
 * 并支持自动清理过期日志（超过30天的日志文件将被删除）。
 */
class Logger {
public:
    /**
     * @brief 日志级别枚举
     * - INFO: 信息级别日志
     * - WARNING: 警告级别日志
     * - ERROR: 错误级别日志
     */
    enum Level {
        INFO,
        WARNING,
        ERROR
    };

    /**
     * @brief 获取Logger单例实例
     * @return Logger& 单例实例的引用
     */
    static Logger &getInstance() {
        static Logger instance;
        return instance;
    }

    /**
     * @brief 设置日志文件夹路径
     * @param logDir 日志文件夹路径
     */
    void setLogDir(const std::string &logDir) {
        std::lock_guard <std::mutex> lock(mutex_);
        logDir_ = logDir;
        ensureLogDirExists(); // 确保日志目录存在
        cleanOldLogs();       // 清理旧日志
    }

    /**
     * @brief 记录一条日志
     * @param level 日志级别
     * @param message 日志信息
     */
    void log(Level level, const std::string &message) {
        std::lock_guard <std::mutex> lock(mutex_);
        std::string levelStr = levelToString(level); // 转换级别为字符串
        std::string timestamp = getTimestamp();     // 获取时间戳

        // 构造完整日志信息
        std::string fullMessage = "[" + timestamp + "] [" + levelStr + "] " + message;

        // 输出到控制台
        std::cout << fullMessage << std::endl;

        // 仅当日志级别为ERROR时，将日志写入文件
        if (level == ERROR && !logDir_.empty()) {
            std::string logFileName = getCurrentLogFileName(); // 获取当前日志文件名
            std::ofstream outFile(logFileName, std::ios::app); // 以追加模式打开文件
            if (outFile.is_open()) {
                outFile << fullMessage << std::endl;
                outFile.close();
            } else {
                std::cerr << "Failed to open log file: " << logFileName << std::endl;
            }
        }
    }

    /**
     * @brief 记录一条异常信息
     * @param e 异常对象
     */
    void logException(const std::exception &e) {
        std::lock_guard <std::mutex> lock(mutex_);
        std::string timestamp = getTimestamp(); // 获取时间戳

        // 构造完整日志信息
        std::string fullMessage = "[" + timestamp + "] [EXCEPTION] " + e.what();

        // 输出到控制台
        std::cout << fullMessage << std::endl;

        // 写入文件
        if (!logDir_.empty()) {
            std::string logFileName = getCurrentLogFileName();
            std::ofstream outFile(logFileName, std::ios::app); // 以追加模式打开文件
            if (outFile.is_open()) {
                outFile << fullMessage << std::endl;
                outFile.close();
            } else {
                std::cerr << "Failed to open log file: " << logFileName << std::endl;
            }
        }
    }

private:
    std::mutex mutex_;              // 用于线程安全的互斥锁
    std::string logDir_;            // 日志文件夹路径

    /**
     * @brief 私有构造函数（单例模式）
     */
    Logger() : logDir_("logs") {
        ensureLogDirExists(); // 确保日志目录存在
        cleanOldLogs();       // 清理过期日志
    }

    // 禁止拷贝构造和赋值操作
    Logger(const Logger &) = delete;

    Logger &operator=(const Logger &) = delete;

    /**
     * @brief 确保日志目录存在
     */
    void ensureLogDirExists() {
        if (!fs::exists(logDir_)) {
            fs::create_directories(logDir_);
        }
    }

    /**
     * @brief 获取当前时间戳（格式：YYYY-MM-DD HH:MM:SS）
     * @return std::string 格式化的时间戳
     */
    std::string getTimestamp() {
        std::time_t now = std::time(nullptr);
        char buf[20];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
        return buf;
    }

    /**
     * @brief 将日志级别转换为字符串
     * @param level 日志级别
     * @return std::string 对应的字符串表示
     */
    std::string levelToString(Level level) {
        switch (level) {
            case INFO:
                return "INFO";
            case WARNING:
                return "WARNING";
            case ERROR:
                return "ERROR";
            default:
                return "UNKNOWN";
        }
    }

    /**
     * @brief 获取当前日志文件名（基于当前日期）
     * @return std::string 日志文件名（格式：logDir/YYYY-MM-DD.log）
     */
    std::string getCurrentLogFileName() {
        std::string date = getCurrentDate();
        return logDir_ + "/" + date + ".log";
    }

    /**
     * @brief 获取当前日期（格式：YYYY-MM-DD）
     * @return std::string 格式化的日期
     */
    std::string getCurrentDate() {
        std::time_t now = std::time(nullptr);
        std::tm *now_tm = std::localtime(&now);
        char buffer[80];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d", now_tm);
        return std::string(buffer);
    }

    /**
     * @brief 清理过期日志（超过30天未修改的文件将被删除）
     */
    void cleanOldLogs() {
        auto now = fs::file_time_type::clock::now();
        for (const auto &entry: fs::directory_iterator(logDir_)) {
            if (entry.is_regular_file()) {
                auto last_write_time = fs::last_write_time(entry.path());
                using namespace std::chrono;
                if (now - last_write_time > hours(30 * 24)) { // 超过30天
                    fs::remove(entry.path());
                }
            }
        }
    }
};

#endif
