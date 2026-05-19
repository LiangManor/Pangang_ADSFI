#pragma once
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include "ara/tsync/synch_master_tb.h"
using DataClock = ara::tsync::SynchMasterTB<ara::tsync::SynchMasterIdentity::k0>;
using ManageClock = ara::tsync::SynchMasterTB<ara::tsync::SynchMasterIdentity::k1>;
class FastLogger
{
public:

// // 数据面时间
// static inline uint64_t nowDataNs()
// {
//     auto tp = DataClock::now();

//     return std::chrono::duration_cast<std::chrono::nanoseconds>(
//             tp.time_since_epoch()).count();
// }

// // 管理面时间
// static inline uint64_t nowManageNs()
// {
//     auto tp = ManageClock::now();

//     return std::chrono::duration_cast<std::chrono::nanoseconds>(
//             tp.time_since_epoch()).count();
// }

// 数据面时间，格式：年月日时分秒毫秒
static inline uint64_t nowDataUs()
{
    auto tp = DataClock::now();

    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
              tp.time_since_epoch()).count();

    time_t sec = ns / 1000000000ULL;
    long   nsec = ns % 1000000000ULL;

    struct tm tm;
    localtime_r(&sec, &tm);

    uint64_t t =
        (uint64_t)(tm.tm_year + 1900) * 10000000000000ULL +
        (uint64_t)(tm.tm_mon + 1)     * 100000000000ULL +
        (uint64_t)(tm.tm_mday)        * 1000000000ULL +
        (uint64_t)(tm.tm_hour)        * 10000000ULL +
        (uint64_t)(tm.tm_min)         * 100000ULL +
        (uint64_t)(tm.tm_sec)         * 1000ULL +
        (uint64_t)(nsec / 1000000);

    return t;
}

// 管理面时间，格式：年月日时分秒毫秒
static inline uint64_t nowManageUs()
{
    auto tp = ManageClock::now();

    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
              tp.time_since_epoch()).count();

    time_t sec = ns / 1000000000ULL;
    long   nsec = ns % 1000000000ULL;

    struct tm tm;
    localtime_r(&sec, &tm);

    uint64_t t =
        (uint64_t)(tm.tm_year + 1900) * 10000000000000ULL +
        (uint64_t)(tm.tm_mon + 1)     * 100000000000ULL +
        (uint64_t)(tm.tm_mday)        * 1000000000ULL +
        (uint64_t)(tm.tm_hour)        * 10000000ULL +
        (uint64_t)(tm.tm_min)         * 100000ULL +
        (uint64_t)(tm.tm_sec)         * 1000ULL +
        (uint64_t)(nsec / 1000000);

    return t;
}

// 传感器时间，格式：年月日时分秒毫秒
static inline uint64_t sensorUs(uint32_t sec, uint32_t nsec)
{
    time_t tsec = sec;

    struct tm tm;
    localtime_r(&tsec, &tm);

    uint64_t t =
        (uint64_t)(tm.tm_year + 1900) * 10000000000000ULL +
        (uint64_t)(tm.tm_mon + 1)     * 100000000000ULL +
        (uint64_t)(tm.tm_mday)        * 1000000000ULL +
        (uint64_t)(tm.tm_hour)        * 10000000ULL +
        (uint64_t)(tm.tm_min)         * 100000ULL +
        (uint64_t)(tm.tm_sec)         * 1000ULL +
        (uint64_t)(nsec / 1000000);

    return t;
}

// 数据面时间，格式：ns
static inline uint64_t nowDataNs()
{
    auto tp = DataClock::now();

    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               tp.time_since_epoch()).count();
}

// 传感器时间，格式：ns
static inline uint64_t sensorNs(uint32_t sec, uint32_t nsec)
{
    return (uint64_t)sec * 1000000000ULL + nsec;
}


//////////************************************************************************************** */
// 重载 log 函数，可以写入 string
static inline void log(
        const std::string& module,
        int stage = 0,
        int value = 0)
{
    log(module.c_str(), stage, value);
}
// 记录日志，格式：管理面时间|数据面时间|模块名|阶段|数值
static inline void log(
        const char* module,
        int stage = 0,
        int value = 0)
{
    uint64_t dp = nowDataUs();
    uint64_t mp = nowManageUs();

    fprintf(fp,
        "%lu|%lu|%s|%d|%d\n",
        mp, dp, module, stage, value);

    fflush(fp);   //有数据就立刻写入文件，调试时快速看到日志，正式发布时可以注释掉这行，提升性能
}
// 记录日志，格式：数据面时间|传感器时间|文字说明|备用|备用
static inline void logTs(
        uint32_t sec,
        uint32_t nsec,
        const char* module,
        int stage = 0,
        int value = 0)
{
    uint64_t dp = nowDataUs();
    uint64_t ts = sensorUs(sec, nsec);

    fprintf(fp,
        "%lu|%lu|%s|%d|%d\n",
        dp, ts, module, stage, value);

    fflush(fp);   //有数据就立刻写入文件，调试时快速看到日志，正式发布时可以注释掉这行，提升性能
}

// 超快速记录 ns 时间戳日志，用于对照系统时间和传感器时间防止时间转换时耗时太久，格式：数据面时间|传感器时间|文字说明|备用|备用
static inline void logNs(
        uint32_t sec,
        uint32_t nsec,
        const char* module,
        int stage = 0,
        int value = 0)
{
    uint64_t dp = nowDataNs();
    uint64_t ts = sensorNs(sec, nsec);

    fprintf(fp,
        "%lu|%lu|%s|%d|%d\n",
        dp, ts, module, stage, value);

    // fflush(fp);   //有数据就立刻写入文件，调试时快速看到日志，正式发布时可以注释掉这行，提升性能
}

// 超快速记录 ns 时间戳日志，用于记录系统时间和传感器时间的差值,格式：数据面时间|传感器时间|文字说明|差值
static inline void logNsDiff(
        uint32_t sec,
        uint32_t nsec,
        const char* module)
{
    uint64_t dp = nowDataNs();
    uint64_t ts = sensorNs(sec, nsec);

    fprintf(fp,
        "%lu|%lu|%s|%lu\n",
        dp, ts, module, dp - ts);

    // fflush(fp);   //有数据就立刻写入文件，调试时快速看到日志，正式发布时可以注释掉这行，提升性能
}

static void init(const char* file)
{
    fp = fopen(file, "a");
    setvbuf(fp, buffer, _IOFBF, sizeof(buffer));
}

static void flush()
{
    fflush(fp);
}

private:

static FILE* fp;
static char buffer[1024*1024];
};

FILE* FastLogger::fp = nullptr;
char FastLogger::buffer[1024*1024];
