class SlidingWindowFilter {
public:
    SlidingWindowFilter(size_t windowSize) : maxSize(windowSize) {}

    // 向滤波器中添加一个新的数据点
    void addValue(double value) {
        // 如果窗口已满，则移除最旧的数据
        if (window.size() >= maxSize) {
            window.pop_front();
        }
        // 添加新数据
        window.push_back(value);
        // 计算新的滤波结果
        currentAverage = calculateAverage();
    }

    // 获取当前滤波后的结果
    double getFilteredValue() const {
        return currentAverage;
    }

private:
    size_t maxSize;                    // 窗口的最大大小
    std::deque<double> window;         // 存储窗口数据的双端队列
    double currentAverage = 0.0;       // 当前平均值

    // 计算窗口中数据的平均值
    double calculateAverage() const {
        if (window.empty()) {
            return 0.0;
        }
        double sum = std::accumulate(window.begin(), window.end(), 0.0);
        return sum / window.size();
    }
};
