#ifndef RATE_COUNTER_HPP
#define RATE_COUNTER_HPP

#include <thread>
#include <atomic>
#include <chrono>

class RateCounter
{
public:
    RateCounter(int desired_rate);
    ~RateCounter();

    void start();
    void stop();
    int getRate() const;
    double getSleepTime() const;
    void count();

private:
    void loop();

    int desired_rate_;
    std::atomic<int> n_;
    std::atomic<int> rate_;
    std::atomic<bool> active_;
    double sleep_time_;
    std::thread thread_;
};

#endif // RATE_COUNTER_HPP
