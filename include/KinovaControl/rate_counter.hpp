#ifndef RATE_COUNTER_HPP
#define RATE_COUNTER_HPP

#include <thread>
#include <chrono>

class RateCounter {
public:
    RateCounter(int desired_rate);
    ~RateCounter();

    void count();
    void start();
    void stop();
    int getRate() const;

private:
    void loop();

    int desired_rate_;
    int n_;
    int rate_;
    double sleep_time_;
    bool active_;
    std::thread loop_thread_;
};

#endif // RATE_COUNTER_HPP
