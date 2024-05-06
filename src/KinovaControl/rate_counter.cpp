#include "KinovaControl/rate_counter.hpp"
#include <chrono>

RateCounter::RateCounter(int desired_rate) :
    desired_rate_(desired_rate),
    n_(0),
    rate_(0),
    active_(false),
    sleep_time_(1.0 / desired_rate_) {}

RateCounter::~RateCounter()
{
    if (active_)
    {
        stop();
    }
}

void RateCounter::start()
{
    active_ = true;
    thread_ = std::thread(&RateCounter::loop, this);
}

void RateCounter::stop()
{
    active_ = false;
    thread_.join();
}

int RateCounter::getRate() const
{
    return rate_;
}

double RateCounter::getSleepTime() const {
    return sleep_time_;
}

void RateCounter::count()
{
    while (active_)
    {
        n_++;
        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time_));
    }
}

void RateCounter::loop()
{
    auto start_time = std::chrono::steady_clock::now();
    while (active_)
    {
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed_seconds = std::chrono::duration<double>(end_time - start_time).count();
        rate_ = n_ / elapsed_seconds;
        n_ = 0;
        start_time = end_time;
        std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
    }
}
