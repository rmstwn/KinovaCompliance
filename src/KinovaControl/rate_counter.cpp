#include "KinovaControl/RateCounter.hpp"

RateCounter::RateCounter(int desired_rate) : desired_rate_(desired_rate), n_(0), rate_(0), sleep_time_(1.0 / desired_rate), active_(false) {}

RateCounter::~RateCounter() {
    stop();
}

void RateCounter::count() {
    n_++;
}

void RateCounter::start() {
    active_ = true;
    loop_thread_ = std::thread(&RateCounter::loop, this);
}

void RateCounter::stop() {
    active_ = false;
    if (loop_thread_.joinable()) {
        loop_thread_.join();
    }
}

int RateCounter::getRate() const {
    return rate_;
}

void RateCounter::loop() {
    while (active_) {
        if (n_ != 0) {
            rate_ = n_;
            n_ = 0;
            sleep_time_ *= rate_ / desired_rate_;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
