#ifndef INCLIO_TIMER_HPP
#define INCLIO_TIMER_HPP

#include <chrono>
#include <string>

namespace IncLIO {

// TODO: Implement a scoped timer for profiling pipeline stages
//       Usage: { ScopedTimer t("NDT registration"); ... }

class ScopedTimer {
public:
    explicit ScopedTimer(const std::string& name);
    ~ScopedTimer();

    double ElapsedMs() const;

private:
    std::string name_;
    std::chrono::high_resolution_clock::time_point start_;
};

} // namespace IncLIO

#endif // INCLIO_TIMER_HPP
