#pragma once

#include <array>
#include <cstddef>
#include <ostream>
#include <vector>

namespace hybrid_cartesian_dynamics {

constexpr std::array<const char*, 23> logColumns{{
    "time_s", "pdes_x", "pcur_x", "vdes_x", "vcur_x",
    "fcur_x", "fcur_y", "fcur_z", "ffiltered_z", "f_feedback_z", "fdes_z", "f_cmd_z",
    "wrench_x", "wrench_z", "a_requested_x", "a_cmd_x",
    "q2", "q4", "qd2", "qd4", "tau_cmd_2", "tau_cmd_4", "a_integral_x"
}};
using LogSample = std::array<double, logColumns.size()>;

template <typename Row>
void writeLogRow(std::ostream& output, const Row& row) {
    output << row.front();
    for (std::size_t i = 1; i < row.size(); ++i) {
        output << ',' << row[i];
    }
    output << '\n';
}

// Both buffers are allocated before contact. The execution-manager mutex protects
// recording and the swap, and the command thread writes the swapped rows to disk.
class LogBuffer {
public:
    explicit LogBuffer(std::size_t capacity) : capacity_(capacity) {
        pending_.reserve(capacity);
        batch_.reserve(capacity);
    }

    void reset() {
        pending_.clear();
        dropped_ = 0;
    }

    void record(const LogSample& sample) {
        if (pending_.size() < capacity_) {
            pending_.push_back(sample);
        } else {
            ++dropped_;
        }
    }

    void swapPending() {
        pending_.swap(batch_);
    }

    void writeBatch(std::ostream& output) {
        for (const auto& sample : batch_) {
            writeLogRow(output, sample);
        }
        batch_.clear();
    }

    std::size_t dropped() const { return dropped_; }

private:
    const std::size_t capacity_;
    std::vector<LogSample> pending_, batch_;
    std::size_t dropped_ = 0;
};

} // namespace hybrid_cartesian_dynamics
