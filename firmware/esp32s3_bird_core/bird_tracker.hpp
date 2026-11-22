#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace bird {

constexpr int kWidth = 256;
constexpr int kHeight = 192;
constexpr int kCells = kWidth * kHeight;
constexpr int kWarmupFrames = 10;
constexpr double kTrackingFarM = 4.50;
using Cell = uint16_t;

struct Vec3 {
    double x, y, z;
};

struct Measurement {
    float x, y, z;
    uint32_t source_row;
};

// Grids use native ring/column order. Zero marks a missing measurement.
struct Frame {
    const Measurement* points;
    double* exact_ranges;
    const Vec3* rays;
    const float* squared_ranges;
    uint32_t* range_cached;

    double range(Cell cell) const;
    void cache_range(Cell cell, double value) const;
};

inline Vec3 point(const Frame& frame, Cell cell) {
    const auto& measured = frame.points[cell];
    return {measured.x, measured.y, measured.z};
}

inline double norm(Vec3 value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

inline double Frame::range(Cell cell) const {
    const uint32_t bit = uint32_t{1} << (cell % 32);
    if ((range_cached[cell / 32] & bit) != 0) {
        return exact_ranges[cell];
    }
    if (squared_ranges[cell] == 0.0f) { return 0.0; }
    const double exact = norm(point(*this, cell));
    cache_range(cell, exact);
    return exact;
}

inline void Frame::cache_range(Cell cell, double value) const {
    exact_ranges[cell] = value;
    range_cached[cell / 32] |= uint32_t{1} << (cell % 32);
}

inline float squared_norm(float x, float y, float z) {
    return x * x + y * y + z * z;
}

inline double distance(Vec3 a, Vec3 b) {
    return norm({a.x - b.x, a.y - b.y, a.z - b.z});
}

struct Component {
    std::vector<Cell> cells;
    std::array<int, 4> box;  // Display coordinates, inclusive corners.
    Vec3 center;
};

void segment_bodies(const Frame& frame, const std::vector<Cell>& parent,
                    std::vector<Component>& out);

enum class MatchReason { Dynamic, Retained };

struct TrackResult {
    uint32_t track_id;
    MatchReason reason;
    Component component;
    double range_m;
    double max_disp_m;
};

struct Result {
    bool warming_up = false;
    int warmup_progress = 0;
    int dynamic_candidates = 0;
    int skipped_large_parents = 0;
    int tentative_count = 0;
    std::vector<TrackResult> locked;
};

class Tracker {
public:
    // Scratch capacities are kWarmupFrames*kCells and kCells for each other buffer.
    Tracker(double* warmup_background, uint8_t* mask, Cell* queue, float* foreground_limit_squared);
    ~Tracker();
    Tracker(const Tracker&) = delete;
    Tracker& operator=(const Tracker&) = delete;
    void reset();
    void clear_tracks();
    bool warming_up() const;
    void locked_context_boxes(std::vector<std::array<int, 4>>& out) const;
    void step(const Frame& frame, Result& result);

private:
    struct Impl;
    Impl* impl_;
};

}  // namespace bird
