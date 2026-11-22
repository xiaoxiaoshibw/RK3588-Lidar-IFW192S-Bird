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
using Cell = uint16_t;

struct Vec3 {
    double x, y, z;
};

struct Measurement {
    float x, y, z;
    uint32_t source_row;
};

// Grids use native ring/column order. Zero range marks a missing measurement.
struct Frame {
    const Measurement* points;
    const double* ranges;
    const Vec3* rays;
};

inline Vec3 point(const Frame& frame, Cell cell) {
    const auto& measured = frame.points[cell];
    return {measured.x, measured.y, measured.z};
}

inline double norm(Vec3 value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
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
    // Scratch capacities are kWarmupFrames*kCells, kCells and kCells elements.
    Tracker(double* warmup_background, uint8_t* mask, Cell* queue);
    ~Tracker();
    Tracker(const Tracker&) = delete;
    Tracker& operator=(const Tracker&) = delete;
    void reset();
    void step(const Frame& frame, Result& result);

private:
    struct Impl;
    Impl* impl_;
};

}  // namespace bird
