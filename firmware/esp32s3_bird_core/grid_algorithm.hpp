#pragma once

#include "bird_tracker.hpp"

namespace bird {

constexpr int kPresenceBytes = kCells / 8;

// Provide distinct buffers that outlive the tracker. Sizes are in elements.
struct GridWorkspace {
    double* background;       // kWarmupFrames * kCells
    double* exact_ranges;     // kCells
    float* squared_ranges;    // kCells, prefer internal SRAM
    float* foreground_limit;  // kCells
    uint8_t* mask;             // kCells, prefer internal SRAM
    Cell* queue;              // kCells
    uint32_t* range_cached;   // kCells / 32, prefer internal SRAM
};

class GridTracker {
public:
    explicit GridTracker(GridWorkspace workspace);

    // A frame supplies kCells measurements in native ring/column order,
    // kPresenceBytes validity bytes (least significant bit first), and
    // kCells calibrated unit rays. Input buffers stay valid during this call.
    // False means invalid input. Tracking state and output are unchanged.
    // Output cells refer to the input of this call.
    bool step(const Measurement* points, const uint8_t* presence,
              const Vec3* unit_rays, Result& result);

    // Active-frame fast path when workspace.squared_ranges and workspace.range_cached
    // are already populated and points[cell] is materialized for all foreground and
    // locked-track retain-context cells.
    bool step_precomputed(const Measurement* points,
                          const Vec3* unit_rays, Result& result);

    // Rebuild the background after a scene or calibration change.
    void reset();
    void clear_tracks() { tracker_.clear_tracks(); }
    bool warming_up() const { return tracker_.warming_up(); }
    void locked_context_boxes(std::vector<std::array<int, 4>>& out) const {
        tracker_.locked_context_boxes(out);
    }

private:
    GridWorkspace workspace_;
    Tracker tracker_;
};

}  // namespace bird
