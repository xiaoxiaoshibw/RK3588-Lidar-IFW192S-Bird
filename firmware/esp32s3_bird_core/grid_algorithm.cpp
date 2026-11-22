#include "grid_algorithm.hpp"

#include <algorithm>

namespace bird {
namespace {

bool present(const uint8_t* bitmap, int cell) {
    return (bitmap[cell / 8] & (1u << (cell % 8))) != 0;
}

bool valid_range(double range) {
    return std::isfinite(range) && range > 0.1;
}

}  // namespace

GridTracker::GridTracker(GridWorkspace workspace)
    : workspace_(workspace),
      tracker_(workspace.background, workspace.mask, workspace.queue, workspace.foreground_limit) {}

bool GridTracker::step(const Measurement* points, const uint8_t* presence,
                       const Vec3* unit_rays, Result& result) {
    if (!points || !presence || !unit_rays) { return false; }
    const Frame frame{points, workspace_.exact_ranges, unit_rays,
                      workspace_.squared_ranges, workspace_.range_cached};

    if (tracker_.warming_up()) {
        for (int cell = 0; cell < kCells; ++cell) {
            if (!present(presence, cell)) {
                workspace_.exact_ranges[cell] = 0.0;
                continue;
            }
            const double range = norm(point(frame, static_cast<Cell>(cell)));
            if (!valid_range(range)) { return false; }
            workspace_.exact_ranges[cell] = range;
        }
    } else {
        std::fill_n(workspace_.range_cached, kCells / 32, uint32_t{0});
        for (int cell = 0; cell < kCells; ++cell) {
            float& squared = workspace_.squared_ranges[cell];
            if (!present(presence, cell)) {
                squared = 0.0f;
                continue;
            }
            const Measurement& measured = points[cell];
            squared = squared_norm(measured.x, measured.y, measured.z);
            // Recheck near the validity cutoff and when float arithmetic overflows.
            if (squared < 0.01f - 0.000001f) { return false; }
            if (!std::isfinite(squared) || squared <= 0.01f + 0.000001f) {
                const double range = norm(point(frame, static_cast<Cell>(cell)));
                if (!valid_range(range)) { return false; }
                frame.cache_range(static_cast<Cell>(cell), range);
            }
        }
    }

    tracker_.step(frame, result);
    return true;
}

bool GridTracker::step_precomputed(const Measurement* points,
                                   const Vec3* unit_rays, Result& result) {
    if (!points || !unit_rays || tracker_.warming_up()) { return false; }
    const Frame frame{points, workspace_.exact_ranges, unit_rays,
                      workspace_.squared_ranges, workspace_.range_cached};
    tracker_.step(frame, result);
    return true;
}

void GridTracker::reset() {
    tracker_.reset();
}

}  // namespace bird
