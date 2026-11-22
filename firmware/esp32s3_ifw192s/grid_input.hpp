#pragma once

#include <limits>

#include "geometry.hpp"
#include "../esp32s3_bird_core/grid_algorithm.hpp"

namespace ifw192s {

struct GridInput {
    bird::Measurement* measurements;
    uint8_t* presence;
};

inline void place_measurement(const WireRecord& point, uint32_t wire_row,
                              const Direction* wire_directions, const RangeCalibration& calibration,
                              GridInput grid, bool fast_range = false,
                              float max_range = std::numeric_limits<float>::infinity()) {
    const float range = fast_range ? live_measured_range(point, calibration) : measured_range(point, calibration);
    if (range == 0.0f || range > max_range) {
        return;
    }
    const uint16_t wire_cell = (uint16_t{point.ring} << 8) | point.column;
    const auto& ray = wire_directions[wire_cell];
    const uint16_t cell = tracker_cell(point);
    grid.measurements[cell] = {range * ray.x, range * ray.y, range * ray.z, wire_row};
    grid.presence[cell / 8] |= uint8_t{1} << (cell % 8);
}

}  // namespace ifw192s
