#include "grid_algorithm.hpp"

#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#ifdef _WIN32
#define BIRD_API extern "C" __declspec(dllexport)
#else
#define BIRD_API extern "C" __attribute__((visibility("default")))
#endif

namespace {

struct BirdTrackC {
    uint32_t track_id;
    uint32_t reason;  // 0 = Dynamic, 1 = Retained
    int32_t box[4];
    double center[3];
    double range_m;
    double max_disp_m;
    uint32_t point_count;
    uint32_t cell_offset;
};

struct BirdStepResultC {
    uint32_t ok;
    uint32_t warming_up;
    int32_t warmup_progress;
    int32_t dynamic_candidates;
    int32_t skipped_large_parents;
    int32_t tentative_count;
    uint32_t locked_count;
    uint32_t total_track_cells;
};

static_assert(sizeof(bird::Measurement) == 16, "Measurement ABI must match <fffI");
static_assert(sizeof(bird::Vec3) == 24, "Vec3 ABI must match <ddd");
static_assert(sizeof(BirdTrackC) == 72, "BirdTrackC ABI size must be 72 bytes");
static_assert(sizeof(BirdStepResultC) == 32, "BirdStepResultC ABI size must be 32 bytes");

struct NativeTrackerHandle {
    std::vector<double> background;
    std::vector<double> exact_ranges;
    std::vector<float> squared_ranges;
    std::vector<float> foreground_limit;
    std::vector<uint8_t> mask;
    std::vector<bird::Cell> queue;
    std::vector<uint32_t> range_cached;
    std::unique_ptr<bird::GridTracker> tracker;

    bird::Result last_result;
    std::vector<BirdTrackC> last_tracks;
    std::vector<uint16_t> last_cells;
    std::vector<uint32_t> last_source_rows;

    NativeTrackerHandle()
        : background(static_cast<size_t>(bird::kWarmupFrames) * bird::kCells, 0.0),
          exact_ranges(bird::kCells, 0.0),
          squared_ranges(bird::kCells, 0.0f),
          foreground_limit(bird::kCells, 0.0f),
          mask(bird::kCells, 0),
          queue(bird::kCells, 0),
          range_cached(bird::kCells / 32, 0) {
        bird::GridWorkspace ws{
            background.data(),
            exact_ranges.data(),
            squared_ranges.data(),
            foreground_limit.data(),
            mask.data(),
            queue.data(),
            range_cached.data(),
        };
        tracker = std::make_unique<bird::GridTracker>(ws);
    }
};

}  // namespace

BIRD_API void* bird_tracker_create() {
    try {
        return new NativeTrackerHandle();
    } catch (...) {
        return nullptr;
    }
}

BIRD_API void bird_tracker_destroy(void* raw_handle) {
    delete static_cast<NativeTrackerHandle*>(raw_handle);
}

BIRD_API void bird_tracker_reset(void* raw_handle) {
    if (!raw_handle) { return; }
    auto* handle = static_cast<NativeTrackerHandle*>(raw_handle);
    handle->tracker->reset();
    handle->last_tracks.clear();
    handle->last_cells.clear();
    handle->last_source_rows.clear();
}

BIRD_API void bird_tracker_clear_tracks(void* raw_handle) {
    if (!raw_handle) { return; }
    auto* handle = static_cast<NativeTrackerHandle*>(raw_handle);
    handle->tracker->clear_tracks();
    handle->last_tracks.clear();
    handle->last_cells.clear();
    handle->last_source_rows.clear();
}

BIRD_API int bird_tracker_step(void* raw_handle,
                               const bird::Measurement* points,
                               const uint8_t* presence,
                               const bird::Vec3* unit_rays,
                               BirdStepResultC* out_result) {
    if (!raw_handle || !points || !presence || !unit_rays || !out_result) {
        return 0;
    }
    auto* handle = static_cast<NativeTrackerHandle*>(raw_handle);
    std::memset(out_result, 0, sizeof(*out_result));
    handle->last_tracks.clear();
    handle->last_cells.clear();
    handle->last_source_rows.clear();

    const bool ok = handle->tracker->step(points, presence, unit_rays, handle->last_result);
    if (!ok) {
        return 0;
    }

    const auto& res = handle->last_result;
    out_result->ok = 1;
    out_result->warming_up = res.warming_up ? 1u : 0u;
    out_result->warmup_progress = res.warmup_progress;
    out_result->dynamic_candidates = res.dynamic_candidates;
    out_result->skipped_large_parents = res.skipped_large_parents;
    out_result->tentative_count = res.tentative_count;
    out_result->locked_count = static_cast<uint32_t>(res.locked.size());

    handle->last_tracks.reserve(res.locked.size());
    for (const auto& track : res.locked) {
        BirdTrackC item{};
        item.track_id = track.track_id;
        item.reason = (track.reason == bird::MatchReason::Retained) ? 1u : 0u;
        for (int i = 0; i < 4; ++i) {
            item.box[i] = track.component.box[i];
        }
        item.center[0] = track.component.center.x;
        item.center[1] = track.component.center.y;
        item.center[2] = track.component.center.z;
        item.range_m = track.range_m;
        item.max_disp_m = track.max_disp_m;
        item.point_count = static_cast<uint32_t>(track.component.cells.size());
        item.cell_offset = static_cast<uint32_t>(handle->last_cells.size());
        handle->last_tracks.push_back(item);

        for (bird::Cell cell : track.component.cells) {
            handle->last_cells.push_back(cell);
            handle->last_source_rows.push_back(points[cell].source_row);
        }
    }
    out_result->total_track_cells = static_cast<uint32_t>(handle->last_cells.size());
    return 1;
}

BIRD_API uint32_t bird_tracker_copy_tracks(void* raw_handle,
                                           BirdTrackC* out_tracks,
                                           uint32_t max_tracks,
                                           uint16_t* out_cells,
                                           uint32_t* out_source_rows,
                                           uint32_t max_cells) {
    if (!raw_handle) { return 0; }
    const auto* handle = static_cast<const NativeTrackerHandle*>(raw_handle);
    const uint32_t track_count = static_cast<uint32_t>(handle->last_tracks.size());
    if (out_tracks && max_tracks >= track_count && track_count > 0) {
        std::memcpy(out_tracks, handle->last_tracks.data(), track_count * sizeof(BirdTrackC));
    }
    const uint32_t cell_count = static_cast<uint32_t>(handle->last_cells.size());
    if (out_cells && out_source_rows && max_cells >= cell_count && cell_count > 0) {
        std::memcpy(out_cells, handle->last_cells.data(), cell_count * sizeof(uint16_t));
        std::memcpy(out_source_rows, handle->last_source_rows.data(), cell_count * sizeof(uint32_t));
    }
    return track_count;
}
