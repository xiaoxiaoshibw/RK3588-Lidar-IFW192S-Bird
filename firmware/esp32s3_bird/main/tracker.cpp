#include "bird_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>
#include <utility>

namespace bird {
namespace {

constexpr double kNearM = 1.50;
constexpr double kFarM = 4.50;
constexpr double kBackgroundMarginM = 0.10;
constexpr int kMaxParentCells = 900;
constexpr int kOccluderMinCells = 25;

using Box = std::array<int, 4>;
constexpr Box kFullGrid = {0, 0, kWidth - 1, kHeight - 1};

struct Track {
    uint32_t id;
    Vec3 init_center;
    Vec3 last_center;
    Box last_box;
    double last_range_m;
    int hits = 1;
    int missed = 0;
    double max_disp_m = 0;
    bool locked = false;
};

struct Match {
    Component* component = nullptr;
    MatchReason reason = MatchReason::Dynamic;
};

Box native_window(const Box& display, int pad) {
    return {
        std::max(0, kWidth - 1 - display[2] - pad),
        std::max(0, kHeight - 1 - display[3] - pad),
        std::min(kWidth - 1, kWidth - 1 - display[0] + pad),
        std::min(kHeight - 1, kHeight - 1 - display[1] + pad),
    };
}

template<class Operation>
void for_cells(const Box& box, Operation operation) {
    for (int y = box[1]; y <= box[3]; ++y) {
        for (int x = box[0]; x <= box[2]; ++x) {
            operation(static_cast<Cell>(y * kWidth + x));
        }
    }
}

}  // namespace

struct Tracker::Impl {
    double* warmup_background;
    uint8_t* mask;
    Cell* queue;
    std::vector<Track> tracks;
    std::vector<Cell> parent;
    uint32_t next_id = 1;
    int warmup_count = 0;

    void finish_warmup() {
        for (int cell = 0; cell < kCells; ++cell) {
            double values[kWarmupFrames];
            int count = 0;
            for (int frame = 0; frame < kWarmupFrames; ++frame) {
                const double range = warmup_background[frame * kCells + cell];
                if (range > 0 && std::isfinite(range)) {
                    int insert = count++;
                    while (insert > 0 && range < values[insert - 1]) {
                        values[insert] = values[insert - 1];
                        --insert;
                    }
                    values[insert] = range;
                }
            }
            double background = 0;
            if (count >= kWarmupFrames / 3) {
                const double p10 = values[count / 10];
                const double p90 = values[std::min(count - 1, count * 9 / 10)];
                background = p90 - p10 > 0.15 ? values[count - 1] : values[(count - 1) / 2];
            }
            warmup_background[cell] = background;
        }
    }

    int flood(const Frame& frame, Cell seed, bool use_xyz) {
        queue[0] = seed;
        mask[seed] = 0;
        int count = 1;
        for (int cursor = 0; cursor < count; ++cursor) {
            const Cell cell = queue[cursor];
            const int y = cell / kWidth;
            const int x = cell % kWidth;
            const Vec3 origin = use_xyz ? point(frame, cell) : Vec3{};
            for (int ny = std::max(0, y - 1); ny <= std::min(kHeight - 1, y + 1); ++ny) {
                for (int nx = std::max(0, x - 1); nx <= std::min(kWidth - 1, x + 1); ++nx) {
                    const Cell neighbor = ny * kWidth + nx;
                    if (mask[neighbor] && (!use_xyz || distance(origin, point(frame, neighbor)) <= 0.08)) {
                        mask[neighbor] = 0;
                        queue[count++] = neighbor;
                    }
                }
            }
            if (!use_xyz && count >= kOccluderMinCells) {
                break;
            }
        }
        return count;
    }

    void extract(const Frame& frame, const Box& seeds, bool seeded,
                 int& skipped, std::vector<Component>& out) {
        struct Group {
            Cell first_cell;
            size_t begin;
            size_t end;
        };
        std::vector<Group> groups;
        for_cells(seeds, [&](Cell seed) {
            if (!mask[seed]) {
                return;
            }
            const int count = flood(frame, seed, true);
            if (count > kMaxParentCells) {
                ++skipped;
            } else if (count >= 12) {
                parent.assign(queue, queue + count);
                std::sort(parent.begin(), parent.end());
                const size_t begin = out.size();
                segment_bodies(frame, parent, out);
                if (seeded && begin != out.size()) {
                    groups.push_back({parent.front(), begin, out.size()});
                }
            }
        });
        // A parent reached from a core seed can start before that seed in native cell order.
        if (groups.size() > 1) {
            std::sort(groups.begin(), groups.end(), [](const Group& a, const Group& b) {
                return a.first_cell < b.first_cell;
            });
            std::vector<Component> ordered;
            ordered.reserve(out.size());
            for (const Group& group : groups) {
                for (size_t i = group.begin; i < group.end; ++i) {
                    ordered.push_back(std::move(out[i]));
                }
            }
            out = std::move(ordered);
        }
    }

    bool retain(const Frame& frame, const Track& track, Component& retained) {
        const Box core = native_window(track.last_box, 8);
        const Box context = native_window(track.last_box, 16);
        const double min_range = std::max(kNearM, track.last_range_m - 0.30);
        const double max_range = std::min(kFarM, track.last_range_m + 0.35);
        std::memset(mask, 0, kCells);
        for_cells(core, [&](Cell cell) {
            const double range = frame.ranges[cell];
            mask[cell] = range > 0 && range < min_range;
        });
        bool occluded = false;
        for_cells(core, [&](Cell cell) {
            if (!occluded && mask[cell] && flood(frame, cell, false) >= kOccluderMinCells) {
                occluded = true;
            }
        });
        if (occluded) {
            return false;
        }

        for_cells(context, [&](Cell cell) {
            const double range = frame.ranges[cell];
            mask[cell] = min_range <= range && range <= max_range;
        });
        std::vector<Component> components;
        int skipped = 0;
        extract(frame, core, true, skipped, components);
        Component* best = nullptr;
        double best_distance = 0.14;
        for (Component& comp : components) {
            const double separation = distance(comp.center, track.last_center);
            if (separation <= best_distance) {
                best = &comp;
                best_distance = separation;
            }
        }
        if (best) {
            retained = std::move(*best);
        }
        return best != nullptr;
    }
};

Tracker::Tracker(double* warmup_background, uint8_t* mask, Cell* queue)
    : impl_(new Impl{warmup_background, mask, queue, {}, {}}) {
    impl_->parent.reserve(kMaxParentCells);
}

Tracker::~Tracker() {
    delete impl_;
}

void Tracker::reset() {
    impl_->tracks.clear();
    impl_->next_id = 1;
    impl_->warmup_count = 0;
}

void Tracker::step(const Frame& frame, Result& result) {
    auto& state = *impl_;
    result.locked.clear();
    result.tentative_count = 0;
    result.dynamic_candidates = 0;
    result.skipped_large_parents = 0;
    result.warming_up = state.warmup_count < kWarmupFrames;
    result.warmup_progress = 0;
    if (result.warming_up) {
        std::memcpy(state.warmup_background + kCells * state.warmup_count,
                    frame.ranges, kCells * sizeof(double));
        if (++state.warmup_count == kWarmupFrames) {
            state.finish_warmup();
        } else {
            result.warmup_progress = state.warmup_count;
        }
        return;
    }

    for (int cell = 0; cell < kCells; ++cell) {
        const double range = frame.ranges[cell];
        state.mask[cell] = kNearM <= range && range <= kFarM &&
                           (state.warmup_background[cell] == 0 ||
                            range < state.warmup_background[cell] - kBackgroundMarginM);
    }
    std::vector<Component> dynamic;
    state.extract(frame, kFullGrid, false, result.skipped_large_parents, dynamic);
    result.dynamic_candidates = static_cast<int>(dynamic.size());

    auto& tracks = state.tracks;
    std::vector<size_t> ordered(tracks.size());
    std::iota(ordered.begin(), ordered.end(), 0);
    std::sort(ordered.begin(), ordered.end(), [&](size_t i, size_t j) {
        const Track& a = tracks[i];
        const Track& b = tracks[j];
        if (a.locked != b.locked) {
            return a.locked;
        }
        if (a.hits != b.hits) {
            return a.hits > b.hits;
        }
        return a.id < b.id;
    });

    std::vector<uint8_t> used(dynamic.size());
    std::vector<Match> matched(tracks.size());
    std::vector<Vec3> claimed;
    for (size_t track_idx : ordered) {
        const Track& track = tracks[track_idx];
        size_t best_idx = dynamic.size();
        double best_dist = 0.18;
        for (size_t i = 0; i < dynamic.size(); ++i) {
            if (!used[i]) {
                const double separation = distance(dynamic[i].center, track.last_center);
                if (separation <= best_dist) {
                    best_dist = separation;
                    best_idx = i;
                }
            }
        }
        if (best_idx != dynamic.size()) {
            used[best_idx] = 1;
            matched[track_idx] = {&dynamic[best_idx], MatchReason::Dynamic};
            claimed.push_back(dynamic[best_idx].center);
        }
    }

    std::vector<Component> retained;
    retained.reserve(tracks.size());
    for (size_t track_idx : ordered) {
        const Track& track = tracks[track_idx];
        if (!track.locked || matched[track_idx].component) {
            continue;
        }
        Component comp;
        if (state.retain(frame, track, comp) &&
            std::none_of(claimed.begin(), claimed.end(), [&](Vec3 center) {
                return distance(comp.center, center) < 0.16;
            })) {
            claimed.push_back(comp.center);
            retained.push_back(std::move(comp));
            matched[track_idx] = {&retained.back(), MatchReason::Retained};
        }
    }

    for (size_t i = 0; i < tracks.size(); ++i) {
        Track& track = tracks[i];
        const Match& match = matched[i];
        if (!match.component) {
            ++track.missed;
            continue;
        }
        Component& comp = *match.component;
        track.max_disp_m = std::max(track.max_disp_m, distance(comp.center, track.init_center));
        track.last_center = comp.center;
        track.last_box = comp.box;
        track.last_range_m = distance(comp.center, {});
        ++track.hits;
        track.missed = 0;
        if (!track.locked && result.skipped_large_parents == 0 &&
            track.hits >= 5 && track.max_disp_m >= 0.045 &&
            std::none_of(tracks.begin(), tracks.end(), [&](const Track& other) {
                return other.locked && other.id != track.id &&
                       distance(track.last_center, other.last_center) < 0.22;
            })) {
            track.locked = true;
        }
        if (track.locked) {
            result.locked.push_back({track.id, match.reason, std::move(comp),
                                     track.last_range_m, track.max_disp_m});
        }
    }

    tracks.erase(std::remove_if(tracks.begin(), tracks.end(), [](const Track& track) {
        return track.missed > (track.locked ? 25 : 3);
    }), tracks.end());
    std::sort(tracks.begin(), tracks.end(), [](const Track& a, const Track& b) {
        return a.locked != b.locked ? a.locked : a.id < b.id;
    });
    auto nearby_locked = [](const auto begin, const auto end, Vec3 center) {
        return std::any_of(begin, end, [center](const Track& other) {
            return other.locked && distance(center, other.last_center) < 0.20;
        });
    };
    size_t kept = 0;
    for (const Track& track : tracks) {
        if (track.locked && nearby_locked(tracks.begin(), tracks.begin() + kept, track.last_center)) {
            result.locked.erase(std::remove_if(result.locked.begin(), result.locked.end(), [&](const auto& item) {
                return item.track_id == track.id;
            }), result.locked.end());
        } else {
            tracks[kept++] = track;
        }
    }
    tracks.resize(kept);

    for (size_t i = 0; i < dynamic.size(); ++i) {
        const Component& comp = dynamic[i];
        if (!used[i] && !nearby_locked(tracks.begin(), tracks.end(), comp.center)) {
            tracks.push_back({state.next_id++, comp.center, comp.center, comp.box, distance(comp.center, {})});
        }
    }
    result.tentative_count = static_cast<int>(std::count_if(tracks.begin(), tracks.end(), [](const Track& track) {
        return !track.locked;
    }));
    std::sort(result.locked.begin(), result.locked.end(), [](const auto& a, const auto& b) {
        return a.track_id < b.track_id;
    });
}

}  // namespace bird
