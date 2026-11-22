#include "replay_wire.hpp"
#include "bird_tracker.hpp"

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

namespace bird {
namespace {

constexpr int kRowsPerRead = 256;
constexpr int kRowBytes = 18;
constexpr int kHeaderBytes = 16;

constexpr auto crc_table() {
    std::array<uint32_t, 256> table{};
    for (uint32_t index = 0; index < table.size(); ++index) {
        uint32_t value = index;
        for (int bit = 0; bit < 8; ++bit) {
            value = (value >> 1) ^ ((value & 1) ? 0xedb88320u : 0u);
        }
        table[index] = value;
    }
    return table;
}

uint32_t update_crc(uint32_t crc, const uint8_t* bytes, size_t size) {
    static constexpr auto table = crc_table();
    for (size_t i = 0; i < size; ++i) {
        crc = table[(crc ^ bytes[i]) & 0xffu] ^ (crc >> 8);
    }
    return crc;
}

template <typename T>
T unpack(const uint8_t* bytes) {
    // Both supported build targets are little endian with IEEE-754 floats.
    T value;
    std::memcpy(&value, bytes, sizeof(value));
    return value;
}

template <typename T>
auto buffer(const ReplayIO& io, size_t size) {
    return std::unique_ptr<T, decltype(&std::free)>(
        static_cast<T*>(io.allocate(size * sizeof(T))), &std::free);
}

bool say(const ReplayIO& io, const char* line) {
    return io.write(line, std::strlen(line));
}

bool error(const ReplayIO& io, const char* message) {
    char line[192];
    std::snprintf(line, sizeof(line), "{\"type\":\"error\",\"message\":\"%s\"}\n", message);
    return say(io, line);
}

bool ready(const ReplayIO& io) {
    char line[120];
    std::snprintf(line, sizeof(line), "{\"type\":\"ready\",\"protocol\":1,\"psram_bytes\":%zu}\n",
                  io.psram_bytes);
    return say(io, line);
}

void append_numbers(std::string& json, const Component& comp, const Frame& frame,
                    bool source_rows) {
    char number[16];
    const char* sep = "";
    for (const Cell cell : comp.cells) {
        const uint32_t value = source_rows ? frame.points[cell].source_row : cell;
        std::snprintf(number, sizeof(number), "%s%" PRIu32, sep, value);
        json += number;
        sep = ",";
    }
}

void append_result(std::string& json, const Result& result, const Frame& frame) {
    char part[384];
    std::snprintf(part, sizeof(part),
                  "{\"phase\":\"%s\",\"tentative_count\":%d,\"dynamic_candidates\":%d,"
                  "\"skipped_large_parents\":%d,",
                  result.warming_up ? "WARMUP" : "ACTIVE", result.tentative_count,
                  result.dynamic_candidates, result.skipped_large_parents);
    json += part;
    if (result.warming_up) {
        std::snprintf(part, sizeof(part), "\"warmup_progress\":\"%d/%d\",",
                      result.warmup_progress, kWarmupFrames);
        json += part;
    }
    json += "\"locked_birds\":[";
    const char* sep = "";
    for (const TrackResult& track : result.locked) {
        const auto& comp = track.component;
        std::snprintf(part, sizeof(part),
                      "%s{\"track_id\":%" PRIu32 ",\"state\":\"CANDIDATE_UNVERIFIED\",\"reason\":\"%s\","
                      "\"point_count\":%zu,\"grid_box_gx0_gy0_gx1_gy1\":[%d,%d,%d,%d],"
                      "\"centroid_xyz_m\":[%.17g,%.17g,%.17g],\"range_m\":%.17g,\"max_disp_m\":%.4f,\"cells\":[",
                      sep, track.track_id,
                      track.reason == MatchReason::Dynamic ? "dynamic_foreground" : "retained_locked_bird",
                      comp.cells.size(), comp.box[0], comp.box[1], comp.box[2], comp.box[3],
                      comp.center.x, comp.center.y, comp.center.z, track.range_m, track.max_disp_m);
        json += part;
        append_numbers(json, comp, frame, false);
        json += "],\"source_rows\":[";
        append_numbers(json, comp, frame, true);
        json += "]}";
        sep = ",";
    }
    json += "]}";
}

}  // namespace

void run_replay(const ReplayIO& io) {
    static_assert(sizeof(Vec3) == 3 * sizeof(double));
    static_assert(sizeof(float) == 4 && sizeof(double) == 8);
    auto measurements = buffer<Measurement>(io, kCells);
    auto ranges = buffer<double>(io, kCells);
    auto rays = buffer<Vec3>(io, kCells);
    auto warmup = buffer<double>(io, kWarmupFrames * kCells);
    auto mask = buffer<uint8_t>(io, kCells);
    auto queue = buffer<Cell>(io, kCells);
    if (!measurements || !ranges || !rays || !warmup || !mask || !queue) {
        error(io, "cannot allocate replay buffers (8 MB PSRAM required on ESP32-S3)");
        return;
    }

    const Frame frame{measurements.get(), ranges.get(), rays.get()};
    Tracker tracker(warmup.get(), mask.get(), queue.get());
    Result result;
    uint32_t ordinal = 0;
    bool calibrated = false;
    uint8_t header[kHeaderBytes];
    uint8_t records[kRowsPerRead * kRowBytes];
    std::string json;
    json.reserve(4096);

    while (io.read(header, sizeof(header))) {
        const uint32_t frame_index = unpack<uint32_t>(header + 4);
        const uint32_t count = unpack<uint32_t>(header + 8);
        const uint32_t expected_crc = unpack<uint32_t>(header + 12);
        if (frame_index == 0 && count == 0 && expected_crc == 0) {
            if (std::memcmp(header, "PING", 4) == 0) {
                if (!ready(io)) { return; }
                continue;
            }
            if (std::memcmp(header, "RSET", 4) == 0) {
                ordinal = 0;
                tracker.reset();
                if (!say(io, "{\"type\":\"reset\"}\n")) { return; }
                continue;
            }
        }
        if (std::memcmp(header, "CAL1", 4) == 0 && frame_index == 0 && count == kCells) {
            calibrated = false;
            uint32_t crc = ~0u;
            bool usable = true;
            for (int i = 0; i < kCells; i += kRowsPerRead) {
                auto* output = rays.get() + i;
                const size_t bytes = kRowsPerRead * sizeof(Vec3);
                if (!io.read(output, bytes)) { return; }
                crc = update_crc(crc, reinterpret_cast<const uint8_t*>(output), bytes);
                for (int j = 0; j < kRowsPerRead; ++j) {
                    if (!(std::abs(norm(output[j]) - 1.0) <= 1e-9)) {
                        usable = false;
                    }
                }
            }
            if (~crc != expected_crc || !usable) {
                if (!error(io, "invalid calibration CRC or unit vectors")) { return; }
                continue;
            }
            calibrated = true;
            ordinal = 0;
            tracker.reset();
            if (!say(io, "{\"type\":\"calibrated\"}\n")) { return; }
            continue;
        }
        if (std::memcmp(header, "BIRD", 4) != 0 || count > kCells) {
            if (!error(io, "invalid request header")) { return; }
            continue;
        }

        const int64_t start = io.now_us();
        std::fill_n(ranges.get(), kCells, 0.0);
        int64_t ingest_us = io.now_us() - start;
        uint32_t crc = ~0u;
        int previous = -1;
        bool usable = true;
        for (uint32_t offset = 0; offset < count;) {
            const uint32_t batch = std::min<uint32_t>(kRowsPerRead, count - offset);
            const size_t bytes = batch * kRowBytes;
            if (!io.read(records, bytes)) { return; }
            const int64_t parse_start = io.now_us();
            crc = update_crc(crc, records, bytes);
            for (uint32_t row = 0; row < batch; ++row) {
                const auto* item = records + row * kRowBytes;
                const int cell = unpack<Cell>(item);
                const Measurement measured{
                    unpack<float>(item + 2), unpack<float>(item + 6),
                    unpack<float>(item + 10), unpack<uint32_t>(item + 14),
                };
                const double range = norm({measured.x, measured.y, measured.z});
                if (cell <= previous || cell >= kCells || !std::isfinite(range) || range <= 0.1) {
                    usable = false;
                    continue;
                }
                measurements.get()[cell] = measured;
                ranges.get()[cell] = range;
                previous = cell;
            }
            ingest_us += io.now_us() - parse_start;
            offset += batch;
        }
        if (!calibrated || frame_index != ordinal || ~crc != expected_crc || !usable) {
            if (!error(io, "invalid frame CRC, measurements, ordinal or calibration")) { return; }
            continue;
        }

        const int64_t tracker_start = io.now_us();
        tracker.step(frame, result);
        const int64_t json_start = io.now_us();
        char part[192];
        std::snprintf(part, sizeof(part),
                      "{\"type\":\"frame\",\"ordinal\":%" PRIu32 ",\"tracker_us\":%" PRId64
                      ",\"ingest_us\":%" PRId64 ",\"result\":",
                      ordinal, json_start - tracker_start, ingest_us);
        json.assign(part);
        append_result(json, result, frame);
        std::snprintf(part, sizeof(part), ",\"json_us\":%" PRId64 "}\n", io.now_us() - json_start);
        json += part;
        if (!io.write(json.data(), json.size())) { return; }
        ++ordinal;
    }
}

}  // namespace bird
