#include <algorithm>
#include <array>
#include <atomic>
#include <cinttypes>
#include <cstdio>
#include <cstring>

#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "grid_input.hpp"
#include "w5500.hpp"

namespace {

using namespace ifw192s;
constexpr int kFrameBuffers = 2;
constexpr auto kPsram = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
constexpr auto kInternal = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
constexpr size_t kCalBytes = sizeof(RangeCalibration) + kFrameRecords * 2 * sizeof(double);
constexpr uint32_t kMaxReplayPackets = 200'000;

struct TaggedGrid {
    int slot;
    FrameBoundary boundary;
};

struct PackedPoint {
    uint16_t source_row;
    uint16_t width;
    uint16_t range_low;
    uint16_t area_low;
    uint16_t high_bytes;

    static PackedPoint pack(const WireRecord& point, uint16_t row) {
        return {row, point.pulse_width, static_cast<uint16_t>(point.range_ticks),
                static_cast<uint16_t>(point.pulse_area),
                static_cast<uint16_t>((point.range_ticks >> 16) | ((point.pulse_area >> 16) << 8))};
    }

    WireRecord unpack(uint16_t cell) const {
        return {static_cast<uint8_t>(cell), static_cast<uint8_t>(cell >> 8), width,
                range_low | (uint32_t{high_bytes & 0xffu} << 16),
                area_low | ((uint32_t{high_bytes} & 0xff00u) << 8)};
    }
};
static_assert(sizeof(PackedPoint) == 10);

struct LiveState {
    QueueHandle_t free_slots;
    QueueHandle_t ready;
    TaskHandle_t owner;
    PackedPoint* input_frames[kFrameBuffers];
    GridInput working_grid;
    RangeCalibration calibration;
    Direction* measured_rays;
    bird::Vec3* tracking_rays;
    bird::GridWorkspace workspace;
    std::atomic<bool> settling_background{false};
    std::atomic<bool> background_ready{false};
    bool live;
};

LiveState state{};

template <class T>
T* allocate(size_t count, bool prefer_internal = false) {
    void* out = prefer_internal ? heap_caps_malloc(sizeof(T) * count, kInternal) : nullptr;
    if (!out) { out = heap_caps_malloc(sizeof(T) * count, kPsram); }
    return static_cast<T*>(out);
}

bool read_exact(void* buffer, size_t size) {
    auto* next = static_cast<uint8_t*>(buffer);
    while (size) {
        const auto got = uart_read_bytes(UART_NUM_0, next, size, pdMS_TO_TICKS(8000));
        if (got <= 0) { return false; }
        size -= got;
        next += got;
    }
    return true;
}

void uart_start() {
    uart_config_t config{};
    config.baud_rate = 2'000'000;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 16 * 1024, 0, 0, nullptr, 0));
    setvbuf(stdout, nullptr, _IONBF, 0);
}

bool init_storage() {
    for (auto& buffer : state.input_frames) {
        buffer = allocate<PackedPoint>(kFrameRecords);
        if (!buffer) { return false; }
    }
    state.working_grid = {allocate<bird::Measurement>(kFrameRecords), allocate<uint8_t>(bird::kPresenceBytes, true)};
    if (!state.working_grid.measurements || !state.working_grid.presence) { return false; }
    state.workspace = {
        allocate<double>(bird::kWarmupFrames * kFrameRecords), allocate<double>(kFrameRecords),
        allocate<float>(kFrameRecords, true), allocate<float>(kFrameRecords),
        allocate<uint8_t>(kFrameRecords, true), allocate<bird::Cell>(kFrameRecords),
        allocate<uint32_t>(kFrameRecords / 32, true),
    };
    const auto& w = state.workspace;
    return w.background && w.exact_ranges && w.squared_ranges && w.foreground_limit &&
           w.mask && w.queue && w.range_cached;
}

void prepare_rays(const uint8_t* calibration) {
    std::memcpy(&state.calibration, calibration, sizeof(state.calibration));
    constexpr double radians = 0.017453292519943295;
    const uint8_t* cursor = calibration + sizeof(state.calibration);
    for (uint32_t cell = 0; cell < kFrameRecords; ++cell) {
        double az, el;
        std::memcpy(&az, cursor, sizeof(az));
        std::memcpy(&el, cursor + sizeof(az), sizeof(el));
        cursor += sizeof(az) + sizeof(el);
        state.measured_rays[cell] = ray_from_calibration(static_cast<float>(az), static_cast<float>(el));
        const double a = az * radians;
        const double e = el * radians;
        const double ce = std::cos(e);
        state.tracking_rays[cell] = {ce * std::cos(a), ce * std::sin(a), std::sin(e)};
    }
}

bool wait_for_calibration() {
    uint8_t header[16];
    if (!read_exact(header, sizeof(header)) || std::memcmp(header, "CAL2", 4) != 0 ||
        ifw192s::read_u32(header + 4) != 0 || ifw192s::read_u32(header + 8) != kCalBytes) {
        return false;
    }
    auto* data = allocate<uint8_t>(kCalBytes);
    if (!data) { return false; }
    const bool okay = read_exact(data, kCalBytes) &&
        esp_rom_crc32_le(0, data, kCalBytes) == ifw192s::read_u32(header + 12);
    if (okay) { prepare_rays(data); }
    heap_caps_free(data);
    return okay;
}

void materialize_grid(const PackedPoint* frame, bool active) {
    const auto grid = state.working_grid;
    std::fill_n(grid.presence, bird::kPresenceBytes, 0);
    // Keep the near side of the active cutoff even when single precision rounds up.
    const float max_range = state.live && active ? static_cast<float>(bird::kTrackingFarM) + 0.00002f
        : std::numeric_limits<float>::infinity();
    for (uint16_t cell = 0; cell < kFrameRecords; ++cell) {
        const auto& point = frame[cell];
        place_measurement(point.unpack(cell), point.source_row, state.measured_rays, state.calibration,
                          grid, state.live, max_range);
    }
}

void materialize_active_grid(const PackedPoint* frame,
                             const std::vector<std::array<int, 4>>& retain_boxes) {
    constexpr float kNearSqLo = 2.25f - 0.001f;
    constexpr float kLimitMargin = 0.001f;
    constexpr float kMaxRange = static_cast<float>(bird::kTrackingFarM) + 0.00002f;
    auto* measurements = state.working_grid.measurements;
    auto* squared = state.workspace.squared_ranges;
    const auto* limit = state.workspace.foreground_limit;
    const auto* rays = state.measured_rays;
    const auto cal = state.calibration;
    std::fill_n(state.workspace.range_cached, bird::kCells / 32, uint32_t{0});

    for (int ring = 0; ring < kRings; ++ring) {
        const uint32_t row_base = static_cast<uint32_t>(ring) << 8;
        const PackedPoint* row_points = frame + row_base;
        const Direction* row_rays = rays + row_base;
        bool ring_in_box = false;
        for (const auto& box : retain_boxes) {
            if (ring >= box[1] && ring <= box[3]) {
                ring_in_box = true;
                break;
            }
        }
        for (int col = 0; col < kColumns; ++col) {
            const uint32_t tcell = row_base | static_cast<uint32_t>(255 - col);
            const auto& pt = row_points[col];
            if (pt.width == 0) {
                squared[tcell] = 0.0f;
                continue;
            }
            const uint32_t wire_cell = row_base | static_cast<uint32_t>(col);
            const float range = live_measured_range(pt.unpack(static_cast<uint16_t>(wire_cell)), cal);
            if (range == 0.0f || range > kMaxRange) {
                squared[tcell] = 0.0f;
                continue;
            }
            const float r2 = range * range;
            bool need_xyz = (r2 >= kNearSqLo && r2 <= limit[tcell] + kLimitMargin);
            if (!need_xyz && ring_in_box) {
                const int tx = 255 - col;
                for (const auto& box : retain_boxes) {
                    if (ring >= box[1] && ring <= box[3] && tx >= box[0] && tx <= box[2]) {
                        need_xyz = true;
                        break;
                    }
                }
            }
            if (need_xyz) {
                const auto& ray = row_rays[col];
                const float x = range * ray.x;
                const float y = range * ray.y;
                const float z = range * ray.z;
                measurements[tcell] = {x, y, z, pt.source_row};
                squared[tcell] = bird::squared_norm(x, y, z);
            } else {
                squared[tcell] = r2;
            }
        }
    }
}

void track_frames() {
    bird::GridTracker tracker(state.workspace);
    bird::Result result;
    std::vector<std::array<int, 4>> retain_boxes;
    uint32_t last_sequence = 0;
    bool have_last = false;
    bool first_after_warmup = false;
    for (;;) {
        TaggedGrid input{};
        xQueueReceive(state.ready, &input, portMAX_DELAY);
        if (input.slot < 0) {
            return;
        }
        const auto& frame = input.boundary;
        const auto grid = state.working_grid;
        const bool discontinuity = have_last && frame.final_sequence - last_sequence != 768;
        if (state.live && (discontinuity || !frame.complete)) {
            tracker.clear_tracks();
        } else if (!state.live &&
                   ((discontinuity && !tracker.warming_up() && !first_after_warmup) || !frame.complete)) {
            tracker.reset();
            result.warming_up = true;
            result.warmup_progress = 0;
        }
        first_after_warmup = false;
        have_last = true;
        last_sequence = frame.final_sequence;
        const bool settling = frame.complete && tracker.warming_up() && result.warmup_progress == 9;
        if (settling) { state.settling_background.store(true); }
        const bool use_fast_active = state.live && !tracker.warming_up();
        if (frame.complete) {
            if (use_fast_active) {
                tracker.locked_context_boxes(retain_boxes);
                materialize_active_grid(state.input_frames[input.slot], retain_boxes);
            } else {
                materialize_grid(state.input_frames[input.slot], !tracker.warming_up());
            }
        }
        const auto grid_ready = esp_timer_get_time();
        xQueueSend(state.free_slots, &input.slot, 0);
        const auto start = esp_timer_get_time();
        const bool valid = frame.complete && (use_fast_active
            ? tracker.step_precomputed(grid.measurements, state.tracking_rays, result)
            : tracker.step(grid.measurements, grid.presence, state.tracking_rays, result));
        const auto ready = esp_timer_get_time();
        if (settling) {
            first_after_warmup = valid && !tracker.warming_up();
            state.background_ready.store(first_after_warmup);
            state.settling_background.store(false);
        }
        if (!valid) {
            if (state.live) { tracker.clear_tracks(); }
            std::printf("{\"type\":\"frame\",\"seq\":%" PRIu32 ",\"state\":\"UNKNOWN\",\"records\":%" PRIu32
                        "}\n", frame.final_sequence, frame.records);
            continue;
        }
        std::printf("{\"type\":\"frame\",\"seq\":%" PRIu32 ",\"state\":\"%s\",\"warmup\":%d,\"locked\":%zu,"
                    "\"candidates\":%d,\"tentative\":%d,"
                    "\"tail_read_us\":%" PRIu64 ",\"grid_ready_us\":%" PRId64 ",\"tracker_start_us\":%" PRId64
                    ",\"result_ready_us\":%" PRId64 "}\n",
                    frame.final_sequence, result.warming_up ? "warmup" : "active", result.warmup_progress,
                    result.locked.size(), result.dynamic_candidates, result.tentative_count,
                    frame.final_packet_read_us, grid_ready, start, ready);
        for (const auto& track : result.locked) {
            const auto& object = track.component;
            std::printf("{\"type\":\"track\",\"seq\":%" PRIu32 ",\"id\":%" PRIu32
                        ",\"box\":[%d,%d,%d,%d],\"xyz_m\":[%.6f,%.6f,%.6f],\"points\":%zu}\n",
                        frame.final_sequence, track.track_id, object.box[0], object.box[1], object.box[2], object.box[3],
                        object.center.x, object.center.y, object.center.z, object.cells.size());
        }
    }
}

void worker(void*) {
    track_frames();
    xTaskNotifyGive(state.owner);
    vTaskDelete(nullptr);
}

struct FrameSink {
    int slot = -1;
    uint32_t no_buffer = 0;
    uint32_t queue_full = 0;
    uint32_t complete = 0;
    uint32_t unknown = 0;
    uint32_t skipped_during_background = 0;
    uint32_t active_no_buffer = 0;
    uint32_t active_unknown = 0;
    bool suspend_frame = false;
    bool active_frame = false;

    void begin_frame() {
        slot = -1;
        suspend_frame = state.settling_background.load();
        active_frame = state.live && state.background_ready.load();
        if (suspend_frame) { return; }
        xQueueReceive(state.free_slots, &slot, 0);
    }
    void record(const WireRecord& point, uint32_t row) const {
        if (slot >= 0) {
            const uint16_t cell = (uint16_t{point.ring} << 8) | point.column;
            state.input_frames[slot][cell] = PackedPoint::pack(point, static_cast<uint16_t>(row));
        }
    }
    void end_frame(const FrameBoundary& frame) {
        complete += frame.complete;
        unknown += !frame.complete;
        active_unknown += active_frame && !frame.complete;
        if (suspend_frame || state.settling_background.load()) {
            ++skipped_during_background;
            discard_partial();
            return;
        }
        if (slot < 0) {
            ++no_buffer;
            active_no_buffer += active_frame;
            return;
        }
        const TaggedGrid ready{slot, frame};
        if (!xQueueSend(state.ready, &ready, 0)) {
            xQueueSend(state.free_slots, &slot, 0);
            ++queue_full;
        }
        slot = -1;
    }
    void discard_partial() {
        if (slot >= 0) { xQueueSend(state.free_slots, &slot, 0); }
        slot = -1;
    }
};

void usb_replay(WireAssembler& wire, FrameSink& sink, uint32_t count) {
    uint8_t payload[kPacketBytes];
    for (uint32_t i = 0; i < count; ++i) {
        if (!read_exact(payload, sizeof(payload))) { break; }
        wire.push(payload, sizeof(payload), esp_timer_get_time(), sink);
    }
}

void ethernet(WireAssembler& wire, FrameSink& sink, uint32_t hz) {
    W5500 nic;
    nic.init();
    if (nic.version() != 4 || !nic.configure(W5500::kIsolatedMac)) {
        nic.reset();
        std::puts("{\"type\":\"error\",\"message\":\"W5500 init failed\"}");
        return;
    }
    const int64_t wait_link_until = esp_timer_get_time() + 8'000'000;
    while (!(nic.phy() & 1) && esp_timer_get_time() < wait_link_until) {
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    if (!(nic.phy() & 1) || !nic.open_udp()) {
        nic.reset();
        std::puts("{\"type\":\"error\",\"message\":\"W5500 link or socket failed\"}");
        return;
    }
    if (hz == 80'000'000) {
        nic.drive(GPIO_DRIVE_CAP_0);
        nic.read_dummy_bits(1);
    }
    if (nic.version(hz) != 4) {
        nic.reset();
        std::puts("{\"type\":\"error\",\"message\":\"W5500 SPI clock failed\"}");
        return;
    }
    uint8_t* buffers[2]{};
    for (auto& buffer : buffers) {
        buffer = static_cast<uint8_t*>(heap_caps_malloc(W5500::kBufferBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    }
    if (!buffers[0] || !buffers[1]) {
        heap_caps_free(buffers[0]);
        heap_caps_free(buffers[1]);
        nic.reset();
        std::puts("{\"type\":\"error\",\"message\":\"SPI DMA buffer allocation failed\"}");
        return;
    }
    const int64_t capture_start = esp_timer_get_time();
    const int64_t deadline = INT64_MAX;
    uint32_t burst_errors = 0;
    const auto process_batch = [&](const uint8_t* batch, size_t available, uint64_t batch_read_us) {
        size_t cursor = 0;
        while (available - cursor >= 8) {
            const auto* p = batch + cursor;
            const size_t length = uint16_t{p[6]} << 8 | p[7];
            if (length > available - cursor - 8) { break; }
            cursor += 8 + length;
            const uint8_t sensor[] = {192, 168, 1, 10, 0x1b, 0x58};
            if (std::memcmp(p, sensor, sizeof(sensor)) == 0) {
                wire.push(p + 8, length, batch_read_us, sink);
            }
        }
        if (cursor != available) {
            ++burst_errors;
            sink.discard_partial();
            wire.reset_alignment();
        }
    };
    unsigned next = 0;
    size_t previous_size = 0;
    uint64_t previous_read_us = 0;
    nic.acquire();
    while (esp_timer_get_time() < deadline) {
        const size_t size = nic.pending_ifw192s();
        if (!size) {
            if (previous_size) {
                process_batch(buffers[next ^ 1], previous_size, previous_read_us);
                previous_size = 0;
            }
            continue;
        }
        nic.start_pop(buffers[next], size);
        if (previous_size) {
            process_batch(buffers[next ^ 1], previous_size, previous_read_us);
        }
        previous_read_us = nic.finish_pop();
        previous_size = size;
        next ^= 1;
    }
    nic.release();
    const auto capture_us = esp_timer_get_time() - capture_start;
    if (previous_size) {
        process_batch(buffers[next ^ 1], previous_size, previous_read_us);
    }
    heap_caps_free(buffers[0]);
    heap_caps_free(buffers[1]);
    nic.read_dummy_bits(0);
    nic.version(W5500::kBaseHz);
    nic.reset();
    std::printf("{\"type\":\"ethernet_done\",\"burst_errors\":%" PRIu32 ",\"capture_us\":%" PRId64 "}\n",
                burst_errors, capture_us);
}

}  // namespace

extern "C" void app_main() {
    uart_start();
    state.measured_rays = allocate<Direction>(kFrameRecords);
    state.tracking_rays = allocate<bird::Vec3>(kFrameRecords);
    if (!state.measured_rays || !state.tracking_rays) {
        std::puts("{\"type\":\"error\",\"message\":\"Cannot allocate calibration rays\"}");
        return;
    }
    std::puts("\n{\"type\":\"ready\",\"protocol\":2,\"profile\":\"geometry_only\"}");
    if (!wait_for_calibration() || !init_storage()) {
        std::puts("{\"type\":\"error\",\"message\":\"Calibration or frame storage failed\"}");
        return;
    }
    state.free_slots = xQueueCreate(kFrameBuffers, sizeof(int));
    state.ready = xQueueCreate(1, sizeof(TaggedGrid));
    state.owner = xTaskGetCurrentTaskHandle();
    if (!state.free_slots || !state.ready) {
        std::puts("{\"type\":\"error\",\"message\":\"Frame queue allocation failed\"}");
        return;
    }
    for (int slot = 0; slot < kFrameBuffers; ++slot) { xQueueSend(state.free_slots, &slot, 0); }
    std::printf("{\"type\":\"calibrated\",\"internal_free\":%zu,\"psram_free\":%zu}\n",
                heap_caps_get_free_size(kInternal), heap_caps_get_free_size(kPsram));
    uint8_t command[8];
    const bool have_command = read_exact(command, sizeof(command));
    const auto argument = have_command ? ifw192s::read_u32(command + 4) : 0;
    const bool from_ethernet = have_command && std::memcmp(command, "UDP2", 4) == 0;
    state.live = from_ethernet;
    if (xTaskCreatePinnedToCore(worker, "bird_tracker", 8192, nullptr, 4, nullptr, 1) != pdPASS) {
        std::puts("{\"type\":\"error\",\"message\":\"Cannot start tracker task\"}");
        return;
    }
    WireAssembler wire(from_ethernet ? PacketLayout::detect : PacketLayout::aligned);
    FrameSink sink;
    if (have_command && std::memcmp(command, "USB2", 4) == 0 && argument > 0 && argument <= kMaxReplayPackets) {
        usb_replay(wire, sink, argument);
    } else if (from_ethernet &&
               (argument == 40'000'000 || argument == 80'000'000)) {
        ethernet(wire, sink, argument);
    } else {
        std::puts("{\"type\":\"error\",\"message\":\"Invalid run command\"}");
    }
    sink.discard_partial();
    const TaggedGrid finished{-1, {}};
    xQueueSend(state.ready, &finished, portMAX_DELAY);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    std::printf("{\"type\":\"summary\",\"packets\":%" PRIu64 ",\"missing\":%" PRIu64 ","
                "\"malformed\":%" PRIu64 ",\"late\":%" PRIu64 ",\"complete\":%" PRIu32
                ",\"unknown\":%" PRIu32 ",\"no_buffer\":%" PRIu32 ",\"queue_full\":%" PRIu32
                ",\"startup_skipped\":%" PRIu32 ",\"first_missing_seq\":%" PRIu32 ",\"invalid_cells\":%" PRIu32
                ",\"duplicate_cells\":%" PRIu32 ",\"active_unknown\":%" PRIu32 ",\"active_no_buffer\":%" PRIu32 "}\n",
                wire.stats().packets, wire.stats().missing, wire.stats().malformed,
                wire.stats().duplicate_or_late, sink.complete, sink.unknown, sink.no_buffer, sink.queue_full,
                sink.skipped_during_background, wire.stats().first_missing_sequence,
                wire.stats().invalid_cells, wire.stats().duplicate_cells, sink.active_unknown, sink.active_no_buffer);
}
