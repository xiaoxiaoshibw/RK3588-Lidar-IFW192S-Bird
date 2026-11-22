#include <cinttypes>
#include <cstdio>
#include <cstring>

#include "esp_attr.h"
#include "wire.hpp"
#include "w5500.hpp"

namespace {

using ifw192s::W5500;
using ifw192s::WireAssembler;
constexpr unsigned kTestBytes = 1024;
DMA_ATTR uint8_t expected[kTestBytes];
DMA_ATTR uint8_t rx[W5500::kBufferBytes];
constexpr unsigned kSamples = 6;
uint8_t samples[kSamples][ifw192s::kPacketBytes];

struct Frames {
    uint32_t complete = 0;
    uint32_t unknown = 0;
    void begin_frame() {}
    void record(const ifw192s::WireRecord&, uint32_t) {}
    void end_frame(const ifw192s::FrameBoundary& boundary) {
        complete += boundary.complete;
        unknown += !boundary.complete;
    }
};

size_t first_bad_byte() {
    for (size_t i = 0; i < sizeof(expected); ++i) {
        if (rx[i] != expected[i]) { return i; }
    }
    return sizeof(expected);
}

bool test_bus(W5500& device, int hz) {
    auto version = device.version(hz);
    if (version != 4) {
        std::printf("BUS_BAD requested_hz=%d actual_khz=%d version=%02x\n", hz, device.clock_khz(), version);
        return false;
    }
    for (int trial = 0; trial < 16; ++trial) {
        device.read_test_bytes(rx, sizeof(expected));
        const size_t bad = first_bad_byte();
        if (bad != sizeof(expected)) {
            std::printf("BUS_BAD requested_hz=%d actual_khz=%d trial=%d offset=%zu expected=%02x got=%02x\n",
                        hz, device.clock_khz(), trial, bad, expected[bad], rx[bad]);
            return false;
        }
    }
    std::printf("BUS_OK requested_hz=%d actual_khz=%d verified_bytes=16384\n", hz, device.clock_khz());
    return true;
}

void probe_stream(W5500& device, bool optimized) {
    WireAssembler wire(ifw192s::PacketLayout::detect);
    Frames frames;
    uint32_t batches = 0, rx_errors = 0, foreign = 0, bad_layout = 0;
    uint32_t first_gap_from = 0, first_gap_to = 0, last_sequence = 0;
    uint32_t first_sequence = 0;
    uint32_t marker127 = 0, marker255 = 0;
    uint32_t ring191 = 0;
    uint8_t max_ring = 0, max_column = 0;
    unsigned sampled = 0;
    size_t peak = 0;
    constexpr size_t header = 8;
    constexpr int64_t test_us = 4'000'000;
    device.acquire();
    const int64_t start = esp_timer_get_time();
    while (esp_timer_get_time() - start < test_us) {
        const size_t waiting = optimized ? device.pending_ifw192s() : device.pending();
        if (waiting == 0) { continue; }
        peak = std::max(peak, waiting);
        const size_t count = waiting;
        if (!device.pop(rx, count, !optimized)) { ++rx_errors; break; }
        const uint64_t read_us = esp_timer_get_time();
        ++batches;
        size_t cursor = 0;
        while (count - cursor >= header) {
            const uint8_t* datagram = rx + cursor;
            const size_t length = uint16_t{datagram[6]} << 8 | datagram[7];
            if (length > count - cursor - header) { break; }
            cursor += header + length;
            const uint8_t lidar[] = {192, 168, 1, 10, 0x1b, 0x58};
            if (std::memcmp(datagram, lidar, sizeof(lidar)) != 0) { ++foreign; continue; }
            const auto missing_before = wire.stats().missing;
            wire.push(datagram + header, length, read_us, frames);
            if (length == ifw192s::kPacketBytes) {
                if (sampled < kSamples && wire.stats().packets >= 1 + sampled * 177u) {
                    std::memcpy(samples[sampled++], datagram + header, ifw192s::kPacketBytes);
                }
                const uint8_t* record = datagram + header + ifw192s::kRecordOffset;
                for (size_t slot = 0; slot < ifw192s::kRecordsPerPacket; ++slot, record += ifw192s::kRecordBytes) {
                    max_column = std::max(max_column, record[13]);
                    max_ring = std::max(max_ring, record[14]);
                    if (record[14] == 191) {
                        ++ring191;
                        marker127 += record[13] == 127;
                        marker255 += record[13] == 255;
                    }
                }
                const uint32_t seq = ifw192s::read_u32(datagram + header + ifw192s::kSequenceOffset);
                if (first_sequence == 0) { first_sequence = seq; }
                if (first_gap_to == 0 && wire.stats().missing > missing_before) {
                    first_gap_from = last_sequence;
                    first_gap_to = seq;
                }
                last_sequence = seq;
            }
        }
        if (cursor != count) {
            if (++bad_layout == 1) {
                std::printf("FIRST_BAD_BATCH bytes=%zu offset=%zu\n", count, cursor);
            }
            wire.reset_alignment();
        }
    }
    device.release();
    const auto& stats = wire.stats();
    std::printf("STREAM optimized=%d actual_khz=%d duration_us=%" PRId64
                " packets=%" PRIu64 " missing=%" PRIu64 " malformed=%" PRIu64 " late=%" PRIu64
                " complete=%" PRIu32 " unknown=%" PRIu32 " peak=%zu batches=%" PRIu32
                " bad_layout=%" PRIu32 " io_error=%" PRIu32 " foreign=%" PRIu32
                " seq_first=%" PRIu32 " seq_last=%" PRIu32 " first_gap=%" PRIu32 ":%" PRIu32
                " stack_min=%u marker127=%" PRIu32 " marker255=%" PRIu32 " ring191=%" PRIu32
                " max_ring=%u max_column=%u\n",
                optimized, device.clock_khz(), esp_timer_get_time() - start, stats.packets,
                stats.missing, stats.malformed, stats.duplicate_or_late, frames.complete, frames.unknown,
                peak, batches, bad_layout, rx_errors, foreign, first_sequence, last_sequence, first_gap_from, first_gap_to,
                static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)), marker127, marker255, ring191,
                max_ring, max_column);
    for (unsigned i = 0; i < sampled; ++i) {
        char text[ifw192s::kPacketBytes * 2 + 1];
        constexpr char hex[] = "0123456789abcdef";
        for (size_t j = 0; j < ifw192s::kPacketBytes; ++j) {
            text[j * 2] = hex[samples[i][j] >> 4];
            text[j * 2 + 1] = hex[samples[i][j] & 15];
        }
        text[sizeof(text) - 1] = '\0';
        std::printf("SAMPLE optimized=%d %s\n", optimized, text);
    }
}

}  // namespace

extern "C" void app_main() {
    W5500 device;
    device.init();
    const auto version = device.version();
    if (version != 4 || !device.configure(W5500::kIsolatedMac)) {
        std::printf("START_FAILED version=%02x\n", version);
        device.reset();
        return;
    }
    const auto deadline = esp_timer_get_time() + 8'000'000;
    while (!(device.phy() & 1) && esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    const auto phy = device.phy();
    std::printf("PHY=%02x\n", phy);
    if (!(phy & 1)) {
        std::puts("LINK_DOWN");
        device.reset();
        return;
    }
    uint32_t state = 0x19fa1625u;
    for (auto& byte : expected) {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        byte = state & 0xff;
    }
    device.set_mode(0);
    device.drive(GPIO_DRIVE_CAP_0);
    constexpr int hz = 80'000'000;
    for (bool optimized : {false, true}) {
        device.read_dummy_bits(0);
        if (device.version(W5500::kBaseHz) != 4 || !device.open_udp()) {
            std::printf("OPEN_FAILED optimized=%d\n", optimized);
            continue;
        }
        device.write_test_bytes(expected, sizeof(expected));
        device.read_dummy_bits(1);
        if (!test_bus(device, hz)) {
            continue;
        }
        // Verify that MOSI also works at this clock before using live RX pointers.
        rx[0] = static_cast<uint8_t>(expected[0] ^ 0xff);
        const uint8_t updated = rx[0];
        device.write_test_bytes(rx, sizeof(expected));
        device.read_dummy_bits(0);
        device.version(W5500::kBaseHz);
        device.read_test_bytes(rx, sizeof(expected));
        if (rx[0] != updated) {
            std::printf("WRITE_BAD expected=%02x got=%02x\n", updated, rx[0]);
            continue;
        }
        device.read_dummy_bits(1);
        device.version(hz);
        probe_stream(device, optimized);
    }
    device.read_dummy_bits(0);
    device.version(W5500::kBaseHz);
    device.reset();
    uint8_t mac[6];
    device.read_mac(mac);
    if (std::memcmp(mac, W5500::kIsolatedMac, sizeof(mac)) != 0) {
        std::printf("BURST_DONE version_after_reset=%02x\n", device.version());
    }
}
