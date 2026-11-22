#pragma once

#include <cstddef>
#include <cstdint>

namespace bird {

struct ReplayIO {
    bool (*read)(void* data, size_t size);
    bool (*write)(const char* data, size_t size);
    int64_t (*now_us)();
    void* (*allocate)(size_t size);
    size_t psram_bytes;
};

void run_replay(const ReplayIO& io);

}  // namespace bird
