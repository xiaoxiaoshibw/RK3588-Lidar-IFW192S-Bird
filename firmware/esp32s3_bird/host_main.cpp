#include "replay_wire.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

int main() {
#ifdef _WIN32
    if (_setmode(_fileno(stdin), _O_BINARY) < 0 || _setmode(_fileno(stdout), _O_BINARY) < 0) {
        return 1;
    }
#endif
    bird::run_replay({
        [](void* data, size_t size) { return std::fread(data, 1, size, stdin) == size; },
        [](const char* data, size_t size) {
            return std::fwrite(data, 1, size, stdout) == size && std::fflush(stdout) == 0;
        },
        []() -> int64_t {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        },
        std::malloc,
        0,
    });
}
