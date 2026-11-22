#include "bird_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace bird {
namespace {

using Index = uint16_t;
using Box = std::array<int, 4>;

struct Node {
    std::array<Index, 8> adjacent{};
    uint8_t degree = 0;
    bool seed_seen = false;
    bool in_body = false;
};

int display_col(Cell cell) {
    return kWidth - 1 - cell % kWidth;
}

int display_row(Cell cell) {
    return kHeight - 1 - cell / kWidth;
}

void expand(Box& box, Cell cell) {
    int x = display_col(cell);
    int y = display_row(cell);
    box = {std::min(box[0], x), std::min(box[1], y),
           std::max(box[2], x), std::max(box[3], y)};
}

bool depth_edge(const Frame& frame, Cell first, Cell second) {
    const auto& a = frame.points[first];
    const auto& b = frame.points[second];
    const double dx = double(a.x) - b.x;
    const double dy = double(a.y) - b.y;
    const double dz = double(a.z) - b.z;
    if (dx * dx + dy * dy + dz * dz > 0.09 * 0.09) {
        return false;
    }

    const auto& u = frame.rays[first];
    const auto& v = frame.rays[second];
    const double cosine = std::clamp(u.x * v.x + u.y * v.y + u.z * v.z, -1.0, 1.0);
    const double r1 = frame.range(first);
    const double r2 = frame.range(second);
    const double near_range = std::min(r1, r2);
    const double denominator = std::max(1e-6, std::abs(r1 - r2) - near_range * (1.0 - cosine));
    // Both arguments of atan2 are nonnegative. Squaring the tangent comparison
    // removes the per-edge sqrt and atan2 from the beta >= 12-degree test.
    constexpr double beta_tangent_squared = 0.04518029190898192;
    return near_range * near_range * (1.0 - cosine * cosine)
        >= beta_tangent_squared * denominator * denominator;
}

std::vector<Node> build_graph(const Frame& frame, const std::vector<Cell>& cells) {
    std::vector<Node> graph(cells.size());
    constexpr int dx[] = {1, -1, 0, 1};
    constexpr int dy[] = {0, 1, 1, 1};
    std::array<std::size_t, 4> next{};
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const Cell cell = cells[i];
        for (std::size_t dir = 0; dir < next.size(); ++dir) {
            const int x = cell % kWidth + dx[dir];
            const int y = cell / kWidth + dy[dir];
            if (x < 0 || x >= kWidth || y >= kHeight) {
                continue;
            }
            const int neighbor = y * kWidth + x;
            auto& j = next[dir];
            // Within each direction the requested indices increase with sorted cells.
            while (j < cells.size() && cells[j] < neighbor) {
                ++j;
            }
            if (j < cells.size() && cells[j] == neighbor && depth_edge(frame, cell, cells[j])) {
                graph[i].adjacent[graph[i].degree++] = static_cast<Index>(j);
                graph[j].adjacent[graph[j].degree++] = static_cast<Index>(i);
            }
        }
    }
    return graph;
}

std::array<double, 3> covariance_eigenvalues(
    double xx, double yy, double zz, double xy, double xz, double yz
) {
    const double off_diag_squared = xy * xy + xz * xz + yz * yz;
    if (off_diag_squared == 0.0) {
        std::array<double, 3> eigenvalues{xx, yy, zz};
        std::sort(eigenvalues.begin(), eigenvalues.end());
        return eigenvalues;
    }

    // Solve the characteristic polynomial of the centered, normalized symmetric
    // 3x3 matrix. The determinant gives cos(3*angle); clamp roundoff at +/-1.
    const double mean = (xx + yy + zz) / 3.0;
    xx -= mean;
    yy -= mean;
    zz -= mean;
    const double scale = std::sqrt(
        (xx * xx + yy * yy + zz * zz + 2.0 * off_diag_squared) / 6.0);
    const double det = xx * yy * zz + 2.0 * xy * xz * yz
        - xx * yz * yz - yy * xz * xz - zz * xy * xy;
    const double angle = std::acos(std::clamp(det / (2.0 * scale * scale * scale), -1.0, 1.0)) / 3.0;
    constexpr double two_pi_over_three = 2.09439510239319549;
    const double largest = mean + 2.0 * scale * std::cos(angle);
    const double smallest = mean + 2.0 * scale * std::cos(angle + two_pi_over_three);
    const double middle = 3.0 * mean - largest - smallest;
    return {smallest, middle, largest};
}

bool accept_structure(const Frame& frame, const std::vector<Cell>& cells,
                      const std::vector<Cell>& parent, Box& box, Vec3& center) {
    box = {kWidth, kHeight, -1, -1};
    for (Cell cell : cells) {
        expand(box, cell);
    }
    const int col_span = box[2] - box[0] + 1;
    const int row_span = box[3] - box[1] + 1;
    if (col_span < 4 || col_span > 20 || row_span < 4 || row_span > 20
        || double(cells.size()) / (row_span * col_span) < 0.45) {
        return false;
    }

    Vec3 low = point(frame, cells.front());
    Vec3 high = low;
    center = {};
    for (Cell cell : cells) {
        const Vec3 p = point(frame, cell);
        low = {std::min(low.x, p.x), std::min(low.y, p.y), std::min(low.z, p.z)};
        high = {std::max(high.x, p.x), std::max(high.y, p.y), std::max(high.z, p.z)};
        center.x += p.x;
        center.y += p.y;
        center.z += p.z;
    }
    const double count = static_cast<double>(cells.size());
    center.x /= count;
    center.y /= count;
    center.z /= count;
    const Vec3 extent{high.x - low.x, high.y - low.y, high.z - low.z};
    if (std::min({extent.x, extent.y, extent.z}) < 0.04
        || std::max({extent.x, extent.y, extent.z}) > 0.42) {
        return false;
    }

    double xx = 0.0;
    double yy = 0.0;
    double zz = 0.0;
    double xy = 0.0;
    double xz = 0.0;
    double yz = 0.0;
    for (Cell cell : cells) {
        const Vec3 p = point(frame, cell);
        const double dx = p.x - center.x;
        const double dy = p.y - center.y;
        const double dz = p.z - center.z;
        xx += dx * dx;
        yy += dy * dy;
        zz += dz * dz;
        xy += dx * dy;
        xz += dx * dz;
        yz += dy * dz;
    }
    const double n = count - 1.0;
    const auto e = covariance_eigenvalues(xx / n, yy / n, zz / n, xy / n, xz / n, yz / n);
    const double largest = std::max(0.0, e[2]);
    const double divisor = std::max(1e-9, largest);
    const double linearity = (largest - std::max(0.0, e[1])) / divisor;
    const double sphericity = std::max(0.0, e[0]) / divisor;
    if (linearity > 0.82 || sphericity < 0.085) {
        return false;
    }

    int below_under = 0;
    for (Cell cell : parent) {
        const int x = display_col(cell);
        const int y = display_row(cell);
        if (y < box[1] - 1) {
            return true;
        }
        if (y > box[3] + 2 && x >= box[0] - 1 && x <= box[2] + 1) {
            ++below_under;
        }
    }
    return below_under < 12;
}

}  // namespace

void segment_bodies(const Frame& frame, const std::vector<Cell>& parent,
                    std::vector<Component>& out) {
    if (parent.size() < 12) {
        return;
    }
    auto graph = build_graph(frame, parent);
    const bool strong_seeds = std::any_of(graph.begin(), graph.end(), [](const Node& node) {
        return node.degree >= 7;
    });
    const int seed_degree = strong_seeds ? 7 : 6;
    std::vector<Index> seeds;
    std::vector<Index> expanded;
    std::vector<Cell> body;
    for (Index seed = 0; seed < graph.size(); ++seed) {
        if (graph[seed].degree < seed_degree || graph[seed].seed_seen) {
            continue;
        }
        seeds.clear();
        expanded.clear();
        seeds.push_back(seed);
        graph[seed].seed_seen = true;
        Box seed_box{kWidth, kHeight, -1, -1};
        for (std::size_t i = 0; i < seeds.size(); ++i) {
            const Index index = seeds[i];
            expand(seed_box, parent[index]);
            const auto& node = graph[index];
            for (std::size_t k = 0; k < node.degree; ++k) {
                const Index other = node.adjacent[k];
                if (graph[other].degree >= seed_degree && !graph[other].seed_seen) {
                    graph[other].seed_seen = true;
                    seeds.push_back(other);
                }
            }
        }

        // Recover one hop from the seed component, restricted to the asymmetric
        // display-grid padding used by the panoramic Python tracker.
        const auto include = [&](Index index) {
            if (!graph[index].in_body) {
                graph[index].in_body = true;
                expanded.push_back(index);
            }
        };
        for (Index index : seeds) {
            include(index);
            const auto& node = graph[index];
            for (std::size_t k = 0; k < node.degree; ++k) {
                const Index other = node.adjacent[k];
                const int x = display_col(parent[other]);
                const int y = display_row(parent[other]);
                if (x >= seed_box[0] - 2 && x <= seed_box[2] + 2
                    && y >= seed_box[1] - 1 && y <= seed_box[3] + 2) {
                    include(other);
                }
            }
        }
        for (Index index : expanded) {
            graph[index].in_body = false;
        }
        if (expanded.size() < 15 || expanded.size() > 160) {
            continue;
        }
        std::sort(expanded.begin(), expanded.end());
        body.clear();
        for (Index index : expanded) {
            body.push_back(parent[index]);
        }
        Box box{};
        Vec3 center{};
        if (accept_structure(frame, body, parent, box, center)) {
            Component component{};
            component.cells = body;
            component.box = box;
            component.center = center;
            out.push_back(std::move(component));
        }
    }
}

}  // namespace bird
