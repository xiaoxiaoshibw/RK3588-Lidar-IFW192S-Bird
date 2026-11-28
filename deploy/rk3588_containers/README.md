# RK3588 双容器部署：人体倒地识别 + 鸟类识别

将人体倒地识别与鸟类识别拆分为两个相互隔离的 Docker 容器运行在 RK3588 上，保留板端现有 `slam-localization` 容器作为共享雷达驱动与 Foxglove 桥接层（不改驱动、不改采集、不抢占网口）：

```text
IFW192S (以太网)
  └── [既有不动] slam-localization 容器
        ├─ roscore (:11311)
        ├─ inno_lidar_node → /innolidar_points
        ├─ foxglove_bridge (:8765)
        └─ capture (:8766)
              ├── ROS1 订阅 ────────────────────→ [容器 1] lidar-human-fall (大核 4,5)
              │                                    发布 /single_target_live/state
              │
              └── Foxglove WS 只读订阅 (:8765) ──→ [容器 2] lidar-bird (大核 6,7)
                                                   本机 C++17 跟踪 + Web 看板 (:8895)
```

## 两个容器的职责与边界

| 容器名 | 算法与代码来源 | 输入方式 | 输出接口 | CPU / 内存 |
| --- | --- | --- | --- | --- |
| `lidar-human-fall` | `range_service_instant_v1.tar`（`--instant-start`，免空场 + 后台静默地面精修，`confirmed=false`） | 同宿主机 ROS1 订阅 `/innolidar_points` | 发布 `/single_target_live/state`（供 Foxglove / LiDAR Studio / Qt 订阅） | 大核 `4,5` / `1536m` |
| `lidar-bird` | 本仓库未改动 C++17 `firmware/esp32s3_bird_core` 编译为 `libbird_tracker.so`（不依赖 ESP32-S3） | 只读订阅 Foxglove `ws://127.0.0.1:8765` `/innolidar_points` | 浏览器看板 `http://<RK3588_IP>:8895/`、`/state`、`/events`、`/healthz`、`/reset-background` | 大核 `6,7` / `512m` |

- 两个容器互不依赖：单独重启、升级或停止其中一个，不会影响另一个和底层雷达驱动。
- `lidar-bird` 保留实测点 `cells` 与原始 `PointCloud2` 的 `source_rows` 溯源；预热期（前 10 帧）保持场景静止，断流或超时后自动清除当前框并保留 `UNKNOWN` 状态。
- 鸟类识别与人体/跌倒识别的真实场景准确率、召回率、误报率仍为 `UNKNOWN`，`confirmed` 保持关闭。

## 连上 RK3588 后一键部署

确保 PC 与 RK3588 网络连通（例如 `192.168.31.58`，SSH 别名 `ldiar-wel`）后，在 `d:\Code\lidar_bird` 根目录运行：

```powershell
python -B deploy/rk3588_containers/deploy_to_rk3588.py --host 192.168.31.58 --user wel
```

脚本会自动完成以下安全步骤：

1. 校验 `D:\Code\ldiar\docs\evidence\2026-11-25_board_instant_deploy\range_service_instant_v1.tar` 内全部 88 个文件的 `MANIFEST.json` SHA256。
2. 打包阶段归档并传到 RK3588，校验 `sha256sum`。
3. 自动检测板端现有 `slam-localization` 使用的本地镜像与 `ROS_MASTER_URI`，**100% 离线构建** `lidar-human-fall:latest` 与 `lidar-bird:latest`（构建失败不会中断任何正在运行的服务）。
4. 仅停止 `slam-localization` 内旧的 `range_service_node.py` 进程（不动驱动、Foxglove、采集和 `current` 软链）；若旧 ESP32-S3 看板 `lidar-bird-s3.service` 占用 8895 端口，先停止该用户服务（保留原目录用于回滚）。
5. 启动 `lidar-human-fall` 与 `lidar-bird` 两个容器（自动适配 `docker compose` / `docker-compose` / `docker run`），并校验两者的健康状态。

### 常用可选参数

- 更换板端 IP：`--host <新IP>`
- 只在本地生成离线部署包（不连板）：
  ```powershell
  python -B deploy/rk3588_containers/deploy_to_rk3588.py --stage-only --out data/rk3588_stage_20261125
  ```
- 启用 `human_physical_v1` 或更换安装俯仰角：`--with-human-physical-v1 --mount-pitch-deg 26.0`
- 给 `lidar-bird` 指定设备专用二进制标定：`--calibration-bin <路径/calibration_sdk_geometry.bin>`
- 指定鸟类看板端口（例如避开 8895）：`--bird-http-port 8897`

## 日常运维与回滚

部署完成后，在 RK3588 上可独立操作任一容器：

```bash
# 查看状态（含健康检查）
docker ps --filter "name=slam-localization" --filter "name=lidar-human-fall" --filter "name=lidar-bird"

# 单独查看日志
docker logs -f --tail 100 lidar-human-fall
docker logs -f --tail 100 lidar-bird

# 单独重启某个容器（不影响另一个）
docker restart lidar-bird
docker restart lidar-human-fall

# 在线重置鸟类识别 10 帧背景（无需重启容器）
curl -X POST http://127.0.0.1:8895/reset-background

# 查看鸟类容器健康与耗时指标
curl http://127.0.0.1:8895/healthz
```

如需停止并移除这两个业务容器（不动 `slam-localization` 底层驱动）：

```powershell
python -B deploy/rk3588_containers/deploy_to_rk3588.py --host 192.168.31.58 --rollback
```

## 离线本地验证（无板端硬件）

在 Windows / Linux 本机可直接验证 C++17 跟踪库编译、ROS1 `PointCloud2` 映射、Foxglove 只读客户端、HTTP/SSE 看板及离线阶段包构建：

```powershell
python -B -W error -m unittest discover -s tests -v
```
