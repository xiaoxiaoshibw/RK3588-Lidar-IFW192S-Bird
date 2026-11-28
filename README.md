# RK3588-Lidar-IFW192S-Bird

基于 IFW192S 激光雷达的鸟模型候选跟踪原型。ESP32-S3 通过 W5500 接收以太网数据，在板上建立背景、提取几何候选并做时序关联。RK3588 通过 USB 串口接收 JSON，再用浏览器展示轨迹 ID、位置和距离。

```text
IFW192S → 以太网 / W5500 → ESP32-S3 → USB 串口 → RK3588 → 浏览器
```

固件用前 10 帧建立背景，随后按原生距离网格提取前景、进行深度角分割和结构筛选。锁定输出使用当前帧实测点。暂失或输入断开时，看板清除当前框，状态缺失保留为 `UNKNOWN`。

## 运行入口

| 用途 | 入口 | 所需输入 |
| --- | --- | --- |
| ESP32-S3 实时固件 | [firmware/esp32s3_ifw192s/live_probe](firmware/esp32s3_ifw192s/live_probe) | ESP-IDF 5.5.3、带 Octal PSRAM 的 ESP32-S3、W5500 与 IFW192S |
| RK3588 实时看板 | [部署说明](deploy/s3_rk3588/README.md) | 已刷入实时固件的 ESP32-S3、USB 串口和该设备的 IFW192S 标定 |
| RK3588 双容器部署 | [双容器说明](deploy/rk3588_containers/README.md) | RK3588 既有 `slam-localization` 容器、IFW192S 标定表与人体倒地发布包 |
| 无硬件看板回放 | [web/s3_live_server.py](web/s3_live_server.py) | 本地 S3 结果 JSONL |
| 原始点云及结果联合回放 | [web/s3_cloud_replay.py](web/s3_cloud_replay.py) | 匹配的 BIRDRAW、计时映射、UART 日志及外部参考解码库 |
| 相机对照视频 | [web/s3_sync_video.py](web/s3_sync_video.py) | S3 结果 JSONL、可选的相机视频及接收时间元数据 |
| C++ 主机协议验证 | [firmware/esp32s3_bird](firmware/esp32s3_bird/README.md) | CMake 和 C++ 编译器 |

在仓库根目录用已有记录启动离线看板，再打开 <http://127.0.0.1:8898/>：

```powershell
python -B web/s3_live_server.py --host 127.0.0.1 --replay data/live_w5500_timing.jsonl --loop
```

`data/live_w5500_timing.jsonl` 是示例路径，替换为你自己的 S3 串口结果记录。该入口只依赖 Python 标准库。使用 `python -B -m web.s3_cloud_replay --help` 或 `python -B web/s3_sync_video.py --help` 可查看其他离线工具的输入参数。

## 开发验证

```powershell
python -m pip install -r requirements.txt
python -B -W error -m unittest discover -s tests -v
```

测试覆盖 S3 JSON 帧组装、点云回放配对、接收时间对齐及串口中断后清除旧状态。依赖本地录制的测试会在缺少录制时跳过。

2026-11-21 的模型鸟移动试验中观察到持续跟踪，期间有丢失。该现场演示和开发回放尚未给出独立摆位的召回率、误报率或真实鸟类识别准确率。相机视频用于离线复核，接收时刻对齐的误差包含传输和处理时间。数据来源及划分见[数据说明](data/README.md)。
