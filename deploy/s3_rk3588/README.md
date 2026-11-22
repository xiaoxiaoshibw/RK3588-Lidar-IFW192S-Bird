# RK3588 实时看板

ESP32-S3 使用 W5500 接收雷达数据并输出当前轨迹。RK3588 的 Python 服务通过 USB 串口向 S3 上传 IFW192S 标定并开始读取结果，再通过 HTTP 和 SSE 提供页面。

## 准备固件

在已激活的 ESP-IDF 5.5.3 终端进入 `firmware/esp32s3_ifw192s/live_probe`：

```powershell
idf.py -DIDF_TARGET=esp32s3 build
idf.py -p COM11 -b 921600 flash
```

将 `COM11` 换成你的烧录端口。默认构建配置对应 16 MB Flash、8 MB Octal PSRAM 的 ESP32-S3 开发板。W5500 默认接收 `192.168.1.35:8080` 的 UDP 数据。接线和网络参数见 [w5500.hpp](../../firmware/esp32s3_ifw192s/burst_probe/main/w5500.hpp)。

## 从源码手动启动

在 RK3588 使用 Python 3.8 或更新版本。连接已烧录的 ESP32-S3，确认串口可访问且只有一个进程读取。准备与雷达匹配的 `calibration_sdk_geometry.bin`，设备专用标定文件由本地发布包提供。

在仓库根目录运行：

```bash
python3 -m pip install pyserial==3.5
ls -l /dev/serial/by-id/
PYTHONPATH=firmware/esp32s3_ifw192s:web python3 -B deploy/s3_rk3588/viewer.py \
  --port /dev/ttyACM0 --calibration /path/to/calibration_sdk_geometry.bin --http-port 8895
```

将串口替换成实际路径。固定部署可使用 `/dev/serial/by-id/` 下对应设备的稳定路径。浏览器访问 RK3588 的 8895 端口。S3 用前 10 帧建立背景，预热时保持场景静止。

## 本地发布包和服务

2026-11-21 本地发布包位于 `data/s3_rk3588_release_20261121_01.zip`，包含已部署版本源码、应用镜像、标定、构建配置及回滚备份。按包内说明校验 `SHA256SUMS` 后安装。仓库中的 [systemd 单元](lidar-bird-s3.service) 使用包内文件布局。安装到不同设备时，在单元的 `ExecStart` 中添加实际的 `--port`。
