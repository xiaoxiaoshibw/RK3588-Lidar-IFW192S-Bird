# ESP32-S3 串口网格回放和主机协议检查

此目录保留 2026-11-19 的串口回放固件与 C++ 主机协议检查。实时以太网固件位于 [esp32s3_ifw192s/live_probe](../esp32s3_ifw192s/live_probe)。

本回放固件接收 PC 解码后的原生网格，在 ESP32-S3 上运行背景预热、前景聚类、结构筛选及轨迹锁定。输出候选保留当前帧的 native cell 和 PointCloud2 源行。曾用同段 45 帧开发录制与 Python 参考比对，逐候选核对了 1,890 条源行。历史 Python 对照需要原参考模块、录制和标定。

## 主机协议检查

在仓库根目录使用 CMake 和兼容的 C++ 编译器。Windows MinGW 示例：

```powershell
cmake -S firmware/esp32s3_bird -B data/s3_host_build -G "MinGW Makefiles" -DBIRD_HOST=ON -DCMAKE_BUILD_TYPE=Release
cmake --build data/s3_host_build
python -B firmware/esp32s3_bird/test_protocol.py data/s3_host_build/bird_host.exe
```

## 输入协议

输入使用小端编码。16 字节头为 `<4sIII>`，依次为命令、回放序号、记录数和有效载荷 CRC32。`PING` 和 `RSET` 的后三项填 0。`CAL1` 发送按 native ring/column 排列的 49,152 个 `float64[3]` 标定单位向量，成功后重置跟踪。`BIRD` 的序号从 0 开始，每条 `<HfffI>` 记录依次提供 cell、实测 XYZ 和原始帧内行号。cell 升序且不重复，未提供的 cell 视为缺失。前 10 帧发送所有有效回波，随后可略去 4.5 m 之外的回波。回复为换行结束的 JSON。
