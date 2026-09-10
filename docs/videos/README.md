# 仿真演示视频

[ground-bottle-demo.mp4](ground-bottle-demo.mp4) 为 2026-09-10 在 Jetson Orin 的 Gazebo 桌面直接录制的单次 A→B 地面取放。底盘固定，过程包含回零、接近、下降、夹取、抬升、搬运、释放和返回。

- MP4 / H.264，1900×1200，采集目标 10 fps，无音频，未剪辑或加速。
- Gazebo 窗口渲染帧率偏低，采集帧率不代表仿真画面更新率，因此播放存在卡顿。
- 任务成功，实际抬升 31.23 mm、放置误差 1.88 mm。完整配置、轨迹及结果见 [recorded-demo.json](../validation/bottle/recorded-demo.json)。
- 此视频不是五次连续验收录像；五次成绩单独保存于 [five-cycles.json](../validation/bottle/five-cycles.json)。
- 瓶子采用假设总质量 175 g 的刚体模型；视频不证明真机载荷能力或瓶壁不变形。

录制使用 Jetson 已有 GStreamer 的 ximagesrc、x264enc 和 mp4mux；开始录制后通过 `bash scripts/pick.sh --cycles 1 --output work/recorded-demo.json` 执行一次任务，任务结束后停止录制。再次演示前需将场景恢复为瓶子在 A 点的初始状态。
