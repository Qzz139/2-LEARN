# Jetson 最后五次实机抓放记录及参数快照

2026-09-12 从 `/home/robot/projects/2-LEARN/` 原样复制到 Mac。

- `config/real_pick.local.json`：Jetson 实际使用的 HOME/A/B、安全高度、夹爪和容差参数。
- `config/real_pick.json`：参数模板。
- `config/ep_connection.json`：通信参数。
- `results/`：按时间选取最新五次 `mode=pick` 记录，未按成功与否筛选；不包含空载、回 HOME、单独松爪等模式。
- `manifest.json`：原始路径、大小和 SHA-256 校验值，复制后全部校验一致。

五次发生于北京时间 2026-09-11 16:31:31–16:32:52。五份 JSON 均为 `commands_completed=true`，`physical_grasp_success=null`。这表示程序流程完成；实物是否抓起、无损放置仍须结合现场观察或录像确认，不能将其直接改写为实物五次全部成功。

文件名和内容均保留原样。该目录是提交材料快照，不改变项目运行配置。
