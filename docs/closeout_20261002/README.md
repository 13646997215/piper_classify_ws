# 本阶段收尾 · 2026-10-02

用户已反馈“目前该项目没什么问题”，要求先完成收尾，后续另行下达任务。当前阶段已收尾，后续工作等待新的明确要求。

- 保留三块基线和五块同色多物体场景；红/蓝入对应料盒，绿块保持原位。
- RViz 手臂/夹爪控制、视觉 ID/yaw、料盒槽位及异常拒绝逻辑保留。
- 既有验收汇总为 5 项接受记录、20 tests 无失败，详见 [多块报告](../multi_object_20261002/README.md) 和 [接受汇总](../multi_object_20261002/accepted_summary.json)。本次未新增运行测试。
- 收尾时本工作区完整仿真进程已停止，运行代码和配置保持现有版本。随后按用户的 GitHub 发布要求，编译产物与缓存已清理；重新运行前须构建。
- 收尾时曾生成源码压缩快照，其 SHA256 和逐文件清单见 [历史版本清单](manifest.json)。GitHub 发布时快照和原始相机帧已移入工作区外的本机归档；当前源码由 Git 保存。历史清单记录收尾当时的文件及散列，不代表发布后仍包含这些本机产物。报告、JSON 结果和截图保留。
- 真机工作区未改动；既有仿真与单位设备支持边界见 [交接说明](../05_交接说明.md)。

发布清理范围见 [发布记录](../github_20261002/README.md)。首次或清理后运行，先按 [根目录 README](../../README.md#首次下载与构建) 构建。

## 再次运行

三块基线：

```bash
cd /home/hanhan/Desktop/piper_classify_ws
bash scripts/reset_classify.sh
```

五块场景：

```bash
cd /home/hanhan/Desktop/piper_classify_ws
bash scripts/reset_classify.sh --world docs/multi_object_20261002/multi_v3_seed11.world
```

默认只启动环境；等待 VISION_READY 后按交接说明调用 `/classify/start`。结束后用 `bash scripts/stop_classify.sh` 关闭。
