# GitHub 发布与清理 · 2026-10-02

仓库目标：`13646997215/piper_classify_ws`，公开，默认分支 `main`。源码、运行配置和已验收的场景保留。首次构建见 [根目录 README](../../README.md#首次下载与构建)。

## 清理范围

- 删除 colcon 的 build/install/log、Python 缓存和编辑器索引；本机编辑器索引位置改到用户缓存目录，避免再次在工作区生成大型数据库。
- 删除根目录无引用、标签不匹配的历史 `$3` 文件。
- 将 530 个原始 RGB-D `.npz` 帧、旧源码快照、官方 PDF、旧截图和原编辑器配置保存到工作区外的本机归档 `~/Desktop/piper_classify_ws_local_evidence_20261002.tar.gz`。此归档不上传 GitHub。
- JSON 结果、记录、测试报告、截图和历史开发说明继续保留；数据记录中的帧文件名对应上述归档。历史 manifest 保持原值，用于描述当时状态，不能作为当前所有文件仍存在的保证。
- 删除两份生成的仿真 URDF，保留原始 URDF/xacro 与生成脚本。生成模型含当前目录的绝对路径，必须在本机生成。
- 添加 Git 忽略规则，避免编译产物、缓存、原始采样、私有环境文件进入后续提交。完整清理清单见 [cleanup.json](cleanup.json)。

## 发布准备

补充首次下载、依赖解析、描述包及其仿真依赖构建、URDF 生成与完整构建步骤。修正 `build_urdf.sh` 缺少描述包时的提示命令，由 `--packages-select` 改为 `--packages-up-to`，使首次构建包含依赖。C++ 运行代码、模型源文件、传感器和任务配置保持收尾版本。

MIT 许可沿用各 ROS 包的既有声明，保留官方 Piper 上游版权。来源见 [THIRD_PARTY.md](../../THIRD_PARTY.md)。发布不包含其他真机工作区或 GitHub 登录凭据；本机提交使用 GitHub noreply 邮箱。

## 验证

全新临时目录执行首次构建流程：依赖初始构建 3 包通过，完整构建 9 包通过，模型网格和控制器绝对路径正确指向新目录；场景结构、相机标定/视场静态检查及启动参数加载通过。4 个包的回归结果为 **20 tests、0 errors、0 failures、0 skipped**。完整记录见 [verification.json](verification.json)，原始测试 XML 保存在 test_results 目录。

本次未重新执行 Gazebo 实体抓放；运行代码、模型源文件和配置与既有验收版本一致。验证目录独立于本工作区，记录保存后删除临时构建目录；本工作区保持没有 build/install/log 的源码状态。
