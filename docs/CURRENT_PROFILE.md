# 2026-09-19：当前高位策略预筛入口

本地fork从69e66c4建立`feat/high-view-current-profile`。保留历史M0/M2默认配置，新增独立入口，避免把旧的2.4m搜索、3.4m升高转运和随航迹转头假设当作当前比赛策略。

## 本轮实现

- `tools/import_current_runs.py`读取已完成SITL批次，保存31–40实际墙/树排除范围/靶位/CameraInfo和原Gate，不重新生成场景、不运行ROS；10场来自20b1a0f与92bd3ee两批，版本边界保留。
- `config/high_view_20260919.yaml`明确高2.6m/低1.4m、固定机头、相机下16cm、55×55×40cm设计包络。55cm已经含用户所述主动膨胀，默认额外余量为0，不重复加入原鲁棒padding。
- `current_scene_screen.py`比较高位闭合骨架与低位牛耕，复用现有A*/相机/LOS。导航障碍为全高柱，视线遮挡仍用物体高度，不因高度高于树就允许越过树投影。
- 路线构建不读目标坐标；随后才用目标真值评价中心/五点几何可见性，累计的是指定的bridge/panzer/red_cross，不是任意三个目标。
- 只给航程、名义完整搜索时间和三类几何可见时刻。检测概率和完整任务时间显式为null；不暗用B100格外legacy回退。
- `tools/report_current_screen.py`生成报告和可选PNG；核心入口只需既有Python/PyYAML，绘图选项使用本机rl_drone已有Matplotlib。

## 结果与意义

[20组预筛报告与航线图](current_profile_20260919/REPORT.md)。高位9/10几何路线可构造，7/10能累计三类中心几何可见，4/10满足更严格的五点近似；低位分别10/10、10/10、9/10。**这些不是检测召回率、任务成功率或新飞行结果。**

几个不应忽略的差异：

- 34高位的0.6m静态端点调整在本近似模型中无解，是端点风险提示，不等同实际飞行必不可达；实际执行还有等待、不同占据图和跳过策略。
- 33实际高位PASS，但固定水平姿态/名义几何路线未齐三类中心。树冠AABB、1.8m假设高度、姿态和实际避障航迹都可能造成差异，不能据模型假阴性否定实跑。
- 37/39实际通过而严格五点口径未齐，也说明几何代理不等同识别/确认。
- 高位和低位时间中位数来自不同成功子集，速度也不同（0.8/0.6m/s）。不得把26s与108s直接相除宣布“节时76%”。

当前用途是快速找几何覆盖缺口、端点风险、航程/视角取舍，并筛少量SITL候选。它不实现实际Fast-Planner投影进度、保持锁、PX4跟踪和近墙超调，因此不能拿它验证第二门停滞已修。

## 运行

以下命令在WSL内执行；Windows外层使用`wsl -e bash -c '...'`。沿用已有环境，不创建uv环境或安装系统Python包。

```bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
python -m unittest discover -s tests
python current_scene_screen.py \
  --scenes data/current_runs/20260919/frozen_scenes.json \
  --profile config/high_view_20260919.yaml \
  --output results/current_profile_20260919/screen.json
python tools/report_current_screen.py \
  --screen results/current_profile_20260919/screen.json \
  --scenes data/current_runs/20260919/frozen_scenes.json \
  --output docs/current_profile_20260919 --plot
```

重新导入时给`--matrix`一个或多个现有批次JSON，`--output`指定新文件；manifest与批次源码不一致、重复run、非零出生XY偏移或不支持的旋转墙体会拒绝，不静默猜坐标。随附冻结几何不依赖原日志绝对路径即可预筛。

## 明确未实现

此入口没有低位三点投递排序/重捕/释放/走廊/落地的执行模型，没有动态姿态或SLAM误差模型。树冠是基于spawn排除半径的AABB，LOS高度1.8m是可配置近似；导航采用已知准确地图，时间是恒速加转向惩罚，转向期间不累计可见性。端点调整也不模拟真实0.3m吸附、8s等待和一次0.6m替代的状态机。

下一步适合加入实际姿态轨迹回放和观测线索时间线，校准几何代理与实录差异；再考虑有来源的分阶段时间模型。没有当前条件下的统计数据前，不扩充随机概率模型或用Monte Carlo次数制造置信度。

上游PR #1已在99cfa33合入KS2A543 B100/C25/D11；它不是待从零补交的需求。本地/fork后续R64功能与本轮新增入口仍属于fork工作，不直接修改上游main。
