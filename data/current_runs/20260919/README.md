# 31–40冻结场景摘要

来源：liftrace-visionwork高位研究分支中`high_fast_five_20260915/matrix.json`与`high_fast_new_seeds_20260915/matrix.json`。对应运行源码分别20b1a0f、92bd3ee。原Gate/已提交槽数保留，不以模型输出覆盖。

`frozen_scenes.json`保存墙碰撞盒、树排除范围、实际靶位、相机K/D及原始run名称；它不是原始点云/图像或实际飞行控制输入。所有来源run均为高位策略，比较表低位路线一行仍引用同一高位来源Gate，不代表存在对应低位实跑。

树冠mesh未重建；使用排除范围AABB与profile中的LOS高度假设。世界偏移/旋转超出导入器支持时明确拒绝。旧数据不重生成，也不因为靠墙或难飞而过滤。
