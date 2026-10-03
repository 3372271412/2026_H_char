# 2026_H_char

2026 年全国大学生电子设计竞赛 H 题「车载平衡滚球运动控制系统」工程（TI MSPM0G3507 + FreeRTOS，六任务：图传 / 循线 / 管球）。

## 目录
- Component/ 各功能模块
- keil/ Keil uVision 工程（MDK 5.43 + ArmClang 6.24）
- docs/ 文档与交接资料
- logs/ 实车测试日志
- 参数默认值：Component/config/task_params.c

## 编译
MSPM0_SDK=C:\ti\mspm0_sdk_2_10_00_04
SYSCONFIG=C:\ti\sysconfig_1.26.2
UV4.exe -b keil\H_BallCar_Task1_6.uvprojx -t H_BallCar_Task1_6 -j0
