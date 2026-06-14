# 抗齿槽补偿条件疑点记录

在移植 `FOC_CtrlISR()` 时发现，C++ 原件中抗齿槽补偿判断为：

```c
anticogging_enabled && anticogging_calibrated && anticogging_calibrating
```

但在抗齿槽校准流程中，开始校准时会设置：

```c
anticogging_calibrated = false;
anticogging_calibrating = true;
```

校准结束时会设置：

```c
anticogging_calibrating = false;
anticogging_calibrated = true;
```

因此 `anticogging_calibrated` 和 `anticogging_calibrating` 正常情况下很难同时为真，抗齿槽补偿可能不会进入。

当前 C 版移植阶段先严格复刻 C++ 原件，不在第一版中修改该条件。后续上板调试时应重点验证这里是否应改为：

```c
anticogging_enabled && anticogging_calibrated && !anticogging_calibrating
```
