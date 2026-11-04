# 蜂鸣器 PL 侧改动（可复现脚本）

目标：在原有 `dual_ov5640_lcd` Vivado 工程上，增加一个 1 bit 的输出端口 `buzzer`，
接到 FPGA 的 **E18**（板上 J21 跳线排的 **PL_TX** 孔），由 PS 通过 AXI GPIO 通道 2 驱动。

## 改动内容（两处）

1. **Block Design**：`axi_gpio_0` 由单通道改为双通道
   - 通道 1 保持 3 bit（LCD ID 输入，不动）
   - 通道 2 新增 1 bit 输出 → 顶层端口 `buzzer`
2. **约束**：`pin.xdc` 末尾追加

```tcl
#----------------------buzzer (J21 PL_TX) ---------------------------
set_property -dict {PACKAGE_PIN E18 IOSTANDARD LVCMOS33} [get_ports buzzer]
```

## 复现方法

在 Vivado 2020.2 的 Tcl Console：

```tcl
cd {<工程目录>}          ;# dual_ov5640_lcd.xpr 所在目录
source pl_buzzer/bd_patch.tcl
# 然后把上面的约束行追加到 dual_ov5640_lcd.srcs/constrs_1/new/pin.xdc
reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
```

产物：`dual_ov5640_lcd.runs/impl_1/system_wrapper.bit`（本目录已附带改好的 `system_wrapper.bit`）。

## 验证记录

- 实现后 IO 报告：`E18 | buzzer[0] | IO_L5N_T0_AD9N_35 | OUTPUT | LVCMOS33 | bank 35`
- 时序：WNS = +0.666 ns，WHS = +0.042 ns（满足）
- 板上实测：注入 256×256 藏马熊测试图，`detections=1 conf=0.893`，
  日志出现 `BUZZ: test-mode alarm`，蜂鸣器发声
