# OpenFWD

OpenFWD是一个开源多轴驱动车辆模型项目。本仓库中存放着相关固件和遥控APP的代码，以及模型，PCB，各种零件的链接。

如果你只是对特定的某个模型感兴趣，可以直接跳到对应名称的分支或是[完整发布版本](#ReleasePacks)部分。

## STM32固件

存放于目录STM32Firmware中，对应的PCB链接：[PCB](undefined)。

### 编译和烧写

使用```cmake```和ST-Link，你可能需要安装[Arm Gnu Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)并配置环境变量。

``` bash
cd STM32Firmware
cmake -B build -G Ninja --preset Release
cmake --build build
```

### 软硬件配置

- 角度传感器：通过I2C1、2两个接口分别连接AS5600霍尔传感器来监控两个马达。注意对应关系，不要对应错误。
- 马达：通过宏```MOTX_ENABLED```来控制马达的存在与否。强烈建议如不需使用则将其取消，以节省时间片。
- 舵机接口：除了作为一般的舵机接口，通过省去上拉电阻也可以将其作为7.4V的信号输出接口，即排针从左到右第二个口接正极，第三个口接负极。注意此时将不再受到保险丝的保护，注意控制电流。固件中暂时不存在相关的代码，还请自行编写。可以通过宏```RUDX_ENABLED```来控制舵机的存在与否。强烈建议如不需使用则将其取消，以节省时间片。舵机7和8由于GPIO不足而交由ESP32控制，见[ESP32舵机](#ESP32RUD)。
- LED：PCB板上没有限流电阻，注意不要烧毁灯珠。LED可以使用PWM控制，但由于时间片紧张而不建议。LED闪烁模式每tick为20ms。偶数下标表示亮起时间，奇数下标为熄灭时间。
- 功率检测：如不需要功率检测，可以将PCB上的检流电阻直接替换为保险。然后取消宏定义```REPORT_TOT_POWER```。电流和电压的单位分别为1mA和1.6mV。
- 串口：串口1用于连接上位机，串口2用于连接下位机。串口2可通过取消宏```UART2_ENABLED```来关闭。使用转发前缀时下标从2开始。

## ESP32C3固件

存放于目录ESPFirmware中，一个可能的开发板购买链接：[ESP32C3 Super Mini](https://item.taobao.com/item.htm?id=811309798924&pisk=gyNiXdcy_Rk_CMSKpWG1CYFe2V6pBfGjdodxDjnVLDoQWxoACoD0DkM46fUTxS4TdPdACfHmiklZhRnxXmc0lubd2_C85PGj3gIRwAtlUjlWgVk2DBkEWj0axj0QGPGjggLpgTCu5laN6upZu27nkqKZ0ooqYyoIoKJa0ckeY40Sgjrqby5ErVAw0mJZYw0soVRZgd-FTVgiuIrqgw4EAqlq7olqLbXZfSPUTicCNszZA4VnSAmz-TO2iLuw204N-IA4tJDi4QnHgIPnSrVVtuOlH0ySXYH3Y6diiyu0XDEPaBcEE8axolfM_feUd5GT9gYSIomtUWHhTH03tzerRyXl-yVmjYPZKEd75f040x2CqQ3i6JDusR_XYPr-j8l_kUjT-Y2nF2keodlTezFjU55MVDHSolnL4_RnTxSrApJzQDdj8ZFehKMZR2mJL5nPlklcGEQh-L7jQ2gs2wbHhKMZR2mR-wvPCAuI50C..&pvid=e304fcfe-e200-42c8-b44c-036e306da7e7&scm=1007.55993.422959.0&skuId=5675158659268&spm=tbpc.mytb_index.repurchaseitem.d1&xxc=home_recommend)。

### 编译和烧写

使用Arduino IDE即可，注意配置开发板。

### 软硬件配置

- 连接关系：

  |ESP32C3 Super Mini|--|主PCB|
  |---|---|---|
  |3V3|--|3V3|
  |GND|--|GND|
  |GPIO20(RX)|--|TX1|
  |GPIO21(TX)|--|RX1|
  |GPIO0(如果用到RUD7)|--|S7|
  |GPIO1(如果用到RUD7)|--|P7|
  |GPIO2(如果用到RUD8)|--|S8|
  |GPIO3(如果用到RUD8)|--|P8|

<a id="ESP32RUD"></a>

- 舵机7和8：由于STM32的GPIO不足，舵机7和8由ESP32控制，在正确连接S7,P7,S8,P8之后即可使用。软件上需要在ESPFirmware.ino中控制调整宏```RUD7_ENABLED```和```RUD8_ENABLED```。
- 连接模式配置：使用Wifi模式还是蓝牙模式只需在```ESPFirmware.ino```中调整宏```BLE_MODE```和```WIFI_MODE```即可，另外的一些选项例如蓝牙UUID，Wifi SSID和密码，服务器IP地址等也在6~14行。为减少干扰，Wifi模式下可以通过将GPIO4接到GND来使其进入STA模式（即连接已存在的Wifi）。GPIO4内部上拉，即默认为AP模式（向外界提供Wifi服务）。如果使用外部路由器，请注意ESP32-C3只支持2.4G频段。

## 遥控APP

TBD

## 模型、五金件和组装指南

TBD

<a id="ReleasePacks"></a>

## 完整发布版本

TBD
