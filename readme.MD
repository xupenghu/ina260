# INA260

## 简介

INA260是一款数字输出电流、功率和电压检测计，具有一个集成高精度分流电阻的I2C和SMBus兼容接口。该器件支持高精度电流和功率测量并在独立于电源电压的共模电压范围（0V 至 36V）实现过流检测。该器件是一款双向、低侧/高侧分流监测计，可测量流经内部电流感测电阻的电流。集成的精密电流感测电阻可使器件获得校准级别的测量精度以及超低温漂，并确保始终可实施针对感测电阻而优化的 Kelvin 布局。

INA260 在 同一 I2C 兼容接口中具有多达 16 个可编程地址。数字接口允许通过编程设定报警阈值，可实现模数转换器 (ADC) 转换时间并求取平均值。为便于使用，可使用内部乘法器直接读取电流值（单位为安培）和功率值（单位为瓦）。

该器件由一个 2.7V 至 5.5V 单电源供电，电源电流为310μA（典型值）。INA260 在 -40°C 至 +125°C 温度范围内额定运行，采用 16 引脚薄型小外形尺寸(TSSOP) 封装。

更多信息请查看官方提供的[数据手册](https://www.ti.com.cn/product/cn/INA260)。

## 特性和优势

- 集成精密分流电阻：
    - 电流感测电阻：2mΩ
    - 等效容差为 0.1%
    - 15A 持续电流（–40°C 至 +85°C）
    - 10ppm/°C 温度系数（0°C 至 +125°C）
- 感测的总线电压范围：0V 至 36V
- 报告电流、电压和功率
- 高精度：
    - ±0.15% 系统增益误差（最大值）
    - 5mA 偏移（最大值）
- 可配置的取平均值选项
- 16 个可编程地址
- 由 2.7V 至 5.5V 电源供电
- 16 引脚薄型小尺寸 (TSSOP) 封装

  
## 基于RT-thread Sensor框架使用说明

### 依赖

- RT-Thread 4.0.0+
- Sensor 组件
- IIC 驱动：INA260使用 IIC 进行数据通讯，需要系统 IIC 驱动支持；

### 获取软件包

使用 INA260 软件包需要在 RT-Thread 的包管理中选中它，具体路径如下：

```
RT-Thread online packages --->
    peripheral libraries and drivers --->
        sensors drivers --->
            [*] INA260: a INA260 package for rt-thread.package
                   [*] Enable INA260 example 
                    Version (latest)  --->
```

**Version**：软件包版本选择

### 使用软件包

刚才的menuconfig默认创建了一个使用例子，编译下载后，使用msh命令行可以打印出当前的被测温度。具体使用方式清参考 **example_ina260.c**

```
[I/I2C] I2C bus [i2c1] registered

 \ | /
- RT -     Thread Operating System
 / | \     4.0.3 build Jan 30 2021
 2006 - 2020 Copyright by rt-thread team
[I/sensor.ina260] ina260 init success. 0
[I/sensor] rt_sensor init success
current : 1.250000 mA,voltage : 0.000000 V ,power : 0.000000 mW
msh >current : 0.000000 mA,voltage : 0.001250 V ,power : 0.000000 mW
current : 0.000000 mA,voltage : 0.001250 V ,power : 0.000000 mW
current : 0.000000 mA,voltage : 0.001250 V ,power : 0.000000 mW
current : 0.000000 mA,voltage : 0.001250 V ,power : 0.000000 mW
current : 0.000000 mA,voltage : 0.000000 V ,power : 0.000000 mW
current : 0.000000 mA,voltage : 0.001250 V ,power : 0.000000 mW
current : 0.000000 mA,voltage : 0.000000 V ,power : 0.000000 mW
current : 0.000000 mA,voltage : 0.001250 V ,power : 0.000000 mW
current : 0.000000 mA,voltage : 0.001250 V ,power : 0.000000 mW
current : 0.000000 mA,voltage : 0.000000 V ,power : 0.000000 mW
current : 1.250000 mA,voltage : 0.000000 V ,power : 0.000000 mW

```
### 注意事项
- 如果读取的数值为0，或者是初始化失败，请注意模拟IIC的IO口输入输出配置

## 裸机或者基于RTOS开发使用说明

这个模式下只需要将 **ina260.c** 和 **ina260.h** 加入工程中即可。

软件包本身不包含I2C驱动，所以需要用户自己实现I2C驱动并实现如下类型的两个读写函数
```
typedef int (*i2c_func)(uint8_t addr, uint16_t *data, uint8_t data_len);
```
以STM32的HAL库为例，实现的IIC驱动函数如下：
```
static int i2c_read(uint8_t addr, uint16_t *data, uint8_t data_len)
{
	return HAL_I2C_Mem_Read(&hi2c1, CHIP_ADDRESS, (uint16_t)addr, 1, (uint8_t *)data, data_len, 100);
}

static int i2c_write(uint8_t addr, uint16_t *data, uint8_t data_len)
{
    return HAL_I2C_Mem_Write(&hi2c1, CHIP_ADDRESS, (uint16_t)addr, 1, (uint8_t *)data, data_len, 100);
}
```
然后调用INA260初始化函数，并将上一步实现的IIC读写函数传入初始化参数中。
```
ret = ina260_init(rt_i2c_read_data, rt_i2c_write_data, device.addr);
if (0 == ret)
{
    LOG_I("ina260 init success.");
}
else
{
    LOG_E("ina260 init error. %d", ret);
}
```

接着就可以在需要的地方调用读取函数来获取传感器的电压电流和功率值。
```
float ma,mv,mw;
ina260_get_current(&ma);
ina260_get_voltage(&mv);
ina260_get_power(&mw);

```


## 联系人信息
- [xph](https://github.com/xupenghu)
- [邮箱](xupenghu@outlook.com)
- [项目地址](https://github.com/xupenghu/ina260)




