/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-30     xph    first version
 */

#include "ti_ina260_sensor_v1.h"
#include "ina260.h"

#include "stdint.h"
#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG
#define DBG_SECTION_NAME "sensor.ina260"
#define DBG_COLOR
#include <rtdbg.h>

#define INA260_DEFAULT_ADDR (0x40)

static rt_ina260_device_t device = {0};

static int rt_i2c_read_data(uint8_t addr, uint16_t *data, uint8_t data_len)
{

    struct rt_i2c_msg msgs[2];
    msgs[0].addr = device.addr; /* Slave address */
    msgs[0].flags = RT_I2C_WR;  /* Write flag */
    msgs[0].buf = &addr;        /* Slave register address */
    msgs[0].len = 1;            /* Number of bytes sent */

    msgs[1].addr = device.addr;    /* Slave address */
    msgs[1].flags = RT_I2C_RD;     /* Read flag */
    msgs[1].buf = (uint8_t *)data; /* Read data pointer */
    msgs[1].len = data_len;        /* Number of bytes read */

    if (rt_i2c_transfer(device.bus, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}
static int rt_i2c_write_data(uint8_t addr, uint16_t *data, uint8_t data_len)
{

    /* Implement the I2C write routine according to the target machine. */
    struct rt_i2c_msg msgs[2];

    msgs[0].addr = device.addr; /* Slave address */
    msgs[0].flags = RT_I2C_WR;  /* Write flag */
    msgs[0].buf = &addr;        /* Slave register address */
    msgs[0].len = 1;            /* Number of bytes sent */

    msgs[1].addr = device.addr;                  /* Slave address */
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START; /* Read flag */
    msgs[1].buf = (uint8_t *)data;                          /* Read data pointer */
    msgs[1].len = data_len;                      /* Number of bytes read */

    if (rt_i2c_transfer(device.bus, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_size_t ina260_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    float current = 0;
    ina260_get_current(&current);
    data->data.ma = (rt_uint32_t)(current*1000);
    data->timestamp = rt_sensor_get_ts();
    data->type = RT_SENSOR_CLASS_TEMP;
    return 1;
}

static RT_SIZE_TYPE ina260_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return ina260_polling_get_data(sensor, buf);
    }

    return 0;
}

static rt_err_t ina260_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t  res = RT_ERROR;
    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_POWER_MONITOR_DATA:
        {
            power_monitor_data_t *hData = (power_monitor_data_t *)args;
            ina260_get_current(&(hData->ma));
            ina260_get_voltage(&(hData->mv));
            ina260_get_power(&(hData->mw));
            res = RT_EOK;
        }
        break;
    
    default:
        break;
    }
    return res;
}

static struct rt_sensor_ops sensor_ops =
{
    ina260_fetch_data,
    ina260_control
};

int rt_hw_ina260_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_err_t ret = RT_EOK;
    rt_memset(&device, 0, sizeof(struct _rt_ina260_device_t));
    if (cfg->intf.user_data)
    {
        device.addr = (uint8_t)(uintptr_t)(cfg->intf.user_data);
    }

    device.bus = (struct rt_i2c_bus_device *)rt_device_find(cfg->intf.dev_name);
    if (device.bus == RT_NULL)
    {
        LOG_E("can not find %s bus.", cfg->intf.dev_name);
        return -RT_ERROR;
    }
    ret = ina260_init(rt_i2c_read_data, rt_i2c_write_data, device.addr);
    if (RT_EOK == ret)
    {
        LOG_I("ina260 init success. %d", ret);
    }
    else
    {
        LOG_E("ina260 init error. %d", ret);
    }

    rt_sensor_t sensor = RT_NULL;
    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (RT_NULL == sensor)
    {
        LOG_E("calloc failed");
        return -RT_ERROR;
    }

    sensor->info.type = RT_SENSOR_CLASS_POWER;
    sensor->info.vendor = RT_SENSOR_VENDOR_TI;
    sensor->info.model = "ina260";
    sensor->info.unit = RT_SENSOR_UNIT_MA;
    sensor->info.intf_type = RT_SENSOR_INTF_I2C;
    sensor->info.range_max = 0xffff;
    sensor->info.range_min = 0xffff;
    sensor->info.period_min = 100;

    sensor->config = *cfg;
    sensor->ops = &sensor_ops;

    ret = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDONLY, &device);
    if (ret != RT_EOK)
    {
        LOG_E("device register err. code: %d", ret);
        rt_free(sensor);
    }

    return RT_EOK;
}
