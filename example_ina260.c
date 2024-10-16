/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-30     xph    first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "ti_ina260_sensor_v1.h"
#include "stdio.h"



/****************************
 *         A0   A1  addr
 *         0    0   0x40
 *         0    1   0x41
 *         1    0   0x44
 *         1    1   0x45
 *************************/
#define INA260_ADDR (0x40)

static void ina260_thread_entry(void *parameter)
{
    rt_device_t dev = RT_NULL;
    struct rt_sensor_data sensor_data;
    char ma[10],mv[10],mw[10];

    power_monitor_data_t power_monitor_data;
    rt_size_t res;
    dev = rt_device_find("pm_ina26");
    if (dev == RT_NULL)
    {
        rt_kprintf("can not find device : mlx90632 \n");
        return;
    }

    if (rt_device_open(dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("open device failed!\n");
        return;
    }

    while (1)
    {
        res = rt_device_control(dev, RT_SENSOR_CTRL_GET_POWER_MONITOR_DATA,&power_monitor_data );
        if (res != RT_EOK)
        {
            rt_kprintf("read data failed! result is %d\n", res);
            rt_device_close(dev);
            return;
        }
        else
        {
            power_monitor_data.mv/=1000.0;
            sprintf(ma,"%f", power_monitor_data.ma);
            sprintf(mv,"%f", power_monitor_data.mv);
            sprintf(mw,"%f", power_monitor_data.mw);
            rt_kprintf("current : %s mA,voltage : %s V ,power : %s mW\n", ma, mv, mw);
        }
        rt_thread_mdelay(500);
    }
}

static int ina260_example(void)
{
    rt_thread_t ina260_thread;
    ina260_thread = rt_thread_create(
        "ina260",
        ina260_thread_entry,
        RT_NULL,
        1024,
        RT_THREAD_PRIORITY_MAX / 2,
        10);
    if (ina260_thread != RT_NULL)
    {
        rt_thread_startup(ina260_thread);
    }
    else
    {
        return -RT_ERROR;
    }
    return RT_EOK;
}

INIT_APP_EXPORT(ina260_example);

static int rt_hw_ina260_port(void)
{
    struct rt_sensor_config cfg;
    cfg.intf.user_data = (void *)INA260_ADDR;
    cfg.intf.dev_name = "i2c1";
    cfg.mode = RT_SENSOR_MODE_POLLING;
    rt_hw_ina260_init("ina260", &cfg);
    return RT_EOK;
}

INIT_COMPONENT_EXPORT(rt_hw_ina260_port);
