/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-30     xph    first version
 */
#include <stdlib.h>
#include "ina260.h"


/* pre-defined device ID per datasheet */
static ina260_identification_t INA260_DEVICE_ID = {
    .revision = 0x00,
    .device = 0x227};

static ina260_device_t ina260_device = {0};

/* private function prototypes */
static int ina260_write_config(void);
static int ina260_read_mask_enable(ina260_mask_enable_t *mask_en);

/* exported functions */

/**
  * @brief    initialize the INA260 with default configuration (continuous
  *           voltage and current measurements, each using 1 sample of a 1.1 ms
  *           ADC conversion)
  *
  * @param    i2c_read - i2c read function  i2c_write - i2c write function
  *           
  * @return   0 : init sunccess -1 : error.
  */
int ina260_init(i2c_func i2c_read, i2c_func i2c_write, uint8_t addr)
{
  if (i2c_read == NULL || i2c_write == NULL)
  {
    ina260_device.i2c_read = NULL;
    ina260_device.i2c_write = NULL;
    return INA_STATUS_ERROR;
  }
  if (addr == 0)
  {
    addr = INA260_SLAVE_ADDRESS;
  }

  ina260_device.i2c_read = i2c_read;
  ina260_device.i2c_write = i2c_write;
  ina260_device.i2c_addr = addr;

  ina260_device.config.type = DEFAULT_OPERATING_TYPE;
  ina260_device.config.mode = DEFAULT_OPERATING_MODE;
  ina260_device.config.ctime = DEFAULT_CURRENT_CTIME;
  ina260_device.config.vtime = DEFAULT_VOLTAGE_CTIME;
  ina260_device.config.ssize = DEFAULT_SAMPLE_SIZE;
  return ina260_reset(0);
}

/**
  * @brief    Read the IC ID to determine if the connection is correct
  *
  * @param    none
  *
  * @return   0 : not busy 
  *          -1 : busy
  *
  */
int ina260_ready(void)
{
  int status = -1;
  ina260_identification_t id;

  if (ina260_device.i2c_read)
  {
    status = ina260_device.i2c_read(INA260_REG_DEV_ID, &(id.u16), 2U);
    if (INA_STATUS_OK == status)
    {
      // INA260 returns MSB first
      id.u16 = __LEu16(&(id.u16));
      if (INA260_DEVICE_ID.u16 != id.u16)
      {
        status = INA_STATUS_ERROR;
      }
    }
  }
  return status;
}

/**
  * @brief    You can implement this function to make sure you don't die waiting
  */
int ina260_wait_until_ready(uint32_t timeout)
{
  return ina260_ready();
}

/**
  * @brief    set all configuration parameters at once
  *
  *
  * @param    operating_type - determines which conversions are performed
  *                            for each reading. may be one of the following:
  *             iotShutdown - put device in power-down state
  *             iotCurrent  - perform current readings only
  *             iotVoltage  - perform voltage readings only
  *             iotPower    - perform current and voltage readings
  *
  * @param    operating_mode - determines how conversions should be performed
  *                            for reading. may be one of the following:
  *             iomTriggered  - perform one-shot reading
  *             iomContinuous - continuously update readings
  *
  * @param    current_ctime - sets the conversion time for the current
  *                           measurement. total measure time is conversion
  *                           time multiplied by number of samples (specified
  *                           by sample_size). may be one of the following:
  *             ictConvert140us   - 140 us
  *             ictConvert204us   - 204 us
  *             ictConvert332us   - 332 us
  *             ictConvert588us   - 588 us
  *             ictConvert1p1ms   - 1.1 ms
  *             ictConvert2p116ms - 2.116 ms
  *             ictConvert4p156ms - 4.156 ms
  *             ictConvert8p244ms - 8.244 ms
  *
  * @param    voltage_ctime - sets the conversion time for the bus voltage
  *                           measurement. total measure time is conversion
  *                           time multiplied by number of samples (specified
  *                           by sample_size). may be one of the following:
  *             ictConvert140us   - 140 us
  *             ictConvert204us   - 204 us
  *             ictConvert332us   - 332 us
  *             ictConvert588us   - 588 us
  *             ictConvert1p1ms   - 1.1 ms
  *             ictConvert2p116ms - 2.116 ms
  *             ictConvert4p156ms - 4.156 ms
  *             ictConvert8p244ms - 8.244 ms
  *
  * @param    sample_size - determines the number of samples that are collected
                            and averaged for each measurement. may be one of the
                            following:
  *             issSample1    - 1 sample
  *             issSample4    - 4 samples
  *             issSample16   - 16 samples
  *             issSample64   - 64 samples
  *             issSample128  - 128 samples
  *             issSample256  - 256 samples
  *             issSample512  - 512 samples
  *             issSample1024 - 1024 samples
  *
  * @return    0 : success
  *           -1 : error
  */
int ina260_set_config(ina260_operating_type_t operating_type,
                      ina260_operating_mode_t operating_mode,
                      ina260_conversion_time_t current_ctime,
                      ina260_conversion_time_t voltage_ctime,
                      ina260_sample_size_t sample_size)
{

  ina260_device.config.type = operating_type;
  ina260_device.config.mode = operating_mode;
  ina260_device.config.ctime = current_ctime;
  ina260_device.config.vtime = voltage_ctime;
  ina260_device.config.ssize = sample_size;

  return ina260_write_config();
}

/**
  * @brief    set the operating type
  *
  *
  * @param    operating_type - determines which conversions are performed for
  *                            each reading. may be one of the following:
  *             iotShutdown - put device in power-down state
  *             iotCurrent  - perform current readings only
  *             iotVoltage  - perform voltage readings only
  *             iotPower    - perform current and voltage readings
  *
  * @return    0 : success
  *           -1 : error
  */
int ina260_set_operating_type(ina260_operating_type_t operating_type)
{
  ina260_device.config.type = operating_type;
  return ina260_write_config();
}

/**
  * @brief    set the operating mode
  *
  *
  * @param    operating_mode - determines how conversions should be performed
  *                            for reading. may be one of the following:
  *             iomTriggered  - perform one-shot reading
  *             iomContinuous - continuously update readings
  *
  * @return    0 : success
  *           -1 : error
  */
int ina260_set_operating_mode(ina260_operating_mode_t operating_mode)
{
  ina260_device.config.mode = operating_mode;

  return ina260_write_config();
}

/**
  * @brief    set the conversion time for both voltage and current readings
  *
  *
  * @param    time - sets the conversion time for both voltage and current
  *                  measurements. total measure time is conversion time
  *                  multiplied by number of samples. may be one of the
  *                  following:
  *             ictConvert140us   - 140 us
  *             ictConvert204us   - 204 us
  *             ictConvert332us   - 332 us
  *             ictConvert588us   - 588 us
  *             ictConvert1p1ms   - 1.1 ms
  *             ictConvert2p116ms - 2.116 ms
  *             ictConvert4p156ms - 4.156 ms
  *             ictConvert8p244ms - 8.244 ms
  *
  * @return    0 : success
  *           -1 : error
  */
int ina260_set_conversion_time(ina260_conversion_time_t time)
{
  ina260_device.config.ctime = time;
  ina260_device.config.vtime = time;

  return ina260_write_config();
}

/**
  * @brief    set the conversion time for current readings
  *
  *
  * @param    time - sets the conversion time for the current measurement. total
  *                  measure time is conversion time multiplied by number of
  *                  samples. may be one of the following:
  *             ictConvert140us   - 140 us
  *             ictConvert204us   - 204 us
  *             ictConvert332us   - 332 us
  *             ictConvert588us   - 588 us
  *             ictConvert1p1ms   - 1.1 ms
  *             ictConvert2p116ms - 2.116 ms
  *             ictConvert4p156ms - 4.156 ms
  *             ictConvert8p244ms - 8.244 ms
  *
  * @return    0 : success
  *           -1 : error
  */
int ina260_set_current_conversion_time(ina260_conversion_time_t time)
{
  ina260_device.config.ctime = time;

  return ina260_write_config();
}

/**
  * @brief    set the conversion time for voltage readings
  *
  *
  * @param    time - sets the conversion time for the bus voltage measurement.
  *                  total measure time is conversion time multiplied by number
  *                  of samples. may be one of the following:
  *             ictConvert140us   - 140 us
  *             ictConvert204us   - 204 us
  *             ictConvert332us   - 332 us
  *             ictConvert588us   - 588 us
  *             ictConvert1p1ms   - 1.1 ms
  *             ictConvert2p116ms - 2.116 ms
  *             ictConvert4p156ms - 4.156 ms
  *             ictConvert8p244ms - 8.244 ms
  *
  * @return    0 : success
  *           -1 : error
  */
int ina260_set_voltage_conversion_time(ina260_conversion_time_t time)
{
  ina260_device.config.vtime = time;

  return ina260_write_config();
}

/**
  * @brief    set the sample size (averaging mode) for voltage and current
  *           readings
  *
  *
  * @param    sample_size - determines the number of samples that are collected
  *                         and averaged for each measurement. may be one of the
  *                         following:
  *             issSample1    - 1 sample
  *             issSample4    - 4 samples
  *             issSample16   - 16 samples
  *             issSample64   - 64 samples
  *             issSample128  - 128 samples
  *             issSample256  - 256 samples
  *             issSample512  - 512 samples
  *             issSample1024 - 1024 samples
  *
  * @return    0 : success
  *           -1 : error
  */
int ina260_set_sample_size(ina260_sample_size_t sample_size)
{
  ina260_device.config.ssize = sample_size;

  return ina260_write_config();
}

/**
  * @brief    software reset the INA260 device via configuration register,
  *           optionally resetting configuration to default or user settings.
  *           the configuration settings stored in the given ina260_t struct
  *           will reflect the actual resulting device configuration in either
  *           case.
  *
  *
  * @param    init - initialize configuration using the current settings stored
  *                  in the ina260_t struct (if non-zero), or use the default
  *                  configuration (if zero).
  *
  * @return    0 : success
  *           -1 : error
  */
int ina260_reset(uint8_t init)
{
  ina260_device.config.reset = 1U;

  int status = ina260_write_config();
  if (INA_STATUS_OK != status)
  {
    return status;
  }

  if (0U == init)
  {

    ina260_device.config.type = DEFAULT_OPERATING_TYPE;
    ina260_device.config.mode = DEFAULT_OPERATING_MODE;
    ina260_device.config.ctime = DEFAULT_CURRENT_CTIME;
    ina260_device.config.vtime = DEFAULT_VOLTAGE_CTIME;
    ina260_device.config.ssize = DEFAULT_SAMPLE_SIZE;

    return INA_STATUS_OK;
  }
  else
  {
    return ina260_write_config();
  }
}

/**
  * @brief    tests if a conversion is ready (following all conversions,
  *           averaging, and multiplications), to coordinate triggered
  *           measurements. once read, the flag is cleared and will not be set
  *           again until the next conversion completes.
  *
  *
  * @return    0 : success
  *           -1 : error
  */
int ina260_conversion_ready(void)
{
  ina260_mask_enable_t mask_en;
  int status = ina260_read_mask_enable(&mask_en);
  if (INA_STATUS_OK != status)
  {
    return status;
  }

  if (INA_STATUS_OK == mask_en.conversion_ready)
  {
    return INA_STATUS_ERROR;
  }
  else
  {
    return INA_STATUS_OK;
  }
}

/**
  * @brief    starts or restarts a triggered "one-shot" conversion using the
  *           current device configuration. if the current operating mode is
  *           continuous, this routine does nothing and will not affect the
  *           device in any way.
  *
  *
  * @return    0 : success
  *           -1 : error
  */
int ina260_conversion_start(void)
{

  if (iomContinuous == ina260_device.config.mode)
  {
    return INA_STATUS_ERROR;
  }

  return ina260_write_config();
}

/**
  * @brief    reads the bus voltage (mV) stored in the INA260 voltage register
  *           for both continuous and triggered conversions
  *
  * @param    voltage - pointer to output float variable to which a successful
  *                     reading of bus voltage (mV) will be written
  *
  * @return    0 : success
  *           -1 : error
  */
int ina260_get_voltage(float *voltage)
{
  int status;
  uint16_t v;

  if (ina260_device.i2c_read)
  {
    if (INA_STATUS_OK != (status = ina260_device.i2c_read(INA260_REG_VOLTAGE, &v, 2U)))
    {
      *voltage = 0;
      return status;
    }
  }

  v = __LEu16(&v);
  *voltage = (float)v * INA260_LSB_VOLTAGE;

  return INA_STATUS_OK;
}

/**
  * @brief    reads the current (mA) stored in the INA260 current register for
  *           both continuous and triggered conversions
  *
  * @param    current - pointer to output float variable to which a successful
  *                     reading of current (mA) will be written
  *
  * @return    0 : success
  *           -1 : error
  */
int ina260_get_current(float *current)
{
  int status;
  uint16_t u;
  int16_t c;

  if (ina260_device.i2c_read)
  {
    if (INA_STATUS_OK != (status = ina260_device.i2c_read(INA260_REG_CURRENT, &u, 2U)))
    {
      *current = 0;
      return status;
    }
  }

  c = (int16_t)__LEu16(&u);
  *current = (float)c * INA260_LSB_CURRENT;

  return INA_STATUS_OK;
}

/**
  * @brief    reads the power (mW) stored in the INA260 power register for both
  *           continuous and triggered conversions
  *
  * @param    power - pointer to output float variable to which a successful
  *                   reading of power (mW) will be written
  *
  * @return    0 : success
  *           -1 : error
  */
int ina260_get_power(float *power)
{
  int status;
  uint16_t p;

  if (ina260_device.i2c_read)
  {
    if (0 != (status = ina260_device.i2c_read(INA260_REG_POWER, &p, 2U)))
    {
      *power = 0;
      return status;
    }
  }

  p = __LEu16(&p);
  *power = (float)p * INA260_LSB_POWER;

  return INA_STATUS_OK;
}

static int ina260_write_config(void)
{
  int status = ina260_wait_until_ready(0);

  ina260_configuration_t conf;
  conf.u16 = __LEu16(&(ina260_device.config.u16));

  if (INA_STATUS_OK == status)
  {
    if (ina260_device.i2c_write)
      status = ina260_device.i2c_write(INA260_REG_CONFIG, &(conf.u16), 2U);
  }

  return status;
}

static int ina260_read_mask_enable(ina260_mask_enable_t *mask_en)
{
  int status;
  ina260_mask_enable_t me;
  if (ina260_device.i2c_read)
  {
    status = ina260_device.i2c_read(INA260_REG_MASK_EN, &(me.u16), 2U);
    if (INA_STATUS_OK != status)
    {
      return status;
    }
  }

  mask_en->u16 = __LEu16(&(me.u16));

  return INA_STATUS_OK;
}
