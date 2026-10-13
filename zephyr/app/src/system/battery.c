/*

* battery.c - LiPo Battery Voltage Monitor and ADC0 Driver Implementation
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Peripheral: ADC0 @ 0x10001000 (Channel 0)
 *
 * Voltage Divider:
 * R1 = 10 kOhm, R2 = 2.2 kOhm
 * Divider Ratio = (10k + 2.2k) / 2.2k = 12.2 / 2.2 = 5.545455
 */

#include "battery.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>

LOG_MODULE_REGISTER(battery, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_NODE_EXISTS(DT_ALIAS(battery_adc))
static const struct device *const adc_dev =
    DEVICE_DT_GET(DT_ALIAS(battery_adc));
#elif DT_NODE_EXISTS(DT_NODELABEL(adc0))
static const struct device *const adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc0));
#else
#error                                                                         \
    "DeviceTree error: Neither alias 'battery_adc' nor nodelabel 'adc0' found."
#endif

static battery_info_t s_bat_info = {.voltage = 0.0f,
                                    .cell_voltage = 0.0f,
                                    .cell_count = LIPO_4S_CELLS,
                                    .status = BATTERY_OK,
                                    .raw_adc = 0};

static int16_t s_adc_sample_buffer = 0;
static bool s_is_initialized = false;
static struct k_spinlock s_bat_lock;

static const struct adc_channel_cfg s_adc_ch_cfg = {
    .gain = ADC_GAIN_1,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = BATTERY_ADC_CHANNEL,
    .differential = 0};

static const struct adc_sequence s_adc_seq = {
    .channels = BIT(BATTERY_ADC_CHANNEL),
    .buffer = &s_adc_sample_buffer,
    .buffer_size = sizeof(s_adc_sample_buffer),
    .resolution = 12};

int battery_init(void) {
  if (!device_is_ready(adc_dev)) {
    LOG_ERR("ADC0 peripheral not ready on THEJAS32!");
    return -ENODEV;
  }

  int ret = adc_channel_setup(adc_dev, &s_adc_ch_cfg);
  if (ret < 0) {
    LOG_ERR("ADC0 Channel 0 setup failed (err %d)", ret);
    return ret;
  }

  LOG_INF("ADC0 Channel 0 configured (Divider: R1=10k, R2=2.2k, Ratio=%.4f).",
          BATTERY_VOLTAGE_DIVIDER_RATIO);

  /* Acquire 20 baseline samples to establish filtered voltage and cell count */
  float sum_v = 0.0f;
  int valid_reads = 0;

  for (int i = 0; i < 20; i++) {
    if (adc_read(adc_dev, &s_adc_seq) == 0) {
      float v_adc =
          ((float)s_adc_sample_buffer / ADC_MAX_VALUE) * ADC_REF_VOLTAGE_V;
      float v_inst = v_adc * BATTERY_VOLTAGE_DIVIDER_RATIO;
      sum_v += v_inst;
      valid_reads++;
    }
    k_msleep(5);
  }

  if (valid_reads > 0) {
    s_bat_info.voltage = sum_v / (float)valid_reads;
  } else {
    LOG_WRN("ADC baseline reads failed, assuming nominal 4S pack (15.0V)");
    s_bat_info.voltage = 15.0f;
  }

  /* Auto-detect 3S vs 4S pack based on voltage threshold */
  if (s_bat_info.voltage > LIPO_CELL_DETECT_THRESHOLD_V) {
    s_bat_info.cell_count = LIPO_4S_CELLS;
  } else {
    s_bat_info.cell_count = LIPO_3S_CELLS;
  }

  s_bat_info.cell_voltage = s_bat_info.voltage / (float)s_bat_info.cell_count;
  s_bat_info.raw_adc = (uint16_t)s_adc_sample_buffer;
  s_bat_info.status = BATTERY_OK;
  s_is_initialized = true;

  LOG_INF("Battery initialized: %dS LiPo detected @ %.2f V (%.2f V/cell)",
          s_bat_info.cell_count, s_bat_info.voltage, s_bat_info.cell_voltage);

  return 0;
}

int battery_update(void) {
  if (!s_is_initialized) {
    return -EAGAIN;
  }

  int ret = adc_read(adc_dev, &s_adc_seq);
  if (ret < 0) {
    LOG_DBG("ADC0 read failed (err %d)", ret);
    return ret;
  }

  /* Instantaneous pack voltage calculation */
  float v_adc =
      ((float)s_adc_sample_buffer / ADC_MAX_VALUE) * ADC_REF_VOLTAGE_V;
  float v_inst = v_adc * BATTERY_VOLTAGE_DIVIDER_RATIO;

  k_spinlock_key_t key = k_spin_lock(&s_bat_lock);

  /* Exponential Moving Average (EMA) filter */
  s_bat_info.voltage = (BATTERY_EMA_ALPHA * v_inst) +
                       ((1.0f - BATTERY_EMA_ALPHA) * s_bat_info.voltage);
  s_bat_info.cell_voltage = s_bat_info.voltage / (float)s_bat_info.cell_count;
  s_bat_info.raw_adc = (uint16_t)s_adc_sample_buffer;

  /* Determine health status based on cell count and voltage limits */
  float warn_thresh =
      (s_bat_info.cell_count == 4) ? LIPO_4S_WARN_V : LIPO_3S_WARN_V;
  float crit_thresh =
      (s_bat_info.cell_count == 4) ? LIPO_4S_CRIT_V : LIPO_3S_CRIT_V;

  if (s_bat_info.voltage <= crit_thresh) {
    s_bat_info.status = BATTERY_CRITICAL;
  } else if (s_bat_info.voltage <= warn_thresh) {
    s_bat_info.status = BATTERY_WARNING;
  } else {
    s_bat_info.status = BATTERY_OK;
  }

  k_spin_unlock(&s_bat_lock, key);
  return 0;
}

void battery_get_info(battery_info_t *info) {
  if (info != NULL) {
    k_spinlock_key_t key = k_spin_lock(&s_bat_lock);
    *info = s_bat_info;
    k_spin_unlock(&s_bat_lock, key);
  }
}

bool battery_is_critical(void) {
  k_spinlock_key_t key = k_spin_lock(&s_bat_lock);
  bool crit = (s_bat_info.status == BATTERY_CRITICAL);
  k_spin_unlock(&s_bat_lock, key);
  return crit;
}

float battery_get_voltage(void) {
  k_spinlock_key_t key = k_spin_lock(&s_bat_lock);
  float v = s_bat_info.voltage;
  k_spin_unlock(&s_bat_lock, key);
  return v;
}
