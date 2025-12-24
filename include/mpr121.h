#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    /**
     * @brief Status Registers (Read Only)
     */
    MPR121_REG_TOUCH_STATUS_L    = 0x00, /*!< Touch Status (Bit 0 = ELE0 ... Bit 7 = ELE7) */
    MPR121_REG_TOUCH_STATUS_H    = 0x01, /*!< Touch Status (Bit 0 = ELE8 ... Bit 4 = ELEPROX/ELE12, Bit 7 = Over Current Flag) */

    MPR121_REG_OOR_STATUS_L      = 0x02, /*!< Out-of-Range Status (Bit 0 = ELE0 ... Bit 7 = ELE7) */
    MPR121_REG_OOR_STATUS_H      = 0x03, /*!< Out-of-Range Status (Bit 0 = ELE8 ... Bit 4 = ELEPROX, Bits 6/7 = Auto-Config Fail) */

    /**
     * @brief Filtered Data (Read Only) - 10-bit Data (LSB/MSB)
     * Address range: 0x04 - 0x1D
     */
    MPR121_REG_FILT_DATA_0_L     = 0x04,
    MPR121_REG_FILT_DATA_0_H     = 0x05,
    MPR121_REG_FILT_DATA_1_L     = 0x06,
    MPR121_REG_FILT_DATA_1_H     = 0x07,
    MPR121_REG_FILT_DATA_2_L     = 0x08,
    MPR121_REG_FILT_DATA_2_H     = 0x09,
    MPR121_REG_FILT_DATA_3_L     = 0x0A,
    MPR121_REG_FILT_DATA_3_H     = 0x0B,
    MPR121_REG_FILT_DATA_4_L     = 0x0C,
    MPR121_REG_FILT_DATA_4_H     = 0x0D,
    MPR121_REG_FILT_DATA_5_L     = 0x0E,
    MPR121_REG_FILT_DATA_5_H     = 0x0F,
    MPR121_REG_FILT_DATA_6_L     = 0x10,
    MPR121_REG_FILT_DATA_6_H     = 0x11,
    MPR121_REG_FILT_DATA_7_L     = 0x12,
    MPR121_REG_FILT_DATA_7_H     = 0x13,
    MPR121_REG_FILT_DATA_8_L     = 0x14,
    MPR121_REG_FILT_DATA_8_H     = 0x15,
    MPR121_REG_FILT_DATA_9_L     = 0x16,
    MPR121_REG_FILT_DATA_9_H     = 0x17,
    MPR121_REG_FILT_DATA_10_L    = 0x18,
    MPR121_REG_FILT_DATA_10_H    = 0x19,
    MPR121_REG_FILT_DATA_11_L    = 0x1A,
    MPR121_REG_FILT_DATA_11_H    = 0x1B,
    MPR121_REG_FILT_DATA_PROX_L  = 0x1C, /*!< ELE12 / ELEPROX */
    MPR121_REG_FILT_DATA_PROX_H  = 0x1D,

    /**
     * @brief Baseline Values (Read/Write)
     * Address range: 0x1E - 0x2A
     */
    MPR121_REG_BASELINE_0        = 0x1E,
    MPR121_REG_BASELINE_1        = 0x1F,
    MPR121_REG_BASELINE_2        = 0x20,
    MPR121_REG_BASELINE_3        = 0x21,
    MPR121_REG_BASELINE_4        = 0x22,
    MPR121_REG_BASELINE_5        = 0x23,
    MPR121_REG_BASELINE_6        = 0x24,
    MPR121_REG_BASELINE_7        = 0x25,
    MPR121_REG_BASELINE_8        = 0x26,
    MPR121_REG_BASELINE_9        = 0x27,
    MPR121_REG_BASELINE_10       = 0x28,
    MPR121_REG_BASELINE_11       = 0x29,
    MPR121_REG_BASELINE_PROX     = 0x2A,

    /**
     * @brief Baseline Filtering Control (Global for ELE0-ELE11)
     * MHD = Max Half Delta, NHD = Noise Half Delta
     * NCL = Noise Count Limit, FDL = Filter Delay Limit
     */
    MPR121_REG_MHD_RISING        = 0x2B,
    MPR121_REG_NHD_RISING        = 0x2C,
    MPR121_REG_NCL_RISING        = 0x2D,
    MPR121_REG_FDL_RISING        = 0x2E,
    
    MPR121_REG_MHD_FALLING       = 0x2F,
    MPR121_REG_NHD_FALLING       = 0x30,
    MPR121_REG_NCL_FALLING       = 0x31,
    MPR121_REG_FDL_FALLING       = 0x32,
    
    MPR121_REG_NHD_TOUCHED       = 0x33,
    MPR121_REG_NCL_TOUCHED       = 0x34,
    MPR121_REG_FDL_TOUCHED       = 0x35,

    /**
     * @brief Proximity (ELE12) Baseline Filtering Control
     */
    MPR121_REG_PROX_MHD_RISING   = 0x36,
    MPR121_REG_PROX_NHD_RISING   = 0x37,
    MPR121_REG_PROX_NCL_RISING   = 0x38,
    MPR121_REG_PROX_FDL_RISING   = 0x39,

    MPR121_REG_PROX_MHD_FALLING  = 0x3A,
    MPR121_REG_PROX_NHD_FALLING  = 0x3B,
    MPR121_REG_PROX_NCL_FALLING  = 0x3C,
    MPR121_REG_PROX_FDL_FALLING  = 0x3D,

    MPR121_REG_PROX_NHD_TOUCHED  = 0x3E,
    MPR121_REG_PROX_NCL_TOUCHED  = 0x3F,
    MPR121_REG_PROX_FDL_TOUCHED  = 0x40,

    /**
     * @brief Touch and Release Thresholds
     * Address range: 0x41 - 0x5A
     */
    MPR121_REG_TOUCH_TH_0        = 0x41,
    MPR121_REG_REL_TH_0          = 0x42,
    MPR121_REG_TOUCH_TH_1        = 0x43,
    MPR121_REG_REL_TH_1          = 0x44,
    MPR121_REG_TOUCH_TH_2        = 0x45,
    MPR121_REG_REL_TH_2          = 0x46,
    MPR121_REG_TOUCH_TH_3        = 0x47,
    MPR121_REG_REL_TH_3          = 0x48,
    MPR121_REG_TOUCH_TH_4        = 0x49,
    MPR121_REG_REL_TH_4          = 0x4A,
    MPR121_REG_TOUCH_TH_5        = 0x4B,
    MPR121_REG_REL_TH_5          = 0x4C,
    MPR121_REG_TOUCH_TH_6        = 0x4D,
    MPR121_REG_REL_TH_6          = 0x4E,
    MPR121_REG_TOUCH_TH_7        = 0x4F,
    MPR121_REG_REL_TH_7          = 0x50,
    MPR121_REG_TOUCH_TH_8        = 0x51,
    MPR121_REG_REL_TH_8          = 0x52,
    MPR121_REG_TOUCH_TH_9        = 0x53,
    MPR121_REG_REL_TH_9          = 0x54,
    MPR121_REG_TOUCH_TH_10       = 0x55,
    MPR121_REG_REL_TH_10         = 0x56,
    MPR121_REG_TOUCH_TH_11       = 0x57,
    MPR121_REG_REL_TH_11         = 0x58,
    MPR121_REG_TOUCH_TH_PROX     = 0x59,
    MPR121_REG_REL_TH_PROX       = 0x5A,

    /**
     * @brief Device Configuration
     */
    MPR121_REG_DEBOUNCE          = 0x5B, /*!< Debounce Touch/Release */
    MPR121_REG_CONFIG_1          = 0x5C, /*!< AFE 1 (FFI, CDC Global) */
    MPR121_REG_CONFIG_2          = 0x5D, /*!< AFE 2 (CDT Global, SFI, ESI) */
    MPR121_REG_ECR               = 0x5E, /*!< Electrode Configuration (Run/Stop) */

    /**
     * @brief Auto-Configuration: Individual Electrode Current (CDC)
     * Used if global CDC in 0x5C is 0
     */
    MPR121_REG_CDC_0             = 0x5F,
    MPR121_REG_CDC_1             = 0x60,
    MPR121_REG_CDC_2             = 0x61,
    MPR121_REG_CDC_3             = 0x62,
    MPR121_REG_CDC_4             = 0x63,
    MPR121_REG_CDC_5             = 0x64,
    MPR121_REG_CDC_6             = 0x65,
    MPR121_REG_CDC_7             = 0x66,
    MPR121_REG_CDC_8             = 0x67,
    MPR121_REG_CDC_9             = 0x68,
    MPR121_REG_CDC_10            = 0x69,
    MPR121_REG_CDC_11            = 0x6A,
    MPR121_REG_CDC_PROX          = 0x6B,

    /**
     * @brief Auto-Configuration: Individual Charge Time (CDT)
     * Used if global CDT in 0x5D is 0
     */
    MPR121_REG_CDT_0_1           = 0x6C,
    MPR121_REG_CDT_2_3           = 0x6D,
    MPR121_REG_CDT_4_5           = 0x6E,
    MPR121_REG_CDT_6_7           = 0x6F,
    MPR121_REG_CDT_8_9           = 0x70,
    MPR121_REG_CDT_10_11         = 0x71,
    MPR121_REG_CDT_PROX          = 0x72,

    /**
     * @brief GPIO Control (ELE4-ELE11 can be GPIOs)
     */
    MPR121_REG_GPIO_CTRL_0       = 0x73,
    MPR121_REG_GPIO_CTRL_1       = 0x74,
    MPR121_REG_GPIO_DATA         = 0x75,
    MPR121_REG_GPIO_DIR          = 0x76,
    MPR121_REG_GPIO_ENABLE       = 0x77,
    MPR121_REG_GPIO_SET          = 0x78,
    MPR121_REG_GPIO_CLEAR        = 0x79,
    MPR121_REG_GPIO_TOGGLE       = 0x7A,

    /**
     * @brief Auto-Configuration Control
     */
    MPR121_REG_AUTOCFG_0         = 0x7B, /*!< USL, Retry, BVA, ARE, ACE */
    MPR121_REG_AUTOCFG_1         = 0x7C, /*!< SCTS, Skip Charge Time, Interrupts */
    MPR121_REG_USL               = 0x7D, /*!< Upper Side Limit */
    MPR121_REG_LSL               = 0x7E, /*!< Lower Side Limit */
    MPR121_REG_TARGET_LEVEL      = 0x7F, /*!< Target Level */

    /**
     * @brief Soft Reset
     */
    MPR121_REG_SOFT_RESET        = 0x80  /*!< Write 0x63 to reset */
} mpr121_reg_t;

typedef enum {
    MPR121_BASELINE_TRACKING_DIS = 0x00,
    MPR121_BASELINE_TRACKING_EN  = 0x80, // Sets CL to 10 (Bit 7)
} mpr121_baseline_t;

/**
 * @brief Handle for MPR121 device
 */
typedef struct mpr121_dev_t *mpr121_handle_t;

/**
 * @brief Configuration for MPR121 device
 */
typedef struct {
    uint8_t device_address;     /*!< I2C address (default 0x5A) */
    uint32_t i2c_speed_hz;      /*!< I2C clock speed (e.g. 400000) */
    uint8_t electrode_count;    // User sets this to 4, 8, 12, etc.
    bool enable_baseline;       // User toggles this
} mpr121_config_t;

/**
 * @brief Create a new MPR121 device handle
 * 
 * @param bus_handle I2C master bus handle
 * @param config Configuration struct
 * @param[out] ret_handle Returned device handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpr121_new(i2c_master_bus_handle_t bus_handle, const mpr121_config_t *config, mpr121_handle_t *ret_handle);

/**
 * @brief Delete MPR121 device handle and free resources
 * 
 * @param handle Device handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpr121_del(mpr121_handle_t handle);

/**
 * @brief Write a single byte to a register
 * 
 * @param reg Register address
 * @param val Value to write
 */
esp_err_t mpr121_write_reg(mpr121_handle_t handle, mpr121_reg_t reg, uint8_t val);

/**
 * @brief Write multiple bytes to registers (Burst Write)
 * 
 * @param reg Start register address
 * @param data Buffer containing data to write
 * @param len Number of bytes to write
 */
esp_err_t mpr121_write_regs(mpr121_handle_t handle, mpr121_reg_t reg, const uint8_t *data, size_t len);

/**
 * @brief Read a single byte from a register
 * 
 * @param reg Register address
 * @param val Pointer to store read value
 */
esp_err_t mpr121_read_reg(mpr121_handle_t handle, mpr121_reg_t reg, uint8_t *val);

/**
 * @brief Read multiple bytes from registers (Burst Read)
 * 
 * @param reg Start register address
 * @param data Buffer to store read data
 * @param len Number of bytes to read
 */
esp_err_t mpr121_read_regs(mpr121_handle_t handle, mpr121_reg_t reg, uint8_t *data, size_t len);

/**
 * @brief Soft Reset
 * Writes 0x63 to register 0x80 to reset the device to default state [3].
 * Useful after a glitch or power cycle.
 */
esp_err_t mpr121_soft_reset(mpr121_handle_t handle);

/**
 * @brief Configure Touch and Release Thresholds
 * Sets sensitivity. Touch must be > Release to prevent hysteresis bouncing.
 * Registers: 0x41 - 0x5A [5].
 * Note: Thresholds must be configured while the device is in Stop Mode (ECR=0x00).
 * 
 * @param touch_th Threshold to trigger a touch (typ. 12)
 * @param release_th Threshold to trigger a release (typ. 6)
 */
esp_err_t mpr121_set_thresholds(mpr121_handle_t handle, uint8_t touch_th, uint8_t release_th);

/**
 * @brief Configure Thresholds for a specific electrode
 * Allows fine-tuning individual electrodes (0-12).
 * 
 * @param electrode Electrode index (0-11 for touch, 12 for prox)
 * @param touch_th Touch threshold
 * @param release_th Release threshold
 */
esp_err_t mpr121_set_electrode_threshold(mpr121_handle_t handle, uint8_t electrode, uint8_t touch_th, uint8_t release_th);

/**
 * @brief Configure Debounce
 * Sets how many consecutive samples must match before status changes.
 * Register: 0x5B [6].
 */
esp_err_t mpr121_set_debounce(mpr121_handle_t handle, uint8_t touch_cnt, uint8_t release_cnt);

/**
 * @brief Read Touch Status
 * Reads registers 0x00 and 0x01 [9].
 * 
 * @param out_status [out] 12-bit mask. Bit 0 = Elec 0, Bit 11 = Elec 11.
 * @return ESP_OK or I2C error.
 */
esp_err_t mpr121_read_touch_status(mpr121_handle_t handle, uint16_t *out_status);

/**
 * @brief Read Filtered Data (Raw Capacitance)
 * Reads registers 0x04 - 0x1D [10].
 * Useful for debugging why a sensor isn't triggering.
 * 
 * @param electrode Electrode index (0-11)
 * @param out_data [out] 10-bit filtered capacitance value.
 */
esp_err_t mpr121_read_filtered_data(mpr121_handle_t handle, uint8_t electrode, uint16_t *out_data);

/**
 * @brief Read Baseline Value
 * Reads registers 0x1E - 0x2A [11].
 * The sensor's calculated "background" capacitance.
 */
esp_err_t mpr121_read_baseline(mpr121_handle_t handle, uint8_t electrode, uint16_t *out_baseline);

/**
 * @brief Configure Baseline Filtering for Rising Signal (Global)
 * Controls how the baseline tracks rising signal changes (drift compensation).
 * Registers: 0x2B - 0x2E.
 * 
 * @param mhd Max Half Delta (1~63). Determines the largest magnitude of variation to pass through the baseline filter.
 * @param nhd Noise Half Delta (1~63). Determines the incremental change when non-noise drift is detected.
 * @param ncl Noise Count Limit (0~255). Determines the number of samples consecutively greater than the Max Half Delta necessary before it can be determined that it is non-noise.
 * @param fdl Filter Delay Limit (0~255). Determines the rate of operation of the filter. A larger number makes it operate slower.
 */
esp_err_t mpr121_config_filter_rising(mpr121_handle_t handle, uint8_t mhd, uint8_t nhd, uint8_t ncl, uint8_t fdl);

/**
 * @brief Configure Baseline Filtering for Falling Signal (Global)
 * Controls how the baseline tracks falling signal changes.
 * Registers: 0x2F - 0x32.
 * 
 * @param mhd Max Half Delta (1~63). Determines the largest magnitude of variation to pass through the baseline filter.
 * @param nhd Noise Half Delta (1~63). Determines the incremental change when non-noise drift is detected.
 * @param ncl Noise Count Limit (0~255). Determines the number of samples consecutively greater than the Max Half Delta necessary before it can be determined that it is non-noise.
 * @param fdl Filter Delay Limit (0~255). Determines the rate of operation of the filter. A larger number makes it operate slower.
 */
esp_err_t mpr121_config_filter_falling(mpr121_handle_t handle, uint8_t mhd, uint8_t nhd, uint8_t ncl, uint8_t fdl);

/**
 * @brief Configure Baseline Filtering for Touched State (Global)
 * Controls baseline tracking while an electrode is touched.
 * Registers: 0x33 - 0x35.
 * 
 * @param nhd Noise Half Delta (1~63). Determines the incremental change when non-noise drift is detected.
 * @param ncl Noise Count Limit (0~255). Determines the number of samples consecutively greater than the Max Half Delta necessary before it can be determined that it is non-noise.
 * @param fdl Filter Delay Limit (0~255). Determines the rate of operation of the filter. A larger number makes it operate slower.
 */
esp_err_t mpr121_config_filter_touched(mpr121_handle_t handle, uint8_t nhd, uint8_t ncl, uint8_t fdl);

/**
 * @brief Configure Proximity (ELE12) Baseline Filtering - Rising
 * Registers: 0x36 - 0x39.
 * 
 * @param mhd Max Half Delta (1~63). Determines the largest magnitude of variation to pass through the baseline filter.
 * @param nhd Noise Half Delta (1~63). Determines the incremental change when non-noise drift is detected.
 * @param ncl Noise Count Limit (0~255). Determines the number of samples consecutively greater than the Max Half Delta necessary before it can be determined that it is non-noise.
 * @param fdl Filter Delay Limit (0~255). Determines the rate of operation of the filter. A larger number makes it operate slower.
 */
esp_err_t mpr121_config_proximity_filter_rising(mpr121_handle_t handle, uint8_t mhd, uint8_t nhd, uint8_t ncl, uint8_t fdl);

/**
 * @brief Configure Proximity (ELE12) Baseline Filtering - Falling
 * Registers: 0x3A - 0x3D.
 * 
 * @param mhd Max Half Delta (1~63). Determines the largest magnitude of variation to pass through the baseline filter.
 * @param nhd Noise Half Delta (1~63). Determines the incremental change when non-noise drift is detected.
 * @param ncl Noise Count Limit (0~255). Determines the number of samples consecutively greater than the Max Half Delta necessary before it can be determined that it is non-noise.
 * @param fdl Filter Delay Limit (0~255). Determines the rate of operation of the filter. A larger number makes it operate slower.
 */
esp_err_t mpr121_config_proximity_filter_falling(mpr121_handle_t handle, uint8_t mhd, uint8_t nhd, uint8_t ncl, uint8_t fdl);

/**
 * @brief Configure Proximity (ELE12) Baseline Filtering - Touched
 * Registers: 0x3E - 0x40.
 * 
 * @param nhd Noise Half Delta (1~63). Determines the incremental change when non-noise drift is detected.
 * @param ncl Noise Count Limit (0~255). Determines the number of samples consecutively greater than the Max Half Delta necessary before it can be determined that it is non-noise.
 * @param fdl Filter Delay Limit (0~255). Determines the rate of operation of the filter. A larger number makes it operate slower.
 */
esp_err_t mpr121_config_proximity_filter_touched(mpr121_handle_t handle, uint8_t nhd, uint8_t ncl, uint8_t fdl);

/**
 * @brief Configure Auto-Calibration Parameters
 * Sets the target levels for the auto-configuration engine.
 * Registers: 0x7D - 0x7F.
 * 
 * @param target_level Desired baseline level (e.g., 0x90 for 3.3V)
 * @param usl Upper Side Limit (error tolerance)
 * @param lsl Lower Side Limit (error tolerance)
 */
esp_err_t mpr121_config_auto_calibration(mpr121_handle_t handle, uint8_t target_level, uint8_t usl, uint8_t lsl);

/**
 * @brief Enable/Disable Auto-Configuration
 * Configures registers 0x7B and 0x7C to enable automatic electrode configuration.
 * 
 * @param enable Enable or disable auto-config
 * @param skip_charge_time If true, skips charge time search (SCTS)
 */
esp_err_t mpr121_enable_auto_config(mpr121_handle_t handle, bool enable, bool skip_charge_time);

/**
 * @brief Configure GPIO Pins (ELE4 - ELE11)
 * Electrodes 4-11 can be used as GPIOs if not enabled for touch.
 * 
 * @param electrode_mask Bitmask of electrodes to configure (Bit 4 = ELE4, etc.)
 * @param is_output True for Output, False for Input
 */
esp_err_t mpr121_gpio_config(mpr121_handle_t handle, uint16_t electrode_mask, bool is_output);

/**
 * @brief Set GPIO Output Level
 * 
 * @param electrode_mask Bitmask of pins to modify
 * @param level 1 for High, 0 for Low
 */
esp_err_t mpr121_gpio_set_level(mpr121_handle_t handle, uint16_t electrode_mask, uint8_t level);

/**
 * @brief Read GPIO Input Level
 * 
 * @param[out] level_mask Returns bitmask of current pin levels
 */
esp_err_t mpr121_gpio_read_level(mpr121_handle_t handle, uint16_t *level_mask);

/**
 * @brief Enable Sensor (Enter Run Mode)
 * Writes to ECR (0x5E) to enable electrodes and baseline tracking [7].
 * REQUIRED to start sensing.
 * 
 */
esp_err_t mpr121_enable(mpr121_handle_t handle);

/**
 * @brief Disable Sensor (Enter Stop Mode)
 * Writes 0x00 to ECR (0x5E).
 * REQUIRED before changing thresholds or configuration [4].
 */
esp_err_t mpr121_disable(mpr121_handle_t handle);

#ifdef __cplusplus
}
#endif