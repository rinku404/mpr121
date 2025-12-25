#include "mpr121.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "mpr121";

typedef struct mpr121_dev_t {
    i2c_master_dev_handle_t i2c_dev;
    mpr121_config_t config;
} mpr121_dev_t;

esp_err_t mpr121_new(i2c_master_bus_handle_t bus_handle, const mpr121_config_t *config, mpr121_handle_t *ret_handle) {
    ESP_RETURN_ON_FALSE(bus_handle && config && ret_handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    mpr121_dev_t *dev = (mpr121_dev_t *)heap_caps_calloc(1, sizeof(mpr121_dev_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "No memory");

    dev->config = *config;
    i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = config->i2c_speed_hz,
        .device_address = config->device_address,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to I2C bus");
        heap_caps_free(dev);
        return ret;
    }

    *ret_handle = dev;
    return ESP_OK;
}

esp_err_t mpr121_del(mpr121_handle_t handle) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    i2c_master_bus_rm_device(handle->i2c_dev);
    heap_caps_free(handle);
    return ESP_OK;
}

esp_err_t mpr121_write_reg(mpr121_handle_t handle, mpr121_reg_t reg, uint8_t val) {
    const uint8_t tx_buf[] = {(uint8_t)reg, val};
    return i2c_master_transmit(handle->i2c_dev, tx_buf, sizeof(tx_buf), CONFIG_MPR121_I2C_TIMEOUT_MS);
}

esp_err_t mpr121_write_regs(mpr121_handle_t handle, mpr121_reg_t reg, const uint8_t *data, size_t len) {
    ESP_RETURN_ON_FALSE(handle && (data || len == 0), ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    if (len == 0) return ESP_OK;

    // Max standard burst is usually thresholds (26 bytes). 64 bytes is safe for stack.
    if (len < 64) {
        uint8_t buffer[65];
        buffer[0] = (uint8_t)reg;
        memcpy(buffer + 1, data, len);
        return i2c_master_transmit(handle->i2c_dev, buffer, len + 1, CONFIG_MPR121_I2C_TIMEOUT_MS);
    }

    // Allocate buffer for Reg Addr + Data
    // Use heap to support arbitrary lengths safely
    uint8_t *buffer = (uint8_t *)heap_caps_malloc(len + 1, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(buffer, ESP_ERR_NO_MEM, TAG, "No memory");

    buffer[0] = (uint8_t)reg;
    memcpy(buffer + 1, data, len);

    esp_err_t ret = i2c_master_transmit(handle->i2c_dev, buffer, len + 1, CONFIG_MPR121_I2C_TIMEOUT_MS);
    heap_caps_free(buffer);
    return ret;
}

esp_err_t mpr121_read_regs(mpr121_handle_t handle, mpr121_reg_t reg, uint8_t *data, size_t len) {
    uint8_t reg_addr = (uint8_t)reg;
    return i2c_master_transmit_receive(handle->i2c_dev, &reg_addr, 1, data, len, CONFIG_MPR121_I2C_TIMEOUT_MS);
}

esp_err_t mpr121_read_reg(mpr121_handle_t handle, mpr121_reg_t reg, uint8_t *val) {
    return mpr121_read_regs(handle, reg, val, 1);
}

esp_err_t mpr121_soft_reset(mpr121_handle_t handle) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    return mpr121_write_reg(handle, MPR121_REG_SOFT_RESET, 0x63);
}

esp_err_t mpr121_set_thresholds(mpr121_handle_t handle, uint8_t touch_th, uint8_t release_th) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    // Note: Thresholds must be configured while the device is in Stop Mode (ECR=0x00).
    // Registers 0x41 to 0x5A (26 registers: 13 electrodes * 2)
    uint8_t data[26];
    for (int i = 0; i < 13; i++) {
        data[i * 2] = touch_th;      // Touch
        data[i * 2 + 1] = release_th; // Release
    }

    return mpr121_write_regs(handle, MPR121_REG_TOUCH_TH_0, data, sizeof(data));
}

esp_err_t mpr121_set_electrode_threshold(mpr121_handle_t handle, uint8_t electrode, uint8_t touch_th, uint8_t release_th) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(electrode <= 12, ESP_ERR_INVALID_ARG, TAG, "Invalid electrode index");

    // Calculate start register: 0x41 + (electrode * 2)
    uint8_t data[] = {touch_th, release_th};
    return mpr121_write_regs(handle, (mpr121_reg_t)(MPR121_REG_TOUCH_TH_0 + (electrode * 2)), data, sizeof(data));
}

esp_err_t mpr121_set_debounce(mpr121_handle_t handle, uint8_t touch_cnt, uint8_t release_cnt) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    // Bits 4-6: Touch debounce, Bits 0-2: Release debounce
    uint8_t val = ((touch_cnt & 0x07) << 4) | (release_cnt & 0x07);
    return mpr121_write_reg(handle, MPR121_REG_DEBOUNCE, val);
}

esp_err_t mpr121_read_touch_status(mpr121_handle_t handle, uint16_t *out_status) {
    ESP_RETURN_ON_FALSE(handle && out_status, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    uint8_t data[2];

    esp_err_t ret = mpr121_read_regs(handle, MPR121_REG_TOUCH_STATUS_L, data, sizeof(data));
    if (ret == ESP_OK) {
        // LSB: ELE0-7, MSB: ELE8-12. Mask 0x1F on MSB to ignore status bits (Over Current).
        *out_status = data[0] | ((uint16_t)(data[1] & 0x1F) << 8);
    }
    return ret;
}

esp_err_t mpr121_read_filtered_data(mpr121_handle_t handle, uint8_t electrode, uint16_t *out_data) {
    ESP_RETURN_ON_FALSE(handle && out_data, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    ESP_RETURN_ON_FALSE(electrode <= 12, ESP_ERR_INVALID_ARG, TAG, "Invalid electrode index");

    uint8_t data[2];

    esp_err_t ret = mpr121_read_regs(handle, MPR121_REG_FILT_DATA_0_L + (electrode * 2), data, sizeof(data));
    if (ret == ESP_OK) {
        *out_data = data[0] | ((uint16_t)data[1] << 8);
    }
    return ret;
}

esp_err_t mpr121_read_baseline(mpr121_handle_t handle, uint8_t electrode, uint16_t *out_baseline) {
    ESP_RETURN_ON_FALSE(handle && out_baseline, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    ESP_RETURN_ON_FALSE(electrode <= 12, ESP_ERR_INVALID_ARG, TAG, "Invalid electrode index");

    uint8_t val = 0;

    esp_err_t ret = mpr121_read_reg(handle, MPR121_REG_BASELINE_0 + electrode, &val);
    if (ret == ESP_OK) {
        // Baseline is the 8 MSBs of the 10-bit value. Shift left by 2 to match filtered data scale.
        *out_baseline = (uint16_t)val << 2;
    }
    return ret;
}


/* Baseline Filtering Configuration [1-3] */
esp_err_t mpr121_config_filter_rising(mpr121_handle_t handle, uint8_t mhd, uint8_t nhd, uint8_t ncl, uint8_t fdl) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uint8_t data[] = {mhd, nhd, ncl, fdl};
    return mpr121_write_regs(handle, MPR121_REG_MHD_RISING, data, sizeof(data));
}

esp_err_t mpr121_config_filter_falling(mpr121_handle_t handle, uint8_t mhd, uint8_t nhd, uint8_t ncl, uint8_t fdl) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uint8_t data[] = {mhd, nhd, ncl, fdl};
    return mpr121_write_regs(handle, MPR121_REG_MHD_FALLING, data, sizeof(data));
}

esp_err_t mpr121_config_filter_touched(mpr121_handle_t handle, uint8_t nhd, uint8_t ncl, uint8_t fdl) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uint8_t data[] = {nhd, ncl, fdl};
    return mpr121_write_regs(handle, MPR121_REG_NHD_TOUCHED, data, sizeof(data));
}

/* Proximity (13th Channel) Filtering Configuration [2] */
esp_err_t mpr121_config_proximity_filter_rising(mpr121_handle_t handle, uint8_t mhd, uint8_t nhd, uint8_t ncl, uint8_t fdl) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uint8_t data[] = {mhd, nhd, ncl, fdl};
    return mpr121_write_regs(handle, MPR121_REG_PROX_MHD_RISING, data, sizeof(data));
}

esp_err_t mpr121_config_proximity_filter_falling(mpr121_handle_t handle, uint8_t mhd, uint8_t nhd, uint8_t ncl, uint8_t fdl) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uint8_t data[] = {mhd, nhd, ncl, fdl};
    return mpr121_write_regs(handle, MPR121_REG_PROX_MHD_FALLING, data, sizeof(data));
}

esp_err_t mpr121_config_proximity_filter_touched(mpr121_handle_t handle, uint8_t nhd, uint8_t ncl, uint8_t fdl) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uint8_t data[] = {nhd, ncl, fdl};
    return mpr121_write_regs(handle, MPR121_REG_PROX_NHD_TOUCHED, data, sizeof(data));
}

esp_err_t mpr121_config_auto_calibration(mpr121_handle_t handle, uint8_t target_level, uint8_t usl, uint8_t lsl) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    // Registers 0x7D (USL), 0x7E (LSL), 0x7F (Target)
    uint8_t data[] = {usl, lsl, target_level};
    return mpr121_write_regs(handle, MPR121_REG_USL, data, sizeof(data));
}

esp_err_t mpr121_enable_auto_config(mpr121_handle_t handle, bool enable, bool skip_charge_time) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    if (!enable) {
        uint8_t data[] = {0x00, 0x00};
        return mpr121_write_regs(handle, MPR121_REG_AUTOCFG_0, data, sizeof(data));
    }

    // Enable: ARE=1, ACE=1. BVA=10 (default).
    uint8_t reg0 = 0x0B; 
    // SCTS=1 (skip) or 0.
    uint8_t reg1 = skip_charge_time ? 0x80 : 0x00;
    
    uint8_t data[] = {reg0, reg1};
    return mpr121_write_regs(handle, MPR121_REG_AUTOCFG_0, data, sizeof(data));
}

esp_err_t mpr121_gpio_config(mpr121_handle_t handle, uint16_t electrode_mask, bool is_output) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    
    // GPIOs are ELE4 (bit 4) to ELE11 (bit 11). Registers map ELE4 to bit 0.
    uint8_t mask = (uint8_t)((electrode_mask & 0x0FF0) >> 4);
    if (mask == 0) return ESP_OK;

    // Read 5 registers: CTRL0(0x73), CTRL1(0x74), DATA(0x75), DIR(0x76), EN(0x77)
    uint8_t regs[5];
    esp_err_t ret = mpr121_read_regs(handle, MPR121_REG_GPIO_CTRL_0, regs, 5);
    if (ret != ESP_OK) return ret;

    if (is_output) {
        regs[4] |= mask; // Enable
        regs[3] |= mask; // Dir = 1 (Out)
        regs[0] |= mask; // Ctrl0 = 1 (CMOS)
        regs[1] &= ~mask; // Ctrl1 = 0 (CMOS)
    } else {
        regs[4] |= mask; // Enable
        regs[3] &= ~mask; // Dir = 0 (In)
        regs[0] |= mask; // Ctrl0 = 1 (Pull-up)
        regs[1] |= mask; // Ctrl1 = 1 (Pull-up)
    }

    return mpr121_write_regs(handle, MPR121_REG_GPIO_CTRL_0, regs, 5);
}

esp_err_t mpr121_gpio_set_level(mpr121_handle_t handle, uint16_t electrode_mask, uint8_t level) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uint8_t mask = (uint8_t)((electrode_mask & 0x0FF0) >> 4);
    if (mask == 0) return ESP_OK;

    if (level) {
        return mpr121_write_reg(handle, MPR121_REG_GPIO_SET, mask);
    } else {
        return mpr121_write_reg(handle, MPR121_REG_GPIO_CLEAR, mask);
    }
}

esp_err_t mpr121_gpio_read_level(mpr121_handle_t handle, uint16_t *level_mask) {
    ESP_RETURN_ON_FALSE(handle && level_mask, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    
    uint8_t val = 0;
    esp_err_t ret = mpr121_read_reg(handle, MPR121_REG_GPIO_DATA, &val);
    if (ret == ESP_OK) {
        *level_mask = ((uint16_t)val << 4);
    }
    return ret;
}



esp_err_t mpr121_enable(mpr121_handle_t handle) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    // 1. Calculate the Configuration Byte
    // Start with 0. 
    uint8_t ecr_value = 0;

    // Set Baseline Tracking (CL bits 7-6)
    // Source [2]: 0b10xxxxxx initializes baseline tracking
    if (handle->config.enable_baseline) {
        ecr_value |= 0x80; 
    }

    // Set Electrode Count (ELE bits 3-0)
    // Source [4]: 0x0C (binary 1100) enables 12 electrodes
    // Limit to max 12 to prevent writing invalid bits
    uint8_t count = (handle->config.electrode_count > 12) ? 12 : handle->config.electrode_count;
    
    // The datasheet allows writing 0-12 directly into the low nibble 
    // (though 12 is technically 0x0C, 0x0F also works for 12)
    ecr_value |= count; 

    // 2. Write to the ECR Register
    // This transitions the device from Stop Mode -> Run Mode
    return mpr121_write_reg(handle, MPR121_REG_ECR, ecr_value);
}

esp_err_t mpr121_disable(mpr121_handle_t handle) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    return mpr121_write_reg(handle, MPR121_REG_ECR, 0x00);
}