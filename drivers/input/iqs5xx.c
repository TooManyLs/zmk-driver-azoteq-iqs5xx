/*
 * Copyright (c) 2025 Mariano Uvalle
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT azoteq_iqs5xx

#include <stdlib.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "iqs5xx.h"

LOG_MODULE_REGISTER(iqs5xx, CONFIG_INPUT_LOG_LEVEL);

/*
 * The IQS5xx only accepts I2C traffic during a communication window (RDY
 * asserted). Accessing it outside the window provokes another RDY assertion,
 * so END_COMM_WINDOW must never be sent on the error path — doing so would
 * create an endless interrupt storm.
 */
#define IQS5XX_I2C_RETRIES 5
#define IQS5XX_I2C_RETRY_DELAY_US 250

/* Stop servicing RDY after consecutive failures to prevent CPU saturation. */
#define IQS5XX_MAX_CONSECUTIVE_FAILURES 8
#define IQS5XX_RECOVERY_PERIOD K_SECONDS(2)

static int iqs5xx_rdy_interrupt_set(const struct device *dev, bool enable) {
    const struct iqs5xx_config *config = dev->config;

    return gpio_pin_interrupt_configure_dt(&config->rdy_gpio,
                                           enable ? GPIO_INT_EDGE_RISING : GPIO_INT_DISABLE);
}

static int iqs5xx_write_buf(const struct device *dev, const uint8_t *buf, size_t len) {
    const struct iqs5xx_config *config = dev->config;
    int ret = -EIO;

    for (int i = 0; i < IQS5XX_I2C_RETRIES; i++) {
        ret = i2c_write_dt(&config->i2c, buf, len);
        if (ret == 0) {
            return 0;
        }

        k_usleep(IQS5XX_I2C_RETRY_DELAY_US);
    }

    return ret;
}

static int iqs5xx_read_buf(const struct device *dev, uint16_t reg, uint8_t *buf, size_t len) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
    int ret = -EIO;

    for (int i = 0; i < IQS5XX_I2C_RETRIES; i++) {
        ret = i2c_write_read_dt(&config->i2c, reg_buf, sizeof(reg_buf), buf, len);
        if (ret == 0) {
            return 0;
        }

        k_usleep(IQS5XX_I2C_RETRY_DELAY_US);
    }

    return ret;
}

static int iqs5xx_write_reg8(const struct device *dev, uint16_t reg, uint8_t val) {
    uint8_t buf[3] = {reg >> 8, reg & 0xFF, val};

    return iqs5xx_write_buf(dev, buf, sizeof(buf));
}

static int iqs5xx_write_reg16(const struct device *dev, uint16_t reg, uint16_t val) {
    uint8_t buf[4] = {reg >> 8, reg & 0xFF, val >> 8, val & 0xFF};

    return iqs5xx_write_buf(dev, buf, sizeof(buf));
}

static int iqs5xx_end_comm_window(const struct device *dev) {
    uint8_t buf[3] = {IQS5XX_END_COMM_WINDOW >> 8, IQS5XX_END_COMM_WINDOW & 0xFF, 0x00};

    return iqs5xx_write_buf(dev, buf, sizeof(buf));
}

static int iqs5xx_setup_device(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    int ret;

    ret = iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONFIG_1,
                            IQS5XX_EVENT_MODE | IQS5XX_TP_EVENT | IQS5XX_GESTURE_EVENT);
    if (ret < 0) {
        LOG_ERR("Failed to configure event mode: %d", ret);
        return ret;
    }

    ret = iqs5xx_write_reg8(dev, IQS5XX_BOTTOM_BETA, config->bottom_beta);
    if (ret < 0) {
        LOG_ERR("Failed to set bottom beta: %d", ret);
        return ret;
    }

    ret = iqs5xx_write_reg8(dev, IQS5XX_STATIONARY_THRESH, config->stationary_threshold);
    if (ret < 0) {
        LOG_ERR("Failed to set bottom stationary threshold: %d", ret);
        return ret;
    }

    // TODO: Expose these through dts bindings.
    ret = iqs5xx_write_reg8(dev, IQS5XX_FILTER_SETTINGS,
                            IQS5XX_IIR_FILTER | IQS5XX_MAV_FILTER | IQS5XX_ALP_COUNT_FILTER);
    if (ret < 0) {
        LOG_ERR("Failed to configure filter settings: %d", ret);
        return ret;
    }

    uint8_t single_finger_gestures = 0;
    single_finger_gestures |= config->one_finger_tap ? IQS5XX_SINGLE_TAP : 0;
    single_finger_gestures |= config->press_and_hold ? IQS5XX_PRESS_AND_HOLD : 0;
    ret = iqs5xx_write_reg8(dev, IQS5XX_SINGLE_FINGER_GESTURES_CONF, single_finger_gestures);
    if (ret < 0) {
        LOG_ERR("Failed to configure single finger gestures: %d", ret);
        return ret;
    }

    ret = iqs5xx_write_reg16(dev, IQS5XX_HOLD_TIME, config->press_and_hold_time);
    if (ret < 0) {
        LOG_ERR("Failed to configure the hold time: %d", ret);
        return ret;
    }

    uint8_t two_finger_gestures = 0;
    two_finger_gestures |= config->two_finger_tap ? IQS5XX_TWO_FINGER_TAP : 0;
    two_finger_gestures |= config->scroll ? IQS5XX_SCROLL : 0;
    ret = iqs5xx_write_reg8(dev, IQS5XX_MULTI_FINGER_GESTURES_CONF, two_finger_gestures);
    if (ret < 0) {
        LOG_ERR("Failed to configure multi finger gestures: %d", ret);
        return ret;
    }

    uint8_t xy_config = 0;
    xy_config |= config->flip_x ? IQS5XX_FLIP_X : 0;
    xy_config |= config->flip_y ? IQS5XX_FLIP_Y : 0;
    xy_config |= config->switch_xy ? IQS5XX_SWITCH_XY_AXIS : 0;
    ret = iqs5xx_write_reg8(dev, IQS5XX_XY_CONFIG_0, xy_config);
    if (ret < 0) {
        LOG_ERR("Failed to configure axes: %d", ret);
        return ret;
    }

    ret = iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONFIG_0, IQS5XX_SETUP_COMPLETE | IQS5XX_WDT);
    if (ret < 0) {
        LOG_ERR("Failed to configure system: %d", ret);
        return ret;
    }

    ret = iqs5xx_end_comm_window(dev);
    if (ret < 0) {
        LOG_ERR("Failed to end comm window during initialization: %d", ret);
        return ret;
    }

    return 0;
}

static int iqs5xx_reconfigure(struct iqs5xx_data *data) {
    const struct device *dev = data->dev;
    int ret;

    data->rdy_armed = false;
    iqs5xx_rdy_interrupt_set(dev, false);

    ret = iqs5xx_setup_device(dev);
    if (ret < 0) {
        return ret;
    }

    ret = iqs5xx_rdy_interrupt_set(dev, true);
    if (ret < 0) {
        LOG_ERR("Failed to arm the data-ready interrupt: %d", ret);
        return ret;
    }

    data->consecutive_failures = 0;
    data->rdy_armed = true;

    return 0;
}

static void iqs5xx_suspend_data_ready(struct iqs5xx_data *data) {
    data->rdy_armed = false;
    iqs5xx_rdy_interrupt_set(data->dev, false);
    k_work_schedule(&data->recovery_work, IQS5XX_RECOVERY_PERIOD);
}

static void iqs5xx_note_success(struct iqs5xx_data *data) {
    if (data->consecutive_failures != 0) {
        LOG_INF("IQS5xx responding again");
        data->consecutive_failures = 0;
    }
}

static void iqs5xx_note_failure(struct iqs5xx_data *data, const char *what, int err) {
    if (data->consecutive_failures == 0) {
        LOG_ERR("Failed to %s: %d", what, err);
    }

    if (data->consecutive_failures < IQS5XX_MAX_CONSECUTIVE_FAILURES) {
        data->consecutive_failures++;
    }

    if (data->consecutive_failures == IQS5XX_MAX_CONSECUTIVE_FAILURES && data->rdy_armed) {
        LOG_WRN("IQS5xx stopped responding, masking the data-ready interrupt");
        iqs5xx_suspend_data_ready(data);
    }
}

static void iqs5xx_button_release_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs5xx_data *data = CONTAINER_OF(dwork, struct iqs5xx_data, button_release_work);

    int btn_idx = data->pending_release_btn;
    if (btn_idx >= 0) {
        uint8_t old_val = data->buttons_pressed;
        data->buttons_pressed &= ~(1 << btn_idx);

        if (old_val & (1 << btn_idx)) {
            if (input_report_key(data->dev, INPUT_BTN_0 + btn_idx, 0, true, K_NO_WAIT) < 0) {
                LOG_WRN("Dropped synthetic button release");
            }
        }
        data->pending_release_btn = -1;
    }
}

static void iqs5xx_recovery_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs5xx_data *data = CONTAINER_OF(dwork, struct iqs5xx_data, recovery_work);

    data->consecutive_failures = 0;

    if (iqs5xx_reconfigure(data) < 0) {
        LOG_DBG("IQS5xx still not responding, retrying later");
        data->consecutive_failures = 0;
        k_work_schedule(&data->recovery_work, IQS5XX_RECOVERY_PERIOD);
        return;
    }

    LOG_INF("IQS5xx trackpad re-initialized");
}

static void iqs5xx_work_handler(struct k_work *work) {
    struct iqs5xx_data *data = CONTAINER_OF(work, struct iqs5xx_data, work);
    const struct device *dev = data->dev;
    const struct iqs5xx_config *config = dev->config;
    uint8_t block[IQS5XX_DATA_BLOCK_LEN];
    int ret;

    if (!data->initialized) {
        return;
    }

    ret = iqs5xx_read_buf(dev, IQS5XX_DATA_BLOCK_START, block, sizeof(block));
    if (ret < 0) {
        iqs5xx_note_failure(data, "read the data block", ret);
        return;
    }

    iqs5xx_note_success(data);

    uint8_t gesture_events_0 = block[0];
    uint8_t gesture_events_1 = block[1];
    uint8_t sys_info_0 = block[2];
    uint8_t sys_info_1 = block[3];
    int16_t rel_x = (int16_t)((block[5] << 8) | block[6]);
    int16_t rel_y = (int16_t)((block[7] << 8) | block[8]);

    if (sys_info_0 & IQS5XX_SHOW_RESET) {
        LOG_INF("Device reset detected");

        ret = iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONTROL_0, IQS5XX_ACK_RESET);
        if (ret < 0) {
            iqs5xx_note_failure(data, "acknowledge the device reset", ret);
            return;
        }

        if (iqs5xx_reconfigure(data) < 0) {
            LOG_ERR("Failed to re-initialize the device after a reset");
            iqs5xx_suspend_data_ready(data);
        }

        return;
    }

    bool tp_movement = (sys_info_1 & IQS5XX_TP_MOVEMENT) != 0;
    bool scroll = (gesture_events_1 & IQS5XX_SCROLL) != 0;
    if (!scroll) {
        data->scroll_x_acc = 0;
        data->scroll_y_acc = 0;
    }

    uint16_t button_code;
    bool button_pressed = false;
    if (gesture_events_0 & IQS5XX_SINGLE_TAP) {
        button_pressed = true;
        button_code = INPUT_BTN_0;
    } else if (gesture_events_1 & IQS5XX_TWO_FINGER_TAP) {
        button_pressed = true;
        button_code = INPUT_BTN_1;
    }

    bool hold_became_active = (gesture_events_0 & IQS5XX_PRESS_AND_HOLD) && !data->active_hold;
    bool hold_released = !(gesture_events_0 & IQS5XX_PRESS_AND_HOLD) && data->active_hold;

    if (hold_became_active) {
        LOG_INF("Hold became active");
        data->active_hold = true;
        if (input_report_key(dev, LEFT_BUTTON_CODE, 1, true, K_NO_WAIT) < 0) {
            LOG_WRN("Dropped hold press event");
        }
    } else if (hold_released) {
        LOG_INF("Hold became inactive");
        data->active_hold = false;
        if (input_report_key(dev, LEFT_BUTTON_CODE, 0, true, K_NO_WAIT) < 0) {
            LOG_WRN("Dropped hold release event");
        }
    } else if (button_pressed) {
        k_work_cancel_delayable(&data->button_release_work);

        int btn_idx = button_code - INPUT_BTN_0;

        data->buttons_pressed |= (1 << btn_idx);
        if (input_report_key(dev, button_code, 1, true, K_NO_WAIT) < 0) {
            LOG_WRN("Dropped button press event");
        }

        data->pending_release_btn = btn_idx;
        k_work_schedule(&data->button_release_work, K_MSEC(25));

    } else if (scroll) {
        // TODO: Expose this divisor.
        int16_t scroll_div = 32;

        if (rel_x != 0) {
            if (!config->natural_scroll_x) {
                rel_x *= -1;
            }
            data->scroll_x_acc += rel_x;
            if (abs(data->scroll_x_acc) >= scroll_div) {
                if (input_report_rel(dev, INPUT_REL_HWHEEL, data->scroll_x_acc / scroll_div, true, K_NO_WAIT) < 0) {
                    LOG_WRN("Dropped horizontal scroll event");
                }
                data->scroll_x_acc %= scroll_div;
            }
            goto end_comm;
        }
        if (rel_y != 0) {
            if (config->natural_scroll_y) {
                rel_y *= -1;
            }
            data->scroll_y_acc += rel_y;
            if (abs(data->scroll_y_acc) >= scroll_div) {
                if (input_report_rel(dev, INPUT_REL_WHEEL, data->scroll_y_acc / scroll_div, true, K_NO_WAIT) < 0) {
                    LOG_WRN("Dropped vertical scroll event");
                }
                data->scroll_y_acc %= scroll_div;
            }

            goto end_comm;
        }
    } else if (tp_movement) {
        if (rel_x != 0 || rel_y != 0) {
            input_report_rel(dev, INPUT_REL_X, rel_x, false, K_NO_WAIT);
            input_report_rel(dev, INPUT_REL_Y, rel_y, true, K_NO_WAIT);
        }
    }

end_comm:
    ret = iqs5xx_end_comm_window(dev);
    if (ret < 0) {
        iqs5xx_note_failure(data, "close the communication window", ret);
    }
}

static void iqs5xx_rdy_handler(const struct device *port, struct gpio_callback *cb,
                               gpio_port_pins_t pins) {
    struct iqs5xx_data *data = CONTAINER_OF(cb, struct iqs5xx_data, rdy_cb);

    if (!data->initialized || !data->rdy_armed) {
        return;
    }

    k_work_submit(&data->work);
}

static int iqs5xx_init(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    struct iqs5xx_data *data = dev->data;
    int ret;

    if (!i2c_is_ready_dt(&config->i2c)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    data->dev = dev;
    k_work_init(&data->work, iqs5xx_work_handler);
    k_work_init_delayable(&data->button_release_work, iqs5xx_button_release_work_handler);
    k_work_init_delayable(&data->recovery_work, iqs5xx_recovery_work_handler);

    data->initialized = false;
    data->rdy_armed = false;
    data->buttons_pressed = 0;
    data->pending_release_btn = -1;
    data->scroll_x_acc = 0;
    data->scroll_y_acc = 0;
    data->consecutive_failures = 0;
    data->active_hold = false;

    if (config->reset_gpio.port) {
        if (!gpio_is_ready_dt(&config->reset_gpio)) {
            LOG_ERR("Reset GPIO not ready");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure reset GPIO: %d", ret);
            return ret;
        }

        gpio_pin_set_dt(&config->reset_gpio, 1);
        k_msleep(10);
        gpio_pin_set_dt(&config->reset_gpio, 0);
        k_msleep(200);
    }

    if (!gpio_is_ready_dt(&config->rdy_gpio)) {
        LOG_ERR("RDY GPIO not ready");
        return -ENODEV;
    }

    gpio_flags_t rdy_flags = GPIO_INPUT;
    if (!(config->rdy_gpio.dt_flags & (GPIO_PULL_UP | GPIO_PULL_DOWN))) {
        rdy_flags |= GPIO_PULL_DOWN;
    }

    ret = gpio_pin_configure_dt(&config->rdy_gpio, rdy_flags);
    if (ret < 0) {
        LOG_ERR("Failed to configure RDY GPIO: %d", ret);
        return ret;
    }

    gpio_init_callback(&data->rdy_cb, iqs5xx_rdy_handler, BIT(config->rdy_gpio.pin));
    ret = gpio_add_callback(config->rdy_gpio.port, &data->rdy_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add RDY callback: %d", ret);
        return ret;
    }

    if (iqs5xx_reconfigure(data) < 0) {
        // The trackpad may simply not be connected. Keep the keyboard usable and
        // re-check at a harmless rate instead of failing the device - or, far
        // worse, retrying at interrupt rate.
        LOG_WRN("IQS5xx not responding, will keep retrying in the background");
        data->consecutive_failures = 0;
        iqs5xx_suspend_data_ready(data);
    } else {
        LOG_INF("IQS5xx trackpad initialized");
    }

    data->initialized = true;

    return 0;
}

#define IQS5XX_INIT(n)                                                                             \
    static struct iqs5xx_data iqs5xx_data_##n;                                                     \
    static const struct iqs5xx_config iqs5xx_config_##n = {                                        \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                                            \
        .rdy_gpio = GPIO_DT_SPEC_INST_GET(n, rdy_gpios),                                           \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),                               \
        .one_finger_tap = DT_INST_PROP(n, one_finger_tap),                                         \
        .press_and_hold = DT_INST_PROP(n, press_and_hold),                                         \
        .two_finger_tap = DT_INST_PROP(n, two_finger_tap),                                         \
        .scroll = DT_INST_PROP(n, scroll),                                                         \
        .natural_scroll_x = DT_INST_PROP(n, natural_scroll_x),                                     \
        .natural_scroll_y = DT_INST_PROP(n, natural_scroll_y),                                     \
        .press_and_hold_time = DT_INST_PROP_OR(n, press_and_hold_time, 250),                       \
        .switch_xy = DT_INST_PROP(n, switch_xy),                                                   \
        .flip_x = DT_INST_PROP(n, flip_x),                                                         \
        .flip_y = DT_INST_PROP(n, flip_y),                                                         \
        .bottom_beta = DT_INST_PROP_OR(n, bottom_beta, 5),                                         \
        .stationary_threshold = DT_INST_PROP_OR(n, stationary_threshold, 5),                       \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, iqs5xx_init, NULL, &iqs5xx_data_##n, &iqs5xx_config_##n, POST_KERNEL, \
                          CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS5XX_INIT)