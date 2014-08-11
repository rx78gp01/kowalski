/*
 * drivers/input/touchscreen/kowalski_touch_driver.c
 *
 * Copyright (c) 2014, Firtecy <admin@firtecy.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/freezer.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <linux/semaphore.h>
#include <linux/irq.h>
#include <linux/freezer.h>
#include <linux/input/kowalski_touch_driver.h>

#ifdef CONFIG_TOUCHSCREEN_DOUBLETAP2WAKE
#include <linux/input/doubletap2wake.h>
#endif

#define init_MUTEX_LOCKED(sem)    sema_init(sem, 0)

#define STAR_FW_UPGRADE
#define STAR_TOUCH_GRIP_SUPPRESSION
#define STAR_FW_VERSION

#define LGE_NOMELT

#define GET_BIT_MASK(_finger_state_reg)    \
    (_finger_state_reg[2] & 0x04)<<7 | (_finger_state_reg[2] & 0x01)<<8 |    \
(_finger_state_reg[1] & 0x40)<<1 | (_finger_state_reg[1] & 0x10)<<2 |(_finger_state_reg[1] & 0x04)<<3 | (_finger_state_reg[1] & 0x01)<<4 |    \
(_finger_state_reg[0] & 0x40)>>3 | (_finger_state_reg[0] & 0x10)>>2 |(_finger_state_reg[0] & 0x04)>>1 | (_finger_state_reg[0] & 0x01)

#define GET_INDEX_FROM_MASK(_index, _bit_mask)    \
    for(; !((_bit_mask>>_index)&0x01) && _index <= SYNAPTICS_FINGER_MAX; _index++);    \
if(_index <= SYNAPTICS_FINGER_MAX) _bit_mask &= ~(_bit_mask & (1<<(_index)));


#define TS_SNTS_GET_X_POSITION(_high_reg, _low_reg) \
    ( ((u16)((_high_reg << 4) & 0x000007F0) | (u16)(_low_reg&0x0F)) * (LGE_TOUCH_RESOLUTION_X - 1) / 1036)
#define TS_SNTS_GET_Y_POSITION(_high_reg, _low_reg) \
    ( ((u16)((_high_reg << 4) & 0x000007F0) | (u16)((_low_reg >> 4) & 0x0F)) * (LGE_TOUCH_RESOLUTION_Y - 1) / 1728)
#define TS_SNTS_GET_WIDTH(_width) \
    ((((_width & 0xf0) >> 4) - (_width & 0x0f)) > 0)? (_width & 0xf0) >> 4 : _width & 0x0f
#define TS_SNTS_GET_PRESSURE(_pressure) \
    _pressure

#if defined (LGE_NOMELT)
#define TS_SNTS_GET_LOWDATA_X_POSITION(_high_reg, _low_reg) \
    ((u16)((_high_reg << 4) & 0x000007F0) | (u16)(_low_reg&0x0F))
#define TS_SNTS_GET_LOWDATA_Y_POSITION(_high_reg, _low_reg) \
    ((u16)((_high_reg << 4) & 0x000007F0) | (u16)((_low_reg >> 4) & 0x0F))
#endif

enum {X_HIGH_POSITION=0, Y_HIGH_POSITION, XY_LOW_POSITION, XY_WIDTH, PRESSURE};

#define ADJUST_LEVEL 5
#define SQUARE(x)        ((x) * (x))

const u16 ADJUST_FACTOR_LEVEL[ADJUST_LEVEL] = {8, 6, 4, 2, 1};
const u16 ADJUST_BASIS_LEVEL[ADJUST_LEVEL] = {5, 11, 19, 29, 40};
const u16 ADJUST_FACTOR_BASE = 4;

static const kowalski_touch_device_capabilities synaptics_capabilities =
{
    1,    //IsMultiTouchSupported
    1,    //isButtonSupported
    10,   //MaxNumberOfFingerCoordReported;
    0,    //IsRelativeDataSupported
    0,    //MaxNumberOfRelativeCoordReported
    15,   //MaxNumberOfWidthReported
    0xFF, //MaxNumberOfPressureReported
    0,    //Gesture
    0,    //IsWidthSupported
    0,    //IsPressureSupported, mandatory for multi-touch
    0,    //IsFingersSupported
    0,    //XMinPosition
    0,    //YMinPosition
    LGE_TOUCH_RESOLUTION_X-1,    //XMaxPosition
    LGE_TOUCH_RESOLUTION_Y-1,    //YMaxPosition
    0,
};

static ts_sensor_data ts_reg_data={0};

#if defined (LGE_NOMELT)
static char mode=1,
            numfinger=0,
            reportcnt=0;
static int current_data_x,
           current_data_y,
           pre_x,
           pre_y,
           first_x,
           first_y;
#endif

extern bool is_star_touch_enable;
bool is_irq_already_free = true;
bool is_irq_already_enabled = false;

bool touch_irq_wake = 0;
#ifdef CONFIG_TOUCHSCREEN_DOUBLETAP2WAKE
#define SHOULD_PREVENT_SLEEP (dt2w_switch == 0)
#else
#define SHOULD_PREVENT_SLEEP (false)
#endif

#ifdef STAR_FW_UPGRADE
#include <linux/input/synaptics_ts_firmware.h>
static int synaptics_ts_fw_upgrade(synaptics_touch_device* hTouch);
#endif

#if defined (LGE_NOMELT)
static void synaptics_set_no_melt_mode (void* h_dev, bool binit);
#endif /* LGE_NOMELT */

#ifdef STAR_TOUCH_GRIP_SUPPRESSION
#define READ_BUFFER_LENGTH 20
u8 touch_grip_suppression_value = 0;

#define IGNORE_IF_POSITION_IS_SUPPRESSION_AREA(_x)    \
    if(_x < touch_grip_suppression_value || _x > STAR_TOUCH_RESOLUTION_X - touch_grip_suppression_value)    \
continue;

/* Sys file attributes for gripsuppression */
ssize_t touch_gripsuppression_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    sprintf(buf, "%d\n", touch_grip_suppression_value);    
    return (ssize_t)(strlen(buf)+1);
}

ssize_t touch_gripsuppression_store(struct device *dev, struct device_attribute *attr, const char *buffer, size_t count)
{
    u8 *input;
    u32 value;

    count = count > READ_BUFFER_LENGTH ? READ_BUFFER_LENGTH : count;
    input = kzalloc(((int)count+1), GFP_KERNEL);
    if (!input) {
        return 0;
    }

    memcpy(input, buffer, count);

    input[count] = '\0';
    value = simple_strtoul(&input[0], '\0', 10);

    touch_grip_suppression_value = value;

    kfree(input);
    return count;
}

DEVICE_ATTR(gripsuppression, 0666, touch_gripsuppression_show, touch_gripsuppression_store);

#endif /* FEATURE_LGE_TOUCH_GRIP_SUPPRESSION */

//Defined
static irqreturn_t touch_irq_handler(int irq, void *dev_id);
int task_handler(void *pdata);

#if defined (STAR_FW_VERSION)
static unsigned char synaptics_get_fw_version(struct i2c_client *client);
static ssize_t show_fw_revision(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    unsigned char firmware_version = 0;
    int ret = 0;

    firmware_version = synaptics_get_fw_version(client);

    ret = sprintf(buf, "%d\n", firmware_version);
    return ret;
}
static DEVICE_ATTR(fw_ver, S_IRUGO | S_IWUSR, show_fw_revision, NULL);
#endif /* STAR_FW_VERSION */

static void tegra_touch_adjust_position(const u16 value_x, const u16 value_y, u16 *adjust_x, u16 *adjust_y)
{
    //Calculate the distance between the adjust_(x|y) and value_(x|y)
    u16 distant = int_sqrt(SQUARE(*adjust_x - value_x) + SQUARE(*adjust_y - value_y));
    u16 i;

    for(i = 0; i < ADJUST_LEVEL; i++){
        if(distant <= ADJUST_BASIS_LEVEL[i]){
            *adjust_x = (value_x * ADJUST_FACTOR_LEVEL[i] + *adjust_x * ADJUST_FACTOR_BASE) 
                / (ADJUST_FACTOR_LEVEL[i] + ADJUST_FACTOR_BASE);
            *adjust_y = (value_y * ADJUST_FACTOR_LEVEL[i] + *adjust_y * ADJUST_FACTOR_BASE) 
                / (ADJUST_FACTOR_LEVEL[i] + ADJUST_FACTOR_BASE);
            break;
        }
    }

    if(*adjust_y < LGE_TOUCH_RESOLUTION_Y + 20 && *adjust_y >= LGE_TOUCH_RESOLUTION_Y)
        *adjust_y = LGE_TOUCH_RESOLUTION_Y - 1;
}

void synaptics_device_close (synaptics_touch_device* hTouch)
{
    kfree(hTouch);
}

bool synaptics_device_open(synaptics_touch_device* h_touch, struct i2c_client *client, u32 touch)
{
    synaptics_touch_device* hTouch = (synaptics_touch_device*)0;
    struct star_synaptics_platform_data *pdata;
    
    // Allocate safely or abort open
    if(unlikely((hTouch = kzalloc(sizeof(synaptics_touch_device), GFP_KERNEL)) == NULL)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        goto err_synaptics_device_open;
    }

    memset(hTouch, 0, sizeof(synaptics_touch_device));

    if(unlikely(i2c_check_functionality(client->adapter, I2C_FUNC_I2C) == 0)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        goto err_synaptics_device_open;
    }

    hTouch->client = client;

    if(unlikely((pdata = client->dev.platform_data) == NULL)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        goto err_synaptics_device_open;
    }

    hTouch->flags       = pdata->irqflags /*| IRQF_NO_SUSPEND | IRQF_ONESHOT*/;
    hTouch->power       = pdata->power;
    hTouch->gpio        = pdata->gpio;
    hTouch->irq_gpio    = client->irq;
    hTouch->touch       = touch;

    memcpy(&hTouch->caps, &synaptics_capabilities, sizeof(kowalski_touch_device_capabilities));

#ifdef STAR_TOUCH_GRIP_SUPPRESSION
    if(unlikely(device_create_file(&client->dev, &dev_attr_gripsuppression) != 0)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        goto err_synaptics_device_open;
    }
#endif    

#if defined (STAR_FW_VERSION)
    if(unlikely(device_create_file(&client->dev, &dev_attr_fw_ver) != 0)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        goto err_synaptics_device_open;
    }
    DEBUG_MSG(3, "[TOUCH_FW] Device File Created\n");
#endif

    *h_touch = *hTouch;

    return true;

err_synaptics_device_open:
    synaptics_device_close(hTouch);
    return false;
} 

static int synaptics_enable_irq_wake(synaptics_touch_device* st_device)
{
	int ret = 0;
	if(!touch_irq_wake) {
		touch_irq_wake = true;
		ret = enable_irq_wake(st_device->irq_gpio);
	}
	return ret;
}

static int synaptics_disable_irq_wake(synaptics_touch_device *st_device)
{
	int ret = 0;
	if(touch_irq_wake) {
		touch_irq_wake = false;
		ret = disable_irq_wake(st_device->irq_gpio);
	}
	return ret;
}

bool input_device_open(synaptics_touch_device* hTouch)
{
    struct input_dev* input_dev;

    if(unlikely((hTouch->input_dev = input_allocate_device()) == NULL)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        goto err_input_device_open2;
    }

    input_dev = hTouch->input_dev;
    input_dev->name = LGE_TOUCH_NAME;

    set_bit(EV_SYN, input_dev->evbit);
    set_bit(EV_KEY, input_dev->evbit);
    set_bit(EV_ABS, input_dev->evbit);
    set_bit(KEY_MENU, input_dev->keybit);
    set_bit(KEY_BACK, input_dev->keybit);
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_P999)
    set_bit(KEY_HOMEPAGE, input_dev->keybit);
#elif defined (CONFIG_MACH_STAR_SU660)
    set_bit(KEY_HOME, input_dev->keybit);
#endif
    set_bit(KEY_SEARCH, input_dev->keybit);

    input_set_abs_params(input_dev, ABS_MT_POSITION_X, hTouch->caps.XMinPosition, hTouch->caps.XMaxPosition, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, hTouch->caps.YMinPosition, hTouch->caps.YMaxPosition, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, hTouch->caps.MaxNumberOfPressureReported , 0, 0);
    input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, hTouch->caps.MaxNumberOfWidthReported, 0, 0);

    if(unlikely(!input_register_device(input_dev) == 0)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        goto err_input_device_open;
    }
    return true;

err_input_device_open:
    input_unregister_device(hTouch->input_dev);
err_input_device_open2:
    input_free_device(hTouch->input_dev);
    return false;
}

void input_device_close(synaptics_touch_device* hTouch)
{
    input_unregister_device(hTouch->input_dev);
    input_free_device(hTouch->input_dev);
}

bool input_device_send_abs(synaptics_touch_device* hTouch, ts_finger_data data, u8 isPress)
{
    if(unlikely(hTouch->input_dev == NULL)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        goto err_input_device_send_abs;
    }

    if(isPress){
        input_report_abs(hTouch->input_dev, ABS_MT_POSITION_X, data.X_position);
        input_report_abs(hTouch->input_dev, ABS_MT_POSITION_Y, data.Y_position);
        input_report_abs(hTouch->input_dev, ABS_MT_TOUCH_MAJOR, data.pressure);
        input_report_abs(hTouch->input_dev, ABS_MT_WIDTH_MAJOR, data.width);
    }
    input_mt_sync(hTouch->input_dev);

    return true;

err_input_device_send_abs:
    return false;
}

bool input_device_send_abs_for_multi(synaptics_touch_device* hTouch, ts_finger_data* data, u8 total_num)
{
    int i=0, check=0;

    if(unlikely(hTouch->input_dev == NULL)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        return false;
    }

    for(i=0; i<total_num; i++){
        if(IS_PANEL(data[i].Y_position)){
            input_report_abs(hTouch->input_dev, ABS_MT_POSITION_X, data[i].X_position);
            input_report_abs(hTouch->input_dev, ABS_MT_POSITION_Y, data[i].Y_position);
            input_report_abs(hTouch->input_dev, ABS_MT_TOUCH_MAJOR, data[i].pressure);
            input_report_abs(hTouch->input_dev, ABS_MT_WIDTH_MAJOR, data[i].width);
            input_mt_sync(hTouch->input_dev);
            DEBUG_MSG(M, "[TOUCH] X[%d], Y[%d]\n", (int)data[i].X_position, (int)data[i].Y_position);
            check++;
        }
    }

    if(!check){
        DEBUG_MSG(M, "[TOUCH] mt_sync. \n");
        input_mt_sync(hTouch->input_dev);
    }

    return true;
}


bool input_device_send_button(synaptics_touch_device* hTouch, u32 type, bool isPress)
{
    if(unlikely(hTouch->input_dev == NULL)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        return false;
    }

    input_report_key(hTouch->input_dev, type, isPress);
    return true;  
}

bool input_device_send_sync(synaptics_touch_device* hTouch, u8 state)
{
    if(unlikely(hTouch->input_dev == NULL)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        return false;
    }

    //We should not sync when the touch is actually locked 
    //since that will not be a event that is needed
    if(state != TOUCH_LOCK){
        input_sync(hTouch->input_dev);
        DEBUG_MSG(M, "[TOUCH] sync.\n");
    }

    return true;
}

bool synaptics_task_open(synaptics_touch_device* hTouch)
{
    if(unlikely((hTouch->task = kthread_create(task_handler, (void*)hTouch->touch, "LGE_touch_thread")) == NULL)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        return false;
    }
	return true;	
}

void synaptics_task_close(synaptics_touch_device* h_dev)
{

}

bool synaptics_task_start(synaptics_touch_device* hTouch)
{
    wake_up_process(hTouch->task);
    return true;
}

bool synaptics_task_stop(synaptics_touch_device* hTouch)
{
    kthread_stop(hTouch->task);
    return true; 
}

bool synaptics_task_is_stopped(synaptics_touch_device* hTouch)
{
    return (hTouch->task->state > 0); 
}

void interrupt_close(synaptics_touch_device* hTouch)
{
    if(!is_irq_already_free) {
        free_irq(hTouch->irq_gpio, (void*)hTouch->touch);
    }
}

bool interrupt_open(synaptics_touch_device* hTouch)
{
    int ret;
    
    init_MUTEX_LOCKED(&hTouch->sem);
    
    ret = request_threaded_irq(hTouch->irq_gpio, //Interrupt line to allocate
                               NULL, //Function to be called when the IRQ occurs
                               touch_irq_handler,   //Function called from the irq handler
                               hTouch->flags, //Interrupt type flags
                               LGE_TOUCH_NAME, //An ascii name for the claiming device
                               (void*)hTouch->touch); //A cookie passed back to the handler function
    
    if(unlikely(!ret == 0)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        
        //Make sure to close/free the irq if we failed
        interrupt_close((void*)hTouch);
        return false;
    }
    
    is_irq_already_free = false;
    return true;    
}

bool interrupt_enable(synaptics_touch_device* hTouch)
{
    if(!is_irq_already_enabled) {
        enable_irq(hTouch->irq_gpio);
        is_irq_already_enabled = true;
    }
    
    return true;
}

bool interrupt_disable(synaptics_touch_device* hTouch)
{
    if (is_irq_already_enabled) {
        disable_irq_nosync(hTouch->irq_gpio);
        is_irq_already_enabled = false;
    }
    return true;
}

/*
 * This function is used to unlock the semaphore. If it gets unlocked the
 * touchthread will continue
 */
bool interrupt_start(synaptics_touch_device* hTouch)
{
    printk("[TOUCH] %s() called and unlocking semaphore\n", __FUNCTION__);
    up(&hTouch->sem);

    return true;
}

#define WAIT_TOUCH_POWER_READY(_client, _num)            \
                                \
int retry = _num;                        \
while (retry-- > 0) {                        \
    int ret = i2c_smbus_read_byte_data(_client, 0xb8);    \
    if (ret >= 0)                        \
    break;                            \
    msleep(100);                        \
}                                \
}

bool interrupt_wait(synaptics_touch_device* hTouch)
{
    int ret;

    do {
        ret = down_interruptible(&hTouch->sem);
        if (ret && !try_to_freeze())
            schedule();
    } while (ret);
    return true;
}

bool synaptics_power_control_on_off(synaptics_touch_device* hTouch, bool onoff)
{
    int ret;
#if defined CONFIG_MACH_STAR_P990 || defined CONFIG_MACH_STAR_P999
#else
    ret = hTouch->power("vdd_onetouch", onoff);
    if(unlikely(ret < 0)) return false;
#endif

    ret = hTouch->power("vcc_touch_3v1", onoff);
    if(unlikely(ret < 0)) return false;
    
    msleep(300);

    ret = hTouch->power("vcc_touch_1v8", onoff);
    if(unlikely(ret < 0)) return false;
    
    if(onoff && !is_star_touch_enable)   
        msleep(400);

    is_star_touch_enable = onoff;
    return true;
}

bool synaptics_power_control(synaptics_touch_device* hTouch, int request_state)
{
    switch(request_state) {
        case SYNAPTICS_POWER_STATE_ON:
            return synaptics_power_control_on_off(hTouch, true);
        case SYNAPTICS_POWER_STATE_OFF:
            return synaptics_power_control_on_off(hTouch, false);
        case SYNAPTICS_POWER_STATE_SLEEP:
            {
                u8 DeviceControl;
                DeviceControl = i2c_smbus_read_byte_data(hTouch->client, SYNAPTICS_DEVICE_CONTROL_REG);
                i2c_smbus_write_byte_data(hTouch->client,
                                      SYNAPTICS_DEVICE_CONTROL_REG,
                                      ((DeviceControl & 0xFC) | SYNAPTICS_DEVICE_SENSOR_SLEEP));
            }
            return true;
        case SYNAPTICS_POWER_STATE_WAKE:
            {
                u8 DeviceControl;
#if defined (LGE_NOMELT)
                // Activate melt 
                i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_MELT_CONTROL, SYNAPTICS_MELT);
                // Reset the values for no melt
                mode = 1;
                numfinger = 0;
                reportcnt = 0;
#endif /* LGE_NOMELT */

                DeviceControl = i2c_smbus_read_byte_data(hTouch->client, SYNAPTICS_DEVICE_CONTROL_REG);
                i2c_smbus_write_byte_data(hTouch->client,
                                      SYNAPTICS_DEVICE_CONTROL_REG,
                                      ((DeviceControl & 0xFC) | SYNAPTICS_DEVICE_NORMAL_OPERATION));
                return true;
            }
            break;
        default:
            return false;
    }
}

bool synaptics_test_device_specific_settings(synaptics_touch_device* hTouch)
{
    u8 configValueX;
    u8 configValueY;
    
    configValueX = i2c_smbus_read_byte_data(hTouch->client, SYNAPTICS_DELTA_X_THRES_REG);
    configValueY = i2c_smbus_read_byte_data(hTouch->client, SYNAPTICS_DELTA_Y_THRES_REG);
    
    return (configValueX == SYNAPTICS_DELTA_THRESHOLD 
                && configValueY == SYNAPTICS_DELTA_THRESHOLD);
}

bool synaptics_get_finger_data(synaptics_touch_device* hTouch, kowalski_touch_finger_data* data)
{
    u16 touch_finger_bit_mask = 0;
    u8  finger_index = 0;
    u8  index = 0;

    //Abort early if the touch isn't enabled
    if(is_star_touch_enable == false)
        return false;

    //Will read a block from i2c  [i2c-device]     [register num]            [size]         [destination]
    i2c_smbus_read_i2c_block_data(hTouch->client, SYNAPTICS_INT_STATUS_REG, sizeof(u8)*4, (u8*)(&ts_reg_data.interrupt_status_reg));
    DEBUG_MSG(M, "[TOUCH] i[%d], 0[%d], 1[%d], 2[%d]", (int)ts_reg_data.interrupt_status_reg,
                                                       (int)ts_reg_data.finger_state_reg[0],
                                                       (int)ts_reg_data.finger_state_reg[1],
                                                       (int)ts_reg_data.finger_state_reg[2]);

    if(ts_reg_data.interrupt_status_reg == SYNAPTICS_INT_ABS0){
        touch_finger_bit_mask = GET_BIT_MASK(ts_reg_data.finger_state_reg);

        while(touch_finger_bit_mask) {
            
            GET_INDEX_FROM_MASK(finger_index, touch_finger_bit_mask)
            
            i2c_smbus_read_i2c_block_data(hTouch->client, SYNAPTICS_REG_FINGER_DATA_START_ADDR + (SYNAPTICS_REG_FINGER_DATA_GAP*finger_index), 
                SYNAPTICS_REG_FINGER_VALID_DATA_SIZE, ts_reg_data.fingers_data[index]);

            // X-POSTITION
            data->curr_data[index].X_position = TS_SNTS_GET_X_POSITION(ts_reg_data.fingers_data[index][X_HIGH_POSITION],
                                                                       ts_reg_data.fingers_data[index][XY_LOW_POSITION]);
            
            // Y-POSITION                                                  
            data->curr_data[index].Y_position = TS_SNTS_GET_Y_POSITION(ts_reg_data.fingers_data[index][Y_HIGH_POSITION],
                                                                       ts_reg_data.fingers_data[index][XY_LOW_POSITION]);
            // WIDTH                                
            data->curr_data[index].width      = TS_SNTS_GET_WIDTH(ts_reg_data.fingers_data[index][XY_WIDTH]);
            
            // PRESSURE
            data->curr_data[index].pressure   = TS_SNTS_GET_PRESSURE(ts_reg_data.fingers_data[index][PRESSURE]);

            DEBUG_MSG(M, "[TOUCH] X[%d], Y[%d], Press[%d], Width[%d]", (int)data->curr_data[index].X_position,
                                                                       (int)data->curr_data[index].Y_position,
                                                                       (int)data->curr_data[index].width,
                                                                       (int)data->curr_data[index].pressure);

            //Adjust the position of both (x and y)
            tegra_touch_adjust_position(data->prev_data[index].X_position, //Previous locations
                                        data->prev_data[index].Y_position,
                                        &data->curr_data[index].X_position, //destinations for new positions
                                        &data->curr_data[index].Y_position);
#if defined (LGE_NOMELT)
            current_data_x = TS_SNTS_GET_LOWDATA_X_POSITION(ts_reg_data.fingers_data[index][X_HIGH_POSITION],
                                                            ts_reg_data.fingers_data[index][XY_LOW_POSITION]);
                                                            
            current_data_y = TS_SNTS_GET_LOWDATA_Y_POSITION(ts_reg_data.fingers_data[index][Y_HIGH_POSITION],
                                                            ts_reg_data.fingers_data[index][XY_LOW_POSITION]);
            DEBUG_MSG(B,"[TOUCH] current_data_x[%d], current_data_y[%d]\n",current_data_x, current_data_y);
#endif /* LGE_NOMELT */
            //We increase the index, so that we know for how many fingers we have pulled the values
            index++;
        }
        //Save the finger count
        data->total_num = index;
        
#if defined (LGE_NOMELT)
        synaptics_set_no_melt_mode (hTouch, false);
#endif /* LGE_NOMELT */

        return true;
    } else return false;
}

u8 synaptics_check_for_button(kowalski_touch_finger_data* data)
{
    /* Notice: Keep in mind, that the touch buttons (menu, home, back and search)
     *         are not hardware buttons. They are a specific region at the bottom
     *         of the touch area. This means we have to check for them here
     */

    u8 tmp_button = KEY_NULL;
    u8 sync = DO_NOT_ANYTHING;
    u8 total_num = data->total_num;
    u8 prev_button = data->prev_button;
    u8 state = data->state;
    ts_finger_data curr_data = data->curr_data[0];

#ifdef CONFIG_TOUCHSCREEN_DOUBLETAP2WAKE
    if(state != TOUCH_LOCK && detect_knock_inputs((int)curr_data.X_position,
                            (int)curr_data.Y_position, total_num != SINGLE_FINGER)) {
       data->curr_button = tmp_button;
       return DO_NOT_ANYTHING;
    }
#endif

    if(total_num == FINGER_RELEASE) {
        data->curr_button = KEY_NULL;
        if(prev_button){
            if(prev_button == KEY_PANEL)
                return ABS_RELEASE;
            else return BUTTON_RELEASE;
        } else return DO_NOT_ANYTHING; 
    }
    
    // If we have a touch lock that means we must not check 
    // for anything, so just return
    if(state == TOUCH_LOCK)
        return TOUCH_LOCK;
    
    //Check if this is multi
    if(total_num != SINGLE_FINGER) {
        data->curr_button = KEY_PANEL;
        if(prev_button && prev_button != KEY_PANEL)
            return BUTTON_RELEASE;
        else return ABS_PRESS;
    }

    // determine the current Button/Key
    if(IS_PANEL(curr_data.Y_position))       tmp_button = KEY_PANEL;
    else if(IS_MENU(curr_data.X_position))   tmp_button = KEY_MENU;
#if defined(CONFIG_MACH_STAR_P990) || defined(CONFIG_MACH_STAR_P999)
    else if(IS_HOME(curr_data.X_position))   tmp_button = KEY_HOMEPAGE;
#else
    else if(IS_HOME(curr_data.X_position))   tmp_button = KEY_HOME;
#endif
    else if(IS_BACK(curr_data.X_position))   tmp_button = KEY_BACK;
    else if(IS_SEARCH(curr_data.X_position)) tmp_button = KEY_SEARCH;
    else                                     tmp_button = KEY_BOUNDARY;

    //Now check for releases and presses
    if(prev_button != KEY_NULL && prev_button != KEY_BOUNDARY) {
        if(prev_button == KEY_PANEL){
            if(prev_button != tmp_button)
                sync = ABS_RELEASE;
            else
                sync = ABS_PRESS;
        } else {
            if(prev_button != tmp_button)
                sync = BUTTON_RELEASE;
            else
                sync = DO_NOT_ANYTHING;
        }
    } else {
        if(tmp_button == KEY_PANEL)
            sync = ABS_PRESS;
        else if(tmp_button == KEY_BOUNDARY)
            sync = DO_NOT_ANYTHING;
        else 
            sync = BUTTON_PRESS;
    }

    data->curr_button = tmp_button;
    return sync;
}

bool synaptics_additional_job(kowalski_touch_finger_data* data, u32 whereis)
{
    switch(whereis){
        case BEFORE_WHILE:
            set_freezable_with_signal();
            break;
        case AFTER_GET_DATA:
            break;
        case AFTER_SYNC:
            if(data->total_num == SINGLE_FINGER){
                if(data->state == ABS_RELEASE || data->state == BUTTON_RELEASE)
                    data->state = TOUCH_LOCK;
            }
            break;
        default:
            break;
    }
    return true;
}

bool init_device_specific_settings(synaptics_touch_device* hTouch)
{
#ifdef STAR_FW_UPGRADE
    if(unlikely(synaptics_ts_fw_upgrade(hTouch) != 0)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        return false;
    }
#endif

    //Will read a block from i2c  [i2c-device]     [register num]            [size]         [destination]
    i2c_smbus_read_i2c_block_data(hTouch->client, SYNAPTICS_INT_STATUS_REG, sizeof(u8)*4, (u8*)(&ts_reg_data.interrupt_status_reg));


    //Will write blocks to i2c  [i2c-device]       [register num]                 [value]
    i2c_smbus_write_byte_data(hTouch->client,   SYNAPTICS_INTERRUPT_ENABLE_REG,   SYNAPTICS_INT_ABS0);

    i2c_smbus_write_byte_data(hTouch->client,   SYNAPTICS_DELTA_X_THRES_REG,      SYNAPTICS_DELTA_THRESHOLD);    // Delta X
    i2c_smbus_write_byte_data(hTouch->client,   SYNAPTICS_DELTA_Y_THRES_REG,      SYNAPTICS_DELTA_THRESHOLD);    // Delta Y

    i2c_smbus_write_byte_data(hTouch->client,   SYNAPTICS_2D_GESTURE_ENABLE1,     0x00);        // 2d gesture enable1 = not use
    i2c_smbus_write_byte_data(hTouch->client,   SYNAPTICS_2D_GESTURE_ENABLE2,     0x00);        // 2d gesture enable2 = not use

    i2c_smbus_write_byte_data(hTouch->client,   SYNAPTICS_REPORT_MODE_REG,        0x08);        // continuous reporting

    i2c_smbus_write_byte_data(hTouch->client,   SYNAPTICS_MELT_CONTROL,           SYNAPTICS_MELT); //mb_jgroh no_mel->melt

    return true;
}


/*
 * synaptics_set_no_melt_mode
 * 20110603, xwolf@lge.com 
 * when user taps the panel by 4, the panel goes into NoMelt Mode
 * No Melt Mode is that there is no release key after 4 secs from starting pressing the panel 
 * cf) Melt Mode prevents ghost finger.
 */
#if defined (LGE_NOMELT)
static void synaptics_set_no_melt_mode (void* h_dev, bool binit)
{
    synaptics_touch_device* hTouch;
    hTouch = (synaptics_touch_device*)h_dev;

    if (binit) {
        mode = 1;
        numfinger = 0;
        reportcnt = 0;
    } else if (mode) {
        if((ts_reg_data.finger_state_reg[0] == 0) 
            && (ts_reg_data.finger_state_reg[1] == 0) 
            && (ts_reg_data.finger_state_reg[2] == 0)) { //No Finger
            
            DEBUG_MSG(E, "[TOUCH] numfinger=%d,reportcnt=%d\n", numfinger, reportcnt);
            
            if((numfinger == 1) & (reportcnt > 6)) {
                DEBUG_MSG(E, "[TOUCH] first_x=%d,first_y=%d\n",first_x,first_y);
                
                if((abs(first_x - current_data_x) > 200) 
                    || (abs(first_y - current_data_y) > 200)) {   //correspond to 1cm
                    
                    //set no melting                
                    if(i2c_smbus_write_byte_data(hTouch->client, SYNAPTICS_MELT_CONTROL, SYNAPTICS_NO_MELT) < 0) {
                        DEBUG_MSG(E,"[TOUCH] ERROR I2C WRITE FAIL SYNAPTICS_MELT_CONTROL\n");
                    }
                    
                    mode = 0;
                    DEBUG_MSG(E, "[TOUCH] No melt mode~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
                }
            }
            numfinger = 0;
            reportcnt = 0;
        } else if((ts_reg_data.finger_state_reg[0] == 1) 
                    && (ts_reg_data.finger_state_reg[1] == 0) 
                    && (ts_reg_data.finger_state_reg[2] == 0)) { // 1 finger
            if(++reportcnt > 10)
                reportcnt = 10;
                
            if(numfinger == 0) {
                numfinger = 1;
                
                first_x = current_data_x;
                first_y = current_data_y;
                
                pre_x = current_data_x;
                pre_y = current_data_y;
                
            } else if(numfinger == 1) {
                if((abs(pre_x-current_data_x) > 500) 
                    || (abs(pre_y-current_data_y) > 500)) {
                    numfinger = 2;
                }
                pre_x = current_data_x;
                pre_y = current_data_y;
            }
        } else { // more than 2 finger
            numfinger = 2;
        }
    }
}
#endif /* LGE_NOMELT */

#ifdef STAR_FW_UPGRADE

#define TOUCH_FW_COMPARE

#define WAIT_UNTIL_PIN_READY(_pin)    \
{    \
    while(gpio_get_value(_pin)){    \
        mdelay(1);    \
    }    \
}

#define WAIT_UNTIL_FLASH_CMD_READY(_cond)\
{    \
    u8 flashValue, temp_data;    \
    do{    \
        flashValue = i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG);    \
        temp_data = i2c_smbus_read_byte_data(client, SYNAPTICS_INT_STATUS_REG);    \
    } while(_cond);\
}

#define WAIT_UNTIL_DEVICE_READY(_cond, _pin)\
    WAIT_UNTIL_PIN_READY(_pin)    \
WAIT_UNTIL_FLASH_CMD_READY(_cond)

#define I2C_R(_state)    DO_SAFE(_state, "I2C READ", return -1)
#define I2C_W(_state)    DO_SAFE(_state, "I2C WRITE", return -1)


static u8 synaptics_get_fw_version(struct i2c_client *client)
{
    u8 RMI_Query_BaseAddr;
    u8 FWVersion_Addr;

    u8 SynapticsFirmVersion;

    I2C_R((RMI_Query_BaseAddr = i2c_smbus_read_byte_data(client, SYNAPTICS_RMI_QUERY_BASE_REG)) < 0)
    FWVersion_Addr = RMI_Query_BaseAddr + 3;
    I2C_R((SynapticsFirmVersion = i2c_smbus_read_byte_data(client, FWVersion_Addr)) < 0)

    DEBUG_MSG(3, "[TOUCH FW] synaptics_GetFWVersion = %x\n", SynapticsFirmVersion)

    return SynapticsFirmVersion;
}

static unsigned long ExtractLongFromHeader(const u8 *SynaImage)  // Endian agnostic 
{
    return((unsigned long)SynaImage[0] +
            (unsigned long)SynaImage[1]*0x100 +
            (unsigned long)SynaImage[2]*0x10000 +
            (unsigned long)SynaImage[3]*0x1000000);
}

static void CalculateChecksum(u16 *data, u16 len, u32 *dataBlock)
{
    unsigned long temp = *data++;
    unsigned long sum1;
    unsigned long sum2;

    *dataBlock = 0xffffffff;

    sum1 = *dataBlock & 0xFFFF;
    sum2 = *dataBlock >> 16;

    while (len--) {
        sum1 += temp;    
        sum2 += sum1;    
        sum1 = (sum1 & 0xffff) + (sum1 >> 16);    
        sum2 = (sum2 & 0xffff) + (sum2 >> 16);
    }

    *dataBlock = sum2 << 16 | sum1;
}

static void SpecialCopyEndianAgnostic(u8 *dest, u16 src) 
{
    dest[0] = src % 0x100;  //Endian agnostic method
    dest[1] = src / 0x100;  
}

#ifdef TOUCH_FW_COMPARE
static int fw_compare(struct i2c_client *client, const u8 BlockDataStartAddr, u16 index, const u16 block_size)
{
    u8 *tmp_block = kmalloc(sizeof(u8)*block_size, GFP_KERNEL);
    u8 i;

    if(i2c_smbus_read_i2c_block_data(client, BlockDataStartAddr, sizeof(tmp_block), tmp_block) < sizeof(tmp_block)) {
        kfree(tmp_block);
        return -1;
    }

    for(i = 0; i < sizeof(tmp_block); i++){
        if(unlikely(tmp_block[i] != SynapticsFirmware[index])){
            kfree(tmp_block);
            return -1;
        }
        DEBUG_MSG(1, "[TOUCH FW] [%x] : tmp[%x] / Firm[%x]\n", i, tmp_block[i], SynapticsFirmware[index]);
        index++;
    }

    //mdelay(100);
    kfree(tmp_block);
    return 0;
}
#endif

static int synaptics_ts_fw_upgrade(synaptics_touch_device* hTouch)
{
    struct i2c_client *client = hTouch->client;

    int i;

    u8 TouchFWVersion;

    u8 FlashQueryBaseAddr, FlashDataBaseAddr;
    u8 RMICommandBaseAddr;

    u8 BootloaderIDAddr;
    u8 BlockSizeAddr;
    u8 FirmwareBlockCountAddr;
    u8 ConfigBlockCountAddr;

    u8 BlockNumAddr;
    u8 BlockDataStartAddr;

    u8 bootloader_id[2];

    u8 temp_array[2], temp_data, m_firmwareImgVersion;
    u8 checkSumCode;

    u16 ts_block_size, ts_config_block_count, ts_fw_block_count;
    u16 m_bootloadImgID;

    u32 ts_config_img_size;
    u32 ts_fw_img_size;
    u32 m_fileSize, m_firmwareImgSize, m_configImgSize, m_FirmwareImgFile_checkSum;

    u8 RMI_Query_BaseAddr;
    u8 product_id_addr;

    u8 product_id[7];

    ////////////////////////////////////////////////////////////////////////////////////

    DEBUG_MSG(3, "[TOUCH FW] Synaptics_UpgradeFirmware :: TM1576 [START]\n")


    ////////////////////////    Product ID Check    ///////////////////////////


    I2C_R((RMI_Query_BaseAddr = i2c_smbus_read_byte_data(client, SYNAPTICS_RMI_QUERY_BASE_REG)) < 0)
    product_id_addr = RMI_Query_BaseAddr + 11;

    I2C_R(i2c_smbus_read_i2c_block_data(client, product_id_addr,
                                        sizeof(product_id) - 1, product_id) < sizeof(product_id) - 1)
    product_id[6] = '\0';

    DEBUG_MSG(E, "[TOUCH FW] Touch controller Product ID = %s\n", product_id)

    if(unlikely(strncmp(product_id, &SynapticsFirmware[0x10], 6) != 0)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        return 0;
    }
    if(unlikely((TouchFWVersion = synaptics_get_fw_version(client)) == -1)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
    }

    if((TouchFWVersion >= 0x64 && SynapticsFirmware[0x1F] >= 0x64) 
        || (TouchFWVersion < 0x64 && SynapticsFirmware[0x1F] < 0x64)) {
        
        if(unlikely(!(TouchFWVersion < SynapticsFirmware[0x1F]))) {
            DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "FW Upgrade is not needed");
            return 0;
        }
    }

    ////////////////////////    Configuration    ///////////////////////////
    I2C_R((FlashQueryBaseAddr = i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_QUERY_BASE_REG)) < 0)

    BootloaderIDAddr = FlashQueryBaseAddr;
    BlockSizeAddr = FlashQueryBaseAddr + 3;
    FirmwareBlockCountAddr = FlashQueryBaseAddr + 5;
    ConfigBlockCountAddr = FlashQueryBaseAddr + 7;

    I2C_R((FlashDataBaseAddr = i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_DATA_BASE_REG)) < 0)

    BlockNumAddr = FlashDataBaseAddr;
    BlockDataStartAddr = FlashDataBaseAddr + 2;

    m_fileSize = sizeof(SynapticsFirmware) - 1;

    checkSumCode         = ExtractLongFromHeader(&(SynapticsFirmware[0]));
    m_bootloadImgID      = (unsigned int)SynapticsFirmware[4] + (unsigned int)SynapticsFirmware[5]*0x100;
    m_firmwareImgVersion = SynapticsFirmware[7]; 
    m_firmwareImgSize    = ExtractLongFromHeader(&(SynapticsFirmware[8]));
    m_configImgSize      = ExtractLongFromHeader(&(SynapticsFirmware[12]));    

    CalculateChecksum((u16*)&(SynapticsFirmware[4]), (u16)(m_fileSize-4)>>1, &m_FirmwareImgFile_checkSum);

    // Get Current Firmware Information
    I2C_R(i2c_smbus_read_i2c_block_data(client, BlockSizeAddr, sizeof(temp_array),
                                            (u8*)&temp_array[0]) < sizeof(temp_array)) 

    ts_block_size = temp_array[0] + (temp_array[1] << 8);

    I2C_R(i2c_smbus_read_i2c_block_data(client, FirmwareBlockCountAddr, sizeof(temp_array),
                                            (u8*)&temp_array[0]) < sizeof(temp_array))
                                            
    ts_fw_block_count = temp_array[0] + (temp_array[1] << 8);
    ts_fw_img_size = ts_block_size * ts_fw_block_count;

    I2C_R(i2c_smbus_read_i2c_block_data(client, ConfigBlockCountAddr, sizeof(temp_array),
                                            (u8*)&temp_array[0]) < sizeof(temp_array))

    ts_config_block_count = temp_array[0] + (temp_array[1] << 8);
    ts_config_img_size = ts_block_size * ts_config_block_count;

    I2C_R(i2c_smbus_read_i2c_block_data(client, BootloaderIDAddr, sizeof(bootloader_id),
                                            (u8*)&bootloader_id[0]) < sizeof(bootloader_id))

    // Compare
    if(unlikely(m_fileSize != (0x100 + m_firmwareImgSize+m_configImgSize))) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        return 0;
    }
    if(unlikely(m_firmwareImgSize != ts_fw_img_size)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        return 0;
    }
    if(unlikely(m_configImgSize != ts_config_img_size)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        return 0;
    }
    if(unlikely(m_firmwareImgVersion == 0 && ((unsigned int)bootloader_id[0] + (unsigned int)bootloader_id[1] * 0x100)
                            != m_bootloadImgID)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        return 0;
    }

    ////////////////////////    Flash Command - Enable    ///////////////////////////
    I2C_W(i2c_smbus_write_i2c_block_data(client, BlockDataStartAddr, sizeof(bootloader_id), bootloader_id) < 0)
    WAIT_UNTIL_FLASH_CMD_READY((flashValue & 0x0f) != 0x00)

    I2C_W(i2c_smbus_write_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_ENABLE) < 0)
    WAIT_UNTIL_DEVICE_READY(flashValue != 0x80, hTouch->gpio)

    DEBUG_MSG(E, "[TOUCH FW] Synaptics_UpgradeFirmware :: Flash Program Enable Setup Complete\n")

    ////////////////////////    Flash Command  - Eraseall    ///////////////////////////
    I2C_W(i2c_smbus_write_i2c_block_data(client, BlockDataStartAddr, sizeof(bootloader_id), bootloader_id) < 0)
    I2C_W(i2c_smbus_write_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_ERASEALL) < 0)

    WAIT_UNTIL_DEVICE_READY(flashValue != 0x80, hTouch->gpio)

    DEBUG_MSG(E, "[TOUCH FW] Synaptics_UpgradeFirmware :: Flash Erase Complete\n")

    ////////////////////////    F/W Data Write    ///////////////////////////
    for(i = 0; i < ts_fw_block_count; ++i)
    {
        temp_array[0] = i & 0xff;
        temp_array[1] = (i & 0xff00) >> 8;

        I2C_W(i2c_smbus_write_i2c_block_data(client, BlockNumAddr, sizeof(temp_array), temp_array) < 0)
        I2C_W(i2c_smbus_write_i2c_block_data(client, BlockDataStartAddr, ts_block_size,
                                (u8*)&SynapticsFirmware[0x100 + i*ts_block_size]) < 0)

#ifdef TOUCH_FW_COMPARE
        if(fw_compare(client, BlockDataStartAddr, 0x100+i*ts_block_size, ts_block_size)){
            DEBUG_MSG(3, "[TOUCH FW] FAIL: Firmware Update[%x]\n", i)
            return -1;
        }
#endif    
        I2C_W(i2c_smbus_write_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_FW_WRITE) < 0)
        WAIT_UNTIL_DEVICE_READY(flashValue != 0x80, hTouch->gpio)
    }
    DEBUG_MSG(E, "[TOUCH FW] Synaptics_UpgradeFirmware :: Flash Firmware Write Complete\n")

    ////////////////////////    F/W Config Write    ///////////////////////////
    for(i = 0; i < ts_config_block_count; i++) {
        SpecialCopyEndianAgnostic(&temp_array[0], i);

        I2C_W(i2c_smbus_write_i2c_block_data(client, BlockNumAddr, sizeof(temp_array), temp_array) < 0)

        I2C_W(i2c_smbus_write_i2c_block_data(client, BlockDataStartAddr, ts_block_size,
                                (u8*)&SynapticsFirmware[0x100+m_firmwareImgSize+ i * ts_block_size]) < 0)
#ifdef TOUCH_FW_COMPARE
        if(fw_compare(client, BlockDataStartAddr, 0x100 + m_firmwareImgSize 
                                + i * ts_block_size, ts_block_size)) {
            DEBUG_MSG(3, "[TOUCH FW] FAIL: Firmware Update[%x]\n", i)
            return -1;
        }
#endif    
        I2C_W(i2c_smbus_write_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_CONFIG_WRITE) < 0)
        WAIT_UNTIL_DEVICE_READY(flashValue != 0x80, hTouch->gpio)
    }
    DEBUG_MSG(E, "[TOUCH FW] Synaptics_UpgradeFirmware :: Flash Config Write Complete\n")

    ////////////////////////    Reset Touch IC    ///////////////////////////
    I2C_W(RMICommandBaseAddr = i2c_smbus_read_byte_data(client, SYNAPTICS_RMI_CMD_BASE_REG) < 0)

    if(RMICommandBaseAddr){
        I2C_W(i2c_smbus_write_byte_data(client, RMICommandBaseAddr, 0x01) < 0)                
        mdelay(200);

        WAIT_UNTIL_DEVICE_READY((flashValue & 0x0f) != 0x00, hTouch->gpio)

        I2C_R((temp_data = i2c_smbus_read_byte_data(client, SYNAPTICS_INT_STATUS_REG)) < 0)

        // Read F01 Status flash prog, ensure the 6th bit is '0'
        while((temp_data = i2c_smbus_read_byte_data(client, SYNAPTICS_DATA_BASE_REG)) != 0);
    }
    else {
        // H/W reset
        if(!synaptics_power_control(hTouch, SYNAPTICS_POWER_STATE_OFF))
            return -1;
        if(!synaptics_power_control(hTouch, SYNAPTICS_POWER_STATE_ON))
            return -1;
    }

    DEBUG_MSG(E, "[TOUCH] Synaptics_UpgradeFirmware :: Complete!\n")
    return 0;    
}
#endif

extern bool is_star_touch_enable;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void touch_early_suspend(struct early_suspend *es);
static void touch_late_resume  (struct early_suspend *es);
#endif

static bool create_touch_object(touch_driver_data **touch_object);
static void remove_touch_object(touch_driver_data *touch);

/*
 * Will allocate all missing data
 */
static bool create_touch_object(touch_driver_data **touch_object)
{
    touch_driver_data *touch;
    if(unlikely(((touch = kzalloc(sizeof(touch_driver_data), GFP_KERNEL)) == NULL)))
    {
        // if the alloc failed we should really try to 
        // free as much objects as possible to avoid leaks
        remove_touch_object(touch);
        return false;
    }
    
    // Set the new builded data
    *touch_object = touch;
    return true;
}

static void remove_touch_object(touch_driver_data *touch)
{
    if(touch) kfree(touch);
}

static irqreturn_t touch_irq_handler(int irq, void *dev_id)
{
	touch_driver_data *touch = (touch_driver_data*)dev_id;
	
	//JUMP
	interrupt_disable(&touch->handle_touch);
    interrupt_start(&touch->handle_touch);

    return IRQ_HANDLED;
}

int task_handler(void *pdata)
{
    touch_driver_data *touch                  = (touch_driver_data*)pdata;
    kowalski_touch_finger_data finger_data    = touch->finger_data;

    synaptics_touch_device *hTouch = &touch->handle_touch;

    printk("[TOUCH] %s() called and locking mutex\n", __FUNCTION__);
    mutex_lock(&touch->mutex);

    if(unlikely(synaptics_additional_job(&finger_data, BEFORE_WHILE) == 0)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        mutex_unlock(&touch->mutex);
        return 0;
    }

    while(1) {
        printk("[TOUCH] %s() called and waiting for interrupt\n", __FUNCTION__);
        if(unlikely(interrupt_wait(hTouch) == 0)) {
            DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
            goto err_in_while;
        }

        if(unlikely(synaptics_get_finger_data(hTouch, &finger_data) == 0)) {
            DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
            goto err_in_while;
        }
        
        if(unlikely(synaptics_additional_job(&finger_data, AFTER_GET_DATA) == 0)) {
            DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
            goto err_in_while;
        }

        finger_data.state = synaptics_check_for_button(&finger_data);

        DEBUG_MSG(B, "[TOUCH] finger_num[%d], state[%d], c_button[%d], p_button[%d]", 
                                            finger_data.total_num,
                                            finger_data.state,
                                            finger_data.curr_button,
                                            finger_data.prev_button);

        if(finger_data.total_num == FINGER_RELEASE) {
            switch(finger_data.state){
                case ABS_RELEASE:
                    input_device_send_abs(hTouch, finger_data.curr_data[0], FINGER_RELEASE);
                    break;
                case BUTTON_RELEASE:
                    input_device_send_button(hTouch, finger_data.prev_button, 0);
                    break;
                default: 
                    break;
            }
        } else if(finger_data.total_num == SINGLE_FINGER) {
            switch(finger_data.state) {
                case ABS_PRESS: 
                    input_device_send_abs(hTouch, finger_data.curr_data[0], SINGLE_FINGER);
                    break;
                case ABS_RELEASE:
                    input_device_send_abs(hTouch, finger_data.curr_data[0], FINGER_RELEASE);
                    break;
                case BUTTON_PRESS:
                    input_device_send_button(hTouch, finger_data.curr_button, 1);
                    break;
                case BUTTON_RELEASE:
                    input_device_send_button(hTouch, finger_data.prev_button, 0);
                    break;
                default: break;
            }
        } else { //multi-finger
            switch(finger_data.state) {
                case ABS_PRESS:
                    input_device_send_abs_for_multi(hTouch, finger_data.curr_data, finger_data.total_num);
                    break;
                case BUTTON_RELEASE:
                    input_device_send_button(hTouch, finger_data.prev_button, 0);
                    break;
                default: break;
            }
        }

        input_device_send_sync(hTouch, finger_data.state);
        if(unlikely(synaptics_additional_job(&finger_data, AFTER_SYNC) == 0)){
            DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
            goto err_in_while;
        }

        memcpy(finger_data.prev_data, finger_data.curr_data, sizeof(finger_data.curr_data));
        finger_data.prev_button = finger_data.curr_button;
err_in_while:
        if(unlikely(interrupt_enable(hTouch) == 0)) {
            DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
            goto err_task_handler;
        }
    }
err_task_handler:
    printk("[TOUCH] %s() called and unlocking mutex\n", __FUNCTION__);
    mutex_unlock(&touch->mutex);
    return 0;
}

static int touch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    synaptics_touch_device* hTouch = NULL;
    touch_driver_data *touch = NULL;

    if(unlikely(create_touch_object(&touch) == 0)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        goto err_kzalloc_probe;
    }
    
    hTouch = &touch->handle_touch;

    if(unlikely(synaptics_device_open(hTouch, client, (u32)touch) == 0)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        goto err_synaptics_device_open_probe;
    }

    if(unlikely(!synaptics_power_control(hTouch, SYNAPTICS_POWER_STATE_ON))) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        goto err_powerCtrl_probe;
    }
    
    if(unlikely(input_device_open(hTouch) == 0)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        goto err_inputDev_probe;
    }
    if(unlikely(interrupt_open(hTouch) == 0)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        goto err_interrupt_probe;
    }

    if(unlikely(init_device_specific_settings(hTouch) == 0)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        goto err_synaptics_device_open_probe;
    }

    if(unlikely(synaptics_task_open(hTouch) == 0)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        goto err_task_probe;
    }

    if(unlikely(synaptics_task_start(hTouch) == 0)) {
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, "");
        goto err_inputDev_probe;
    }

    mutex_init(&touch->mutex);
    i2c_set_clientdata(client, touch);

#ifdef CONFIG_HAS_EARLYSUSPEND
    touch->early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    touch->early_suspend.suspend = touch_early_suspend;
    touch->early_suspend.resume  = touch_late_resume;
    register_early_suspend(&touch->early_suspend);
#endif

    DEBUG_MSG(M, "[TOUCH] touch_driver is initialized.\n");

    return 0;

err_inputDev_probe:
    input_device_close(hTouch);
err_task_probe:
    synaptics_task_close(hTouch);
err_interrupt_probe:
    interrupt_close(hTouch);
err_powerCtrl_probe:
    synaptics_power_control(hTouch, SYNAPTICS_POWER_STATE_OFF);
err_synaptics_device_open_probe:
    synaptics_device_close(hTouch);
err_kzalloc_probe:
    remove_touch_object(touch);
    return -1;
}

static int touch_remove(struct i2c_client *client)
{
    touch_driver_data *touch = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&touch->early_suspend);
#endif
    input_device_close(&touch->handle_touch);
    synaptics_task_close(&touch->handle_touch);
    interrupt_close(&touch->handle_touch);
    synaptics_device_close(&touch->handle_touch);
    remove_touch_object(touch);
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void touch_early_suspend(struct early_suspend *es)
{
    touch_driver_data *touch = container_of(es, touch_driver_data, early_suspend);

    printk("[TOUCH] %s() called\n", __FUNCTION__);

#ifdef CONFIG_TOUCHSCREEN_DOUBLETAP2WAKE
    if(dt2w_switch == 0)
#endif
    {
        interrupt_disable(&touch->handle_touch);
        synaptics_power_control(&touch->handle_touch, SYNAPTICS_POWER_STATE_SLEEP);
    }
}

static void touch_late_resume(struct early_suspend *es)
{
    touch_driver_data *touch = container_of(es, touch_driver_data, early_suspend);

    printk("[TOUCH] %s called\n", __FUNCTION__);
#ifdef CONFIG_TOUCHSCREEN_DOUBLETAP2WAKE
    if(dt2w_switch != 0 || !synaptics_test_device_specific_settings(&touch->handle_touch))
#else
    if(!synaptics_test_device_specific_settings(&touch->handle_touch))
#endif
    {
        printk("[TOUCH] %s resetting hardware and setting values\n", __FUNCTION__);
        
        //Do a HW - Reset
        synaptics_power_control(&touch->handle_touch, SYNAPTICS_POWER_STATE_OFF);
        synaptics_power_control(&touch->handle_touch, SYNAPTICS_POWER_STATE_ON);
        
        //Write missing settings
        init_device_specific_settings(&touch->handle_touch);

        // Do all missing wake up stuff
        synaptics_power_control(&touch->handle_touch, SYNAPTICS_POWER_STATE_WAKE);
    }
    
    // Wake up the task if it was stopped
    if(synaptics_task_is_stopped(&touch->handle_touch)) {
        synaptics_task_start(&touch->handle_touch);
        printk("[TOUCH] %s restarting task\n", __FUNCTION__);
    }

    interrupt_enable(&touch->handle_touch);
}
#endif

static const struct i2c_device_id LGE_ts_id[] = {
    { LGE_TOUCH_NAME, 0 },
};

static struct i2c_driver touch_driver = {
    .probe      = touch_probe,
    .remove     = touch_remove,
    .id_table   = LGE_ts_id,
    .driver     = {
        .name   = LGE_TOUCH_NAME,
    },
};

static int __devinit touch_init(void)
{
    return i2c_add_driver(&touch_driver);
}

static void __exit touch_exit(void)
{
    i2c_del_driver(&touch_driver);
}

module_init(touch_init);
module_exit(touch_exit);

MODULE_AUTHOR("FIRTECY <admin@firtecy.de>");
MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");

