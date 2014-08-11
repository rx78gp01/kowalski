/* drivers/input/touchscreen/kowalski_touch_driver.h
 *
 * Copyright (C) 2014 Firtecy 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef INCLUDED_TOUCH_SYNAPTICS_H
#define INCLUDED_TOUCH_SYNAPTICS_H

#ifndef DEFINE_TOUCH_NAME
#define DEFINE_TOUCH_NAME
#define LGE_TOUCH_NAME "star_synaptics"
#endif

#define LGE_TOUCH_ADDR 0x20

//#define SYNAPTICS_TOUCH_DEBUG_LEVEL 1

#ifdef SYNAPTICS_TOUCH_DEBUG_LEVEL
#define DEBUG_MSG(_level, args...)  \
    if(_level >= SYNAPTICS_TOUCH_DEBUG_LEVEL)    \
        printk(KERN_INFO args);
#else
#define DEBUG_MSG(_level, args...)
#endif

enum{M=1,    // touch_movement_event_log || check_some_log
    B,        // touch_button_event_log || sync_event_log
    E,        // essential
    ERR,    //error
    NOTHING
};

#define DO_SAFE(_do, _log, _err_handling)    \
    if(unlikely(_do)){    \
        DEBUG_MSG(ERR, "[TOUCH] ERROR : %s[%u] %s\n", __FUNCTION__, __LINE__, _log);    \
        _err_handling;    \
    }

// do safely when allocate...
#define DO_A(_statement, _jump_place)    DO_SAFE((_statement) == NULL, "", goto _jump_place)

// do safely when call the function...
#define DO_F(_statement, _jump_place)    DO_SAFE((_statement) == 0, "", goto _jump_place)

// do safely when compare...
#define DO_C(_statement, _jump_place)    DO_SAFE((_statement), "", goto _jump_place)

#define IS_MENU(_x)            (_x > 15  && _x < 115)
#define IS_HOME(_x)            (_x > 145 && _x < 225)
#define IS_BACK(_x)            (_x > 255 && _x < 335)
#define IS_SEARCH(_x)          (_x > 365 && _x < 465)
#define IS_PANEL(_y)           (_y >= 0  && _y <= 800)

#define SYNAPTICS_DEVICE_NORMAL_OPERATION    0
#define SYNAPTICS_DEVICE_SENSOR_SLEEP        1

#define SYNAPTICS_POWER_STATE_ON    1
#define SYNAPTICS_POWER_STATE_OFF   2
#define SYNAPTICS_POWER_STATE_SLEEP 3
#define SYNAPTICS_POWER_STATE_WAKE  4

enum {KEY_NULL=0, KEY_PANEL, KEY_BOUNDARY};
enum {BEFORE_WHILE=1, AFTER_GET_DATA, BEFORE_SYNC, AFTER_SYNC};
enum {FINGER_RELEASE=0, SINGLE_FINGER, MULTI_FINGER};
enum {DO_NOT_ANYTHING=0, ABS_PRESS, ABS_RELEASE, BUTTON_PRESS, BUTTON_RELEASE, TOUCH_LOCK};


struct star_synaptics_platform_data {
    u32 gpio;    
    int (*power)(char* reg_id, bool on);    
    unsigned long irqflags;
};

#define LGE_TOUCH_RESOLUTION_X                  480
#define LGE_TOUCH_RESOLUTION_Y                  800

#define SYNAPTICS_TOUCH_DEVICE_GUID             NV_ODM_GUID('s','y','n','t','o','u','c','h')

#define SYNAPTICS_I2C_SPEED_KHZ                 400
#define SYNAPTICS_I2C_TIMEOUT                   10
#define SYNAPTICS_I2C_RETRY_COUNT               5
#define SYNAPTICS_LOW_SAMPLE_RATE               0        //40 reports per-second
#define SYNAPTICS_HIGH_SAMPLE_RATE              1        //80 reports per-second

#define SYNAPTICS_SCREEN_ANGLE_MODE             1        //0=Landscape, 1=Portrait
#define SYNAPTICS_POR_DELAY_MS                  100        //Delay after Power-On Reset
#define SYNAPTICS_DEBOUNCE_TIME_MS              0
#define SYNAPTICS_FINGER_MAX                    10

#define SYNAPTICS_MELT_SUPPORT_VER              4

#define SYNAPTICS_NEW_PANEL_BASE_FW_VER         11

#define SYNAPTICS_DELTA_THRESHOLD               0x01

#define SYNAPTICS_FLASH_CONTROL_REG             0x12
#define SYNAPTICS_DATA_BASE_REG                 0x13
#define SYNAPTICS_INT_STATUS_REG                0x14

#define SYNAPTICS_REG_FINGER_DATA_START_ADDR    0x18

#define SYNAPTICS_DEVICE_CONTROL_REG            0x4F
#define SYNAPTICS_INTERRUPT_ENABLE_REG          0x50
#define SYNAPTICS_REPORT_MODE_REG               0x51
#define SYNAPTICS_PALM_DETECT_REG               0x52
#define SYNAPTICS_DELTA_X_THRES_REG             0x53
#define SYNAPTICS_DELTA_Y_THRES_REG             0x54
#define SYNAPTICS_VELOCITY_REG                  0x55
#define SYNAPTICS_ACCELERATION_REG              0x56
#define SYNAPTICS_MAX_X_POSITION_LOW_REG        0x57
#define SYNAPTICS_MAX_X_POSITION_HIGH_REG       0x58
#define SYNAPTICS_MAX_Y_POSITION_LOW_REG        0x59
#define SYNAPTICS_MAX_Y_POSITION_HIGH_REG       0x5A
#define SYNAPTICS_2D_GESTURE_ENABLE1            0x5B
#define SYNAPTICS_2D_GESTURE_ENABLE2            0x5C

#define SYNAPTICS_MAX_TAP_TIME_REG              0x9A
#define SYNAPTICS_MIN_PRESS_TIME_REG            0x9B
#define SYNAPTICS_MIN_TAP_DIST_REG              0x9C
#define SYNAPTICS_MIN_FLICK_DIST_REG            0x9D
#define SYNAPTICS_MIN_FLICK_SPEED_REG           0x9E

#define SYNAPTICS_RMI_QUERY_BASE_REG            0xE3
#define SYNAPTICS_RMI_CMD_BASE_REG              0xE4
#define SYNAPTICS_FLASH_QUERY_BASE_REG          0xE9
#define SYNAPTICS_FLASH_DATA_BASE_REG           0xEC

#define SYNAPTICS_MELT_CONTROL                  0xF0

#define SYNAPTICS_INT_FLASH                     1<<0
#define SYNAPTICS_INT_STATUS                    1<<1
#define SYNAPTICS_INT_ABS0                      1<<2

#define SYNAPTICS_CONTROL_SLEEP                 (1<<0)
#define SYNAPTICS_CONTROL_NOSLEEP               (1<<2)

#define SYNAPTICS_REG_FINGER_DATA_GAP           0x05
#define SYNAPTICS_REG_FINGER_VALID_DATA_SIZE    0x05

#define SYNAPTICS_NO_MELT                       0x00
#define SYNAPTICS_MELT                          0x01

#define SYNAPTICS_FLASH_CMD_FW_CRC              0x01
#define SYNAPTICS_FLASH_CMD_FW_WRITE            0x02
#define SYNAPTICS_FLASH_CMD_ERASEALL            0x03
#define SYNAPTICS_FLASH_CMD_CONFIG_READ         0x05
#define SYNAPTICS_FLASH_CMD_CONFIG_WRITE        0x06
#define SYNAPTICS_FLASH_CMD_CONFIG_ERASE        0x07
#define SYNAPTICS_FLASH_CMD_ENABLE              0x0F
#define SYNAPTICS_FLASH_NORMAL_RESULT           0x80


typedef struct
{
    bool    IsMultiTouchSupported; // single touch[0] / multi-touch[1]
    bool    IsButtonSupported; // yes[1] / no[0]
    u32     MaxNumberOfFingerCoordReported;
    bool    IsRelativeDataSupported; // support[1] / not[0]
    u32     MaxNumberOfRelativeCoordReported;
    u32     MaxNumberOfWidthReported;
    u32     MaxNumberOfPressureReported;
    u32     Gesture;
    bool    IsWidthSupported;
    bool    IsPressureSupported;
    bool    IsFingersSupported;
    u32     XMinPosition;
    u32     YMinPosition;
    u32     XMaxPosition;
    u32     YMaxPosition;
    u32     Orientation;
} kowalski_touch_device_capabilities;
typedef kowalski_touch_device_capabilities* kowalski_touch_device_capabilities_handle;

typedef struct {
    u16 X_position;
    u16 Y_position;
    u16 width;
    u16 pressure;
} ts_finger_data;

typedef struct {
    u8              total_num;
    ts_finger_data  curr_data[10];
    ts_finger_data  prev_data[10];
    u8              curr_button;
    u8              prev_button;
    u8              state;
} kowalski_touch_finger_data;

typedef struct 
{
    kowalski_touch_device_capabilities caps;
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct semaphore sem;
    struct task_struct*    task;
    u32 gpio;
    u32 irq_gpio;
    u32 flags;
    int (*power)(char* reg_id, bool on);
    u32 touch;
} synaptics_touch_device;

typedef struct
{
    u8 device_status_reg;                       //0x13
    u8 interrupt_status_reg;                    //0x14
    u8 finger_state_reg[3];                     //0x15 ~ 0x17

    u8 fingers_data[SYNAPTICS_FINGER_MAX][5];   //0x18 ~ 0x49

    u8 gesture_flag0;                           //0x4A
    u8 gesture_flag1;                           //0x4B
    u8 pinch_motion_X_flick_distance;           //0x4C
    u8 rotation_motion_Y_flick_distance;        //0x4D
    u8 finger_separation_flick_time;            //0x4E
} ts_sensor_data;

typedef struct touch_driver
{
    synaptics_touch_device      handle_touch;
    struct early_suspend        early_suspend;
    struct mutex                mutex;
    kowalski_touch_finger_data  finger_data;
} touch_driver_data;

#endif /* _LINUX_STAR_SYNAPTICS_H*/

