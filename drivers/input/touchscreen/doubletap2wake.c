/*
 * drivers/input/touchscreen/doubletap2wake.c
 *
 * Copyright (c) 2013, Dennis Rassmann <showp1984@gmail.com>
 * Copyright for additions: (c) 2014, Firtecy <admin@firtecy.de>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/input/doubletap2wake.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#ifndef CONFIG_HAS_EARLYSUSPEND
#include <linux/lcd_notify.h>
#else
#include <linux/earlysuspend.h>
#endif
#include <linux/hrtimer.h>
#include <asm-generic/cputime.h>

/* uncomment since no touchscreen defines android touch, do that here */
//#define ANDROID_TOUCH_DECLARED

/* if Sweep2Wake is compiled it will already have taken care of this */
#ifdef CONFIG_TOUCHSCREEN_SWEEP2WAKE
#define ANDROID_TOUCH_DECLARED
#endif

/* Version, author, desc, etc */
#define DRIVER_AUTHOR "Dennis Rassmann <showp1984@gmail.com>"
#define DRIVER_DESCRIPTION "Doubletap2wake for almost any device"
#define DRIVER_VERSION "1.0"
#define LOGTAG "[doubletap2wake][TOUCH]: "

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPLv2");

/* Tuneables */
#define DT2W_DEBUG        0
#define DT2W_DEFAULT      1

#define DT2W_PWRKEY_DUR   60
#define DT2W_FEATHER      200
#define DT2W_TIME         700

#ifdef CONFIG_TOUCHSCREEN_KNOCKCODE_UNLOCK
#define TAP_LOC_UP_RIGHT    1
#define TAP_LOC_UP_LEFT     2
#define TAP_LOC_DOWN_LEFT   3
#define TAP_LOC_DOWN_RIGHT  4

#define MAX_NUM_COORDS 4

#define SQUARE(_x) (_x * _x)
#define MIN(_x, _y) ((_x < _y) ? _x : _y)
#define MAX(_x, _y) ((_x > _y) ? _x : _y)
#endif

/* Resources */
int dt2w_switch = DT2W_DEFAULT;
static cputime64_t tap_time_pre = 0;
static int touch_nr = 0,
           x_pre = 0,
           y_pre = 0;


#ifdef CONFIG_TOUCHSCREEN_KNOCKCODE_UNLOCK
// For the knock code implementation
typedef struct {
    int x_coor;
    int y_coor;
    int x_rel;
    int y_rel;
    double tmp_diff;
    cputime64_t tap_time;
} tap_data;

// Positions:
//  +---+---+
//  | 2 | 1 |
//  +---+---+
//  | 3 | 4 |
//  +---+---+

static tap_data* knock_code_tap_data [MAX_NUM_COORDS];
static u8 knock_code_pattern [MAX_NUM_COORDS] = {1, 2, 3, 0};
static int knock_code_pattern_length = 3;
#endif

static bool scr_suspended = false,
            exec_count = true;
            
static unsigned int current_index;
static unsigned int wait_for_index;

#ifndef CONFIG_HAS_EARLYSUSPEND
static struct notifier_block dt2w_lcd_notif;
#endif
static struct input_dev * doubletap2wake_pwrdev;
static DEFINE_MUTEX(pwrkeyworklock);
static struct workqueue_struct *dt2w_input_wq;

/* Read cmdline for dt2w */
static int __init read_dt2w_cmdline(char *dt2w)
{
    if (strcmp(dt2w, "1") == 0) {
        pr_info("[cmdline_dt2w]: DoubleTap2Wake enabled. | dt2w='%s'\n", dt2w);
        dt2w_switch = 1;
    } else if (strcmp(dt2w, "0") == 0) {
        pr_info("[cmdline_dt2w]: DoubleTap2Wake disabled. | dt2w='%s'\n", dt2w);
        dt2w_switch = 0;
    }
#ifdef CONFIG_TOUCHSCREEN_KNOCKCODE_UNLOCK
    else if (strcmp(dt2w, "2") == 0) {
        pr_info("[cmdline_dt2w]: DoubleTap2Wake is in knock code mode. | dt2w='%s'\n", dt2w);
        dt2w_switch = 2;
    }
#endif
    else {
        pr_info("[cmdline_dt2w]: No valid input found. Going with default: | dt2w='%u'\n", dt2w_switch);
    }
    return 1;
}
__setup("dt2w=", read_dt2w_cmdline);

/* reset on finger release */
static void doubletap2wake_reset(void) {
    exec_count = true;
    touch_nr = 0;
    tap_time_pre = 0;
    x_pre = 0;
    y_pre = 0;
}

/* PowerKey work func */
static void doubletap2wake_presspwr(struct work_struct * doubletap2wake_presspwr_work) {
    if(current_index != wait_for_index) {
#if DT2W_DEBUG
        pr_info(LOGTAG"Aborting wakeup, because the ids don't match: %d != %d\n", current_index, wait_for_index);
#endif
        goto end_presspwr;
    }

    if(!scr_suspended)
        goto end_presspwr;

    if (!mutex_trylock(&pwrkeyworklock))
        goto end_presspwr;
               
    input_event(doubletap2wake_pwrdev, EV_KEY, KEY_POWER, 1);
    input_event(doubletap2wake_pwrdev, EV_SYN, 0, 0);
    msleep(DT2W_PWRKEY_DUR);
    input_event(doubletap2wake_pwrdev, EV_KEY, KEY_POWER, 0);
    input_event(doubletap2wake_pwrdev, EV_SYN, 0, 0);
    msleep(DT2W_PWRKEY_DUR);
        mutex_unlock(&pwrkeyworklock);
 
end_presspwr:
    wait_for_index = 0;
    return;
}
static DECLARE_DELAYED_WORK(doubletap2wake_presspwr_work, doubletap2wake_presspwr);

/* PowerKey trigger */
static void doubletap2wake_pwrtrigger(void) {
    if(wait_for_index == 0) {
        wait_for_index = current_index;
        schedule_delayed_work(&doubletap2wake_presspwr_work, msecs_to_jiffies(DT2W_TIME));
    }
    return;
}

/* unsigned */
static unsigned int calc_feather(int coord, int prev_coord) {
    int calc_coord = 0;
    calc_coord = coord-prev_coord;
    if (calc_coord < 0)
        calc_coord = calc_coord * (-1);
    return calc_coord;
}

/* init a new touch */
static void new_touch(int x, int y) {
    tap_time_pre = ktime_to_ms(ktime_get());
    if(current_index > 3000)
        current_index = 0;
    else current_index++;
    x_pre = x;
    y_pre = y;
    touch_nr++;
}

/* Doubletap2wake main function */
static bool detect_doubletap2wake(int x, int y, bool st)
{
    bool single_touch = st;
#if DT2W_DEBUG
    pr_info(LOGTAG"x,y(%4d,%4d) single:%s scr_suspended:%s\n",
            x, y, (single_touch) ? "true" : "false", (scr_suspended) ? "true" : "false");
#endif
    if(!scr_suspended)
        return false;
    
    if ((single_touch) && (dt2w_switch > 0) && (exec_count)) {
        if (touch_nr == 0) {
            new_touch(x, y);
        } else if (touch_nr == 1) {
            if ((calc_feather(x, x_pre) < DT2W_FEATHER) &&
                (calc_feather(y, y_pre) < DT2W_FEATHER) &&
                ((ktime_to_ms(ktime_get())-tap_time_pre) < DT2W_TIME))
                touch_nr++;
            else {
                doubletap2wake_reset();
                new_touch(x, y);
            }
        } else {
            doubletap2wake_reset();
            new_touch(x, y);
        }
        if ((touch_nr > 1)) {
            pr_info(LOGTAG"ON\n");
            doubletap2wake_pwrtrigger();
            doubletap2wake_reset();
        }
    }
    return true;
}

#ifdef CONFIG_TOUCHSCREEN_KNOCKCODE_UNLOCK
/* init a new touch */
static void new_tap_touch(int x, int y) {
    int i;
    for(i = 0;i < MAX_NUM_COORDS - 1;i++) {
        memcpy(&knock_code_tap_data[i + 1], &knock_code_tap_data[i], sizeof(tap_data));
    }
    
    tap_data* data = knock_code_tap_data[0];
    
    data->x_coor = x;
    data->y_coor = y;
    data->tmp_diff = 4000.0;
    data->tap_time = ktime_to_ms(ktime_get());
}

double sqrt(const double fg)
{ 
    double n = fg / 2.0;
    double lstX = 0.0; 
    while(n != lstX) { 
        lstX = n;
        n = (n + fg/n) / 2.0; 
    }
    return n;
}  

static void knock_code_reset(void) {
    int i = 0;
    for(i = 0;i < MAX_NUM_COORDS;i++) {
        memset(knock_code_tap_data, 0, sizeof(tap_data));
    }
}

static int knock_code_get_nearest(int p_length, int to_x, int to_y) {
    int cur_nearest = 0;
    int i;
    
    for(i = 0; i < p_length;i++) {
        tap_data* data = knock_code_tap_data[i];
        
        data->tmp_diff = sqrt(SQUARE(data->x_rel - to_x) + SQUARE(data->y_rel - to_y));
        if(i != 0 && data->tmp_diff < knock_code_tap_data[cur_nearest]->tmp_diff)
            cur_nearest = i;
    }
    return cur_nearest;
}

static bool detect_knock_code(int x, int y, bool singletouch) {
    if(!scr_suspended && singletouch)
        return false;

    new_tap_touch(x, y);
    
    int i;
    int p_length = 0;
    for(i = 0; i < MAX_NUM_COORDS;i++) {
        if(knock_code_tap_data[i] == NULL)
            i = MAX_NUM_COORDS + 1;
        else p_length++;
    }
    
    if(p_length < knock_code_pattern_length) {
        pr_info(LOGTAG"Abort. We have not the right data, p_length=%d;knock_length=%d", p_length, knock_code_pattern_length);
        return false;
    }
    
    pr_info(LOGTAG"checking for the right time difference, p_length=%d", p_length);
    
    for(i = 1; i < p_length;i++) {
        if((knock_code_tap_data[i]->tap_time - knock_code_tap_data[i - 1]->tap_time) > DT2W_TIME) {
            knock_code_reset();
            return false;
        }
    }
    
    int min_x = 480;
    int min_y = 800;
    int max_x = 0;
    int max_y = 0;
    
    pr_info(LOGTAG"Now checking for min and max");
    
    for(i = 0; i < p_length; i++) {
        x = knock_code_tap_data[i]->x_coor;
        y = knock_code_tap_data[i]->y_coor;
        
        min_x = MIN(x, min_x);
        min_y = MIN(y, min_y);
        max_x = MAX(x, max_x);
        max_y = MAX(y, max_y);
    }
    
    pr_info(LOGTAG" min(x|y) => (%d|%d) ; max(x|y) => (%d|%d)", min_x, min_y, max_x, max_y);
    
    for(i = 0; i < p_length; i++) {
        x = knock_code_tap_data[i]->x_coor;
        y = knock_code_tap_data[i]->y_coor;
        
        knock_code_tap_data[i]->x_rel = x - min_x;
        knock_code_tap_data[i]->y_rel = y - min_y;
    }
    
    pr_info(LOGTAG"Set up relative positions");
    
    for(i = 0; i < p_length;i++) {
        int nearest = 0;
        pr_info(LOGTAG"Now checking (%d|%d) for corner", knock_code_tap_data[i]->x_coor, knock_code_tap_data[i]->y_coor);
        switch(knock_code_pattern[p_length - i]) {
            case TAP_LOC_UP_RIGHT:
                nearest = knock_code_get_nearest(p_length, max_x, min_y);
                break;
            case TAP_LOC_DOWN_RIGHT:
                nearest = knock_code_get_nearest(p_length, max_x, max_y);
                break;
            case TAP_LOC_UP_LEFT:
                nearest = knock_code_get_nearest(p_length, min_x, min_y);
                break;
            case TAP_LOC_DOWN_LEFT:
                nearest = knock_code_get_nearest(p_length, max_x, max_y);
                break;
            default:
                break;
        }
        if(nearest != p_length - i) {
            pr_info(LOGTAG"Compare failed at index %d, because nearest=%d", i, nearest);
            knock_code_reset();
            return false;
        }
    }
    
    pr_info(LOGTAG"ON");
    doubletap2wake_pwrtrigger();
    knock_code_reset();
    
    return true;
}
#endif

extern bool detect_knock_inputs(int x, int y , bool singletouch) {
    if(dt2w_switch == 1) {
        return detect_doubletap2wake(x, y, singletouch);
    } else if(dt2w_switch == 2) {
#ifdef CONFIG_TOUCHSCREEN_KNOCKCODE_UNLOCK
        return detect_knock_code(x, y, singletouch);
#endif
    }
    return false;
}

#ifndef CONFIG_HAS_EARLYSUSPEND
static int lcd_notifier_callback(struct notifier_block *this,
                unsigned long event, void *data)
{
    switch (event) {
    case LCD_EVENT_ON_END:
        scr_suspended = false;
        break;
    case LCD_EVENT_OFF_END:
        scr_suspended = true;
        break;
    default:
        break;
    }

    return 0;
}
#else
static void dt2w_early_suspend(struct early_suspend *h) {
    scr_suspended = true;
}

static void dt2w_late_resume(struct early_suspend *h) {
    scr_suspended = false;
    wait_for_index = 0;
}

static struct early_suspend dt2w_early_suspend_handler = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = dt2w_early_suspend,
    .resume = dt2w_late_resume,
};
#endif

/*
 * SYSFS stuff below here
 */
static ssize_t dt2w_doubletap2wake_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    size_t count = 0;

    count += sprintf(buf, "%d\n", dt2w_switch);

    return count;
}

static ssize_t dt2w_doubletap2wake_dump(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    if (buf[0] >= '0' && buf[0] <= '2' && buf[1] == '\n')
                if (dt2w_switch != buf[0] - '0')
                dt2w_switch = buf[0] - '0';

    return count;
}

static DEVICE_ATTR(doubletap2wake, (S_IWUSR|S_IRUGO),
    dt2w_doubletap2wake_show, dt2w_doubletap2wake_dump);

static ssize_t dt2w_version_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    size_t count = 0;

    count += sprintf(buf, "%s\n", DRIVER_VERSION);

    return count;
}

static ssize_t dt2w_version_dump(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    return count;
}

static DEVICE_ATTR(doubletap2wake_version, (S_IWUSR|S_IRUGO),
    dt2w_version_show, dt2w_version_dump);

/*
 * INIT / EXIT stuff below here
 */
#ifdef ANDROID_TOUCH_DECLARED
extern struct kobject *android_touch_kobj;
#else
struct kobject *android_touch_kobj;
EXPORT_SYMBOL_GPL(android_touch_kobj);
#endif
static int __init doubletap2wake_init(void)
{
    int rc = 0;

    doubletap2wake_pwrdev = input_allocate_device();
    if (!doubletap2wake_pwrdev) {
        pr_err("Can't allocate suspend autotest power button\n");
        goto err_alloc_dev;
    }

    input_set_capability(doubletap2wake_pwrdev, EV_KEY, KEY_POWER);
    doubletap2wake_pwrdev->name = "dt2w_pwrkey";
    doubletap2wake_pwrdev->phys = "dt2w_pwrkey/input0";

    rc = input_register_device(doubletap2wake_pwrdev);
    if (rc) {
        pr_err("%s: input_register_device err=%d\n", __func__, rc);
        goto err_input_dev;
    }

#ifndef CONFIG_HAS_EARLYSUSPEND
    dt2w_lcd_notif.notifier_call = lcd_notifier_callback;
    if (lcd_register_client(&dt2w_lcd_notif) != 0) {
        pr_err("%s: Failed to register lcd callback\n", __func__);
    }
#else
    register_early_suspend(&dt2w_early_suspend_handler);
#endif

#ifndef ANDROID_TOUCH_DECLARED
    android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
    if (android_touch_kobj == NULL) {
        pr_warn("%s: android_touch_kobj create_and_add failed\n", __func__);
    }
#endif
    rc = sysfs_create_file(android_touch_kobj, &dev_attr_doubletap2wake.attr);
    if (rc) {
        pr_warn("%s: sysfs_create_file failed for doubletap2wake\n", __func__);
    }
    rc = sysfs_create_file(android_touch_kobj, &dev_attr_doubletap2wake_version.attr);
    if (rc) {
        pr_warn("%s: sysfs_create_file failed for doubletap2wake_version\n", __func__);
    }
    
#ifdef CONFIG_TOUCHSCREEN_KNOCKCODE_UNLOCK
    int i = 0;
    for(i = 0;i < MAX_NUM_COORDS;i++) {
        knock_code_tap_data[i] = (tap_data*) kzalloc(sizeof(tap_data), GFP_KERNEL);
    }
#endif

err_input_dev:
    input_free_device(doubletap2wake_pwrdev);
err_alloc_dev:
    pr_info(LOGTAG"%s done\n", __func__);

    return 0;
}

static void __exit doubletap2wake_exit(void)
{
#ifndef ANDROID_TOUCH_DECLARED
    kobject_del(android_touch_kobj);
#endif
#ifndef CONFIG_HAS_EARLYSUSPEND
    lcd_unregister_client(&dt2w_lcd_notif);
#endif
    destroy_workqueue(dt2w_input_wq);
    input_unregister_device(doubletap2wake_pwrdev);
    input_free_device(doubletap2wake_pwrdev);
#ifdef CONFIG_TOUCHSCREEN_KNOCKCODE_UNLOCK
    kfree(knock_code_tap_data);
#endif
    return;
}

module_init(doubletap2wake_init);
module_exit(doubletap2wake_exit);

