/*
 *      bmi_sensor.c
 *
 *      BMI sensor device driver
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 *      Include files
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <asm/uaccess.h>

#include <linux/bmi.h>
#include <linux/bmi-ids.h>
#include <linux/bmi/bmi-control.h>
#include <linux/bmi/bmi-slot.h>
#include <linux/bmi/bmi_sensor.h>


#define BMISENSOR_VERSION       "2.0"

#define work_to_sensor(w) (struct bmi_sensor*) container_of(w, struct bmi_sensor, work_item)
#define dev_to_bmi_device(d) (struct bmi_device*) container_of(d, struct bmi_device, dev)

/*
 *      Global variables
 */

static struct i2c_board_info mee_info = {
    I2C_BOARD_INFO("SENSOR_MEE", BMI_MEE_I2C_ADDRESS),
};

static struct i2c_board_info iox_info = {
    I2C_BOARD_INFO("SENSOR_IOX", BMI_IOX_I2C_ADDRESS),
};

static struct i2c_board_info adc_info = {
    I2C_BOARD_INFO("SENSOR_ADC", BMI_ADC_I2C_ADDRESS),
};

static struct i2c_board_info pl_info = {
    I2C_BOARD_INFO("SENSOR_PL", BMI_PL_I2C_ADDRESS),
};

static struct i2c_board_info dlight_info = {
    I2C_BOARD_INFO("SENSOR_DLIGHT", BMI_DLIGHT_I2C_ADDRESS),
};

static struct i2c_board_info temp_info = {
    I2C_BOARD_INFO("SENSOR_TEMP", BMI_TEMP_I2C_ADDRESS),
};

static struct i2c_board_info acc_info = {
    I2C_BOARD_INFO("SENSOR_ACC", BMI_ACC_I2C_ADDRESS),
};

static struct i2c_board_info dcomp_info = {
    I2C_BOARD_INFO("SENSOR_DCOMP", BMI_DCOMP_I2C_ADDRESS),
};

static ushort factory_test = 0;
static int eeprom_init = 0;
static ushort xdac_init = 0;
static ushort ydac_init = 0;
static ushort zdac_init = 0;
static ushort fcc_test = 0;

// private device structure
struct bmi_sensor
{
    struct semaphore                sem;                    // bmi_sensor mutex
    struct bmi_device               *bdev;                  // BMI device
    struct cdev                     cdev;                   // control device
    struct device                   *class_dev;             // control class device
    struct sensor_eeprom_raw        eeprom;                         // eeprom contents
    char                            int_name[20];           // interrupt name
    struct workqueue_struct         *workqueue;             // interrupt work queue
    struct work_struct              work_item;              // interrupt work structure
    char                            work_name[20];          // workqueue name
    wait_queue_head_t               pl_wait_queue;          // Proximity/Light interrupt wait queue
    unsigned char                   pl_int_en;              // Proximity/Light interrupts are enabled
    unsigned char                   pl_int_fl;              // Proximity/Light interrupt occurred
    wait_queue_head_t               temp_wait_queue;        // Temperature interrupt wait queue
    unsigned char                   temp_int_en;            // Temperature interrupts are enabled
    unsigned char                   temp_int_fl;            // Temperature interrupt occurred
    wait_queue_head_t               mot_wait_queue;         // Motion interrupt wait queue
    unsigned char                   mot_int_en;             // Motion interrupts are enabled
    unsigned char                   mot_int_fl;             // Motion interrupt occurred
    unsigned int                    mot_state;              // previous motion detector state
    wait_queue_head_t               acc_wait1_queue;        // Accelerometer interrupt wait queue
    unsigned char                   acc_int1_en;            // Accelerometer interrupts are enabled
    unsigned char                   acc_int1_fl;            // Accelerometer interrupt occurred
    wait_queue_head_t               acc_wait2_queue;        // Accelerometer interrupt wait queue
    unsigned char                   acc_int2_en;            // Accelerometer interrupts are enabled
    unsigned char                   acc_int2_fl;            // Accelerometer interrupt occurred
    wait_queue_head_t               usb_wait_queue;         // USB interrupt wait queue
    unsigned char                   usb_int_en;             // USB interrupts are enabled
    unsigned char                   usb_int_fl;             // USB interrupt occurred
    wait_queue_head_t               dcomp_wait_queue;       // Digital compass interrupt wait queue
    unsigned char                   dcomp_int_en;           // Digital compass interrupts are enabled
    unsigned char                   dcomp_int_fl;           // Digital compass interrupt occurred
    unsigned int                    aprox_duration;         // Analog Proximity LED burst duration (ms)
    struct timer_list               aprox_timer;            // Analog Proximity LED burst timer
    wait_queue_head_t               aprox_wait_queue;       // Analog Proximity timer wait queue
    unsigned char                   aprox_int_en;           // Analog Proximity timer are enabled
    unsigned char                   aprox_int_fl;           // Analog Proximity timer occurred
    wait_queue_head_t               dlight_wait_queue;      // Digital Light interrupt wait queue
    unsigned char                   dlight_int_en;          // Digital Light interrupts are enabled
    unsigned char                   dlight_int_fl;          // Digital Light interrupt occurred
    unsigned int                    comp_xsf;               // Compass calibration
    unsigned int                    comp_ysf;               // Compass calibration
    unsigned int                    comp_zsf;               // Compass calibration
    unsigned int                    comp_xoff;              // Compass calibration
    unsigned int                    comp_yoff;              // Compass calibration
    unsigned int                    comp_zoff;              // Compass calibration
    struct i2c_client *             mee_i2c_client;
    struct i2c_client *             iox_i2c_client;
    struct i2c_client *             adc_i2c_client;
    struct i2c_client *             pl_i2c_client;
    struct i2c_client *             dlight_i2c_client;
    struct i2c_client *             temp_i2c_client;
    struct i2c_client *             acc_i2c_client;
    struct i2c_client *             dcomp_i2c_client;
};

static struct bmi_sensor bmi_sensor[4];         // per slot device structure
static int major;                       // control device major

/*
 *      BMI set up
 */

// BMI device ID table
static struct bmi_device_id bmi_sensor_tbl[] =
{
    {
        .match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT,
        .vendor   = BMI_VENDOR_BUG_LABS,
        .product  = BMI_PRODUCT_SENSOR,
        .revision = BMI_ANY,
    },
    { 0, },           /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_sensor_tbl);

int     bmi_sensor_probe(struct bmi_device *bdev);
void    bmi_sensor_remove(struct bmi_device *bdev);

// BMI driver structure
static struct bmi_driver bmi_sensor_driver =
{
    .name = "bmi_sensor",
    .id_table = bmi_sensor_tbl,
    .probe   = bmi_sensor_probe,
    .remove  = bmi_sensor_remove,
};

/*
 *      I2C set up
 */


static int get_mot_det_state(int slot)
{
    return (bmi_slot_gpio_get_all(slot) & (2 ^SENSOR_GPIO_MOT_DET)) ? 1 : 0;
}

// IOX
// write byte to I2C IO expander
static int WriteByte_IOX(struct i2c_client *client, unsigned char offset, unsigned char data)
{
    int ret = 0;
    unsigned char msg[2];

    msg[0] = offset;
    msg[1] = data;
    ret = i2c_master_send(client, msg, sizeof(msg));

    if (ret < 0)
        printk (KERN_ERR "WriteByte_IOX() - i2c_transfer() failed...%d\n",ret);

    return ret;
}

// read byte from I2C IO expander
static int ReadByte_IOX(struct i2c_client *client, unsigned char offset, unsigned char *data)
{
    int ret = 0;

    ret = i2c_master_send(client, &offset, 1);

    if (ret == 1)
        ret = i2c_master_recv(client, data, 1);

    if (ret < 0)
        printk (KERN_ERR "ReadByte_IOX() - i2c_transfer() failed...%d\n",ret);

    return ret;
}

// ADC
// write byte to ADC
static int WriteByte_ADC(struct i2c_client *client, unsigned char data)
{
    int ret = 0;

    ret = i2c_master_send(client, &data, sizeof(data));

    if (ret < 0)
        printk (KERN_ERR "WriteByte_ADC() - i2c_transfer() failed...%d\n",ret);

    return ret;
}

// read data from ADC
static int ReadByte_ADC(struct i2c_client *client, unsigned char *data)
{
    int ret = 0;

    ret = i2c_master_recv(client, data, 2);

    if (ret < 0)
        printk (KERN_ERR "ReadByte_ADC() - i2c_transfer() failed...%d\n",ret);

    return ret;
}

// Proximity/Light and Digital Light (same I2c address and format)
// write byte to I2C PL
static int WriteByte_PL(struct i2c_client *client, unsigned char offset, unsigned char data)
{
    int ret = 0;
    unsigned char msg[2];

    msg[0] = offset;
    msg[1] = data;
    ret = i2c_master_send(client, msg, sizeof(msg));

    if (ret < 0)
        printk (KERN_ERR "WriteByte_PL() - i2c_transfer() failed...%d\n",ret);

    return ret;
}

// read byte from I2C PL
static int ReadByte_PL(struct i2c_client *client, unsigned char offset, unsigned char *data)
{
    int ret = 0;

    ret = i2c_master_send(client, &offset, 1);
    if (ret == 1)
        ret = i2c_master_recv(client, data, 1);
    if (ret < 0)
        printk (KERN_ERR "ReadByte_PL() - i2c_transfer() failed...%d\n",ret);
    return ret;
}

// write byte to I2C PL SYNC
static int WriteByte_PL_SYNC(struct i2c_client *client)
{
    int ret = 0;
    unsigned char offset = SENSOR_PL_EXT_SYNC;

    ret = i2c_master_send(client, &offset, 1);
    if (ret < 0)
        printk (KERN_ERR "WriteByte_PL_SYNC() - i2c_transfer() failed...%d\n",ret);
    return ret;
}

// write byte to I2C DL Interrupt Clear
static int WriteByte_DL_IC(struct i2c_client *client)
{
    int ret = 0;
    unsigned char offset = SENSOR_DL_INT_CLR;

    ret = i2c_master_send(client, &offset, 1);
    if (ret < 0)
        printk (KERN_ERR "WriteByte_DL_IC() - i2c_transfer() failed...%d\n",ret);
    return ret;
}

// Temperature
// write byte to Temperature sensor
static int WriteByte_TEMP(struct i2c_client *client, unsigned char offset, unsigned char data)
{
    int ret = 0;
    unsigned char msg[2];

    msg[0] = offset;
    msg[1] = data;
    ret = i2c_master_send(client, msg, sizeof(msg));

    if (ret < 0)
        printk (KERN_ERR "WriteByte_TEMP() - i2c_transfer() failed...%d\n",ret);

    return ret;
}

// read byte from Temperature sensor
static int ReadByte_TEMP(struct i2c_client *client, unsigned char offset, unsigned char *data)
{
    int ret = 0;

    ret = i2c_master_send(client, &offset, 1);
    if (ret == 1)
        ret = i2c_master_recv(client, data, 1);
    if (ret < 0)
        printk (KERN_ERR "ReadByte_TEMP() - i2c_transfer() failed...%d\n",ret);
    return ret;
}

// Accelerometer
// write byte to I2C Accelerometer
static int WriteByte_ACC(struct i2c_client *client, struct sensor_acc_rw *acc_rw)
{
    int ret = 0;
    unsigned char msg[2];

    msg[0] = acc_rw->address;
    msg[1] = acc_rw->data[0];
    ret = i2c_master_send(client, msg, sizeof(msg));

    if (ret < 0)
        printk (KERN_ERR "WriteByte_ACC() - i2c_transfer() failed...%d\n",ret);

    return ret;
}

// read byte(s) from Acceleromter
static int ReadByte_ACC(struct i2c_client *client, struct sensor_acc_rw *acc_rw)
{
    int ret = 0;

    ret = i2c_master_send(client, &acc_rw->address, 1);
    if (ret == 1)
        ret = i2c_master_recv(client, acc_rw->data, acc_rw->count);
    if (ret < 0)
        printk (KERN_ERR "ReadByte_ACC() - i2c_transfer() failed...%d\n",ret);
    return ret;
}

// digital compass
// write byte to digital compass
static int WriteByte_DCOMP(struct i2c_client *client, unsigned char offset, unsigned char data)
{
    int ret = 0;
    unsigned char msg[2];

    msg[0] = offset;
    msg[1] = data;
    ret = i2c_master_send(client, msg, sizeof(msg));

    if (ret < 0)
        printk (KERN_ERR "WriteByte_DCOMP() - i2c_transfer() failed...%d\n",ret);

    return ret;
}

// read byte from digital compass
static int ReadByte_DCOMP(struct i2c_client *client, unsigned char offset, unsigned char *data)
{
    int ret = 0;

    ret = i2c_master_send(client, &offset, 1);
    if (ret == 1)
        ret = i2c_master_recv(client, data, 1);
    if (ret < 0)
        printk (KERN_ERR "ReadByte_DCOMP() - i2c_transfer() failed...%d\n",ret);
    return ret;
}

// EEPROM
// write byte to I2C EEPROM
static int WriteByte_EE(struct i2c_client *client, unsigned char offset, unsigned char data)
{
    int ret = 0;
    unsigned char msg[2];

    msg[0] = offset;
    msg[1] = data;
    ret = i2c_master_send(client, msg, sizeof(msg));

    if (ret < 0)
        printk (KERN_ERR "WriteByte_EE() - i2c_transfer() failed...%d\n",ret);

    return ret;
}

// read byte from I2C EEPROM
static int ReadByte_EE(struct i2c_client *client, unsigned char offset, unsigned char *data)
{
    int ret = 0;

    ret = i2c_master_send(client, &offset, 1);
    if (ret == 1)
        ret = i2c_master_recv(client, data, 1);
    if (ret < 0)
        printk (KERN_ERR "ReadByte_EE() - i2c_transfer() failed...%d\n",ret);
    return ret;
}

/*
 *      control device operations
 */

// open
int cntl_open(struct inode *inode, struct file *file)
{
    struct bmi_sensor *sensor = container_of(inode->i_cdev, struct bmi_sensor, cdev);
    file->private_data = sensor;
    return 0;

}

// release
int cntl_release(struct inode *inode, struct file *file)
{
    return 0;
}

// analog proximity timer function
void aptimer(unsigned long arg)
{
    struct bmi_sensor *sensor = (struct bmi_sensor *) arg;
    int ret;

    del_timer (&sensor->aprox_timer);

    // wake sleepers
    ret = down_interruptible(&sensor->sem);
    sensor->aprox_int_en = 0;
    sensor->aprox_int_fl = 1;
    up(&sensor->sem);
    wake_up_all(&sensor->aprox_wait_queue);
}

// ioctl
int cntl_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
               unsigned long arg)
{
    unsigned char iox_data;
    struct bmi_sensor *sensor = (struct bmi_sensor *)(file->private_data);
    int slot = sensor->bdev->slot->slotnum;
    int ret = 0;

    // error if sensor not present
    if(sensor->bdev == 0)
        return -ENODEV;

    // ioctl's
    switch(cmd) {

    case BMI_SENSOR_RLEDOFF:
        bmi_slot_gpio_set_value(slot, SENSOR_GPIO_RED_LED, 0);
        break;

    case BMI_SENSOR_RLEDON:
        bmi_slot_gpio_set_value(slot, SENSOR_GPIO_RED_LED, 1);
        break;

    case BMI_SENSOR_GLEDOFF:
        bmi_slot_gpio_set_value(slot, SENSOR_GPIO_GREEN_LED, 0);
        break;

    case BMI_SENSOR_GLEDON:
        bmi_slot_gpio_set_value(slot, SENSOR_GPIO_GREEN_LED, 1);
        break;

    case BMI_SENSOR_GETSTAT:
    {
        int read_data;

        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT0_REG, &iox_data) < 0)
            return -ENODEV;
        read_data = iox_data;

        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0)
            return -ENODEV;
        read_data |= (iox_data << 8) | (bmi_slot_gpio_get_all(slot) << 16);

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_ADCWR:
    {
        unsigned char adc_data;

        if(sensor->eeprom.adc_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        adc_data = (unsigned char) (arg & 0xFF);
        if(WriteByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
            return -ENODEV;
    }
    break;

    case BMI_SENSOR_ADCRD:
    {
        unsigned char adc_data[2];
        int read_data;

        if(sensor->eeprom.adc_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
            return -ENODEV;
        read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_HUMRD:
    {
        unsigned char adc_data[2];
        int read_data;

        if(sensor->eeprom.humidity_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_HUMIDITY | SENSOR_ADC_PD_OFF) < 0)
            return -ENODEV;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
            return -ENODEV;
        read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_ACOMPRST:
    {
        if(sensor->eeprom.acompass_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT0_REG, &iox_data) < 0)
            return -ENODEV;

        iox_data &= ~(0x1 << SENSOR_IOX_COMP_RS_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, iox_data) < 0)
            return -ENODEV;

        mdelay(5);

        iox_data |= (0x1 << SENSOR_IOX_COMP_RS_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, iox_data) < 0)
            return -ENODEV;
    }
    break;

    case BMI_SENSOR_ACOMPXRD:
    {
        unsigned char adc_data[2];
        int read_data;

        if(sensor->eeprom.acompass_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_X | SENSOR_ADC_PD_OFF) < 0)
            return -ENODEV;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
            return -ENODEV;
        read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_ACOMPYRD:
    {
        unsigned char adc_data[2];
        int read_data;

        if(sensor->eeprom.acompass_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_Y | SENSOR_ADC_PD_OFF) < 0)
            return -ENODEV;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
            return -ENODEV;
        read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_ACOMPZRD:
    {
        unsigned char adc_data[2];
        int read_data;

        if(sensor->eeprom.acompass_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_Z | SENSOR_ADC_PD_OFF) < 0)
            return -ENODEV;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
            return -ENODEV;
        read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_PLWR:
    {
        struct sensor_pl_rw *pl = NULL;
        unsigned char pl_data;

        if(sensor->eeprom.light_proximity_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if ((pl = kmalloc(sizeof(struct sensor_pl_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(pl, (struct sensor_pl_rw *) arg, sizeof(struct sensor_pl_rw))) {
            kfree(pl);
            return -EFAULT;
        }

        pl_data = pl->cmd1;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->cmd2;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD2, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->int_lt_lsb;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_INT_LT_LSB, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->int_lt_msb;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_INT_LT_MSB, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->int_ht_lsb;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_INT_HT_LSB, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->int_ht_msb;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_INT_HT_MSB, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        kfree(pl);
    }
    break;

    case BMI_SENSOR_PLRD:
    {
        struct sensor_pl_rw *pl = NULL;
        unsigned char pl_data;

        if(sensor->eeprom.light_proximity_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if ((pl = kmalloc(sizeof(struct sensor_pl_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, &pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }
        pl->cmd1 = pl_data;

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }
        pl->dm = pl_data;

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_LSB, &pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }
        pl->dl = pl_data;

        if(copy_to_user((struct sensor_pl_rw *) arg, pl, sizeof(struct sensor_pl_rw))) {
            kfree(pl);
            return -EFAULT;
        }

        kfree(pl);
    }
    break;

    case BMI_SENSOR_PL_SYNC:
    {
        if(sensor->eeprom.light_proximity_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if(WriteByte_PL_SYNC(sensor->pl_i2c_client) < 0)
            return -ENODEV;
    }
    break;

    case BMI_SENSOR_PL_IWAIT:
    {
        struct sensor_pl_rw *pl = NULL;
        unsigned char pl_data;

        if(sensor->eeprom.light_proximity_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if ((pl = kmalloc(sizeof(struct sensor_pl_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(pl, (struct sensor_pl_rw *) arg, sizeof(struct sensor_pl_rw))) {
            kfree(pl);
            return -EFAULT;
        }

        pl_data = pl->cmd1;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->cmd2;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD2, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->int_lt_lsb;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_INT_LT_LSB, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->int_lt_msb;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_INT_LT_MSB, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->int_ht_lsb;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_INT_HT_LSB, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        pl_data = pl->int_ht_msb;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_INT_HT_MSB, pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }

        ret = down_interruptible(&sensor->sem);
        sensor->pl_int_en = 1;
        sensor->pl_int_fl = 0;
        up(&sensor->sem);
        ret = wait_event_interruptible(sensor->pl_wait_queue, (sensor->pl_int_fl == 1));
        if(ret)
            return ret;

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, &pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }
        pl->cmd1 = pl_data;

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }
        pl->dm = pl_data;

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_LSB, &pl_data) < 0) {
            kfree(pl);
            return -ENODEV;
        }
        pl->dl = pl_data;

        if(copy_to_user((struct sensor_pl_rw *) arg, pl, sizeof(struct sensor_pl_rw))) {
            kfree(pl);
            return -EFAULT;
        }

        kfree(pl);
    }
    break;

    case BMI_SENSOR_SNDARD:
    {
        unsigned char adc_data[2];
        int read_data;

        if(sensor->eeprom.sound_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_SOUND_AVG | SENSOR_ADC_PD_OFF) < 0)
            return -ENODEV;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
            return -ENODEV;
        read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_SNDPRD:
    case BMI_SENSOR_SNDIRD:
    {
        unsigned char adc_data[2];
        int read_data;

        if(sensor->eeprom.sound_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        // read peak
        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_SOUND_PEAK | SENSOR_ADC_PD_OFF) < 0)
            return -ENODEV;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
            return -ENODEV;
        read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

        // clear peak
        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0)
            return -ENODEV;

        iox_data &= ~(0x1 << SENSOR_IOX_S_PK_CLR_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
            return -ENODEV;

        mdelay(1);

        iox_data |= (0x1 << SENSOR_IOX_S_PK_CLR_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
            return -ENODEV;

        if(cmd == BMI_SENSOR_SNDPRD) {
            // return data
            if(put_user(read_data, (int __user *) arg))
                return -EFAULT;
        } else {

            // read peak
            if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_SOUND_PEAK | SENSOR_ADC_PD_OFF) < 0)
                return -ENODEV;

            mdelay(1);

            if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
                return -ENODEV;
            read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

            // return data
            if(put_user(read_data, (int __user *) arg))
                return -EFAULT;
        }

    }
    break;

    case BMI_SENSOR_TEMPWR:
    {
        struct sensor_temp_rw *temp = NULL;
        unsigned char temp_addr;
        unsigned char temp_data;

        if(sensor->eeprom.temperature_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if ((temp = kmalloc(sizeof(struct sensor_temp_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(temp, (struct sensor_temp_rw *) arg, sizeof(struct sensor_temp_rw))) {
            kfree(temp);
            return -EFAULT;
        }

        temp_addr = temp->address;
        temp_data = temp->d1;
        if(WriteByte_TEMP(sensor->temp_i2c_client, temp_addr, temp_data) < 0) {
            kfree(temp);
            return -ENODEV;
        }

        kfree(temp);
    }
    break;

    case BMI_SENSOR_TEMPRD:
    {
        struct sensor_temp_rw *temp = NULL;
        unsigned char temp_addr;
        unsigned char temp_data;

        if(sensor->eeprom.temperature_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if ((temp = kmalloc(sizeof(struct sensor_temp_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(temp, (struct sensor_temp_rw *) arg, sizeof(struct sensor_temp_rw))) {
            kfree(temp);
            return -EFAULT;
        }

        temp_addr = temp->address;
        if(ReadByte_TEMP(sensor->temp_i2c_client, temp_addr, &temp_data) < 0) {
            kfree(temp);
            return -ENODEV;
        }

        temp->d1 = temp_data;
        if(copy_to_user((struct sensor_temp_rw *) arg, temp, sizeof(struct sensor_temp_rw))) {
            kfree(temp);
            return -EFAULT;
        }

        kfree(temp);
    }
    break;

    case BMI_SENSOR_TEMPRD_SL:
    {
        unsigned int read_data;
        unsigned char temp_datam;
        unsigned char temp_datal;

        if(sensor->eeprom.temperature_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_LOC_MSB, &temp_datam) < 0) {
            return -ENODEV;
        }

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_LOC_LSB, &temp_datal) < 0) {
            return -ENODEV;
        }

        read_data = (temp_datam << 8) | temp_datal;
        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;

    }
    break;

    case BMI_SENSOR_TEMPRD_SR:
    {
        unsigned int read_data;
        unsigned char temp_datam;
        unsigned char temp_datal;

        if(sensor->eeprom.temperature_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_REM_MSB, &temp_datam) < 0) {
            return -ENODEV;
        }

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_REM_LSB, &temp_datal) < 0) {
            return -ENODEV;
        }

        read_data = (temp_datam << 8) | temp_datal;
        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;

    }
    break;

    case BMI_SENSOR_TEMPRD_UR:
    {
        unsigned int read_data;
        unsigned char temp_datam;
        unsigned char temp_datal;

        if(sensor->eeprom.temperature_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_UREM_MSB, &temp_datam) < 0) {
            return -ENODEV;
        }

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_UREM_LSB, &temp_datal) < 0) {
            return -ENODEV;
        }

        read_data = (temp_datam << 8) | temp_datal;
        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;

    }
    break;

    case BMI_SENSOR_TEMP_IWAIT:
    {
        if(sensor->eeprom.temperature_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        ret = down_interruptible(&sensor->sem);
        sensor->temp_int_en = 1;
        sensor->temp_int_fl = 0;
        up(&sensor->sem);
        ret = wait_event_interruptible(sensor->temp_wait_queue, (sensor->temp_int_fl == 1));
        if(ret)
            return ret;
    }
    break;

    case BMI_SENSOR_MOTRD:
    {
        unsigned int read_data;

        if(sensor->eeprom.motion_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        read_data = get_mot_det_state(slot);
        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;

    }
    break;

    case BMI_SENSOR_MOT_IWAIT:
    {
        unsigned int read_data;

        if(sensor->eeprom.motion_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        ret = down_interruptible(&sensor->sem);
        sensor->mot_int_en = 1;
        sensor->mot_int_fl = 0;
        up(&sensor->sem);
        ret = wait_event_interruptible(sensor->mot_wait_queue, (sensor->mot_int_fl == 1));
        if(ret)
            return ret;

        read_data = get_mot_det_state(slot);
        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;

    }
    break;

    case BMI_SENSOR_ACCWR:
    {
        struct sensor_acc_rw *acc = NULL;

        if((sensor->eeprom.acc_present != SENSOR_DEVICE_PRESENT)
           && (sensor->eeprom.acc302_present != SENSOR_DEVICE_PRESENT))
            return -ENODEV;

        if ((acc = kmalloc(sizeof(struct sensor_acc_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(acc, (struct sensor_acc_rw *) arg, sizeof(struct sensor_acc_rw))) {
            kfree(acc);
            return -EFAULT;
        }

        if(WriteByte_ACC(sensor->acc_i2c_client, acc) < 0) {
            kfree(acc);
            return -ENODEV;
        }

        kfree(acc);
    }
    break;

    case BMI_SENSOR_ACCRD:
    {
        struct sensor_acc_rw *acc = NULL;

        if((sensor->eeprom.acc_present != SENSOR_DEVICE_PRESENT)
           && (sensor->eeprom.acc302_present != SENSOR_DEVICE_PRESENT))
            return -ENODEV;

        if ((acc = kmalloc(sizeof(struct sensor_acc_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(acc, (struct sensor_acc_rw *) arg, sizeof(struct sensor_acc_rw))) {
            kfree(acc);
            return -EFAULT;
        }

        if(ReadByte_ACC(sensor->acc_i2c_client, acc) < 0) {
            kfree(acc);
            return -ENODEV;
        }

        if(copy_to_user((struct sensor_acc_rw *) arg, acc, sizeof(struct sensor_acc_rw))) {
            kfree(acc);
            return -EFAULT;
        }

        kfree(acc);
    }
    break;

    case BMI_SENSOR_ACCXRD:
    {
        struct sensor_acc_rw *acc = NULL;
        unsigned int read_data;

        if(sensor->eeprom.acc_present == SENSOR_DEVICE_PRESENT) {

            if ((acc = kmalloc(sizeof(struct sensor_acc_rw), GFP_KERNEL)) == NULL)
                return -ENOMEM;

            acc->address = SENSOR_ACC_DX0;
            acc->count = 2;

            if(ReadByte_ACC(sensor->acc_i2c_client, acc) < 0) {
                kfree(acc);
                return -ENODEV;
            }

            read_data = (acc->data[1] << 8) | acc->data[0];
            if(put_user(read_data, (int __user *) arg)) {
                kfree(acc);
                return -EFAULT;
            }
        } else if(sensor->eeprom.acc302_present == SENSOR_DEVICE_PRESENT) {
            if ((acc = kmalloc(sizeof(struct sensor_acc_rw), GFP_KERNEL)) == NULL)
                return -ENOMEM;

            acc->address = SENSOR_A3_OUTX;
            acc->count = 1;

            if(ReadByte_ACC(sensor->acc_i2c_client, acc) < 0) {
                kfree(acc);
                return -ENODEV;
            }

            read_data = acc->data[0];
            if(put_user(read_data, (int __user *) arg)) {
                kfree(acc);
                return -EFAULT;
            }
        } else {
            return -ENODEV;
        }

        kfree(acc);
    }
    break;

    case BMI_SENSOR_ACCYRD:
    {
        struct sensor_acc_rw *acc = NULL;
        unsigned int read_data;

        if(sensor->eeprom.acc_present != SENSOR_DEVICE_PRESENT) {
            if ((acc = kmalloc(sizeof(struct sensor_acc_rw), GFP_KERNEL)) == NULL)
                return -ENOMEM;

            acc->address = SENSOR_ACC_DY0;
            acc->count = 2;

            if(ReadByte_ACC(sensor->acc_i2c_client, acc) < 0) {
                kfree(acc);
                return -ENODEV;
            }

            read_data = (acc->data[1] << 8) | acc->data[0];
            if(put_user(read_data, (int __user *) arg)) {
                kfree(acc);
                return -EFAULT;
            }
        } else if(sensor->eeprom.acc302_present == SENSOR_DEVICE_PRESENT) {
            if ((acc = kmalloc(sizeof(struct sensor_acc_rw), GFP_KERNEL)) == NULL)
                return -ENOMEM;

            acc->address = SENSOR_A3_OUTY;
            acc->count = 1;

            if(ReadByte_ACC(sensor->acc_i2c_client, acc) < 0) {
                kfree(acc);
                return -ENODEV;
            }

            read_data = acc->data[0];
            if(put_user(read_data, (int __user *) arg)) {
                kfree(acc);
                return -EFAULT;
            }
        } else {
            return -ENODEV;
        }

        kfree(acc);
    }
    break;

    case BMI_SENSOR_ACCZRD:
    {
        struct sensor_acc_rw *acc = NULL;
        unsigned int read_data;

        if(sensor->eeprom.acc_present != SENSOR_DEVICE_PRESENT) {
            if ((acc = kmalloc(sizeof(struct sensor_acc_rw), GFP_KERNEL)) == NULL)
                return -ENOMEM;

            acc->address = SENSOR_ACC_DZ0;
            acc->count = 2;

            if(ReadByte_ACC(sensor->acc_i2c_client, acc) < 0) {
                kfree(acc);
                return -ENODEV;
            }

            read_data = (acc->data[1] << 8) | acc->data[0];
            if(put_user(read_data, (int __user *) arg)) {
                kfree(acc);
                return -EFAULT;
            }
        } else if(sensor->eeprom.acc302_present == SENSOR_DEVICE_PRESENT) {
            if ((acc = kmalloc(sizeof(struct sensor_acc_rw), GFP_KERNEL)) == NULL)
                return -ENOMEM;

            acc->address = SENSOR_A3_OUTZ;
            acc->count = 1;

            if(ReadByte_ACC(sensor->acc_i2c_client, acc) < 0) {
                kfree(acc);
                return -ENODEV;
            }

            read_data = acc->data[0];
            if(put_user(read_data, (int __user *) arg)) {
                kfree(acc);
                return -EFAULT;
            }
        } else {
            return -ENODEV;
        }

        kfree(acc);
    }
    break;

    case BMI_SENSOR_ACC_I1WAIT:
    {
        if((sensor->eeprom.acc_present != SENSOR_DEVICE_PRESENT)
           && (sensor->eeprom.acc302_present != SENSOR_DEVICE_PRESENT))
            return -ENODEV;

        ret = down_interruptible(&sensor->sem);
        sensor->acc_int1_en = 1;
        sensor->acc_int1_fl = 0;
        up(&sensor->sem);
        ret = wait_event_interruptible(sensor->acc_wait1_queue, (sensor->acc_int1_fl == 1));
        if(ret)
            return ret;
    }
    break;

    case BMI_SENSOR_ACC_I2WAIT:
    {
        if((sensor->eeprom.acc_present != SENSOR_DEVICE_PRESENT)
           && (sensor->eeprom.acc302_present != SENSOR_DEVICE_PRESENT))
            return -ENODEV;

        ret = down_interruptible(&sensor->sem);
        sensor->acc_int2_en = 1;
        sensor->acc_int2_fl = 0;
        up(&sensor->sem);
        ret = wait_event_interruptible(sensor->acc_wait2_queue, (sensor->acc_int2_fl == 1));
        if(ret)
            return ret;
    }
    break;

    case BMI_SENSOR_EEWR:
    {
        struct sensor_rw *ee = NULL;
        unsigned char ee_addr;
        unsigned char ee_data;

        if ((ee = kmalloc(sizeof(struct sensor_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(ee, (struct sensor_rw *) arg, sizeof(struct sensor_rw))) {
            kfree(ee);
            return -EFAULT;
        }

        ee_addr = ee->address;
        ee_data = ee->data;
        if(WriteByte_EE(sensor->mee_i2c_client, ee_addr, ee_data) < 0) {
            kfree(ee);
            return -ENODEV;
        }

        kfree(ee);
    }
    break;

    case BMI_SENSOR_EERD:
    {
        struct sensor_rw *ee = NULL;
        unsigned char ee_addr;
        unsigned char ee_data;

        if ((ee = kmalloc(sizeof(struct sensor_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(ee, (struct sensor_rw *) arg, sizeof(struct sensor_rw))) {
            kfree(ee);
            return -EFAULT;
        }

        ee_addr = ee->address;
        if(ReadByte_EE(sensor->mee_i2c_client, ee_addr, &ee_data) < 0) {
            kfree(ee);
            return -ENODEV;
        }

        ee->data = ee_data;
        if(copy_to_user((struct sensor_rw *) arg, ee, sizeof(struct sensor_rw))) {
            kfree(ee);
            return -EFAULT;
        }

        kfree(ee);
    }
    break;

    case BMI_SENSOR_MOT_IE:
    {
        if(sensor->eeprom.motion_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, &iox_data) < 0)
            return -ENODEV;

        if(arg == BMI_SENSOR_ON)
            iox_data |= (0x1 << SENSOR_IOX_MOT_EN);
        else
            iox_data &= ~(0x1 << SENSOR_IOX_MOT_EN);

        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, iox_data) < 0)
            return -ENODEV;
    }
    break;

    case BMI_SENSOR_USB_IWAIT:
    {
        ret = down_interruptible(&sensor->sem);
        sensor->usb_int_en = 1;
        sensor->usb_int_fl = 0;
        up(&sensor->sem);
        ret = wait_event_interruptible(sensor->usb_wait_queue, (sensor->usb_int_fl == 1));
        if(ret)
            return ret;
    }
    break;

    case BMI_SENSOR_USB_PWR_EN:
    {
        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, &iox_data) < 0)
            return -ENODEV;

        if(arg == BMI_SENSOR_ON)
            iox_data |= (0x1 << SENSOR_IOX_USB_EN);
        else
            iox_data &= ~(0x1 << SENSOR_IOX_USB_EN);

        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, iox_data) < 0)
            return -ENODEV;
    }
    break;

    case BMI_SENSOR_HUM_PWR_EN:
    {
        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, &iox_data) < 0)
            return -ENODEV;

        if(arg == BMI_SENSOR_ON)
            iox_data |= (0x1 << SENSOR_IOX_HUM_EN);
        else
            iox_data &= ~(0x1 << SENSOR_IOX_HUM_EN);

        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, iox_data) < 0)
            return -ENODEV;
    }
    break;

    case BMI_SENSOR_DCOM_RST:
    {
        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, &iox_data) < 0)
            return -ENODEV;

        if(arg == BMI_SENSOR_ON)
            iox_data |= (0x1 << SENSOR_IOX_COMP_RS_N);
        else
            iox_data &= ~(0x1 << SENSOR_IOX_COMP_RS_N);

        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, iox_data) < 0)
            return -ENODEV;

    }
    break;

    case BMI_SENSOR_COM_GCAL:
    {
        struct sensor_comp_cal *cal = NULL;
        unsigned char ee_datam;
        unsigned char ee_datal;

        if ((cal = kmalloc(sizeof(struct sensor_comp_cal), GFP_KERNEL)) == NULL)
            return -ENOMEM;

        if(ReadByte_EE(sensor->mee_i2c_client, 0x0, &ee_datam) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        if(ReadByte_EE(sensor->mee_i2c_client, 0x1, &ee_datal) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        cal->xsf = (ee_datam << 8) | ee_datal;

        if(ReadByte_EE(sensor->mee_i2c_client, 0x2, &ee_datam) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        if(ReadByte_EE(sensor->mee_i2c_client, 0x3, &ee_datal) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        cal->ysf = (ee_datam << 8) | ee_datal;

        if(ReadByte_EE(sensor->mee_i2c_client, 0x4, &ee_datam) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        if(ReadByte_EE(sensor->mee_i2c_client, 0x5, &ee_datal) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        cal->zsf = (ee_datam << 8) | ee_datal;

        if(ReadByte_EE(sensor->mee_i2c_client, 0x6, &ee_datam) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        if(ReadByte_EE(sensor->mee_i2c_client, 0x7, &ee_datal) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        cal->xoff = (ee_datam << 8) | ee_datal;

        if(ReadByte_EE(sensor->mee_i2c_client, 0x8, &ee_datam) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        if(ReadByte_EE(sensor->mee_i2c_client, 0x9, &ee_datal) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        cal->yoff = (ee_datam << 8) | ee_datal;

        if(ReadByte_EE(sensor->mee_i2c_client, 0xA, &ee_datam) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        if(ReadByte_EE(sensor->mee_i2c_client, 0xB, &ee_datal) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        cal->zoff = (ee_datam << 8) | ee_datal;

        if(copy_to_user((struct sensor_comp_cal *) arg, cal, sizeof(struct sensor_comp_cal))) {
            kfree(cal);
            return -EFAULT;
        }

        kfree(cal);
    }
    break;

    case BMI_SENSOR_COM_SCAL:
    {
        struct sensor_comp_cal *cal = NULL;
        unsigned char ee_datam;
        unsigned char ee_datal;

        if ((cal = kmalloc(sizeof(struct sensor_comp_cal), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(cal, (struct sensor_comp_cal *) arg, sizeof(struct sensor_comp_cal))) {
            kfree(cal);
            return -EFAULT;
        }

        sensor->comp_xsf = cal->xsf;
        sensor->comp_ysf = cal->ysf;
        sensor->comp_zsf = cal->zsf;
        sensor->comp_xoff = cal->xoff;
        sensor->comp_xoff = cal->xoff;
        sensor->comp_zoff = cal->zoff;

        ee_datam = (unsigned char) ((cal->xsf >> 8) & 0xff);
        ee_datal = (unsigned char) (cal->xsf & 0xff);

        if(WriteByte_EE(sensor->mee_i2c_client, 0x0, ee_datam) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        if(WriteByte_EE(sensor->mee_i2c_client, 0x1, ee_datal) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        ee_datam = (unsigned char) ((cal->ysf >> 8) & 0xff);
        ee_datal = (unsigned char) (cal->ysf & 0xff);

        if(WriteByte_EE(sensor->mee_i2c_client, 0x2, ee_datam) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        if(WriteByte_EE(sensor->mee_i2c_client, 0x3, ee_datal) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        ee_datam = (unsigned char) ((cal->zsf >> 8) & 0xff);
        ee_datal = (unsigned char) (cal->zsf & 0xff);

        if(WriteByte_EE(sensor->mee_i2c_client, 0x4, ee_datam) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        if(WriteByte_EE(sensor->mee_i2c_client, 0x5, ee_datal) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        ee_datam = (unsigned char) ((cal->xoff >> 8) & 0xff);
        ee_datal = (unsigned char) (cal->xoff & 0xff);

        if(WriteByte_EE(sensor->mee_i2c_client, 0x6, ee_datam) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        if(WriteByte_EE(sensor->mee_i2c_client, 0x7, ee_datal) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        ee_datam = (unsigned char) ((cal->yoff >> 8) & 0xff);
        ee_datal = (unsigned char) (cal->yoff & 0xff);

        if(WriteByte_EE(sensor->mee_i2c_client, 0x8, ee_datam) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        if(WriteByte_EE(sensor->mee_i2c_client, 0x9, ee_datal) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        ee_datam = (unsigned char) ((cal->zoff >> 8) & 0xff);
        ee_datal = (unsigned char) (cal->zoff & 0xff);

        if(WriteByte_EE(sensor->mee_i2c_client, 0xA, ee_datam) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        if(WriteByte_EE(sensor->mee_i2c_client, 0xB, ee_datal) < 0) {
            kfree(cal);
            return -ENODEV;
        }

        kfree(cal);
    }
    break;

    case BMI_SENSOR_DCWR:
    {
        struct sensor_rw *dc = NULL;
        unsigned char dc_addr;
        unsigned char dc_data;

        if(sensor->eeprom.dcompass_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if ((dc = kmalloc(sizeof(struct sensor_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(dc, (struct sensor_rw *) arg, sizeof(struct sensor_rw))) {
            kfree(dc);
            return -EFAULT;
        }

        dc_addr = dc->address;
        dc_data = dc->data;
        if(WriteByte_DCOMP(sensor->dcomp_i2c_client, dc_addr, dc_data) < 0) {
            kfree(dc);
            return -ENODEV;
        }

        kfree(dc);
    }
    break;

    case BMI_SENSOR_DCRD:
    {
        struct sensor_rw *dc = NULL;
        unsigned char dc_addr;
        unsigned char dc_data;

        if(sensor->eeprom.dcompass_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if ((dc = kmalloc(sizeof(struct sensor_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(dc, (struct sensor_rw *) arg, sizeof(struct sensor_rw))) {
            kfree(dc);
            return -EFAULT;
        }

        dc_addr = dc->address;
        if(ReadByte_DCOMP(sensor->dcomp_i2c_client, dc_addr, &dc_data) < 0) {
            kfree(dc);
            return -ENODEV;
        }

        dc->data = dc_data;
        if(copy_to_user((struct sensor_rw *) arg, dc, sizeof(struct sensor_rw))) {
            kfree(dc);
            return -EFAULT;
        }

        kfree(dc);
    }
    break;

    case BMI_SENSOR_DC_GDAC:
    {
        struct sensor_comp_dac *dac = NULL;

        if(sensor->eeprom.dcompass_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if ((dac = kmalloc(sizeof(struct sensor_comp_dac), GFP_KERNEL)) == NULL)
            return -ENOMEM;

        dac->xdac = sensor->eeprom.xdac;
        dac->ydac = sensor->eeprom.ydac;
        dac->zdac = sensor->eeprom.zdac;

        if(copy_to_user((struct sensor_comp_dac *) arg, dac, sizeof(struct sensor_comp_dac))) {
            kfree(dac);
            return -EFAULT;
        }

        kfree(dac);
    }
    break;

    case BMI_SENSOR_DC_SDAC:
    {
        struct sensor_comp_dac *dac = NULL;

        if(sensor->eeprom.dcompass_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if ((dac = kmalloc(sizeof(struct sensor_comp_dac), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(dac, (struct sensor_comp_dac *) arg, sizeof(struct sensor_comp_dac))) {
            kfree(dac);
            return -EFAULT;
        }

        sensor->eeprom.xdac = dac->xdac;
        sensor->eeprom.ydac = dac->ydac;
        sensor->eeprom.zdac = dac->zdac;

        if(WriteByte_EE(sensor->mee_i2c_client, SENSOR_EE_XDAC, dac->xdac & 0xFF) < 0) {
            kfree(dac);
            return -ENODEV;
        }
        mdelay(5);

        if(WriteByte_EE(sensor->mee_i2c_client, SENSOR_EE_YDAC, dac->ydac & 0xFF) < 0) {
            kfree(dac);
            return -ENODEV;
        }
        mdelay(5);

        if(WriteByte_EE(sensor->mee_i2c_client, SENSOR_EE_ZDAC, dac->zdac & 0xFF) < 0) {
            kfree(dac);
            return -ENODEV;
        }
        mdelay(5);

        kfree(dac);
    }
    break;

    case BMI_SENSOR_DC_IWAIT:
    {
        if(sensor->eeprom.dcompass_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        ret = down_interruptible(&sensor->sem);
        sensor->dcomp_int_en = 1;
        sensor->dcomp_int_fl = 0;
        up(&sensor->sem);
        ret = wait_event_interruptible(sensor->dcomp_wait_queue, (sensor->dcomp_int_fl == 1));
        if(ret)
            return ret;
    }
    break;

    case BMI_SENSOR_APROX_DUR:
    {
        if(sensor->eeprom.aproximity_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if(arg < 2)
            sensor->aprox_duration = HZ/50;
        else if(arg > 100)
            sensor->aprox_duration = HZ/10;
        else
            sensor->aprox_duration = (HZ/100) * arg;
    }
    break;

    case BMI_SENSOR_APROXRD:
    {
        unsigned char aprox_data[2];
        int read_data;

        if(sensor->eeprom.aproximity_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        // start burst to LED
        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0)
            return -ENODEV;
        iox_data |= (0x1 << SENSOR_IOX_PROX_RST_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
            return -ENODEV;

        // set up timer
        sensor->aprox_timer.expires = jiffies + sensor->aprox_duration;
        add_timer (&sensor->aprox_timer);

        // wait for timer
        ret = down_interruptible(&sensor->sem);
        sensor->aprox_int_en = 1;
        sensor->aprox_int_fl = 0;
        up(&sensor->sem);
        ret = wait_event_interruptible(sensor->aprox_wait_queue, (sensor->aprox_int_fl == 1));
        if(ret)
            return ret;

        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0)
            return -ENODEV;

        // digital output
        read_data = (iox_data & (0x1 << SENSOR_IOX_PROX_OUT)) << 14;

        // read ADC - analog output
        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_APROXIMITY | SENSOR_ADC_PD_OFF) < 0)
            return -ENODEV;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, aprox_data) < 0)
            return -ENODEV;
        read_data |= (aprox_data[0] << 8) | aprox_data[1];

        // stop burst to LED
        iox_data &= ~(0x1 << SENSOR_IOX_PROX_RST_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
            return -ENODEV;

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_ALIGHTRD:
    {
        unsigned char adc_data[2];
        int read_data;

        if(sensor->eeprom.alight_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_LIGHT | SENSOR_ADC_PD_OFF) < 0)
            return -ENODEV;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0)
            return -ENODEV;
        read_data = ((adc_data[0] & SENSOR_ADC_DATA_MSB) << 8) | adc_data[1];

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_DLIGHTWR:
    {
        struct sensor_dl_rw *dl = NULL;
        unsigned char dl_data;

        if(sensor->eeprom.dlight_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if ((dl = kmalloc(sizeof(struct sensor_dl_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(dl, (struct sensor_dl_rw *) arg, sizeof(struct sensor_dl_rw))) {
            kfree(dl);
            return -EFAULT;
        }

        dl_data = dl->cmd;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_CMD, dl_data) < 0) {
            kfree(dl);
            return -ENODEV;
        }

        dl_data = dl->control;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_CONT, dl_data) < 0) {
            kfree(dl);
            return -ENODEV;
        }

        dl_data = dl->int_thi;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_INT_THI, dl_data) < 0) {
            kfree(dl);
            return -ENODEV;
        }

        dl_data = dl->int_tlo;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_INT_TLO, dl_data) < 0) {
            kfree(dl);
            return -ENODEV;
        }

        kfree(dl);
    }
    break;

    case BMI_SENSOR_DLIGHTRD:
    {
        unsigned char dl_data;
        unsigned int read_data;

        if(sensor->eeprom.dlight_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        // read sensor data
        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_SENSOR_MSB, &dl_data) < 0) {
            return -ENODEV;
        }
        read_data = ((unsigned int) (dl_data)) << 8;

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_SENSOR_LSB, &dl_data) < 0) {
            return -ENODEV;
        }
        read_data |= dl_data;

        if(put_user(read_data, (int __user *) arg))
            return -EFAULT;
    }
    break;

    case BMI_SENSOR_DLIGHT_IC:
    {
        if(sensor->eeprom.dlight_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if(WriteByte_DL_IC(sensor->pl_i2c_client) < 0)
            return -ENODEV;
    }
    break;

    case BMI_SENSOR_DLIGHT_IWAIT:
    {
        struct sensor_dl_rw *dl = NULL;
        unsigned char dl_data;
        unsigned int read_data;

        if(sensor->eeprom.dlight_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        // write all register
        if ((dl = kmalloc(sizeof(struct sensor_dl_rw), GFP_KERNEL)) == NULL)
            return -ENOMEM;
        if (copy_from_user(dl, (struct sensor_dl_rw *) arg, sizeof(struct sensor_dl_rw))) {
            kfree(dl);
            return -EFAULT;
        }

        dl_data = dl->cmd;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_CMD, dl_data) < 0) {
            kfree(dl);
            return -ENODEV;
        }

        dl_data = dl->control;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_CONT, dl_data) < 0) {
            kfree(dl);
            return -ENODEV;
        }

        dl_data = dl->int_thi;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_INT_THI, dl_data) < 0) {
            kfree(dl);
            return -ENODEV;
        }

        dl_data = dl->int_tlo;
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_INT_TLO, dl_data) < 0) {
            kfree(dl);
            return -ENODEV;
        }

        // enable interrupt
        ret = down_interruptible(&sensor->sem);
        sensor->dlight_int_en = 1;
        sensor->dlight_int_fl = 0;
        up(&sensor->sem);
        // wait on interrupt
        ret = wait_event_interruptible(sensor->dlight_wait_queue, (sensor->dlight_int_fl == 1));
        if(ret)
            return ret;

        // read sensor data
        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_SENSOR_MSB, &dl_data) < 0) {
            return -ENODEV;
        }
        read_data = ((unsigned int) (dl_data)) << 8;

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_SENSOR_LSB, &dl_data) < 0) {
            return -ENODEV;
        }
        read_data |= dl_data;
        dl->sensor_data = read_data;

        if(copy_to_user((struct sensor_dl_rw *) arg, dl, sizeof(struct sensor_dl_rw))) {
            kfree(dl);
            return -EFAULT;
        }

        kfree(dl);
    }
    break;

    case BMI_SENSOR_MIC_EN:
    {
        if(sensor->eeprom.sound_present != SENSOR_DEVICE_PRESENT)
            return -ENODEV;

        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, &iox_data) < 0)
            return -ENODEV;

        if(arg == BMI_SENSOR_ON)
            iox_data |= (0x1 << SENSOR_IOX_MIC_EN);
        else
            iox_data &= ~(0x1 << SENSOR_IOX_MIC_EN);

        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
            return -ENODEV;
    }
    break;

    default:
        return -ENOTTY;
    }

    return 0;
}

// control file operations
struct file_operations cntl_fops = {
    .owner = THIS_MODULE,
    .ioctl = cntl_ioctl,
    .open = cntl_open,
    .release = cntl_release,
};

/*
 *      PIM functions
 */

// interrupt handler
static void sensor_work_handler(struct work_struct * work)
{
    struct bmi_sensor *sensor = work_to_sensor(work);
    int slot = sensor->bdev->slot->slotnum;
    unsigned char iox0;
    unsigned char iox1;
    unsigned char i2c_dummy;
    struct sensor_acc_rw acc_rw;

    if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT0_REG, &iox0) < 0) {
        printk(KERN_ERR "bmi_sensor.c: sensor_work_handler - IOX0 error\n");
        return;
    }

    if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox1) < 0) {
        printk(KERN_ERR "bmi_sensor.c: sensor_work_handler - IOX1 error\n");
        return;
    }

    if(sensor->eeprom.light_proximity_present == SENSOR_DEVICE_PRESENT) {
        if(sensor->pl_int_en) {
            if((iox1 & (0x1 << SENSOR_IOX_PL_INT)) == 0) {
                sensor->pl_int_en = 0;
                sensor->pl_int_fl = 1;
                // clear interrupts
                if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, &i2c_dummy) < 0) {
                    printk(KERN_ERR "bmi_sensor.c: PL read error\n");
                }
                wake_up_all(&sensor->pl_wait_queue);
            }
        }
    }

    if(sensor->eeprom.temperature_present == SENSOR_DEVICE_PRESENT) {
        if(sensor->temp_int_en) {
            if((iox1 & (0x1 << SENSOR_IOX_TEMP_INT)) == 0) {
                sensor->temp_int_en = 0;
                sensor->temp_int_fl = 1;
                // disable interrupts
                if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_CONF1_WR, SENSOR_TEMP_CONF1_STOP) < 0) {
                    printk(KERN_ERR "bmi_sensor.c: TEMP write error\n");
                }
                wake_up_all(&sensor->temp_wait_queue);
            }
        }
    }

    if(sensor->eeprom.motion_present == SENSOR_DEVICE_PRESENT) {
        if(sensor->mot_int_en) {
            if(sensor->mot_state != get_mot_det_state(slot)) {
                sensor->mot_state = get_mot_det_state(slot);
                sensor->mot_int_en = 0;
                sensor->mot_int_fl = 1;
                wake_up_all(&sensor->mot_wait_queue);
            }
        }
    }

    if((sensor->eeprom.acc_present == SENSOR_DEVICE_PRESENT)
       || (sensor->eeprom.acc302_present == SENSOR_DEVICE_PRESENT)) {
        if(sensor->acc_int1_en) {
            if((iox0 & (0x1 << SENSOR_IOX_ACC_INT1)) == 0) {
                sensor->acc_int1_en = 0;
                sensor->acc_int1_fl = 1;
                wake_up_all(&sensor->acc_wait1_queue);
            }
        }

        if(sensor->acc_int2_en) {
            if((iox0 & (0x1 << SENSOR_IOX_ACC_INT2)) == 0) {
                sensor->acc_int2_en = 0;
                sensor->acc_int2_fl = 1;
                wake_up_all(&sensor->acc_wait2_queue);
            }
        }

        if(sensor->eeprom.acc_present == SENSOR_DEVICE_PRESENT)  {
            // clear interrupts
            acc_rw.address = SENSOR_ACC_IS;
            acc_rw.count = 1;
            if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
                printk(KERN_ERR "bmi_sensor.c: ACC_IS read error\n");
            }

            acc_rw.address = SENSOR_ACC_DX0;
            acc_rw.count = 2;
            if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
                printk(KERN_ERR "bmi_sensor.c: ACC read (DX0) error\n");
            }

            acc_rw.address = SENSOR_ACC_DY0;
            acc_rw.count = 2;
            if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
                printk(KERN_ERR "bmi_sensor.c: ACC read (DY0) error\n");
            }

            acc_rw.address = SENSOR_ACC_DZ0;
            acc_rw.count = 2;
            if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
                printk(KERN_ERR "bmi_sensor.c: ACC read (DZ0) error\n");
            }
        } else { // LIS302DL
            // clear interrupts
            if(sensor->acc_int1_en) {
                if((iox0 & (0x1 << SENSOR_IOX_ACC_INT1)) == 0) {
                    acc_rw.address = SENSOR_A3_SRC1;
                    acc_rw.count = 1;
                    if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
                        printk(KERN_ERR "bmi_sensor.c: A3_SRC1 read error\n");
                    }
                }
            }

            if(sensor->acc_int2_en) {
                if((iox0 & (0x1 << SENSOR_IOX_ACC_INT2)) == 0) {
                    acc_rw.address = SENSOR_A3_SRC2;
                    acc_rw.count = 1;
                    if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
                        printk(KERN_ERR "bmi_sensor.c: A3_SRC2 read error\n");
                    }
                }
            }

            acc_rw.address = SENSOR_A3_STAT;
            acc_rw.count = 1;
            if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
                printk(KERN_ERR "bmi_sensor.c: A3_STAT read error\n");
            }

            acc_rw.address = SENSOR_A3_OUTX;
            acc_rw.count = 1;
            if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
                printk(KERN_ERR "bmi_sensor.c: A3 read (OUTX) error\n");
            }

            acc_rw.address = SENSOR_A3_OUTY;
            acc_rw.count = 1;
            if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
                printk(KERN_ERR "bmi_sensor.c: A3 read (OUTY) error\n");
            }

            acc_rw.address = SENSOR_A3_OUTZ;
            acc_rw.count = 1;
            if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
                printk(KERN_ERR "bmi_sensor.c: A3 read (OUTZ) error\n");
            }
        }
    }

    if(sensor->eeprom.dcompass_present == SENSOR_DEVICE_PRESENT) {
        if(sensor->dcomp_int_en) {
            if((iox1 & (0x1 << SENSOR_IOX_DCOMP_INT)) != 0) {
                sensor->dcomp_int_en = 0;
                sensor->dcomp_int_fl = 1;
                // clear interrupts
                if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_TMPS, &i2c_dummy) < 0) {
                    printk(KERN_ERR "bmi_sensor.c: TMPS error\n");
                }

                wake_up_all(&sensor->dcomp_wait_queue);
            }
        }
    }

    if(sensor->eeprom.dlight_present == SENSOR_DEVICE_PRESENT) {
        if(sensor->dlight_int_en) {
            if((iox1 & (0x1 << SENSOR_IOX_PL_INT)) == 0) {
                sensor->dlight_int_en = 0;
                sensor->dlight_int_fl = 1;
                // clear interrupts
                if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_CONT, &i2c_dummy) < 0) {
                    printk(KERN_ERR "bmi_sensor.c: DL read error\n");
                }
                i2c_dummy &= ~(SENSOR_DL_CONT_INT);
                if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_CONT, i2c_dummy) < 0) {
                    printk(KERN_ERR "bmi_sensor.c: DL write error\n");
                }
                if(WriteByte_DL_IC(sensor->pl_i2c_client) < 0) {
                    printk(KERN_ERR "bmi_sensor.c: DL interrupt clear error\n");
                }
                wake_up_all(&sensor->pl_wait_queue);
            }
        }
    }


    if((iox0 & (0x1 << SENSOR_IOX_USB_FL_N)) == 0) {
        sensor->usb_int_en = 0;
        sensor->usb_int_fl = 1;
        // disable USB power
        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT0_REG, &i2c_dummy) < 0) // clear IOX interrupts
            printk(KERN_ERR "bmi_sensor.c: USB IOX read error\n");
        wake_up_all(&sensor->usb_wait_queue);
    }

    return;
}

/*
 *      BMI functions
 */

/*-------------------------------------
 *
 *      bmi device sysfs attributes
 *
 *-------------------------------------
 */

static ssize_t show_humidity(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char adc_data[2];

    if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_HUMIDITY) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
    }

    mdelay(1);

    if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
    }

    return sprintf(buf, "0x%x\n", (adc_data[0] << 8) | adc_data[1]);
}
static DEVICE_ATTR(humidity, S_IRUGO, show_humidity, NULL);

static ssize_t show_acompass(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char adc_data[2];
    unsigned int compass_x;
    unsigned int compass_y;
    unsigned int compass_z;

    if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_X) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
    }

    mdelay(1);

    if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
    }
    compass_x = (adc_data[0] << 8) | adc_data[1];

    if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_Y) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
    }

    mdelay(1);

    if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
    }
    compass_y = (adc_data[0] << 8) | adc_data[1];

    if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_Z) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
    }

    mdelay(1);

    if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
    }
    compass_z = (adc_data[0] << 8) | adc_data[1];

    return sprintf(buf, "X=0x%x\nY=0x%x\nZ=0x%x\n",
                   compass_x, compass_y, compass_z);
}
static DEVICE_ATTR(acompass, S_IRUGO, show_acompass, NULL);

static ssize_t show_dcompass(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char compass_i;
    unsigned char compass_t;
    unsigned char compass_x;
    unsigned char compass_y;
    unsigned char compass_z;

    if(WriteByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_MS1, SENSOR_DCOMP_MS1_SENSOR) < 0) {
        printk(KERN_ERR "bmi_sensor.c: DCOMP MS1 write error\n");
    }

    mdelay(20);

    if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_ST, &compass_i) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
    }
    if((compass_i & SENSOR_DCOMP_ST_INT) == 0) {
        printk(KERN_ERR "bmi_sensor.c: DCOMP interrupt error\n");
    }

    if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_TMPS, &compass_t) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TMPS error\n");
    }

    if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_H1X, &compass_x) < 0) {
        printk(KERN_ERR "bmi_sensor.c: H1X error\n");
    }

    if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_H1Y, &compass_y) < 0) {
        printk(KERN_ERR "bmi_sensor.c: H1Y error\n");
    }

    if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_H1Z, &compass_z) < 0) {
        printk(KERN_ERR "bmi_sensor.c: H1Z error\n");
    }

    return sprintf(buf, "T=0x%x\nX=0x%x\nY=0x%x\nZ=0x%x\n",
                   compass_t, compass_x, compass_y, compass_z);
}
static DEVICE_ATTR(dcompass, S_IRUGO, show_dcompass, NULL);

static ssize_t show_als(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char pl_data[2];

    if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, SENSOR_PL_CMD1_ALS_1X) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL write error\n");
    }

    mdelay(20);

    if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[0]) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL read error\n");
    }

    if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_LSB, &pl_data[1]) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL read error\n");
    }

    return sprintf(buf, "0x%x\n", (pl_data[0] << 8) | pl_data[1]);
}
static DEVICE_ATTR(als, S_IRUGO, show_als, NULL);

static ssize_t show_ir(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char pl_data[2];

    if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, SENSOR_PL_CMD1_IR_1X) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL write error\n");
    }

    mdelay(20);

    if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[0]) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL read error\n");
    }

    if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_LSB, &pl_data[1]) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL read error\n");
    }

    return sprintf(buf, "0x%x\n", (pl_data[0] << 8) | pl_data[1]);
}
static DEVICE_ATTR(ir, S_IRUGO, show_ir, NULL);

static ssize_t show_proximity(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char pl_data[2];

    if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, SENSOR_PL_CMD1_PROX_1X) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL write error\n");
    }

    mdelay(20);

    if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[0]) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL read error\n");
    }

    if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_LSB, &pl_data[1]) < 0) {
        printk(KERN_ERR "bmi_sensor.c: PL read error\n");
    }

    return sprintf(buf, "0x%x\n", (pl_data[0] << 8) | pl_data[1]);
}
static DEVICE_ATTR(proximity, S_IRUGO, show_proximity, NULL);

static ssize_t show_snda(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char adc_data[2];

    if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_SOUND_AVG) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
    }

    mdelay(1);

    if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
    }

    return sprintf(buf, "0x%x\n", (adc_data[0] << 8) | adc_data[1]);
}
static DEVICE_ATTR(sound_avg, S_IRUGO, show_snda, NULL);

static ssize_t show_sndp(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char adc_data[2];

    if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_SOUND_PEAK) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
    }

    mdelay(1);

    if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
    }

    return sprintf(buf, "0x%x\n", (adc_data[0] << 8) | adc_data[1]);
}
static DEVICE_ATTR(sound_peak, S_IRUGO, show_sndp, NULL);

static ssize_t show_temp_l(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char temp_datam;
    unsigned char temp_datal;

    if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_CONF1_WR, SENSOR_TEMP_CONF1_STOP) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
    }

    if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_ONE_SHOT, 0x0) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
    }

    mdelay(400);

    if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_LOC_MSB, &temp_datam) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP read (LOCAL MSB) error\n");
    }

    if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_LOC_LSB, &temp_datal) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP read (LOCAL LSB) error\n");
    }

    return sprintf(buf, "0x%x\n", (temp_datam << 8) | temp_datal);
}
static DEVICE_ATTR(temp_local, S_IRUGO, show_temp_l, NULL);

static ssize_t show_temp_sr(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char temp_datam;
    unsigned char temp_datal;

    if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_CONF1_WR, SENSOR_TEMP_CONF1_STOP) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
    }

    if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_ONE_SHOT, 0x0) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
    }

    mdelay(400);

    if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_REM_MSB, &temp_datam) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP read (REM MSB) error\n");
    }

    if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_REM_LSB, &temp_datal) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP read (REM LSB) error\n");
    }

    return sprintf(buf, "0x%x\n", (temp_datam << 8) | temp_datal);
}
static DEVICE_ATTR(temp_sremote, S_IRUGO, show_temp_sr, NULL);

static ssize_t show_temp_ur(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char temp_datam;
    unsigned char temp_datal;

    if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_CONF1_WR, SENSOR_TEMP_CONF1_STOP) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
    }

    if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_ONE_SHOT, 0x0) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
    }

    mdelay(400);

    if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_UREM_MSB, &temp_datam) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP read (UREM MSB) error\n");
    }

    if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_UREM_LSB, &temp_datal) < 0) {
        printk(KERN_ERR "bmi_sensor.c: TEMP read (UREM LSB) error\n");
    }

    return sprintf(buf, "0x%x\n", (temp_datam << 8) | temp_datal);
}
static DEVICE_ATTR(temp_uremote, S_IRUGO, show_temp_ur, NULL);

static ssize_t show_motion(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    return sprintf(buf, "0x%x\n", get_mot_det_state(bdev->slot->slotnum));
}
static DEVICE_ATTR(motion, S_IRUGO, show_motion, NULL);

static ssize_t show_accel(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];

    struct sensor_acc_rw acc_rw;
    int x;
    int y;
    int z;

    if (sensor->eeprom.acc_present != SENSOR_DEVICE_PRESENT) {
        acc_rw.address = SENSOR_ACC_DX0;
        acc_rw.count = 2;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC read (DX0) error\n");
        }
        x = (acc_rw.data[0] << 8) | acc_rw.data[1];

        acc_rw.address = SENSOR_ACC_DY0;
        acc_rw.count = 2;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC read (DY0) error\n");
        }
        y = (acc_rw.data[0] << 8) | acc_rw.data[1];

        acc_rw.address = SENSOR_ACC_DZ0;
        acc_rw.count = 2;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC read (DZ0) error\n");
        }
        z = (acc_rw.data[0] << 8) | acc_rw.data[1];
    } else {
        acc_rw.address = SENSOR_A3_OUTX;
        acc_rw.count = 1;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC read (OUTX) error\n");
        }
        x = acc_rw.data[0];

        acc_rw.address = SENSOR_A3_OUTY;
        acc_rw.count = 1;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC read (OUTY) error\n");
        }
        y = acc_rw.data[0];

        acc_rw.address = SENSOR_A3_OUTZ;
        acc_rw.count = 1;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC read (OUTZ) error\n");
        }
        z = acc_rw.data[0];
    }
    return sprintf(buf, "X=0x%x\nY=0x%x\nZ=0x%x\n", x, y, z);
}
static DEVICE_ATTR(accel, S_IRUGO, show_accel, NULL);

static ssize_t show_aprox(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned int read_data;
    unsigned char iox_data;
    unsigned char aprox_data;
    int ret;

    // start burst to LED
    if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0)
        printk(KERN_ERR "bmi_sensor.c: IOX read error\n");
    iox_data |= (0x1 << SENSOR_IOX_PROX_RST_N);
    if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
        printk(KERN_ERR "bmi_sensor.c: IOX write error\n");

    // set up timer
    sensor->aprox_timer.expires = jiffies + sensor->aprox_duration;
    add_timer (&sensor->aprox_timer);

    // wait for timer
    ret = down_interruptible(&sensor->sem);
    sensor->aprox_int_en = 1;
    sensor->aprox_int_fl = 0;
    up(&sensor->sem);
    wait_event_interruptible(sensor->aprox_wait_queue, (sensor->aprox_int_fl == 1));

    // stop burst to LED
    if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0)
        printk(KERN_ERR "bmi_sensor.c: IOX read error\n");
    iox_data &= ~(0x1 << SENSOR_IOX_PROX_RST_N);
    if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
        printk(KERN_ERR "bmi_sensor.c: IOX write error\n");

    // digital output
    read_data = (iox_data & (0x1 << SENSOR_IOX_PROX_OUT)) << 14;

    // read ADC - analog output
    if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_APROXIMITY | SENSOR_ADC_PD_OFF) < 0)
        printk(KERN_ERR "bmi_sensor.c: IOX write error\n");

    mdelay(1);

    if(ReadByte_ADC(sensor->adc_i2c_client, &aprox_data) < 0)
        printk(KERN_ERR "bmi_sensor.c: IOX read error\n");
    read_data |= aprox_data;

    return sprintf(buf, "Analog Proxiimity = 0x%x\n", read_data);
}
static DEVICE_ATTR(aprox, S_IRUGO, show_aprox, NULL);

static ssize_t show_alight(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char adc_data[2];

    if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_LIGHT) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
    }

    mdelay(1);

    if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
        printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
    }

    return sprintf(buf, "0x%x\n", (adc_data[0] << 8) | adc_data[1]);
}
static DEVICE_ATTR(alight, S_IRUGO, show_alight, NULL);

static ssize_t show_dlight(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_device *bdev = dev_to_bmi_device(dev);
    struct bmi_sensor *sensor = &bmi_sensor[bdev->slot->slotnum];
    unsigned char dl_data[2];

    if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_SENSOR_MSB, &dl_data[0]) < 0) {
        printk(KERN_ERR "bmi_sensor.c: DL read error\n");
    }

    if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_SENSOR_LSB, &dl_data[1]) < 0) {
        printk(KERN_ERR "bmi_sensor.c: DL read error\n");
    }

    return sprintf(buf, "0x%x\n", (dl_data[0] << 8) | dl_data[1]);
}
static DEVICE_ATTR(dlight, S_IRUGO, show_dlight, NULL);


// read calibration/equipage EEPROM
int read_eeprom(struct i2c_client *client, struct sensor_eeprom_raw *eeprom)
{
    unsigned char ee_data;

    if(ReadByte_EE(client, 0x0, &ee_data) < 0)
        return -ENODEV;
    eeprom->xsf_msb = (__u8) ee_data;

    if(ReadByte_EE(client, 0x1, &ee_data) < 0)
        return -ENODEV;
    eeprom->xsf_lsb = (__u8) ee_data;

    if(ReadByte_EE(client, 0x2, &ee_data) < 0)
        return -ENODEV;
    eeprom->ysf_msb = (__u8) ee_data;

    if(ReadByte_EE(client, 0x3, &ee_data) < 0)
        return -ENODEV;
    eeprom->ysf_lsb = (__u8) ee_data;

    if(ReadByte_EE(client, 0x4, &ee_data) < 0)
        return -ENODEV;
    eeprom->zsf_msb = (__u8) ee_data;

    if(ReadByte_EE(client, 0x5, &ee_data) < 0)
        return -ENODEV;
    eeprom->zsf_lsb = (__u8) ee_data;

    if(ReadByte_EE(client, 0x6, &ee_data) < 0)
        return -ENODEV;
    eeprom->xoff_msb = (__u8) ee_data;

    if(ReadByte_EE(client, 0x7, &ee_data) < 0)
        return -ENODEV;
    eeprom->xoff_lsb = (__u8) ee_data;

    if(ReadByte_EE(client, 0x8, &ee_data) < 0)
        return -ENODEV;
    eeprom->yoff_msb = (__u8) ee_data;

    if(ReadByte_EE(client, 0x9, &ee_data) < 0)
        return -ENODEV;
    eeprom->zoff_lsb = (__u8) ee_data;

    if(ReadByte_EE(client, 0xA, &ee_data) < 0)
        return -ENODEV;
    eeprom->zoff_msb = (__u8) ee_data;

    if(ReadByte_EE(client, 0xB, &ee_data) < 0)
        return -ENODEV;
    eeprom->yoff_lsb = (__u8) ee_data;

    if(ReadByte_EE(client, 0xC, &ee_data) < 0)
        return -ENODEV;
    eeprom->xdac = (__u8) ee_data;

    if(ReadByte_EE(client, 0xD, &ee_data) < 0)
        return -ENODEV;
    eeprom->ydac = (__u8) ee_data;

    if(ReadByte_EE(client, 0xE, &ee_data) < 0)
        return -ENODEV;
    eeprom->zdac = (__u8) ee_data;

    if(ReadByte_EE(client, 0xF, &ee_data) < 0)
        return -ENODEV;
    eeprom->adc_present = (__u8) ee_data;

    if(ReadByte_EE(client, 0x10, &ee_data) < 0)
        return -ENODEV;
    eeprom->humidity_present = (__u8) ee_data;

    if(ReadByte_EE(client, 0x11, &ee_data) < 0)
        return -ENODEV;
    eeprom->acompass_present = (__u8) ee_data;

    if(ReadByte_EE(client, 0x12, &ee_data) < 0)
        return -ENODEV;
    eeprom->light_proximity_present = (__u8) ee_data;

    if(ReadByte_EE(client, 0x13, &ee_data) < 0)
        return -ENODEV;
    eeprom->sound_present = (__u8) ee_data;

    if(ReadByte_EE(client, 0x14, &ee_data) < 0)
        return -ENODEV;
    eeprom->temperature_present = (__u8) ee_data;

    if(ReadByte_EE(client, 0x15, &ee_data) < 0)
        return -ENODEV;
    eeprom->motion_present = (__u8) ee_data;

    if(ReadByte_EE(client, 0x16, &ee_data) < 0)
        return -ENODEV;
    eeprom->acc_present = (__u8) ee_data;

    if(ReadByte_EE(client, 0x17, &ee_data) < 0)
        return -ENODEV;
    eeprom->dcompass_present = (__u8) ee_data;

    if(ReadByte_EE(client, 0x18, &ee_data) < 0)
        return -ENODEV;
    eeprom->aproximity_present = (__u8) ee_data;

    if(ReadByte_EE(client, 0x19, &ee_data) < 0)
        return -ENODEV;
    eeprom->alight_present = (__u8) ee_data;

    if(ReadByte_EE(client, 0x1A, &ee_data) < 0)
        return -ENODEV;
    eeprom->dlight_present = (__u8) ee_data;

    if(ReadByte_EE(client, 0x1B, &ee_data) < 0)
        return -ENODEV;
    eeprom->acc302_present = (__u8) ee_data;

    return 0;
}

static void cleanup_i2c_devices(struct bmi_sensor *sensor)
{
    if (sensor->mee_i2c_client != NULL)
    {
        i2c_unregister_device(sensor->mee_i2c_client);
        sensor->mee_i2c_client = NULL;
    }

    if (sensor->iox_i2c_client != NULL)
    {
        i2c_unregister_device(sensor->iox_i2c_client);
        sensor->iox_i2c_client = NULL;
    }

    if (sensor->adc_i2c_client != NULL)
    {
        i2c_unregister_device(sensor->adc_i2c_client);
        sensor->adc_i2c_client = NULL;
    }

    if (sensor->pl_i2c_client != NULL)
    {
        i2c_unregister_device(sensor->pl_i2c_client);
        sensor->pl_i2c_client = NULL;
    }

    if (sensor->dlight_i2c_client != NULL)
    {
        i2c_unregister_device(sensor->dlight_i2c_client);
        sensor->dlight_i2c_client = NULL;
    }

    if (sensor->temp_i2c_client != NULL)
    {
        i2c_unregister_device(sensor->temp_i2c_client);
        sensor->temp_i2c_client = NULL;
    }

    if (sensor->acc_i2c_client != NULL)
    {
        i2c_unregister_device(sensor->acc_i2c_client);
        sensor->acc_i2c_client = NULL;
    }

    if (sensor->dcomp_i2c_client != NULL)
    {
        i2c_unregister_device(sensor->dcomp_i2c_client);
        sensor->dcomp_i2c_client = NULL;
    }
}

// probe - insert PIM
int bmi_sensor_probe(struct bmi_device *bdev)
{
    int err = 0;
    int slot = bdev->slot->slotnum;
    struct bmi_sensor *sensor = &bmi_sensor[slot];
    struct cdev *cdev;
    struct class *bmi_class;
    dev_t dev_id;
    int irq;
    unsigned char iox_data;

    sensor->bdev = 0;

    // Create 1 minor device
    cdev = &sensor->cdev;
    cdev_init(cdev, &cntl_fops);

    dev_id = MKDEV(major, slot);
    err = cdev_add(cdev, dev_id, 1);
    if(err) {
        return err;
    }

    // Create class device
    bmi_class = bmi_get_class();
    sensor->class_dev = device_create(bmi_class, NULL,
                                      MKDEV(major, slot), NULL,
                                      "bmi_sensor_cntl_m%i", slot+1);

    if(IS_ERR(sensor->class_dev)) {
        printk(KERN_ERR "Unable to create "
               "class_device for bmi_sensor_cntl_m%i; errno = %ld\n",
               slot+1, PTR_ERR(sensor->class_dev));
        goto error;
    }

    // bind driver and bmi_device
    sensor->bdev = bdev;

    sensor->mee_i2c_client = i2c_new_device(bdev->slot->adap, &mee_info);
    if (sensor->mee_i2c_client == NULL)
        printk(KERN_ERR "MEE NULL...\n");

    sensor->iox_i2c_client = i2c_new_device(bdev->slot->adap, &iox_info);
    if (sensor->iox_i2c_client == NULL)
        printk(KERN_ERR "IOX NULL...\n");

    sensor->adc_i2c_client = i2c_new_device(bdev->slot->adap, &adc_info);
    if (sensor->adc_i2c_client == NULL)
        printk(KERN_ERR "ADC NULL...\n");

    sensor->pl_i2c_client = i2c_new_device(bdev->slot->adap, &pl_info);
    if (sensor->pl_i2c_client == NULL)
        printk(KERN_ERR "PL NULL...\n");

    sensor->dlight_i2c_client = i2c_new_device(bdev->slot->adap, &dlight_info);
    if (sensor->dlight_i2c_client == NULL)
        printk(KERN_ERR "DLIGHT NULL...\n");

    sensor->temp_i2c_client = i2c_new_device(bdev->slot->adap, &temp_info);
    if (sensor->temp_i2c_client == NULL)
        printk(KERN_ERR "TEMP NULL...\n");

    sensor->acc_i2c_client = i2c_new_device(bdev->slot->adap, &acc_info);
    if (sensor->acc_i2c_client == NULL)
        printk(KERN_ERR "ACCELO NULL...\n");

    sensor->dcomp_i2c_client = i2c_new_device(bdev->slot->adap, &dcomp_info);
    if (sensor->dcomp_i2c_client == NULL)
        printk(KERN_ERR "DCOMP NULL...\n");

    bmi_device_set_drvdata(bdev, sensor);

    printk(KERN_INFO "bmi_sensor.c: probe slot %d\n", slot);

    // configure IOX
    if(factory_test || fcc_test) {
        // USB/HUM on, MOT_INT off, COMPASS RST High
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, 0x98) < 0) {
            printk(KERN_ERR "bmi_sensor.c: IOX error in slot %d setting IOX_OUTPUT0_REG to 0x98\n", slot);
            goto error;
        }

        // Speaker Peak Clear, all other outputs low
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, 0x08) < 0) {
            printk(KERN_ERR "bmi_sensor.c: IOX error in slot %d setting IOX_OUTPUT1_REG to 0x08\n", slot);
            goto error;
        }
    } else {
        // USB/HUM/MOT_INT off, COMPASS RST High
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT0_REG, 0x80) < 0) {
            printk(KERN_ERR "bmi_sensor.c: IOX error in slot %d setting IOX_OUTPUT0_REG to 0x80\n", slot);
            goto error;
        }

        // Speaker Peak Clear/analog proximity enable high, all other outputs low
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, 0x0A) < 0) {
            printk(KERN_ERR "bmi_sensor.c: IOX error in slot %d settng IOX_OUTPUT1_REG to 0x0A\n", slot);
            goto error;
        }
    }

    // IOX[5,2:0]=IN, IOX[7:6,4:3]=OUT
    if(WriteByte_IOX(sensor->iox_i2c_client, IOX_CONTROL0_REG, 0x27) < 0) {
        printk(KERN_ERR "bmi_sensor.c: IOX error in slot %d setting IOX_CONTROL0_REG to 0x27\n", slot);
        goto error;
    }

    // IOX[7,5:4,2]=IN, IOX[6,3,1:0]=OUT
    if(WriteByte_IOX(sensor->iox_i2c_client, IOX_CONTROL1_REG, 0xb4) < 0) {
        printk(KERN_ERR "bmi_sensor.c: IOX error in slot %d setting IOX_CONTROL1_REG to 0xb4\n", slot);
        goto error;
    }

    // Initialize GPIOs (turn LED's on)
    bmi_slot_gpio_direction_out(slot, SENSOR_GPIO_RED_LED, 1);
    bmi_slot_gpio_direction_out(slot, SENSOR_GPIO_GREEN_LED, 1);
    bmi_slot_gpio_direction_in(slot, SENSOR_GPIO_PDOUT);
    bmi_slot_gpio_direction_in(slot, SENSOR_GPIO_MOT_DET);

    mdelay(200);

    // turn LED's off
    bmi_slot_gpio_set_value(slot, SENSOR_GPIO_RED_LED, 0);
    bmi_slot_gpio_set_value(slot, SENSOR_GPIO_GREEN_LED, 0);

    // bmi_sensor initialization
    init_MUTEX(&sensor->sem);
    sprintf(sensor->work_name, "sensor_m%d", slot + 1);
    init_waitqueue_head(&sensor->pl_wait_queue);
    sensor->pl_int_en = 0;
    sensor->pl_int_fl = 0;
    init_waitqueue_head(&sensor->temp_wait_queue);
    sensor->temp_int_en = 0;
    sensor->temp_int_fl = 0;
    init_waitqueue_head(&sensor->mot_wait_queue);
    sensor->mot_int_en = 0;
    sensor->mot_int_fl = 0;
    sensor->mot_state = get_mot_det_state(slot);
    init_waitqueue_head(&sensor->acc_wait1_queue);
    sensor->acc_int1_en = 0;
    sensor->acc_int1_fl = 0;
    init_waitqueue_head(&sensor->acc_wait2_queue);
    sensor->acc_int2_en = 0;
    sensor->acc_int2_fl = 0;
    init_waitqueue_head(&sensor->usb_wait_queue);
    sensor->usb_int_en = 0;
    sensor->usb_int_fl = 0;
    init_waitqueue_head(&sensor->dcomp_wait_queue);
    sensor->dcomp_int_en = 0;
    sensor->dcomp_int_fl = 0;
    sensor->aprox_duration = 200;
    init_timer(&sensor->aprox_timer);
    sensor->aprox_timer.data = (unsigned long) &bmi_sensor[slot];
    sensor->aprox_timer.function = aptimer;
    init_waitqueue_head(&sensor->aprox_wait_queue);
    sensor->aprox_int_en = 0;
    sensor->aprox_int_fl = 0;

    sensor->workqueue = create_singlethread_workqueue(sensor->work_name);
    if (!sensor->workqueue) {
        printk(KERN_ERR "bmi_sensor.c: Can't create_singlethread_workqueue() in slot %d\n",
               slot);
        goto error;
    }
    INIT_WORK(&sensor->work_item, sensor_work_handler);

    // initialize EEPROM for presence
    if(factory_test && eeprom_init) {
        unsigned char addr = SENSOR_PRESENT_START;

        // presence
        while(addr <= SENSOR_PRESENT_END) {
            if(eeprom_init & 0x1) {
                WriteByte_EE(sensor->mee_i2c_client, addr++, SENSOR_DEVICE_PRESENT);
            } else {
                WriteByte_EE(sensor->mee_i2c_client, addr++, SENSOR_DEVICE_NOT_PRESENT);
            }
            eeprom_init = eeprom_init >> 1;
            mdelay(5);
        }
    }

    if(factory_test && xdac_init) {
        WriteByte_EE(sensor->mee_i2c_client, SENSOR_EE_XDAC, xdac_init & 0xFF);
        mdelay(5);
    }

    if(factory_test && ydac_init) {
        WriteByte_EE(sensor->mee_i2c_client, SENSOR_EE_YDAC, ydac_init & 0xFF);
        mdelay(5);
    }

    if(factory_test && zdac_init) {
        WriteByte_EE(sensor->mee_i2c_client, SENSOR_EE_ZDAC, zdac_init & 0xFF);
        mdelay(5);
    }

    // read EEPROM for calibration/presence
    if(read_eeprom(sensor->mee_i2c_client, &sensor->eeprom)) {
        printk(KERN_ERR "bmi_sensor.c: Can't read calibration EEPROM in slot %d\n",
               slot);
        goto error;
    }

    sensor->comp_xsf = (sensor->eeprom.xsf_msb << 8) | sensor->eeprom.xsf_lsb;
    sensor->comp_ysf = (sensor->eeprom.ysf_msb << 8) | sensor->eeprom.ysf_lsb;
    sensor->comp_zsf = (sensor->eeprom.zsf_msb << 8) | sensor->eeprom.zsf_lsb;
    sensor->comp_xoff = (sensor->eeprom.xoff_msb << 8) | sensor->eeprom.xoff_lsb;
    sensor->comp_yoff = (sensor->eeprom.yoff_msb << 8) | sensor->eeprom.yoff_lsb;
    sensor->comp_zoff = (sensor->eeprom.zoff_msb << 8) | sensor->eeprom.zoff_lsb;

    if(sensor->eeprom.adc_present == SENSOR_DEVICE_PRESENT) {
        unsigned char adc_data[2];

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_CH0) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
            goto error;
        }

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
            goto error;
        }

        if((adc_data[0] & 0xF0) != 0x0) {
            printk(KERN_ERR "bmi_sensor.c: ADC compare error\n");
            goto error;
        }

        if(factory_test == 1) {
            printk(KERN_INFO "bmi_sensor.c: ADC present in slot %d\n", slot);
        }
    }

    if(sensor->eeprom.humidity_present == SENSOR_DEVICE_PRESENT) {
        unsigned char adc_data[2];

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_HUMIDITY) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC write (Humidity) error\n");
            goto error;
        }

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read (Humidity) error\n");
            goto error;
        }

        printk(KERN_INFO "bmi_sensor.c: initial Humidity = %d\n",
               (adc_data[0] << 8) | adc_data[1]);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_humidity)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (humidity) failed.\n",
                    slot);
            goto sysfs_err1;
        }

        if(factory_test == 1) {
            printk(KERN_INFO "bmi_sensor.c: HUMIDITY Sensor present in slot %d\n", slot);
        }
    }

    if(sensor->eeprom.acompass_present == SENSOR_DEVICE_PRESENT) {
        unsigned char adc_data[2];
        unsigned int compass_x;
        unsigned int compass_y;
        unsigned int compass_z;

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_X) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
            goto sysfs_err1;

        }

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
            device_remove_file(&sensor->bdev->dev, &dev_attr_humidity);
            goto sysfs_err1;
        }
        compass_x = (adc_data[0] << 8) | adc_data[1];

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_Y) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
            goto sysfs_err1;
        }

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
            goto sysfs_err1;
        }
        compass_y = (adc_data[0] << 8) | adc_data[1];

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_ACOMPASS_Z) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC write error\n");
            goto sysfs_err1;
        }

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read error\n");
            goto sysfs_err1;
        }
        compass_z = (adc_data[0] << 8) | adc_data[1];

        printk(KERN_INFO "bmi_sensor.c: initial COMPASS (X,Y,Z) = %d,%d,%d\n",
               compass_x, compass_y, compass_z);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_acompass)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (acompass) failed.\n",
                    slot);
            goto sysfs_err1;
        }

        if(factory_test == 1) {
            printk(KERN_INFO "bmi_sensor.c: ACOMPASS Sensor present in slot %d\n", slot);
        }
    }

    if(sensor->eeprom.dcompass_present == SENSOR_DEVICE_PRESENT) {
        unsigned char hxga;
        unsigned char hyga;
        unsigned char hzga;
        unsigned char compass_i;
        unsigned char compass_t;
        unsigned char compass_x;
        unsigned char compass_y;
        unsigned char compass_z;

        if(WriteByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_MS1, SENSOR_DCOMP_MS1_EEPROM) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DCOMP_MS1 write error\n");
            goto sysfs_err2;
        }

        if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_EE_EHXGA, &hxga) < 0) {
            printk(KERN_ERR "bmi_sensor.c: EE_EHXGA read error\n");
            goto sysfs_err2;
        }

        if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_EE_EHYGA, &hyga) < 0) {
            printk(KERN_ERR "bmi_sensor.c: EE_EHYGA read error\n");
            goto sysfs_err2;
        }

        if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_EE_EHZGA, &hzga) < 0) {
            printk(KERN_ERR "bmi_sensor.c: EE_EHZGA read error\n");
            goto sysfs_err2;
        }

        if(WriteByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_MS1, SENSOR_DCOMP_MS1_PD) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DCOMP_MS1 write error\n");
            goto sysfs_err2;
        }

        if(WriteByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_HXGA, hxga) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DCOMP_HXGA write error\n");
            goto sysfs_err2;
        }

        if(WriteByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_HYGA, hyga) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DCOMP_HYGA write error\n");
            goto sysfs_err2;
        }

        if(WriteByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_HZGA, hzga) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DCOMP_HZGA write error\n");
            goto sysfs_err2;
        }

        if(WriteByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_HXDA, sensor->eeprom.xdac) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DCOMP_HXDA write error\n");
            goto sysfs_err2;

        }

        if(WriteByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_HYDA, sensor->eeprom.ydac) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DCOMP_HYDA write error\n");
            goto sysfs_err2;

        }

        if(WriteByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_HZDA, sensor->eeprom.zdac) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DCOMP_HZDA write error\n");
            goto sysfs_err2;

        }

        if(WriteByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_MS1, SENSOR_DCOMP_MS1_SENSOR) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DCOMP_MS1 write error\n");
            goto sysfs_err2;

        }

        mdelay(20);

        if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_ST, &compass_i) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DCOMP_ST read error\n");
            goto sysfs_err2;
        }

        if((compass_i & SENSOR_DCOMP_ST_INT) == 0) {
            printk(KERN_ERR "bmi_sensor.c: DCOMP interrupt error\n");
            goto sysfs_err2;
        }

        if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_TMPS, &compass_t) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DCOMP_TMPS error\n");
            goto sysfs_err2;
        }

        if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_H1X, &compass_x) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DCOMP_H1X error\n");
            goto sysfs_err2;
        }

        if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_H1Y, &compass_y) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DCOMP_H1Y error\n");
            goto sysfs_err2;
        }

        if(ReadByte_DCOMP(sensor->dcomp_i2c_client, SENSOR_DCOMP_H1Z, &compass_z) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DCOMP_H1Z error\n");
            goto sysfs_err2;
        }

        printk(KERN_INFO "bmi_sensor.c: initial COMPASS (T,X,Y,Z) = 0x%x,0x%x,0x%x,0x%x\n",
               compass_t, compass_x, compass_y, compass_z);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_dcompass)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (dcompass) failed.\n",
                    slot);
            goto sysfs_err2;
        }

        if(factory_test == 1) {
            printk(KERN_INFO "bmi_sensor.c: DCOMPASS Sensor present in slot %d\n", slot);
        }
    }

    if(sensor->eeprom.light_proximity_present == SENSOR_DEVICE_PRESENT) {
        unsigned char pl_data[2];

        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, SENSOR_PL_CMD1_ALS_1X) < 0) {
            printk(KERN_ERR "bmi_sensor.c: PL write (ALS) error\n");
            goto sysfs_err3;
        }

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[0]) < 0) {
            printk(KERN_ERR "bmi_sensor.c: PL read (ALS) error\n");
            goto sysfs_err3;
        }

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[1]) < 0) {
            printk(KERN_ERR "bmi_sensor.c: PL read (ALS) error\n");
            goto sysfs_err3;
        }
        printk(KERN_INFO "bmi_sensor.c: initial PL ALS = %d\n",
               (pl_data[0] << 8) | pl_data[1]);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_als)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (ALS) failed.\n",
                    slot);
            goto sysfs_err3;
        }

        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, SENSOR_PL_CMD1_IR_1X) < 0) {
            printk(KERN_ERR "bmi_sensor.c: PL write (IR) error\n");
            goto sysfs_err4;
        }

        mdelay(20);

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[0]) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read (IR) error\n");
            goto sysfs_err4;
        }

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[1]) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read (IR) error\n");
            goto sysfs_err4;
        }
        printk(KERN_INFO "bmi_sensor.c: initial IR = %d\n",
               (pl_data[0] << 8) | pl_data[1]);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_ir)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (IR) failed.\n",
                    slot);
            goto sysfs_err4;
        }

        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_PL_CMD1, SENSOR_PL_CMD1_PROX_1X) < 0) {
            printk(KERN_ERR "bmi_sensor.c: PL write (Proximity) error\n");
            goto sysfs_err5;
        }

        mdelay(20);

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[0]) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read (Proximity) error\n");
            goto sysfs_err5;
        }

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_PL_DATA_MSB, &pl_data[1]) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read (Proximity) error\n");
            goto sysfs_err5;
        }
        printk(KERN_INFO "bmi_sensor.c: initial Proximity = %d\n",
               (pl_data[0] << 8) | pl_data[1]);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_proximity)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (Proximity) failed.\n",
                    slot);
            goto sysfs_err5;
        }

        if(factory_test == 1) {
            printk(KERN_INFO "bmi_sensor.c: Combined LIGHT/PROXIMITY Sensor present in slot %d\n", slot);
        }
    }

    if(sensor->eeprom.sound_present == SENSOR_DEVICE_PRESENT) {
        unsigned char adc_data[2];

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_SOUND_AVG) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC write (Sound Average) error\n");
            goto sysfs_err6;
        }

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read (Sound Average) error\n");
            goto sysfs_err6;
        }

        printk(KERN_INFO "bmi_sensor.c: initial Sound Average = %d\n",
               (adc_data[0] << 8) | adc_data[1]);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_sound_avg)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (Sound Average) failed.\n",
                    slot);
            goto sysfs_err6;
        }

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_SOUND_PEAK) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC write (Sound Peak) error\n");
            goto sysfs_err7;
        }

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read (Sound Peak) error\n");
            goto sysfs_err7;
        }

        printk(KERN_INFO "bmi_sensor.c: initial Sound Peak = %d\n",
               (adc_data[0] << 8) | adc_data[1]);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_sound_peak)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (Sound Peak) failed.\n",
                    slot);
            goto sysfs_err7;
        }

        if(factory_test == 1) {
            printk(KERN_INFO "bmi_sensor.c: SOUND PRESSURE Sensor present in slot %d\n", slot);
        }
    }

    if(sensor->eeprom.temperature_present == SENSOR_DEVICE_PRESENT) {
        unsigned char temp_datam;
        unsigned char temp_datal;

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_MAN_ID, &temp_datam) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP read (Manufacturer ID) error\n");
            goto sysfs_err8;
        }

        if(temp_datam != SENSOR_TEMP_MAN_ID_DATA) {
            printk(KERN_ERR "bmi_sensor.c: TEMP MAN ID error (read=0x%x, expected=0x%x\n",
                   temp_datam, SENSOR_TEMP_MAN_ID_DATA);
            goto sysfs_err8;
        }

        printk(KERN_INFO "bmi_sensor.c: TEMP Manufacturer ID = 0x%x\n", temp_datam);

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_REV_ID, &temp_datam) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP read (Revision ID) error\n");
            goto sysfs_err8;
        }

        if(temp_datam != SENSOR_TEMP_REV_ID_DATA) {
            printk(KERN_ERR "bmi_sensor.c: TEMP REV ID error (read=0x%x, expected=0x%x\n",
                   temp_datam, SENSOR_TEMP_REV_ID_DATA);
            goto sysfs_err8;
        }

        printk(KERN_INFO "bmi_sensor.c: TEMP Revision ID = 0x%x\n", temp_datam);

        if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_CONF2, 0x0) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF2) error\n");
            goto sysfs_err8;
        }

        if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_CONF1_WR, SENSOR_TEMP_CONF1_STOP) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
            goto sysfs_err8;
        }

        if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_CONV_WR, SENSOR_TEMP_CONV_P364) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
            goto sysfs_err8;
        }

        if(WriteByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_ONE_SHOT, 0x0) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP write (CONF1) error\n");
            goto sysfs_err8;
        }

        mdelay(400);

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_LOC_MSB, &temp_datam) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP read (LOCAL MSB) error\n");
            goto sysfs_err8;
        }

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_LOC_LSB, &temp_datal) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP read (LOCAL LSB) error\n");
            goto sysfs_err8;
        }

        printk(KERN_INFO "bmi_sensor.c: initial Local temperature = 0x%x\n",
               (temp_datam << 8) | temp_datal);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_temp_local)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (TEMP local) failed.\n",
                    slot);
            goto sysfs_err8;
        }

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_REM_MSB, &temp_datam) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP read (REM MSB) error\n");
            goto sysfs_err9;
        }

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_REM_LSB, &temp_datal) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP read (REM LSB) error\n");
            goto sysfs_err9;
        }

        printk(KERN_INFO "bmi_sensor.c: initial Remote temperature = 0x%x\n",
               (temp_datam << 8) | temp_datal);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_temp_sremote)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (TEMP sremote) failed.\n",
                    slot);
            goto sysfs_err9;
        }

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_UREM_MSB, &temp_datam) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP read (UREM MSB) error\n");
            goto sysfs_err10;
        }

        if(ReadByte_TEMP(sensor->temp_i2c_client, SENSOR_TEMP_UREM_LSB, &temp_datal) < 0) {
            printk(KERN_ERR "bmi_sensor.c: TEMP read (UREM LSB) error\n");
            goto sysfs_err10;
        }

        if(device_create_file(&sensor->bdev->dev, &dev_attr_temp_uremote)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (TEMP uremote) failed.\n",
                    slot);
            goto sysfs_err10;
        }

        printk(KERN_INFO "bmi_sensor.c: initial Remote temperature (unsigned) = 0x%x\n",
               (temp_datam << 8) | temp_datal);

        if(factory_test == 1) {
            printk(KERN_INFO "bmi_sensor.c: TEMPERATURE Sensor present in slot %d\n", slot);
        }
    }

    if(sensor->eeprom.motion_present == SENSOR_DEVICE_PRESENT) {
        if(device_create_file(&sensor->bdev->dev, &dev_attr_motion)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (motion) failed.\n",
                    slot);
            goto sysfs_err11;
        }

        printk(KERN_INFO "bmi_sensor.c: initial Motion state = 0x%x\n",
               get_mot_det_state(slot));

        if(factory_test == 1) {
            printk(KERN_INFO "bmi_sensor.c: MOTION Sensor present in slot %d\n", slot);
        }
    }

    if(sensor->eeprom.acc_present == SENSOR_DEVICE_PRESENT) {
        struct sensor_acc_rw acc_rw;

        acc_rw.address = SENSOR_ACC_ID;
        acc_rw.count = 1;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC read (ID) error\n");
            goto sysfs_err12;
        }

        if(acc_rw.data[0] != SENSOR_ACC_ID_DATA) {
            printk(KERN_ERR "bmi_sensor.c: ACC ID error (read=0x%x, expected=0x%x)\n",
                   acc_rw.data[0], SENSOR_ACC_ID_DATA);
            goto sysfs_err12;
        }

        acc_rw.address = SENSOR_ACC_RATE;
        acc_rw.count = 1;
        acc_rw.data[0] = SENSOR_ACC_RC_3200_1600;
        if(WriteByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC write (RATE) error\n");
            goto sysfs_err12;
        }

        acc_rw.address = SENSOR_ACC_POWER;
        acc_rw.count = 1;
        acc_rw.data[0] = SENSOR_ACC_P_NORM;
        if(WriteByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC write (RATE) error\n");
            goto sysfs_err12;
        }

        acc_rw.address = SENSOR_ACC_DF;
        acc_rw.count = 1;
        acc_rw.data[0] = SENSOR_ACC_DF_LENGTH;
        if(WriteByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC write (DF) error\n");
            goto sysfs_err12;
        }

        mdelay(20);

        acc_rw.address = SENSOR_ACC_DX0;
        acc_rw.count = 2;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC read (DX0) error\n");
            goto sysfs_err12;
        }

        printk(KERN_INFO "bmi_sensor.c: initial ACC X state = 0x%x\n",
               (acc_rw.data[0] << 8) | acc_rw.data[1]);

        acc_rw.address = SENSOR_ACC_DY0;
        acc_rw.count = 2;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC read (DY0) error\n");
            goto sysfs_err12;
        }

        printk(KERN_INFO "bmi_sensor.c: initial ACC Y state = 0x%x\n",
               (acc_rw.data[0] << 8) | acc_rw.data[1]);

        acc_rw.address = SENSOR_ACC_DZ0;
        acc_rw.count = 2;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC read (DZ0) error\n");
            goto sysfs_err12;
        }

        printk(KERN_INFO "bmi_sensor.c: initial ACC Z state = 0x%x\n",
               (acc_rw.data[0] << 8) | acc_rw.data[1]);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_accel)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (accel) failed.\n",
                    slot);
            goto sysfs_err12;
        }

        if(factory_test == 1) {
            printk(KERN_INFO "bmi_sensor.c: ACCELEROMETER Sensor present in slot %d\n", slot);
        }
    }

    if(sensor->eeprom.acc302_present == SENSOR_DEVICE_PRESENT) {
        struct sensor_acc_rw acc_rw;

        acc_rw.address = SENSOR_A3_WAI;
        acc_rw.count = 1;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC302 read (WAI) error\n");
            goto sysfs_err12;
        }

        if(acc_rw.data[0] != SENSOR_A3_WAI_ID) {
            printk(KERN_ERR "bmi_sensor.c: ACC302 ID error (read=0x%x, expected=0x%x)\n",
                   acc_rw.data[0], SENSOR_A3_WAI_ID);
            goto sysfs_err12;
        }

        acc_rw.address = SENSOR_A3_CTRL1;
        acc_rw.count = 1;
        if(factory_test)
            acc_rw.data[0] = SENSOR_A3_CTRL1_DR400 | SENSOR_A3_CTRL1_PU |
                SENSOR_A3_CTRL1_STP | SENSOR_A3_CTRL1_STM | SENSOR_A3_CTRL1_XYZEN;
        else
            acc_rw.data[0] = SENSOR_A3_CTRL1_DR400 | SENSOR_A3_CTRL1_PU
                | SENSOR_A3_CTRL1_XYZEN;
        if(WriteByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC302 write (RATE) error\n");
            goto sysfs_err12;
        }

        acc_rw.address = SENSOR_A3_CTRL3;
        acc_rw.count = 1;
        acc_rw.data[0] = SENSOR_A3_CTRL3_IL;
        if(WriteByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC302 write (RATE) error\n");
            goto sysfs_err12;
        }

        mdelay(20);

        acc_rw.address = SENSOR_A3_OUTX;
        acc_rw.count = 1;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC302 read (OUTX) error\n");
            goto sysfs_err12;
        }

        printk(KERN_INFO "bmi_sensor.c: initial ACC302 X state = 0x%x\n",
               acc_rw.data[0]);

        acc_rw.address = SENSOR_A3_OUTY;
        acc_rw.count = 1;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC302 read (OUTY) error\n");
            goto sysfs_err12;
        }

        printk(KERN_INFO "bmi_sensor.c: initial ACC302 Y state = 0x%x\n",
               acc_rw.data[0]);

        acc_rw.address = SENSOR_A3_OUTZ;
        acc_rw.count = 1;
        if(ReadByte_ACC(sensor->acc_i2c_client, &acc_rw) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ACC302 read (OUTZ) error\n");
            goto sysfs_err12;
        }

        printk(KERN_INFO "bmi_sensor.c: initial ACC302 Z state = 0x%x\n",
               acc_rw.data[0]);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_accel)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (accel) failed.\n",
                    slot);
            goto sysfs_err12;
        }

        if(factory_test == 1) {
            printk(KERN_INFO "bmi_sensor.c: ISL302 ACCELEROMETER Sensor present in slot %d\n", slot);
        }
    }

    if(sensor->eeprom.aproximity_present == SENSOR_DEVICE_PRESENT) {
        unsigned char aprox_data;
        unsigned int read_data;
        int ret;

        // enable sensor
        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: IOX error in slot %d\n", slot);
            goto sysfs_err13;
        }
        iox_data &= ~(0x1 << SENSOR_IOX_PROX_EN_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: IOX error in slot %d\n", slot);
            goto sysfs_err13;
        }

        // start burst to LED
        iox_data |= (0x1 << SENSOR_IOX_PROX_RST_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
            goto sysfs_err13;

        // set up timer
        sensor->aprox_timer.expires = jiffies + sensor->aprox_duration;
        add_timer (&sensor->aprox_timer);

        // wait for timer
        ret = down_interruptible(&sensor->sem);
        sensor->aprox_int_en = 1;
        sensor->aprox_int_fl = 0;
        up(&sensor->sem);
        wait_event_interruptible(sensor->aprox_wait_queue, (sensor->aprox_int_fl == 1));

        // stop burst to LED
        if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0)
            goto sysfs_err13;
        iox_data &= ~(0x1 << SENSOR_IOX_PROX_RST_N);
        if(WriteByte_IOX(sensor->iox_i2c_client, IOX_OUTPUT1_REG, iox_data) < 0)
            goto sysfs_err13;

        // digital output
        read_data = (iox_data & (0x1 << SENSOR_IOX_PROX_OUT)) << 14;

        // read ADC - analog output
        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_APROXIMITY | SENSOR_ADC_PD_OFF) < 0)
            goto sysfs_err13;

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, &aprox_data) < 0)
            goto sysfs_err13;
        read_data |= aprox_data;

        printk(KERN_INFO "bmi_sensor.c: initial analog proximity = 0x%x\n", read_data);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_aprox)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (aprox) failed.\n",
                    slot);
            goto sysfs_err13;
        }

        if(factory_test == 1) {
            printk(KERN_INFO "bmi_sensor.c: Analog PROXIMITY Sensor present in slot %d\n", slot);
        }
    }

    if(sensor->eeprom.alight_present == SENSOR_DEVICE_PRESENT) {
        unsigned char adc_data[2];

        if(WriteByte_ADC(sensor->adc_i2c_client, SENSOR_ADC_LIGHT) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC write (Analog Light) error\n");
            goto sysfs_err14;
        }

        mdelay(1);

        if(ReadByte_ADC(sensor->adc_i2c_client, adc_data) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read (Humidity) error\n");
            goto sysfs_err14;
        }

        printk(KERN_INFO "bmi_sensor.c: initial Analog Light = 0x%x\n",
               (adc_data[0] << 8) | adc_data[1]);

        if(device_create_file(&sensor->bdev->dev, &dev_attr_alight)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (alight) failed.\n",
                    slot);
            goto sysfs_err14;
        }

        if(factory_test == 1) {
            printk(KERN_INFO "bmi_sensor.c: Analog LIGHT Sensor present in slot %d\n", slot);
        }
    }

    if(sensor->eeprom.dlight_present == SENSOR_DEVICE_PRESENT) {
        unsigned char dl_data[2];

        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_CMD, SENSOR_DL_CMD_ADC_EN) < 0) {
            printk(KERN_ERR "bmi_sensor.c: PL write (Digital Light) error\n");
            goto sysfs_err15;
        }

        mdelay(20);

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_SENSOR_MSB, &dl_data[0]) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read (Digital Light) error\n");
            goto sysfs_err15;
        }

        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_SENSOR_MSB, &dl_data[1]) < 0) {
            printk(KERN_ERR "bmi_sensor.c: ADC read (Digital Light) error\n");
            goto sysfs_err15;
        }
        printk(KERN_INFO "bmi_sensor.c: initial Digital Light = %d\n",
               (dl_data[0] << 8) | dl_data[1]);

        // clear interrupts
        if(ReadByte_PL(sensor->pl_i2c_client, SENSOR_DL_CONT, &dl_data[0]) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DL read error\n");
        }
        dl_data[0] &= ~(SENSOR_DL_CONT_INT);
        if(WriteByte_PL(sensor->pl_i2c_client, SENSOR_DL_CONT, dl_data[0]) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DL write error\n");
        }
        if(WriteByte_DL_IC(sensor->pl_i2c_client) < 0) {
            printk(KERN_ERR "bmi_sensor.c: DL interrupt clear error\n");
        }

        if(device_create_file(&sensor->bdev->dev, &dev_attr_dlight)) {
            printk (KERN_ERR
                    "bmi_sensor.c (%d): attr (Digital Light) failed.\n",
                    slot);
            goto sysfs_err15;
        }

        if(factory_test == 1) {
            printk(KERN_INFO "bmi_sensor.c: Digital LIGHT Sensor present in slot %d\n", slot);
        }
    }

    if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT0_REG, &iox_data) < 0) { // clear IOX interrupts
        printk (KERN_ERR
                "bmi_sensor.c(%d): IOX0 interrupt clear fail.\n",
                slot);
        goto sysfs_err16;
    }

    if(ReadByte_IOX(sensor->iox_i2c_client, IOX_INPUT1_REG, &iox_data) < 0) { // clear IOX interrupts
        printk (KERN_ERR
                "bmi_sensor.c(%d): IOX1 interrupt clear fail.\n",
                slot);
        goto sysfs_err16;
    }

    // request PIM interrupt
    irq = bdev->slot->status_irq;
    sprintf(sensor->int_name, "bmi_sensor%d", slot);
    //pjg if(request_irq(irq, &module_irq_handler, 0, sensor->int_name, sensor)) {
    //pjg printk(KERN_ERR "bmi_sensor.c: Can't allocate irq %d or find Sensor in slot %d\n",
    //pjg irq, slot);
    //pjg goto sysfs_err16;
    //pjg }

    return 0;

sysfs_err16:
    if(sensor->eeprom.dlight_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_dlight);
    }
sysfs_err15:
    if(sensor->eeprom.alight_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_alight);
    }
sysfs_err14:
    if(sensor->eeprom.aproximity_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_aprox);
    }
sysfs_err13:
    if(sensor->eeprom.acc_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_accel);
    }
sysfs_err12:
    if(sensor->eeprom.motion_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_motion);
    }
sysfs_err11:
    if(sensor->eeprom.temperature_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_temp_uremote);
    }
sysfs_err10:
    if(sensor->eeprom.temperature_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_temp_sremote);
    }
sysfs_err9:
    if(sensor->eeprom.temperature_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_temp_local);
    }
sysfs_err8:
    if(sensor->eeprom.sound_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_sound_peak);
    }
sysfs_err7:
    if(sensor->eeprom.sound_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_sound_avg);
    }
sysfs_err6:
    if(sensor->eeprom.light_proximity_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_proximity);
    }
sysfs_err5:
    if(sensor->eeprom.light_proximity_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_ir);
    }
sysfs_err4:
    if(sensor->eeprom.light_proximity_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_als);
    }
sysfs_err3:
    if(sensor->eeprom.dcompass_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_dcompass);
    }
sysfs_err2:
    if(sensor->eeprom.acompass_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_acompass);
    }
sysfs_err1:
    if(sensor->eeprom.humidity_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_humidity);
    }
error:
    cleanup_i2c_devices(sensor);

    sensor->class_dev = NULL;
    cdev_del(&sensor->cdev);
    device_destroy(bmi_class, MKDEV(major, slot));
    bmi_device_set_drvdata(bdev, 0);
    sensor->bdev = 0;
    printk(KERN_ERR "bmi_sensor.c: probe slot %d FAILED\n", slot);
    return -ENODEV;

}

// remove PIM
void bmi_sensor_remove(struct bmi_device *bdev)
{
    int slot;
    struct bmi_sensor *sensor;
    struct class *bmi_class;
    int irq;

    slot = bdev->slot->slotnum;
    sensor = &bmi_sensor[slot];

    irq = bdev->slot->status_irq;
    free_irq(irq, sensor);

    destroy_workqueue(sensor->workqueue);

    if(sensor->eeprom.humidity_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_humidity);
    }
    if(sensor->eeprom.acompass_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_acompass);
    }
    if(sensor->eeprom.dcompass_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_dcompass);
    }
    if(sensor->eeprom.light_proximity_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_als);
        device_remove_file(&sensor->bdev->dev, &dev_attr_ir);
        device_remove_file(&sensor->bdev->dev, &dev_attr_proximity);
    }
    if(sensor->eeprom.sound_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_sound_avg);
        device_remove_file(&sensor->bdev->dev, &dev_attr_sound_peak);
    }
    if(sensor->eeprom.temperature_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_temp_local);
        device_remove_file(&sensor->bdev->dev, &dev_attr_temp_sremote);
        device_remove_file(&sensor->bdev->dev, &dev_attr_temp_uremote);
    }
    if(sensor->eeprom.motion_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_motion);
    }
    if(sensor->eeprom.acc302_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_accel);
    }
    if(sensor->eeprom.acc_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_accel);
    }
    if(sensor->eeprom.aproximity_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_aprox);
        del_timer(&sensor->aprox_timer);
    }
    if(sensor->eeprom.alight_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_alight);
    }
    if(sensor->eeprom.dlight_present == SENSOR_DEVICE_PRESENT) {
        device_remove_file(&sensor->bdev->dev, &dev_attr_dlight);
    }

    cleanup_i2c_devices(sensor);

    bmi_slot_gpio_direction_in(slot, SENSOR_GPIO_RED_LED);
    bmi_slot_gpio_direction_in(slot, SENSOR_GPIO_GREEN_LED);

    bmi_class = bmi_get_class();
    device_destroy(bmi_class, MKDEV(major, slot));

    sensor->class_dev = 0;

    cdev_del(&sensor->cdev);

    // de-attach driver-specific struct from bmi_device structure
    bmi_device_set_drvdata(bdev, 0);
    sensor->bdev = 0;

    return;
}

/*
 *      module routines
 */

static int __init bmi_sensor_init(void)
{
    dev_t   dev_id;
    int     retval;

    printk(KERN_ERR "bmi_sensor.c: init");
    // alloc char driver with 4 minor numbers
    retval = alloc_chrdev_region(&dev_id, 0, 4, "BMI SENSOR Driver");
    if(retval) {
        return -ENODEV;
    }

    major = MAJOR(dev_id);
    retval = bmi_register_driver(&bmi_sensor_driver);
    if(retval) {
        unregister_chrdev_region(dev_id, 4);
        return -ENODEV;
    }

    if(factory_test) {
        printk(KERN_INFO "bmi_sensor.c: Factory Test mode enabled\n");
        if(eeprom_init)
            printk(KERN_INFO "bmi_sensor.c: eeprom init = 0x%x\n", eeprom_init);
        if(xdac_init)
            printk(KERN_INFO "bmi_sensor.c: XDAC init = 0x%x\n", xdac_init);
        if(ydac_init)
            printk(KERN_INFO "bmi_sensor.c: YDAC init = 0x%x\n", ydac_init);
        if(zdac_init)
            printk(KERN_INFO "bmi_sensor.c: ZDAC init = 0x%x\n", zdac_init);
    }

    if(fcc_test)
        printk(KERN_INFO "bmi_sensor.c: FCC Test mode enabled\n");

    printk(KERN_INFO "bmi_sensor.c: BMI_SENSOR Driver v%s \n", BMISENSOR_VERSION);

    return 0;
}

static void __exit bmi_sensor_cleanup(void)
{
    dev_t dev_id;

    bmi_unregister_driver(&bmi_sensor_driver);

    dev_id = MKDEV(major, 0);
    unregister_chrdev_region(dev_id, 4);
    return;
}

module_init(bmi_sensor_init);
module_exit(bmi_sensor_cleanup);

module_param(factory_test, ushort, S_IRUGO);
MODULE_PARM_DESC(factory_test, "Factory Test code enable");

module_param(eeprom_init, int, S_IRUGO);
MODULE_PARM_DESC(eeprom_init, "Factory presence EEPROM programming");

module_param(xdac_init, ushort, S_IRUGO);
MODULE_PARM_DESC(xdac_init, "Factory EEPROM XDAC programming");

module_param(ydac_init, ushort, S_IRUGO);
MODULE_PARM_DESC(ydac_init, "Factory EEPROM YDAC programming");

module_param(zdac_init, ushort, S_IRUGO);
MODULE_PARM_DESC(zdac_init, "Factory EEPROM ZDAC programming");

module_param(fcc_test, ushort, S_IRUGO);
MODULE_PARM_DESC(fcc_test, "FCC Test code enable");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter Giacomini <p.giacomini@encadis.com>");
MODULE_DESCRIPTION("BMI Sensor device driver");
MODULE_SUPPORTED_DEVICE("bmi_sensor_cntl_mX");

