#include "acc.h"
#include <asm/uaccess.h>
#include <linux/bmi.h>
#include <linux/device.h>

#define BMI_SLOT_NUM 4

static dev_t acc_dev_number;

static struct file_operations acc_fops = {
  .owner = THIS_MODULE,
  .open = acc_open,
  .read = acc_read,
  .release = acc_release
};


int acc_open(struct inode *inode, struct file *file)
{  
  struct acc_dev * acc;
  
  printk(KERN_DEBUG "ACC_OPEN\n");

  acc = container_of(inode->i_cdev, struct acc_dev, cdev);

  file->private_data = acc;

  return 0;
}

int acc_release(struct inode *inode, struct file *file)
{
  printk(KERN_DEBUG "ACC_RELEASE");

  return 0;
}

int acc_read(struct file *file, char __user  *buf, size_t count, loff_t *ppos)
{
  struct acc_dev * acc = file->private_data;
  int result = 0;

  if(count < 6) {
    return -EINVAL;
  }

  if(wait_event_interruptible(acc->wq, acc->flag != 0)) {
    return -ERESTARTSYS;
  }

  result = copy_to_user(buf, acc->sample, 6);
  acc->flag = 0;
  if(result) {
    return -EFAULT;
  }

  return 6;
}

int acc_init()
{
  if(alloc_chrdev_region(&acc_dev_number, 0, BMI_SLOT_NUM, "bmi_lcd_acc") < 0) {
    printk(KERN_DEBUG "Unable to register accelerometer device\n");
    return -1;
  }
  
  return 0;
}

int acc_clean()
{
  unregister_chrdev_region(MAJOR(acc_dev_number), BMI_SLOT_NUM);
  return 0;
}

/* BMI Functions */
int  acc_probe (struct acc_dev *acc, int slot) 
{
  struct class * bmi_class;
  struct cdev * cdev;
  int ret;

  printk(__FUNCTION__);
  cdev = &acc->cdev;
  printk(KERN_DEBUG "\nAbout to cdev_init acc=%p cdev=%p\n acc_fops=%p\n", acc, cdev, &acc_fops);
  cdev_init(cdev, &acc_fops);
  printk(KERN_DEBUG "After cdev_init\n");

  ret = cdev_add(cdev, acc_dev_number + slot, 1);
  printk(KERN_DEBUG "After cdev_add" );

  bmi_class = (struct class *) bmi_get_bmi_class();
  printk(KERN_DEBUG "After bmi_get_bmi_class" );
  acc->class_dev = device_create(bmi_class, NULL, acc_dev_number + slot, acc, "bmi_lcd_acc_m%d", slot + 1);
  printk(KERN_DEBUG "After class_device_create" );

  init_waitqueue_head(&acc->wq);
  printk(KERN_DEBUG "After init_waitqueue_head" );
  return ret;
}

void acc_remove (struct acc_dev *acc, int slot) 
{
  struct class *bmi_class;
  int acc_major = MAJOR(acc_dev_number);

  bmi_class = (struct class *) bmi_get_bmi_class();
  device_destroy (bmi_class, MKDEV(acc_major, slot));

  acc->class_dev = 0;

  cdev_del (&acc->cdev);
}

