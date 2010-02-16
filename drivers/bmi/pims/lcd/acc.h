#ifndef _ACC_H_
#define _ACC_H_

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>

/* Accelerometer device structure */
struct acc_dev {
  struct cdev cdev;
  u8 sample[6];
  u8 flag;
  struct device * class_dev;
  wait_queue_head_t wq;
};


int acc_open(struct inode *inode, struct file *file);

int acc_release(struct inode *inode, struct file *file);

int acc_read(struct file *file, char __user  *buf, size_t count, loff_t *ppos);

int acc_init(void);

int acc_clean(void);

/*BMI Functions */
void acc_remove(struct acc_dev *acc, int slot);

int acc_probe(struct acc_dev *acc, int slot);

#endif //_ACC_H_
