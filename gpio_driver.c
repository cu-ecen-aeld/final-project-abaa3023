/*
 * @file: gpio_driver.c
 * @desc: This modules implements a GPIO device driver. This module can read and write to a GPIO
 *  Pin. In the present code it can control two GPIO Pins 17 and 18, but the code is easily
 *  scalable for multiple GPIO Pins.
 * @Author: Abijith Ananda Krishnan; abaa3023@colorado.edu
 * @citation: https://embetronicx.com/tutorials/linux/device-drivers/gpio-driver-basic-using-raspberry-pi/
 * @citation: https://github.com/cu-ecen-aeld/final-project-Jahnavi-pinnamaneni/blob/main/GPIO_Driver/gpio_driver.c
 */
 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>

/* Meta Information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Abijith Ananda Krishnan");
MODULE_DESCRIPTION("A kernel Module for GPIO Driver");

/* Variables for device and device class */
static dev_t my_device_nr;
static struct class *my_class;
static struct cdev my_device;

#define DRIVER_NAME "my_gpio_driver"
#define DRIVER_CLASS "MyModuleClass"
#define GPIO_SEVENTEEN 17
#define GPIO_EIGHTTEEN 18

/**
 * @brief Read data out of the buffer
 */
static ssize_t driver_read(struct file *File, char *user_buffer, size_t count, loff_t *offs) {
	int to_copy, not_copied, delta;
	char tmp[3] = " \n";

	/* Get amount of data to copy */
	to_copy = min(count, sizeof(tmp));

	/* Read value of GPIO */
	tmp[0] = gpio_get_value(GPIO_SEVENTEEN) + '0';

	/* Copy data to user */
	not_copied = copy_to_user(user_buffer, &tmp, to_copy);

	/* Calculate data */
	delta = to_copy - not_copied;

	return delta;
}

/**
 * @brief Write data to buffer
 */
static ssize_t driver_write(struct file *File, const char *user_buffer, size_t count, loff_t *offs) {
	int to_copy, not_copied, delta;
	char value;

	/* Get amount of data to copy */
	to_copy = min(count, sizeof(value));

	/* Copy data to user */
	not_copied = copy_from_user(&value, user_buffer, to_copy);
	printk("Value received: %d\n", value);

	/* Setting the GPIO */
	switch(value) {
		case '0':
		gpio_set_value(GPIO_EIGHTTEEN, 0);
		break;
		case '1':
		gpio_set_value(GPIO_EIGHTTEEN, 1);
		break;
		default:
		printk("Invalid Input!\n");
		break;
	}

	/* Calculate data */
	delta = to_copy - not_copied;

	return delta;
}

/**
 * @brief This function is called, when the device file is opened
 */
static int driver_open(struct inode *device_file, struct file *instance) {
	printk("dev_nr - open was called!\n");
	return 0;
}

/**
 * @brief This function is called, when the device file is opened
 */
static int driver_close(struct inode *device_file, struct file *instance) {
	printk("dev_nr - close was called!\n");
	return 0;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = driver_open,
	.release = driver_close,
	.read = driver_read,
	.write = driver_write
};

/**
 * @brief This function is called, when the module is loaded into the kernel
 */
static int __init ModuleInit(void) {
	printk("Hello, Kernel!\n");

	/* Allocate a device nr */
	if( alloc_chrdev_region(&my_device_nr, 0, 1, DRIVER_NAME) < 0) {
		printk("Device Nr. could not be allocated!\n");
		return -1;
	}
	printk("read_write - Device Nr. Major: %d, Minor: %d was registered!\n", my_device_nr >> 20, my_device_nr && 0xfffff);

	/* Create device class */
	if((my_class = class_create(THIS_MODULE, DRIVER_CLASS)) == NULL) {
		printk("Device class can not e created!\n");
		goto ClassError;
	}

	/* create device file */
	if(device_create(my_class, NULL, my_device_nr, NULL, DRIVER_NAME) == NULL) {
		printk("Can not create device file!\n");
		goto FileError;
	}

	/* Initialize device file */
	cdev_init(&my_device, &fops);

	/* Regisering device to kernel */
	if(cdev_add(&my_device, my_device_nr, 1) == -1) {
		printk("Registering of device to kernel failed!\n");
		goto AddError;
	}

	/* GPIO 17 init */
	if(gpio_request(GPIO_SEVENTEEN, "rpi-gpio-17")) {
		printk("Can not allocate GPIO 4\n");
		goto AddError;
	}

	/* Set GPIO 17 direction */
	if(gpio_direction_input(GPIO_SEVENTEEN)) {
		printk("Can not set GPIO 17 to input!\n");
		goto Gpio17Error;
	}

	/* GPIO 18 init */
	if(gpio_request(GPIO_EIGHTTEEN, "rpi-gpio-18")) {
		printk("Can not allocate GPIO 18\n");
		goto Gpio18Error;
	}

	/* Set GPIO 18 direction */
	if(gpio_direction_output(GPIO_EIGHTTEEN, 0)) {
		printk("Can not set GPIO 18 to output!\n");
		goto Gpio18Error;
	}

	return 0;
	Gpio17Error:
	gpio_free(GPIO_SEVENTEEN);
	Gpio18Error:
	gpio_free(GPIO_EIGHTTEEN);
	AddError:
	device_destroy(my_class, my_device_nr);
	FileError:
	class_destroy(my_class);
	ClassError:
	unregister_chrdev_region(my_device_nr, 1);
	return -1;
}

/**
 * @brief This function is called, when the module is removed from the kernel
 */
static void __exit ModuleExit(void) {
	gpio_set_value(GPIO_EIGHTTEEN, 0);
	gpio_free(GPIO_SEVENTEEN);
	gpio_free(GPIO_EIGHTTEEN);
	cdev_del(&my_device);
	device_destroy(my_class, my_device_nr);
	class_destroy(my_class);
	unregister_chrdev_region(my_device_nr, 1);
	printk("Goodbye, Kernel\n");
}

module_init(ModuleInit);
module_exit(ModuleExit);
