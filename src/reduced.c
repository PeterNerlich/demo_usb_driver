/************************************************
 * USB driver for the OSR FX2 board             *
 * Nick Mikstas                                 *
 * Based on usb-skeleton.c and osrfx2.c         *
 ************************************************/

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>

#define VENDOR_ID     0x0547       
#define PRODUCT_ID    0x1002

#define MINOR_BASE    192

/*********************OSR FX2 vendor commands************************/
#define READ_SWITCHES 0xD6
#define IS_HIGH_SPEED 0xD9

/**********************Function prototypes***************************/
static int osrfx2_open(struct inode * inode, struct file * file);
static int osrfx2_release(struct inode * inode, struct file * file);
static ssize_t osrfx2_read(struct file * file, char * buffer, size_t count, loff_t * ppos);
static int osrfx2_probe(struct usb_interface * interface, const struct usb_device_id * id);
static void osrfx2_disconnect(struct usb_interface * interface);
static void osrfx2_delete(struct kref * kref);
static void interrupt_handler(struct urb * urb);
static ssize_t get_switches(struct device *dev, struct device_attribute *attr, char *buf);

/***********************Module structures****************************/
/*Table of devices that work with this driver*/
static const struct usb_device_id osrfx2_id_table [] = {
    { USB_DEVICE( VENDOR_ID, PRODUCT_ID ) },
    { },
};

/* Create table for modules.usbmap, telling which devices this driver can handle */
MODULE_DEVICE_TABLE(usb, osrfx2_id_table);

/*OSR FX2 private device context structure*/
struct osrfx2 {    
    struct usb_device    * udev;        /* the usb device for this device */
    struct usb_interface * interface;       /* the interface for this device */    
    
    wait_queue_head_t FieldEventQueue;      /*Queue for poll and irq methods*/    

    unsigned char * int_in_buffer;      /*Transfer Buffers*/

    size_t int_in_size;             /*Buffer sizes*/

    __u8  int_in_endpointAddr;          /*USB endpoints*/

    __u8  int_in_endpointInterval;      /*Endpoint intervals*/

    struct urb * int_in_urb;            /*URBs*/
    
    struct kref kref;               /*Reference counter*/

    unsigned char switches;         /*Switch status*/

    size_t pending_data;            /*Data tracking for read write*/

    struct mutex io_mutex;          /*used during cleanup after disconnect*/
};

/* Declare device options and their respective routines in this driver */
static const struct file_operations osrfx2_fops = {
    .owner   = THIS_MODULE,
    .open    = osrfx2_open,
    .release = osrfx2_release,
    .read    = osrfx2_read,
};

/* Declare probe and disconnect routines as well as id table */
static struct usb_driver osrfx2_driver = {
    .name        = "osrfx2",
    .probe       = osrfx2_probe,
    .disconnect  = osrfx2_disconnect,
    .id_table    = osrfx2_id_table,
};

/*Used to get a minor number from the usb core
  and register device with devfs and driver core*/
static struct usb_class_driver osrfx2_class = {
    .name       = "device/osrfx2_%d",
    .fops       = &osrfx2_fops,
    .minor_base = MINOR_BASE,
};

/***********************Module functions*****************************/
/*Create device attribute switches*/
static DEVICE_ATTR(switches, S_IRUGO, get_switches, NULL);

/*insmod*/
int init_module(void) {
    int retval;

    retval = usb_register(&osrfx2_driver);

    if(retval)
        pr_err("usb_register failed. Error number %d", retval);

    return retval;
}

/*rmmod*/
void cleanup_module(void) {
    usb_deregister(&osrfx2_driver);
}

/* "Probe", e.g. attempt to connect the device, after enumeration */
static int osrfx2_probe(struct usb_interface * intf, const struct usb_device_id * id) {
    struct usb_device *udev = interface_to_usbdev(intf);
    struct osrfx2 *fx2dev = NULL;
    struct usb_endpoint_descriptor *endpoint;
    int retval, i, pipe;

    /*Create and initialize context struct*/
    fx2dev = kmalloc(sizeof(struct osrfx2), GFP_KERNEL);
    if (fx2dev == NULL) {
        retval = -ENOMEM;
        dev_err(&intf->dev, "OSR USB-FX2 device probe failed: %d.\n", retval);
        if (fx2dev) kref_put( &fx2dev->kref, osrfx2_delete );
        return retval;
    }

    /*Zero out fx2dev struct*/
    memset(fx2dev, 0, sizeof(*fx2dev));

    /*Set initial fx2dev struct members*/
    kref_init( &fx2dev->kref );
    mutex_init(&fx2dev->io_mutex);
    init_waitqueue_head(&fx2dev->FieldEventQueue);
    fx2dev->udev = usb_get_dev(udev);
    fx2dev->interface = intf;
    usb_set_intfdata(intf, fx2dev);

    /*create sysfs attribute files for device components.*/
    retval = device_create_file(&intf->dev, &dev_attr_switches);
    if (retval != 0) {
        dev_err(&intf->dev, "OSR FX2 device probe failed: %d.\n", retval);
        if (fx2dev) kref_put( &fx2dev->kref, osrfx2_delete );
        return retval;
    }

    /*Set up the endpoint information*/
    for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
        endpoint = &intf->cur_altsetting->endpoint[i].desc;

        if(usb_endpoint_is_int_in(endpoint)) { /*Interrupt in*/
            fx2dev->int_in_endpointAddr = endpoint->bEndpointAddress;
            fx2dev->int_in_endpointInterval = endpoint->bInterval;
            fx2dev->int_in_size = endpoint->wMaxPacketSize;
        }
    }
    /*Error if incorrect number of endpoints found*/
    if (fx2dev->int_in_endpointAddr   == 0) {
        retval = -ENODEV;
        dev_err(&intf->dev, "OSR FX2 device probe failed: %d\n", retval);
        if (fx2dev) kref_put( &fx2dev->kref, osrfx2_delete );
        return retval;
    }

    /*Initialize interrupts*/
    pipe = usb_rcvintpipe(fx2dev->udev, fx2dev->int_in_endpointAddr);
    
    fx2dev->int_in_size = sizeof(fx2dev->switches);

    /*Create interrupt endpoint buffer*/
    fx2dev->int_in_buffer = kmalloc(fx2dev->int_in_size, GFP_KERNEL);
    if (!fx2dev->int_in_buffer) {
        retval = -ENOMEM;
        dev_err(&intf->dev, "OSR FX2 device probe failed: %d.\n", retval);
        if (fx2dev) kref_put( &fx2dev->kref, osrfx2_delete );
        return retval;
    }

    /*Create interrupt endpoint urb*/
    fx2dev->int_in_urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!fx2dev->int_in_urb) {
        retval = -ENOMEM;
        dev_err(&intf->dev, "OSR FX2 device probe failed: %d.\n", retval);
        if (fx2dev) kref_put(&fx2dev->kref, osrfx2_delete);
        return retval;
    }

    /*Fill interrupt endpoint urb*/
    usb_fill_int_urb(fx2dev->int_in_urb, fx2dev->udev, pipe, fx2dev->int_in_buffer,
                     fx2dev->int_in_size, interrupt_handler, fx2dev,
                     fx2dev->int_in_endpointInterval);

    /*Submit urb to USB core*/
    retval = usb_submit_urb( fx2dev->int_in_urb, GFP_KERNEL );
    if (retval != 0) {
        dev_err(&fx2dev->udev->dev, "usb_submit_urb error: %d \n", retval);
        if (fx2dev) kref_put(&fx2dev->kref, osrfx2_delete);
        return retval;
    }

    /*Register device*/
    retval = usb_register_dev(intf, &osrfx2_class);
    if (retval != 0) {
        usb_set_intfdata(intf, NULL);
    }

    dev_info(&intf->dev, "OSR FX2 device now attached\n");

    return 0;
}

/* Handle device disconnect */
static void osrfx2_disconnect(struct usb_interface * intf) {
    struct osrfx2 * fx2dev;

    fx2dev = usb_get_intfdata(intf);
    usb_set_intfdata(intf, NULL);

    /*Give back minor*/
    usb_deregister_dev(intf, &osrfx2_class);

    /*Prevent more I/O from starting*/
    mutex_lock(&fx2dev->io_mutex);
    fx2dev->interface = NULL;
    mutex_unlock(&fx2dev->io_mutex);

    /*Release interrupt urb resources*/
    usb_kill_urb(fx2dev->int_in_urb);

    /*Remove sysfs files*/
    device_remove_file(&intf->dev, &dev_attr_switches);

    /*Decrement usage count*/
    kref_put( &fx2dev->kref, osrfx2_delete );

    dev_info(&intf->dev, "OSR FX2 disconnected.\n");
}

/*Delete resources used by this device*/
static void osrfx2_delete(struct kref * kref) {
    struct osrfx2 *fx2dev = container_of(kref, struct osrfx2, kref);

    usb_put_dev(fx2dev->udev);
    
    if (fx2dev->int_in_urb)
        usb_free_urb(fx2dev->int_in_urb);
    if (fx2dev->int_in_buffer)
        kfree(fx2dev->int_in_buffer);

    kfree(fx2dev);
}

/*Open device for reading and writing*/
static int osrfx2_open(struct inode * inode, struct file * file) {
    struct usb_interface *interface;
    struct osrfx2        *fx2dev;
    int retval;
    
    interface = usb_find_interface(&osrfx2_driver, iminor(inode));
    if (!interface) return -ENODEV;

    fx2dev = usb_get_intfdata(interface);
    if (!fx2dev) return -ENODEV;

    /*Set this device as non-seekable*/
    retval = nonseekable_open(inode, file);
    if (retval) return retval;

    /*Increment our usage count for the device*/
    kref_get(&fx2dev->kref);

    /*Save pointer to device instance in the file's private structure*/
    file->private_data = fx2dev;

    return 0;
}

/*Release device*/
static int osrfx2_release(struct inode * inode, struct file * file) {
    struct osrfx2 * fx2dev;
    int flags;

    fx2dev = (struct osrfx2 *)file->private_data;
    if (!fx2dev)
        return -ENODEV;
 
    /*Decrement the ref-count on the device instance*/
    kref_put(&fx2dev->kref, osrfx2_delete);

    return 0;
}

/*Read from /dev/osrfx2_0*/
static ssize_t osrfx2_read(struct file * file, char * buffer, size_t count, loff_t * ppos) {
    struct osrfx2 *fx2dev;
    int retval = 0;

    fx2dev = (struct osrfx2 *)file->private_data;

    fx2dev->pending_data -= retval;

    retval = sprintf(buffer, "%s%s%s%s%s%s%s%s", /*left sw --> right sw*/
                    (fx2dev->switches & 0x80) ? "1" : "0",
                    (fx2dev->switches & 0x40) ? "1" : "0",
                    (fx2dev->switches & 0x20) ? "1" : "0",
                    (fx2dev->switches & 0x10) ? "1" : "0",
                    (fx2dev->switches & 0x08) ? "1" : "0",
                    (fx2dev->switches & 0x04) ? "1" : "0",
                    (fx2dev->switches & 0x02) ? "1" : "0",
                    (fx2dev->switches & 0x01) ? "1" : "0");

    return retval;
}

/*DIP switch interrupt handler*/
static void interrupt_handler(struct urb * urb) {
    struct osrfx2 *fx2dev = urb->context;
    unsigned char *buf = urb->transfer_buffer;
    int retval;

    if (urb->status == 0) {
        fx2dev->switches = *buf; /*Get new switch state*/

        wake_up(&(fx2dev->FieldEventQueue)); /*Wake-up any requests enqueued*/

        retval = usb_submit_urb(urb, GFP_ATOMIC); /*Restart interrupt urb*/
        if (retval != 0)
            dev_err(&urb->dev->dev, "%s - error %d submitting interrupt urb\n", __FUNCTION__, retval);

        return; /*Success*/   
    }

    /*Error*/
    dev_err(&urb->dev->dev, "%s - non-zero urb status received: %d\n", __FUNCTION__, urb->status);
}

/*Retreive the values of the switches*/
static ssize_t get_switches(struct device *dev, struct device_attribute *attr, char *buf) {
    struct usb_interface   *intf   = to_usb_interface(dev);
    struct osrfx2          *fx2dev = usb_get_intfdata(intf);    
    int retval;

    retval = sprintf(buf, "%s%s%s%s%s%s%s%s", /*left sw --> right sw*/
                    (fx2dev->switches & 0x80) ? "1" : "0",
                    (fx2dev->switches & 0x40) ? "1" : "0",
                    (fx2dev->switches & 0x20) ? "1" : "0",
                    (fx2dev->switches & 0x10) ? "1" : "0",
                    (fx2dev->switches & 0x08) ? "1" : "0",
                    (fx2dev->switches & 0x04) ? "1" : "0",
                    (fx2dev->switches & 0x02) ? "1" : "0",
                    (fx2dev->switches & 0x01) ? "1" : "0");

    return retval;
}

MODULE_DESCRIPTION("OSR FX2 Linux Driver, reduced to dip switches only");
MODULE_AUTHOR("Nick Mikstas, modified by Peter Nerlich");
MODULE_LICENSE("GPL");
