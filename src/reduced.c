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
static ssize_t osrfx2_write(struct file * file, const char * user_buffer, size_t count, loff_t * ppos);
static int osrfx2_probe(struct usb_interface * interface, const struct usb_device_id * id);
static void osrfx2_disconnect(struct usb_interface * interface);
static void osrfx2_delete(struct kref * kref);
static void write_bulk_callback(struct urb *urb);
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
   
    unsigned char * int_in_buffer;      
    unsigned char * bulk_in_buffer;     /*Transfer Buffers*/
    unsigned char * bulk_out_buffer;        
    
    size_t int_in_size;
    size_t bulk_in_size;            /*Buffer sizes*/
    size_t bulk_out_size;
    
    __u8  int_in_endpointAddr;
    __u8  bulk_in_endpointAddr;         /*USB endpoints*/
    __u8  bulk_out_endpointAddr;
    
    __u8  int_in_endpointInterval;
    __u8  bulk_in_endpointInterval;     /*Endpoint intervals*/
    __u8  bulk_out_endpointInterval;
    
    struct urb * bulk_in_urb;
    struct urb * int_in_urb;            /*URBs*/
    struct urb * bulk_out_urb;
    
    struct kref kref;               /*Reference counter*/

    unsigned char switches;         /*Switch status*/

    atomic_t bulk_write_available;      /*Track usage of the bulk pipes*/
    atomic_t bulk_read_available;

    size_t pending_data;            /*Data tracking for read write*/

    struct mutex io_mutex;          /*used during cleanup after disconnect*/
};

/* Declare device options and their respective routines in this driver */
static const struct file_operations osrfx2_fops = {
    .owner   = THIS_MODULE,
    .open    = osrfx2_open,
    .release = osrfx2_release,
    .read    = osrfx2_read,
    .write   = osrfx2_write,
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
    fx2dev->bulk_write_available = (atomic_t) ATOMIC_INIT(1);
    fx2dev->bulk_read_available  = (atomic_t) ATOMIC_INIT(1);
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

        if(usb_endpoint_is_bulk_in(endpoint)) { /*Bulk in*/
            fx2dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
            fx2dev->bulk_in_endpointInterval = endpoint->bInterval;
            fx2dev->bulk_in_size = endpoint->wMaxPacketSize;
        }
        if(usb_endpoint_is_bulk_out(endpoint)) { /*Bulk out*/
            fx2dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
            fx2dev->bulk_out_endpointInterval = endpoint->bInterval;
            fx2dev->bulk_out_size = endpoint->wMaxPacketSize;
        }
        if(usb_endpoint_is_int_in(endpoint)) { /*Interrupt in*/
            fx2dev->int_in_endpointAddr = endpoint->bEndpointAddress;
            fx2dev->int_in_endpointInterval = endpoint->bInterval;
            fx2dev->int_in_size = endpoint->wMaxPacketSize;
        }
    }
    /*Error if incorrect number of endpoints found*/
    if (fx2dev->int_in_endpointAddr   == 0 ||
        fx2dev->bulk_in_endpointAddr  == 0 ||
        fx2dev->bulk_out_endpointAddr == 0) {
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

    /*Initialize bulk endpoint buffers*/
    fx2dev->bulk_in_buffer = kmalloc(fx2dev->bulk_in_size, GFP_KERNEL);
    if (!fx2dev->bulk_in_buffer) {
        retval = -ENOMEM;
        dev_err(&intf->dev, "OSR FX2 device probe failed: %d.\n", retval);
        if (fx2dev) kref_put(&fx2dev->kref, osrfx2_delete);
        return retval;
    }
    fx2dev->bulk_out_buffer = kmalloc(fx2dev->bulk_out_size, GFP_KERNEL);
    if (!fx2dev->bulk_out_buffer) {
        retval = -ENOMEM;
        dev_err(&intf->dev, "OSR FX2 device probe failed: %d.\n", retval);
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
    if (fx2dev->bulk_in_buffer)
        kfree(fx2dev->bulk_in_buffer);
    if (fx2dev->bulk_out_buffer)
        kfree(fx2dev->bulk_out_buffer);

    kfree(fx2dev);
}

/*Open device for reading and writing*/
static int osrfx2_open(struct inode * inode, struct file * file) {
    struct usb_interface *interface;
    struct osrfx2        *fx2dev;
    int retval;
    int flags;
    
    interface = usb_find_interface(&osrfx2_driver, iminor(inode));
    if (!interface) return -ENODEV;

    fx2dev = usb_get_intfdata(interface);
    if (!fx2dev) return -ENODEV;

    /*Serialize access to each of the bulk pipes*/
    flags = (file->f_flags & O_ACCMODE);

    if ((flags == O_WRONLY) || (flags == O_RDWR)) {
        if (!atomic_dec_and_test( &fx2dev->bulk_write_available )) {
            atomic_inc( &fx2dev->bulk_write_available );
            return -EBUSY;
        }

        /*The write interface is serialized, so reset bulk-out pipe (ep-6)*/
        retval = usb_clear_halt(fx2dev->udev, fx2dev->bulk_out_endpointAddr);
        if ((retval != 0) && (retval != -EPIPE)) {
            dev_err(&interface->dev, "%s - error(%d) usb_clear_halt(%02X)\n",
                    __FUNCTION__, retval, fx2dev->bulk_out_endpointAddr);
        }
    }

    if ((flags == O_RDONLY) || (flags == O_RDWR)) {
        if (!atomic_dec_and_test( &fx2dev->bulk_read_available )) {
            atomic_inc( &fx2dev->bulk_read_available );
            if (flags == O_RDWR)
                atomic_inc( &fx2dev->bulk_write_available );
            return -EBUSY;
        }

        /*The read interface is serialized, so reset bulk-in pipe (ep-8)*/
        retval = usb_clear_halt(fx2dev->udev, fx2dev->bulk_in_endpointAddr);
        if ((retval != 0) && (retval != -EPIPE)) {
            dev_err(&interface->dev, "%s - error(%d) usb_clear_halt(%02X)\n",
                    __FUNCTION__, retval, fx2dev->bulk_in_endpointAddr);
        }
    }

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

    /*Release any bulk_[write|read]_available serialization*/
    flags = (file->f_flags & O_ACCMODE);

    if ((flags == O_WRONLY) || (flags == O_RDWR))
        atomic_inc( &fx2dev->bulk_write_available );

    if ((flags == O_RDONLY) || (flags == O_RDWR))
        atomic_inc( &fx2dev->bulk_read_available );
 
    /*Decrement the ref-count on the device instance*/
    kref_put(&fx2dev->kref, osrfx2_delete);

    return 0;
}

/*Read from /dev/osrfx2_0*/
static ssize_t osrfx2_read(struct file * file, char * buffer, size_t count, loff_t * ppos) {
    struct osrfx2 *fx2dev;
    int retval = 0;
    int bytes_read;
    int pipe;

    fx2dev = (struct osrfx2 *)file->private_data;

    /*Initialize pipe*/
    pipe = usb_rcvbulkpipe(fx2dev->udev, fx2dev->bulk_in_endpointAddr),

    /*Do a blocking bulk read to get data from the device*/
    retval = usb_bulk_msg(fx2dev->udev, pipe, fx2dev->bulk_in_buffer, min(fx2dev->bulk_in_size, count),
                          &bytes_read, 10000);

    /*If the read was successful, copy the data to userspace */
    if (!retval) {
        if (copy_to_user(buffer, fx2dev->bulk_in_buffer, bytes_read))
            retval = -EFAULT;
        else
            retval = bytes_read;        
        
        /*Increment the pending_data counter by the byte count received*/
        fx2dev->pending_data -= retval;
    }

    return retval;
}

/*Write to bulk endpoint*/
static ssize_t osrfx2_write(struct file * file, const char * user_buffer, size_t count, loff_t * ppos) {
    struct osrfx2 *fx2dev;
    struct urb *urb = NULL;
    char *buf = NULL;
    int pipe;
    int retval = 0;

    fx2dev = (struct osrfx2 *)file->private_data;

    if (!count) return count;
 
    /*Create a urb*/
    urb = usb_alloc_urb(0, GFP_KERNEL);

    if(!urb) {
        retval = -ENOMEM;
        usb_free_coherent(fx2dev->udev, count, buf, urb->transfer_dma);
        usb_free_urb(urb);
        return retval;
    }

    /*Create urb buffer*/
    buf = usb_alloc_coherent(fx2dev->udev, count, GFP_KERNEL, &urb->transfer_dma);

    if(!buf) {
        retval = -ENOMEM;
        usb_free_coherent(fx2dev->udev, count, buf, urb->transfer_dma);
        usb_free_urb(urb);
        return retval;
    }

    /*Copy the data to the buffer*/
    if(copy_from_user(buf, user_buffer, count)) {
        retval = -EFAULT;
        usb_free_coherent(fx2dev->udev, count, buf, urb->transfer_dma);
        usb_free_urb(urb);
        return retval;
    }

    /*Initialize the urb*/
    pipe = usb_sndbulkpipe(fx2dev->udev, fx2dev->bulk_out_endpointAddr);
    usb_fill_bulk_urb( urb, fx2dev->udev, pipe, buf, count, write_bulk_callback, fx2dev);
    urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

    /*Send the data out the bulk port*/
    retval = usb_submit_urb(urb, GFP_KERNEL);

    if (retval) {
        dev_err(&fx2dev->interface->dev, "%s - usb_submit_urb failed: %d\n", __FUNCTION__, retval);
        usb_free_coherent(fx2dev->udev, count, buf, urb->transfer_dma);
        usb_free_urb(urb);
        return retval;
    }

    /*Increment the pending_data counter by the byte count sent*/
    fx2dev->pending_data += count;
     
    /*Release the reference to this urb*/
    usb_free_urb(urb);

    return count;
}

static void write_bulk_callback(struct urb * urb) {
    struct osrfx2 *fx2dev = (struct osrfx2 *)urb->context;
 
    /*  Filter sync and async unlink events as non-errors*/
    if(urb->status && !(urb->status == -ENOENT || urb->status == -ECONNRESET || urb->status == -ESHUTDOWN))
        dev_err(&fx2dev->interface->dev, "%s - non-zero status received: %d\n", __FUNCTION__, urb->status);
 
    /*Free the spent buffer*/
    usb_free_coherent( urb->dev, urb->transfer_buffer_length, urb->transfer_buffer, urb->transfer_dma );
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
