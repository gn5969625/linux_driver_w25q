#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <sound/core.h>
#include <linux/spi/spi.h>
#include <asm/uaccess.h>
#include <linux/timer.h>

//#include <mach/hardware.h>
//#include <mach/regs-gpio.h>

#include <linux/gpio.h>
//#include <plat/gpio-cfg.h>

//#include <linux/mtd/cfi.h>
/* 参考:
 * drivers\mtd\devices\mtdram.c
 * drivers/mtd/devices/m25p80.c
 */

#define DISABLE_MTD 0
#if DISABLE_MTD
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#endif
static struct spi_device *spi_flash;
static unsigned int addr;
void SPIFlashReadID(int *pMID, int *pDID)
{
    unsigned char tx_buf[1];
    unsigned char rx_buf[5];
    int status;
    //printk("%s \n",__func__);
    tx_buf[0] = 0x90;
    //printk("%s %d\n",__func__,0);
    status = spi_write_then_read(spi_flash, tx_buf, 1, rx_buf, 5);
    //printk("status = %d\n",status);
    //printk("%s %d\n",__func__,1);
    *pMID = rx_buf[3];
    *pDID = rx_buf[4];
    //printk("%s %d\n",__func__,2);
}

static void SPIFlashWriteEnable(int enable)
{
    unsigned char val = enable ? 0x06 : 0x04;
    spi_write(spi_flash, &val, 1);    
}

static unsigned char SPIFlashReadStatusReg1(void)
{
    unsigned char val;
    unsigned char cmd = 0x05;
    spi_write_then_read(spi_flash, &cmd, 1, &val, 1);
    return val;
}

static unsigned char SPIFlashReadStatusReg2(void)
{
    unsigned char val;
    unsigned char cmd = 0x35;
    spi_write_then_read(spi_flash, &cmd, 1, &val, 1);
    return val;
}

static void SPIFlashWaitWhenBusy(void)
{
    while (SPIFlashReadStatusReg1() & 1)
    {
        /* 休眠一段时间 */
        /* Sector erase time : 60ms
         * Page program time : 0.7ms
         * Write status reg time : 10ms
         */
	set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(HZ/100);  /* 休眠10MS后再次判断 */
    }
}

static void SPIFlashWriteStatusReg(unsigned char reg1, unsigned char reg2)
{    
    unsigned char tx_buf[4];
    SPIFlashWriteEnable(1);  
    tx_buf[0] = 0x01;
    tx_buf[1] = reg1;
    tx_buf[2] = reg2;
    spi_write(spi_flash, tx_buf, 3);   
    SPIFlashWaitWhenBusy();
}

static void SPIFlashClearProtectForStatusReg(void)
{
    unsigned char reg1, reg2;
    reg1 = SPIFlashReadStatusReg1();
    reg2 = SPIFlashReadStatusReg2();
    reg1 &= ~(1<<7);
    reg2 &= ~(1<<0);
    SPIFlashWriteStatusReg(reg1, reg2);
}

static void SPIFlashClearProtectForData(void)
{
    /* cmp=0,bp2,1,0=0b000 */
    unsigned char reg1, reg2;
    reg1 = SPIFlashReadStatusReg1();
    reg2 = SPIFlashReadStatusReg2();
    reg1 &= ~(7<<2);
    reg2 &= ~(1<<6);
    SPIFlashWriteStatusReg(reg1, reg2);
}

/* erase 4K */
void SPIFlashEraseSector(unsigned int addr)
{
    unsigned char tx_buf[4];
    tx_buf[0] = 0x20;
    tx_buf[1] = addr >> 16;
    tx_buf[2] = addr >> 8;
    tx_buf[3] = addr & 0xff;
    SPIFlashWriteEnable(1);  
    spi_write(spi_flash, tx_buf, 4);
    SPIFlashWaitWhenBusy();
}

/* program */
void SPIFlashProgram(unsigned int addr, unsigned char *buf, int len)
{
    unsigned char tx_buf[4];   
    struct spi_transfer	t[] = {
        {
    	 .tx_buf = tx_buf,
    	 .len = 4,
        },
	{
         .tx_buf = buf,
         .len = len,
	},
    };
    struct spi_message m;
    tx_buf[0] = 0x02;
    tx_buf[1] = addr >> 16;
    tx_buf[2] = addr >> 8;
    tx_buf[3] = addr & 0xff;
    SPIFlashWriteEnable(1);  
    spi_message_init(&m);
    spi_message_add_tail(&t[0], &m);
    spi_message_add_tail(&t[1], &m);
    spi_sync(spi_flash, &m);
    SPIFlashWaitWhenBusy();    
}

void SPIFlashRead(unsigned int addr, unsigned char *buf, int len)
{
    /* spi_write_then_read规定了tx_cnt+rx_cnt < 32
     * 所以对于大量数据的读取，不能使用该函数
     */
     
    unsigned char tx_buf[4];   
    struct spi_transfer	t[] = {
        {
         .tx_buf = tx_buf,
         .len = 4,
        },
	{
    	 .rx_buf = buf,
    	 .len = len,
	},
    };
    struct spi_message m;
    tx_buf[0] = 0x03;
    tx_buf[1] = addr >> 16;
    tx_buf[2] = addr >> 8;
    tx_buf[3] = addr & 0xff;
    spi_message_init(&m);
    spi_message_add_tail(&t[0], &m);
    spi_message_add_tail(&t[1], &m);
    spi_sync(spi_flash, &m); 
}


static void SPIFlashInit(void)
{
    printk("%s \n",__func__);
    SPIFlashClearProtectForStatusReg();
    SPIFlashClearProtectForData();
}



//we don't use mtd to operate the w25q spi flash,so we create sysfs device node to handle it
static ssize_t w25q_show_id(struct device *dev,struct device_attribute *attr, char *buf) {
    int mid,did;
    SPIFlashReadID(&mid, &did);
    return sprintf(buf,"SPI Flash ID: %02x %02x\n", mid, did);
}
static DEVICE_ATTR(w25q_id, 0644 , w25q_show_id, NULL);

static ssize_t w25q_show_addr(struct device *dev,struct device_attribute *attr, char *buf) {

    return 0;
}
static ssize_t w25q_set_addr(struct device *dev,struct device_attribute *attr, const char *buf, size_t count) {

    return 0;
}
static DEVICE_ATTR(w25q_addr, 0644 , w25q_show_addr, w25q_set_addr);

static ssize_t w25q_read_data(struct device *dev,struct device_attribute *attr, char *buf) {

    return 0;
}
static ssize_t w25q_write_data(struct device *dev,struct device_attribute *attr, const char *buf, size_t count) {

    return 0;
}
static DEVICE_ATTR(w25q_data, 0644 , w25q_read_data, w25q_write_data);
static struct attribute *w25q_attrs[] = {
    &dev_attr_w25q_data.attr,
    &dev_attr_w25q_addr.attr,
    &dev_attr_w25q_id.attr,
    NULL
};
static struct attribute_group w25q_attr_group = {
    .name = "w25q",
    .attrs = w25q_attrs,
};

/* 构造注册一个mtd_info
 * mtd_device_register(master, parts, nr_parts)
 *
 */

/* 首先: 构造注册spi_driver
 * 然后: 在spi_driver的probe函数里构造注册mtd_info
 */
#if DISABLE_MTD
static struct mtd_info spi_flash_dev;
static int spi_flash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
    unsigned int addr = instr->addr;
    unsigned int len  = 0;

    /* 判断参数 */
    if ((addr & (spi_flash_dev.erasesize - 1)) || (instr->len & (spi_flash_dev.erasesize - 1))) {
        printk("addr/len is not aligned\n");
        return -EINVAL;
    }

    for (len = 0; len < instr->len; len += 4096) {
        SPIFlashEraseSector(addr);
        addr += 4096;
    }
    instr->state = MTD_ERASE_DONE;
    mtd_erase_callback(instr);
    return 0;
}

static int spi_flash_read(struct mtd_info *mtd, loff_t from, size_t len,size_t *retlen, u_char *buf)
{
    SPIFlashRead(from, buf, len);
    *retlen = len;
    return 0;
}

static int spi_flash_write(struct mtd_info *mtd, loff_t to, size_t len,size_t *retlen, const u_char *buf)
{
    unsigned int addr = to;
    unsigned int wlen  = 0;
    /* 判断参数 */
    if ((to & (spi_flash_dev.erasesize - 1)) || (len & (spi_flash_dev.erasesize - 1))) {
        printk("addr/len is not aligned\n");
        return -EINVAL;
    }

    for (wlen = 0; wlen < len; wlen += 256) {
        SPIFlashProgram(addr, (unsigned char *)buf, 256);
        addr += 256;
        buf += 256;
    }
    *retlen = len;
    return 0;
}
#endif

static int spi_flash_probe(struct spi_device *spi)
{
    int ret;
    //int mid,did;
    spi_flash = spi;
    addr = 0;
    //s3c2410_gpio_cfgpin(spi->chip_select, S3C2410_GPIO_OUTPUT);
    dev_dbg(&spi->dev, "%s\n", __func__);
    spi->mode = 0;
    spi->max_speed_hz = 1000 * 1000 * 10;
    spi->bits_per_word = 8;
    spi_flash = spi;
    //SPIFlashInit();
    //SPIFlashReadID(&mid, &did);
    //printk("SPI Flash ID: %02x %02x\n", mid, did);
    SPIFlashInit();
    ret = sysfs_create_group(&spi->dev.kobj, &w25q_attr_group);
    printk("Finish probe: %s\n",__func__);

#if DISABLE_MTD
    memset(&spi_flash_dev, 0, sizeof(spi_flash_dev));
    spi_flash_dev.name = "100ask_spi_flash";
    spi_flash_dev.type = MTD_NORFLASH;
    spi_flash_dev.flags = MTD_CAP_NORFLASH;
    spi_flash_dev.size = 0x200000;  // 2M
    spi_flash_dev.writesize = 1;
    spi_flash_dev.writebufsize = 4096; // 没有用到
    spi_flash_dev.erasesize = 4096;  // 擦除的最小单位    
    spi_flash_dev.owner = THIS_MODULE;
    spi_flash_dev._erase = spi_flash_erase;
    spi_flash_dev._read  = spi_flash_read;
    spi_flash_dev._write = spi_flash_write;
    mtd_device_register(&spi_flash_dev, NULL, 0);
#endif
    return 0;
}

static int spi_flash_remove(struct spi_device *spi)
{
#if DISABLE_MTD
    mtd_device_unregister(&spi_flash_dev);
#endif
    sysfs_remove_group(&spi->dev.kobj, &w25q_attr_group);
    return 0;
}

static const struct of_device_id flash_spi_match[] = {
	{.compatible = "winbond,w25q16d"},
	{},
};
MODULE_DEVICE_TABLE(of, flash_spi_match);
static struct spi_driver spi_flash_drv = {
	.driver = {
		.name	= "w25q16d_spi_flash",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(flash_spi_match),
	},
	.probe		= spi_flash_probe,
	.remove		= spi_flash_remove,
};

static int spi_flash_init(void)
{
    printk("%s \n",__func__);
    return spi_register_driver(&spi_flash_drv);
}

static void spi_flash_exit(void)
{
    spi_unregister_driver(&spi_flash_drv);
}

module_init(spi_flash_init);
module_exit(spi_flash_exit);
MODULE_DESCRIPTION("W25Q16D Flash SPI Driver");
MODULE_AUTHOR("Kevin.Shen gn5969625@gmail.com");
MODULE_LICENSE("GPL");
