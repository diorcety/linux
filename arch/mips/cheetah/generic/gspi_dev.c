#include <linux/spi/spi.h>
#include <linux/spi/spi_camelot.h>
//sflash gspi interface
#if defined(CONFIG_MTD_M25P80_MODULE) | defined(CONFIG_MTD_M25P80)
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/flash.h>
static struct mtd_partition bfin_spi_flash_partitions[] = {
    {
        .name       = "second_flash",
        .size       = 0x0100000,
        .offset     = 0,
    },
    {
        .name       = "parition 1",
        .size       = 0x0020000,
        .offset     = 0x0000000,        //MTDPART_OFS_APPEND
    },
    {
        .name       = "parition 2",
        .size       = 0x0040000,
        .offset     = 0x0020000,        //MTDPART_OFS_APPEND
    },
    {
        .name       = "parition 3",
        .size       = 0x0040000,        //MTDPART_SIZ_FULL
        .offset     = 0x0060000,        //MTDPART_OFS_APPEND
    },
    {
        .name       = "parition 4",
        .size       = 0x0060000,        //MTDPART_SIZ_FULL
        .offset     = 0x00a0000,        //MTDPART_OFS_APPEND
    },

    /* ALWAYS keep this part at the end of the structure to for indicating the partition numbers */
    {
        .size       = 0,
        .offset     = 0,
    },
};

static struct flash_platform_data spi_flash_data = {
	.name = "m25p80",
	.parts = bfin_spi_flash_partitions,
	.nr_parts = ARRAY_SIZE(bfin_spi_flash_partitions),
};
#endif

//marvell 8686 spi interface
#if defined(CONFIG_LIBERTAS_SPI_MODULE) | defined(CONFIG_LIBERTAS_SPI)
#include <linux/spi/libertas_spi.h>
static int spi_libertas_setup(struct spi_device *spi)
{
    spi->bits_per_word = 16;
    spi_setup(spi);
    return 0;
}

static struct libertas_spi_platform_data camelot_libertas_pdata = {
	.use_dummy_writes = 0, //set 1 can't work
	.setup = spi_libertas_setup, 
	.teardown = NULL,
};
#endif

struct spi_board_info cml_gspi_devices[] __initdata = {
//sflash gspi interface
#if defined(CONFIG_MTD_M25P80_MODULE) | defined(CONFIG_MTD_M25P80)
    {
        .modalias       = "m25p80",
        .max_speed_hz	= 25000000,//sflash M25P80 max frequency 25MHz, 110nm is 75MHz
        .bus_num        = CML_GSPI_BUS_ID,
        .chip_select	= CML_GSPI_CHIPSELECT_ONE,
        .platform_data	= &spi_flash_data,
    	.mode = SPI_MODE_0,
    },
#endif
//marvell 8686 spi interface
#if defined(CONFIG_LIBERTAS_SPI_MODULE) | defined(CONFIG_LIBERTAS_SPI)
    {
        .modalias       = "libertas_spi",
        .max_speed_hz	= 50000000, //Marvell w8686 max frequency 50MHz
        .bus_num        = CML_GSPI_BUS_ID,
        .chip_select	= CML_GSPI_CHIPSELECT_TWO,
        .platform_data	= &camelot_libertas_pdata,
    	.mode = SPI_MODE_0,
		.irq = 2,
    },
#endif
//ip301 spi interface
    {
        .modalias       = "ip301_spi",
#if defined(CONFIG_CHEETAH_FPGA)
        .max_speed_hz	= 1000000,
#else        
        .max_speed_hz	= 6250000, //ip301 max frequency 20MHz
#endif        
        .bus_num        = CML_GSPI_BUS_ID,
        .chip_select	= CML_GSPI_CHIPSELECT_TWO,
    	.mode = (SPI_CPHA|SPI_3WIRE|SPI_LSB_FIRST),
    },
};

int cml_gspi_devices_size = ARRAY_SIZE(cml_gspi_devices);
