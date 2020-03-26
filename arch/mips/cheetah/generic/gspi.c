/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                           |
|                                                                              |
+=============================================================================*/
/*! 
*   \file gspi.c 
*   \brief CAMELOT Generic SPI Driver
*   \author Montage
*/
//#ifdef    CONFIG_GSPI
/*=============================================================================+
| Included Files                                                               |
+=============================================================================*/
#include <linux/module.h>
#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/gspi.h>
/*=============================================================================+
| Define                                                                       |
+=============================================================================*/
#define GSPI_CSR    (0x0)   /* GSPI Control/Status Register offset */
#define GSPI_CR     (0x4)   /* GSPI Control Register offset */
#define GSPI_DR     (0x8)   /* GSPI Data Register offset */

#define MI_SROMEN	(MI_BASE + 0x8)

#define SPI_TX_WAIT_LOOP    10000
#define SPI_RX_WAIT_LOOP    10000

#define SPIREG(reg) (*(volatile unsigned int*)(reg))

#define spi_udelay(n) {int _www; for (_www=0; _www< 100; _www++); }
#if 0
#define gspi_log printf
#else
#define gspi_log(x , ...)
#endif
/*=============================================================================+
| Variables                                                                    |
+=============================================================================*/
static unsigned int gspi_cbase[2]={MI_BASE+0x30,MI_BASE+0x40};

/*=============================================================================+
| Function Prototypes                                                          |
+=============================================================================*/
int spi_ready(unsigned int idx);
int spi_read(unsigned int idx, unsigned int data_bit_len, unsigned char * rx_buf);
int spi_write(unsigned int idx, unsigned int data_bit_len, unsigned char * tx_buf);
void spi_keep_cs(unsigned int idx, int keep);

/*=============================================================================+
| Functions                                                                    |
+=============================================================================*/
void SET_REG(unsigned int addr, unsigned int mask, unsigned int value)
{
    unsigned int temp;  
    temp = SPIREG(addr) & mask;
    SPIREG(addr) = temp | value;
}

void spi_keep_cs(unsigned int idx, int keep)
{
    if (keep == 1 || keep == 0)
    {
        SET_REG(gspi_cbase[idx] + GSPI_CSR, ~(1<<6), (keep<<6));
    }                   
}
EXPORT_SYMBOL(spi_keep_cs);

void spi_set_len(unsigned int idx, unsigned int len)
{
    SPIREG(gspi_cbase[idx] + GSPI_CR) = len;
}

int spi_ready(unsigned int idx)
{
    unsigned int i,rc;
    for (i=0; i<SPI_RX_WAIT_LOOP; i++)
    {
        rc = SPIREG(gspi_cbase[idx] + GSPI_CSR);
        if (((rc & SPI_TXEMPTY) != 0) && ((rc & SPI_RXNONEMPTY) == 0))
            return 1;
    }
    gspi_log("SPI wait ready timeout\n");
    return 0;
}
EXPORT_SYMBOL(spi_ready);

int spi_read(unsigned int idx, unsigned int data_bit_len, unsigned char * rx_buf)
{
    // maximum SPI_Length = 2^16 - 1 bits
    unsigned int i;
    volatile unsigned int * temp;
    // check status
    if (!spi_ready(idx))
        return -2;  
    // set SKIP and SPI_Length
    spi_set_len(idx, data_bit_len);
    gspi_log("bit_len=%d\n",data_bit_len);
    // read all data from SPI Data Register
    for (i=0; i<(data_bit_len +7)/8; i+=4)
    {
        temp = (int *)(rx_buf + i);
        *temp = SPIREG(gspi_cbase[idx] + GSPI_DR);
        gspi_log("SPI read =0x%08x\n",*temp); 
    }
    // wait till RX buffer is empty
//  for (i=0; i<SPI_RX_WAIT_LOOP; i++)
//  {
//      rc = SPIREG(gspi_cbase[idx] + GSPI_CSR);
//      if ((rc & SPI_RXNONEMPTY) == 0)
//          return 0;
//      spi_udelay(1);
//  }
    return 0; 
}
EXPORT_SYMBOL(spi_read);

int spi_write(unsigned int idx, unsigned int data_bit_len, unsigned char * tx_buf)
{
    // maximum SPI_Length = 2^16 - 1 bits
    unsigned int i,j,rc;
    volatile unsigned int temp;
    // check status
    if (!spi_ready(idx))
        return -2;  
    // set SKIP and SPI_Length
    spi_set_len(idx, data_bit_len);
    gspi_log("bit_len=%d\n",data_bit_len);
    // write all data into SPI_DR 
    for (i=0; i<(data_bit_len +7)/8; i+=4)
    {
        // wait TX buffer isn't full
        for (j=0; j<SPI_TX_WAIT_LOOP; j++)
        {
            rc = SPIREG(gspi_cbase[idx] + GSPI_CSR);
            if ((rc & SPI_TXFULL) == 0)
                break;
//          spi_udelay(1);
        }
        if (j == SPI_TX_WAIT_LOOP)
            return -1;
        temp = *(int *)(tx_buf + i);
        SPIREG(gspi_cbase[idx] + GSPI_DR) = temp; 
gspi_log("SPI write =0x%08x\n",temp);
    }
    // wait till TX buffer is empty
//  for (i=0; i<SPI_TX_WAIT_LOOP; i++)
//  {
//      rc = SPIREG(gspi_cbase[idx] + GSPI_CSR);
//      if ((rc & SPI_TXEMPTY) != 0)
//          return 0;
//      spi_udelay(1);
//  }
    return 0; 
}
EXPORT_SYMBOL(spi_write);

void spi_init(unsigned int idx, unsigned int init_val) // SPI_CLK_W(15~8) SPI_CLK_P(5) SPI_MOD(4) SPI_SAM(3)
{
    // set serial mode for GSPI0
	if(idx == 0)
    	SPIREG(MI_SROMEN) &= ~(1<<1);
    // SPI init SPI_CLK_width,SPI_CLK_Park,SPI_Mode,SPI Rising/Falling Sample Data
    SET_REG(gspi_cbase[idx] + GSPI_CSR, SPI_INIT_MASK, init_val);
    // reset SPI state machine
    SET_REG(gspi_cbase[idx] + GSPI_CSR, ~(1<<7), (1<<7));
    SET_REG(gspi_cbase[idx] + GSPI_CSR, ~(1<<7), (0<<7));
}
EXPORT_SYMBOL(spi_init);

#if 0
int do_spi(int argc, char *argv[])
{
    int i,rc;
    int rw=0xff;
    int failed=0;
    unsigned char rx_buf[32];
    unsigned int data_len = 0xbfc20000;
    unsigned int idx   = 1;

    if (argc > 0)
    {
        if (!hextoul(argv[0], &idx))
        goto err1;
    }
    if (argc > 1)
    {
        if (!hextoul(argv[1], &rw))
        goto err1;
    }
    if (argc > 2)
    {
        if (!hextoul(argv[2], &data_len))
        goto err1;
    }

    if (rw== 1)
    {
        if ((rc = spi_read(idx,data_len,rx_buf)))
        {
            failed |= (1<<1);
        }
        else
        {
            gspi_log("read data =");
            for(i=0; i<(data_len +7)/8; i+=4)
            {   gspi_log("%2x ",rx_buf[i]);}
        }
    }
    else if (rw == 2)
    {
        if ((rc = spi_write(idx,data_len,argv[2])))
        {
            failed |= (1<<2);
        }
    }
    else if (rw == 3)
    {
        spi_keep_cs(idx, data_len);
    }
    else
    {
        failed |= (1<<20);
    }
    gspi_log("\nDone: %s \n", (0==failed)? "PASSED" : "FAILED");
    return (0==failed)? ERR_OK : ERR_PARM;

help:
err1:
    return ERR_PARM;
}

cmdt cmdt_spi_test[] __attribute__ ((section("cmdt"))) =
{
    { "spi", do_spi, "spi idx 1(read)/2(write) bit_length string"},
};
#endif

