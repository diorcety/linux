
#ifndef _CTA_PCM_H
#define _CTA_PCM_H

struct cta_pcm_dma_params {
    char *name;                 /* stream identifier */
    unsigned int transfer_type; /* READ or WRITE DMA transfer */
    struct cta_master *master;
    void (*desc_init)(struct snd_pcm_substream *substream);
};

typedef struct cta_dma_desc {
    volatile u32 info;
    volatile u32 dtadr;
} cta_dma_desc;

#ifdef CONFIG_PCMGPIOCONTROL
struct cta_pcm_event{
    int action;                    // start:0, stop:1;
    struct delayed_work gpioctrl;
};
#endif

struct cta_runtime_data {
    struct cta_device *device;
    struct timer_list poll_timer;
    int active;
    unsigned int period;
    unsigned int periods;
//  int tx_spin;
    int irq_requested;
    spinlock_t lock;
    int cur;
    int next;
    int codec_init;
    snd_pcm_uframes_t hw_ptr;              /* hw ptr */
    dma_addr_t dma_desc_array_phys;        /* channel 0 descriptor base physical address */
    dma_addr_t dma_desc_array_phys2;       /* channel 1 descriptor base physical address */
    cta_dma_desc *dma_desc_array;          /* channel 0 descriptor base address */
    cta_dma_desc *dma_desc_array2;         /* channel 1 descriptor base address */
#ifdef CONFIG_PCMGPIOCONTROL
    struct cta_pcm_event pcm_event; 
#endif
};

/* platform data */
extern struct snd_soc_platform cheetah_snd_soc_platform;

#if defined (CONFIG_GPIO_AUDIO_AMP_POWER) && (CONFIG_GPIO_AUDIO_AMP_POWER_NUM)
#define CFG_IO_AUDIO_AMP_POWER CONFIG_GPIO_AUDIO_AMP_POWER_NUM
#define AUDIO_AMP_POWER CFG_IO_AUDIO_AMP_POWER

#if defined (CONFIG_GPIO_AUDIO_AMP_POWER_LOW_ACTIVE)
 #define AUDIO_AMP_ACTIVE_LEVEL 0
#else
 #define AUDIO_AMP_ACTIVE_LEVEL 1
#endif

int cta_spk_on(int enable);
#endif

#endif
