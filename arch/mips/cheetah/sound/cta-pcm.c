/*
 * ALSA PCM interface for the Cheetah chip
 *
 */

#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <asm/delay.h>

#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-generic/dma-coherence.h>
#include <asm/io.h>

#include "cta-i2s.h"
#include "cta-pcm.h"

#ifdef CFG_IO_AUDIO_AMP_POWER
#include <asm/mach-cheetah/gpio.h>
#endif

#include <linux/workqueue.h>

//#define CTA_DEBUG
#ifdef CTA_DEBUG
#undef pr_debug
#define pr_debug(fmt, ...) \
    printk(KERN_EMERG pr_fmt(fmt), ##__VA_ARGS__)
#endif

#ifdef CONFIG_PCMGPIOCONTROL
#define hotplug_path uevent_helper
static void cta_pcm_event_hook(struct work_struct *work);
#endif

static int dma_new_period(struct snd_pcm_substream *substream);


#if defined (CFG_IO_AUDIO_AMP_POWER)
static int gpio_status = -1;
#endif

int cta_spk_on(int enable)
{
#if defined (CFG_IO_AUDIO_AMP_POWER)
    if (!gpio_status) {
        if (enable)
            gpio_set_value(AUDIO_AMP_POWER, AUDIO_AMP_ACTIVE_LEVEL);
        else
            gpio_set_value(AUDIO_AMP_POWER, !AUDIO_AMP_ACTIVE_LEVEL);
    }
#endif
	return 0;
}


/*--- these hardware parameter sets snd_pcm_substream's rumtime->hw by snd_soc_set_runtime_hwparams ---*/
static const struct snd_pcm_hardware cta_pcm_hardware = {
    .info           = SNDRV_PCM_INFO_NONINTERLEAVED | SNDRV_PCM_INFO_MMAP, 
    .formats        = (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_BE | SNDRV_PCM_FMTBIT_S16_LE), //does not affect transfered data endian
    .rate_min       = 8000,
    .rate_max       = 96000,
    .channels_min   = 2,  
    .channels_max   = 2,
    .period_bytes_min   = CTA_MAX_BUFSIZE,
    .period_bytes_max   = CTA_MAX_BUFSIZE,
    .periods_min        = 1,
    .periods_max        = 128,
    .buffer_bytes_max   = CTA_MAX_BUFSIZE * CTA_BOTH_DESC_NUM,
    .fifo_size      = 32,
};

static inline void cta_desc_inc(unsigned int * no)
{
    (*no)++;
    if (CTA_DESC_NUM == (*no))
        (*no) = 0;
}

static inline unsigned int cta_desc_space(unsigned int curno, unsigned int nextno)
{
    if (curno < nextno)
        curno += CTA_DESC_NUM;
    return (curno - nextno + 1);
}

static int cta_pcm_poll(unsigned long arg)
{
    struct snd_pcm_substream *substream = (struct snd_pcm_substream *)arg;
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct cta_runtime_data *prtd = runtime->private_data;
    volatile struct cta_desc *des;

    if (prtd->active){
        des = ((struct cta_desc *)prtd->dma_desc_array) + prtd->cur;
        while(des->info & DES_OWN){
            if (!prtd->active)
                break;
#if 0 //measure tx underrun
            if ((des->info & CNT_MASK))
                printk(KERN_EMERG "TX_UNRUN_COUNT=%d\n", (des->info & CNT_MASK));
#endif
            if (cta_desc_space(prtd->cur, prtd->next) >= CTA_ONE_REQ)
            {
                prtd->periods += 1;
                prtd->periods %= runtime->periods;
                snd_pcm_period_elapsed(substream);
                /*--- avoid transfer the data which is already obsolete into dma descriptor again due to we can't stop transfer precisely ---*/ 
                if (!((runtime->control->appl_ptr <= prtd->hw_ptr) && (substream->runtime->status->state == SNDRV_PCM_STATE_DRAINING)))
                    dma_new_period(substream);
            }
            cta_desc_inc(&prtd->cur);
            des = ((struct cta_desc *)prtd->dma_desc_array) + prtd->cur;
        }
    }
    else{
            prtd->period = 0;
            prtd->cur = prtd->next;
    }
    mod_timer(&prtd->poll_timer, jiffies + 2);
    return 0;
}

static int dma_new_period(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct cta_runtime_data *prtd = runtime->private_data;
    volatile struct cta_desc *des;

    unsigned int dma_size;
    unsigned int offset;
    dma_addr_t mem_addr_0, mem_addr_1;
    int reqs = CTA_ONE_REQ;
    int ret = 0;
    int i;

    if (prtd->active) {

        /*-------------------------------------------------------------------------------------------
        since our PCM module is NONINTERLEAVED, we have to deal each channel's data by ourselves.
        The dma_buffer is no different from INTERLEAVED while source is mono,
        however, it would separate into 2 parts while source is stereo (maybe more parts when source is multiple channel).
        Each part in dma_buffer stands for each channel. 
        
        e.g.
        In mono
                +------------------------------------------+------------------------------------------+
                |                                 Channel 1's data                                    |
                +------------------------------------------+------------------------------------------+

        In stereo
                +------------------------------------------+------------------------------------------+
                |            Channel 1's data              |             Channel 2's data             |
                +------------------------------------------+------------------------------------------+
        
        Thus, we have to consider this situation when we fetch data then padding into dma descriptor.
        ---------------------------------------------------------------------------------------------*/

        dma_size = frames_to_bytes(runtime, runtime->period_size);

        /*--- rumtime->channels means the file format(mono or stereo) ---*/
        if (runtime->channels == 1){
            offset = dma_size * prtd->period;
            prtd->hw_ptr += bytes_to_frames(runtime, (reqs * (CTA_MAX_BUFSIZE / CTA_ONE_REQ) ));
        }
        else{
            //fix offset for fetching channel 2 (or more) data
            offset = (dma_size >> 1) * prtd->period  ;
            reqs = CTA_ONE_REQ >> 1;
            prtd->hw_ptr += bytes_to_frames(runtime, (reqs * (CTA_MAX_BUFSIZE / CTA_ONE_REQ) * 2));
        }

        pr_debug("%s: period (%d) out of (%d)\n", __func__,
                prtd->period, runtime->periods);
        pr_debug("period_size %d frames\n offset %d bytes\n",
                (unsigned int)runtime->period_size, offset);
        pr_debug("dma_size %d bytes\n", dma_size);

        snd_BUG_ON(dma_size > cta_pcm_hardware.period_bytes_max);
#if 0
        /*--- this section dumps DMA buffer ---*/
        pr_debug("%s: dma_addr = 0x%x, dma_area = 0x%x\n", __func__, (unsigned int) runtime->dma_addr, (unsigned int) runtime->dma_area);
        pr_debug("%s: mem_addr = %x, dma_size = %d\n", __func__, (unsigned int) mem_addr_0, dma_size);
        print_hex_dump(KERN_EMERG, "hexdump:", DUMP_PREFIX_OFFSET,
                    16, 1,
                    runtime->dma_area + offset, dma_size, false);
#endif
        /*--- set the channel's data in dma_buffer, channel 2 data is exactly the same while source is mono,
        add fix offset from channel 1's address otherwise. ---*/
        mem_addr_0 = (dma_addr_t)(runtime->dma_addr + offset);
        mem_addr_1 = (runtime->channels > 1) ? mem_addr_0 + (dma_addr_t)runtime->dma_bytes / 2 : mem_addr_0;

        /*--- Expand period size max CTA_ONE_REQ(16) times, so we could decrease polling rate max 16 times.
           And we pad max 16 descriptor at a time. ---*/
        for(i=0 ; i < reqs ; i++)
        {
            /*--padding channel 0's descriptor--*/
            des = ((struct cta_desc *)prtd->dma_desc_array) + prtd->next;

#if 0       /*--- check descriptor own bit, the own bit would not clear if underrun occur ---*/
            if(!(des->info & DES_OWN))
                printk(KERN_EMERG "desc is in use, continue ...\n");
#endif
            des->buf = (void *)mem_addr_0 + ((CTA_MAX_BUFSIZE / CTA_ONE_REQ) * i);
            /*--switch own bit to hw--*/
            des->info &= ~DES_OWN;

            /*--padding channel 1's descriptor--*/
            des = ((struct cta_desc *)prtd->dma_desc_array2) + prtd->next;
            des->buf = (void *)mem_addr_1 + ((CTA_MAX_BUFSIZE / CTA_ONE_REQ) * i);
            /*--switch own bit to hw--*/
            des->info &= ~DES_OWN;

            cta_desc_inc(&prtd->next);
        }

        prtd->period += 1;
        prtd->period %= runtime->periods;

        /*--- this section aims for underrun recovery ---*/
        {
            struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

            if (likely(cpu_dai->driver->ops->trigger)) {
                ret = cpu_dai->driver->ops->trigger(substream, SNDRV_PCM_TRIGGER_START, cpu_dai);
                if (unlikely(ret < 0)) {
                    printk(KERN_EMERG "re trigger failed\n");
                }
            }
        }
    }
    return ret;
}

static int audio_stop_dma(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct cta_runtime_data *prtd = runtime->private_data;

    pr_debug("%s\n", __func__);
    spin_lock_irq(&prtd->lock);
    prtd->active = 0;
    prtd->period = 0;
    prtd->periods = 0;
    spin_unlock_irq(&prtd->lock);

    return 0;
}

static int audio_wait_dma(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct cta_runtime_data *prtd = runtime->private_data;
    volatile struct cta_desc *des;
    struct cta_desc *dma_des;
    int cur, cnt, i;

    for (i=0; i<runtime->channels; i++) {
        if (i == 0) {
            des = dma_des = ((struct cta_desc *)prtd->dma_desc_array);
        }
        else {
            des = dma_des = ((struct cta_desc *)prtd->dma_desc_array2);
        }
        cur = 0;
        /* The worst case:
           CTA_MAX_BUFSIZE(320) * CTA_DESC_NUM(32) = 10240
           at PCM 8K 8bit data width
           10240 / 8K = 1.25(s) = 1250(ms)
           use 1500 > 1250          by nady
         */
        cnt = 1500;
        do {
            des = dma_des + cur;
            while (!(des->info & DES_OWN)) {
                if (cnt-- <= 0) {
                    printk(KERN_ERR "ERR: PCM occupies desc=0x%x\n", (unsigned int)des);
                    break;
                }
                mdelay(1);
            }
            cta_desc_inc(&cur);
        } while (cur != 0);
    }

    return 0;
}

static int dma_rx_new_period(struct snd_pcm_substream *substream){
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct cta_runtime_data *prtd = runtime->private_data;
    volatile struct cta_desc *des;

    unsigned int dma_size;
    unsigned int offset;
    dma_addr_t mem_addr_0, mem_addr_1;
    int ret = 0;
    int i;
    int one_req = CTA_ONE_REQ;

    if (prtd->active) {
        dma_size = frames_to_bytes(runtime, runtime->period_size);
        if (runtime->channels == 1){
            offset = dma_size * prtd->period;
        }
        else{
            offset = (dma_size >> 1) * prtd->period ;
            one_req = CTA_ONE_REQ >> 1;
        }
        snd_BUG_ON(dma_size > cta_pcm_hardware.period_bytes_max);

        mem_addr_0 = (dma_addr_t)(runtime->dma_addr + offset);
        mem_addr_1 = mem_addr_0 + (dma_addr_t)(runtime->dma_bytes >> 1);
        for(i=0 ; i < one_req ; ++i){
            des = ((struct cta_desc *)prtd->dma_desc_array) + prtd->next;

            if(unlikely(!(des->info & DES_OWN)))
                printk(KERN_ERR "desc is in use, continue ...\n");
            des->buf = (void *)mem_addr_0 + ((CTA_MAX_BUFSIZE / CTA_ONE_REQ) * i);
            des->info &= ~DES_OWN;

            /*--- deal channel 2's data only when recording stereo file --*/ 
            if (runtime->channels > 1){    
                des = ((struct cta_desc *)prtd->dma_desc_array2) + prtd->next;
                des->buf = (void *)mem_addr_1 + ((CTA_MAX_BUFSIZE / CTA_ONE_REQ) * i);
                des->info &= ~DES_OWN;
            }
            cta_desc_inc(&prtd->next);
        }

#if 0 
        pr_debug("%s: dma_addr = 0x%x, dma_area = 0x%x\n", __func__, (unsigned int) runtime->dma_addr, (unsigned int) runtime->dma_area);
        pr_debug("%s: mem_addr = %x, dma_size = %d\n", __func__, (unsigned int) mem_addr_0, dma_size);
        print_hex_dump(KERN_EMERG, "hexdump:", DUMP_PREFIX_OFFSET,
            16, 1,
            runtime->dma_area + offset, dma_size, false);
#endif

        prtd->period += 1;
        prtd->period %= runtime->periods;

        /*--- this section aims for overrun recovery ---*/
        {
            struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

            if (cpu_dai->driver->ops->trigger) {
                ret = cpu_dai->driver->ops->trigger(substream, SNDRV_PCM_TRIGGER_START, cpu_dai);
                if (ret < 0) {
                    printk(KERN_EMERG "re trigger failed\n");
                }
            }
        }
    }
    return ret;
}

static irqreturn_t rx_interrupt(int irq, void *dev){
    struct snd_pcm_substream *substream = (struct snd_pcm_substream *) dev;
//    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct cta_runtime_data *prtd = runtime->private_data;
    volatile struct cta_desc *des;

//    pr_debug("%s:%d name:%s, type:%d\n", __func__, __LINE__, dma_data->name, dma_data->transfer_type)m;

    if (prtd->active){
        des = ((struct cta_desc *)prtd->dma_desc_array) + prtd->cur;
        while(des->info & DES_OWN){
            if (!prtd->active)
                break;
            if (cta_desc_space(prtd->cur, prtd->next) >= CTA_ONE_REQ)
            {
                prtd->periods += 1;
                prtd->periods %= runtime->periods;
                snd_pcm_period_elapsed(substream);
                dma_rx_new_period(substream);
            }
            cta_desc_inc(&prtd->cur);
            des = ((struct cta_desc *)prtd->dma_desc_array) + prtd->cur;
        }
    }
    return IRQ_HANDLED;
}

static int cta_dma_setup_handlers(int channel, irq_handler_t irq_handler, void *data)
{
    int ret;
    ret = request_irq(IRQ_I2S, irq_handler, IRQF_SHARED, "AI_RX", (void *) data);
    if (unlikely(ret)) {
        printk(KERN_CRIT "Can't register IRQ %d for DMA channel %d\n", IRQ_I2S, channel);
        return ret;
    }
    return ret;
}

static void cta_desc_init(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct cta_runtime_data *prtd = runtime->private_data;
    volatile struct cta_desc *des = (struct cta_desc *)prtd->dma_desc_array;
    int dir = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? 0: 1;
    int ch, no;

    pr_debug("%s:%d , dir=%d\n", __func__, __LINE__,  dir);

    /* init channel1 descriptors ptr base */
    prtd->dma_desc_array2 = prtd->dma_desc_array + CTA_DESC_NUM;
    prtd->dma_desc_array_phys2 = prtd->dma_desc_array_phys + (CTA_DESC_BUFSIZE / 2);

    for (ch = CTA_CH0; ch < NUM_CH; ch++)
    {
        des = (struct cta_desc *)((ch == CTA_CH1) ? prtd->dma_desc_array2 : prtd->dma_desc_array);

        /* init descriptors */         
        for (no = CTA_DESC_NUM; no > 0; no--, des++)
        {
            /* des own bit setting */      
            des->info = DES_OWN;           
            /* des buf setting */          
            des->buf = 0;          
        }
        /* last descriptor is the end of ring descripotr */
        des--;
        des->info |= DES_EOR;
    }

    pr_debug("%s:%d dma_desc_array=0x%x, dma_desc_array_phys=0x%x\n", __func__, __LINE__,
        (unsigned int)prtd->dma_desc_array, (unsigned int)prtd->dma_desc_array_phys);
    pr_debug("%s:%d dma_desc_array2=0x%x, dma_desc_array_phys2=0x%x\n", __func__, __LINE__,
        (unsigned int)prtd->dma_desc_array2, (unsigned int)prtd->dma_desc_array_phys2);
}

static int cta_pcm_open(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct cta_runtime_data *prtd;
    int ret;

    pr_debug("%s:%d\n", __func__, __LINE__);

    snd_soc_set_runtime_hwparams(substream, &cta_pcm_hardware);

    ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
    if (unlikely(ret < 0))
        goto out;

    ret = -ENOMEM;
    prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
    if (unlikely(!prtd))
        goto out;

    ret = -ENODEV;
    pr_debug("alloc_coherent size: %d\n", CTA_DESC_BUFSIZE);
    prtd->dma_desc_array =   //dma_desc_array2 speace includes in dma_desc_array, no need to allocate
    dma_alloc_coherent(substream->pcm->card->dev, CTA_DESC_BUFSIZE,
                        &prtd->dma_desc_array_phys, GFP_KERNEL);
    if (unlikely(!prtd->dma_desc_array))
        goto err1;

    pr_debug("%s:%d dma_desc_array=0x%x, dma_desc_array_phys=0x%x\n", __func__, __LINE__, 
        (unsigned int)prtd->dma_desc_array, (unsigned int)prtd->dma_desc_array_phys);

    if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
    {
        pr_debug("%s: Setting rx dma callback function : cpu_dai id: %d\n", __func__, rtd->cpu_dai->id);
        ret = cta_dma_setup_handlers(rtd->cpu_dai->id, rx_interrupt, (void *)substream);
        if (unlikely(ret < 0)) {
            printk(KERN_ERR "ERR: Error %d setting dma callback function\n", ret);
            goto err1;
        }
        prtd->irq_requested = 1;
    }

    runtime->private_data = prtd;
    prtd->codec_init = 0;       //init codec state flag for waiting codec CJC8988
    cta_desc_init(substream);
    spin_lock_init(&prtd->lock);

    /* initial timer interrupt for playback dma polling*/
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
        init_timer(&prtd->poll_timer);
        prtd->poll_timer.function = (void *)cta_pcm_poll;
        prtd->poll_timer.data = (unsigned long)substream;
        prtd->poll_timer.expires = jiffies + 2;
        add_timer(&prtd->poll_timer);
    }

#ifdef CONFIG_PCMGPIOCONTROL
    INIT_DELAYED_WORK(&prtd->pcm_event.gpioctrl, cta_pcm_event_hook);
#endif
    return 0;

 err1:
    kfree(prtd);
 out:
    return ret;
}

static int cta_pcm_close(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct cta_runtime_data *prtd = runtime->private_data;

    pr_debug("%s:%d\n", __func__, __LINE__);
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
        del_timer_sync(&prtd->poll_timer);
    }
    else {
        if (likely(prtd->irq_requested)){
            pr_debug("try to free irq %d\n", IRQ_I2S);
            free_irq(IRQ_I2S, (void *)substream);
            prtd->irq_requested = 0;
        }
    }

    /*--- dma_desc_array2 speace includes in dma_desc_array, no need to release ---*/
    dma_free_coherent(substream->pcm->card->dev, CTA_DESC_BUFSIZE,
                  prtd->dma_desc_array, prtd->dma_desc_array_phys);
    kfree(prtd);
    return 0;
}

static int cta_pcm_hw_params(struct snd_pcm_substream *substream,
    struct snd_pcm_hw_params *params)
{
    struct snd_pcm_runtime *runtime = substream->runtime;

    pr_debug("%s:%d buffer_bytes=%d\n", __func__, __LINE__, params_buffer_bytes(params));

    snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
    runtime->dma_bytes = params_buffer_bytes(params);


    pr_debug("%s: snd_cta_audio_hw_params runtime->dma_addr 0x(%x)\n",
        __func__, (unsigned int)runtime->dma_addr);
    pr_debug("%s: snd_cta_audio_hw_params runtime->dma_area 0x(%x)\n",
        __func__, (unsigned int)runtime->dma_area);
    pr_debug("%s: snd_cta_audio_hw_params runtime->dma_bytes 0x(%x)\n",
        __func__, (unsigned int)runtime->dma_bytes);
  
    return 0;  
}

static int cta_pcm_hw_free(struct snd_pcm_substream *substream)
{
    pr_debug("%s:%d\n", __func__, __LINE__);

    snd_pcm_set_runtime_buffer(substream, NULL);

    return 0;
}

static int cta_pcm_prepare(struct snd_pcm_substream *substream)
{
    struct cta_runtime_data *prtd = substream->runtime->private_data;

    prtd->period = 0;
    prtd->periods = 0;
    prtd->cur = 0;
    prtd->next = 0;
    return 0;
}

#ifdef CONFIG_PCMGPIOCONTROL
static void cta_pcm_event_hook(struct work_struct *work)
{
    struct cta_pcm_event *pcm_event = (struct cta_pcm_event *)container_of(work, struct cta_pcm_event, gpioctrl.work);
    struct kobj_uevent_env *env = kzalloc(sizeof(struct kobj_uevent_env), GFP_KERNEL);
    char *argv[3] = {
        hotplug_path,
        "pcm",
        NULL
    };

    if (!env) {
        pr_debug("[%s:%d]pcm ENOMEM\n",__func__,__LINE__);
        return;
    }

    if (add_uevent_var(env, "ACTION=%s", (pcm_event->action) ? "stop" : "start")) {
        pr_debug("[%s:%d]pcm ENOMEM\n",__func__,__LINE__);
        kfree(env);
        return;
    }

    call_usermodehelper(argv[0], argv, env->envp, UMH_WAIT_EXEC);
    kfree(env);
}
#endif

static int cta_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct cta_runtime_data *prtd = runtime->private_data;
    int ret = 0;

    pr_debug("%s:%d cmd=%d\n", __func__, __LINE__, cmd);
    pr_debug("%s:%d runtime->channels=%d\n", __func__, __LINE__, runtime->channels);
    spin_lock_irq(&prtd->lock);
    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
        /* requested stream startup */
        if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
#ifdef CONFIG_PCMGPIOCONTROL
            prtd->pcm_event.action = 0;
            schedule_delayed_work(&prtd->pcm_event.gpioctrl, 1); 
            pr_debug("%s:%d requested stream startup \n", __func__, __LINE__);
#endif
#if defined (CFG_IO_AUDIO_AMP_POWER)
            if (!gpio_status) {
                pr_debug("%s:%d GPIO%d Audio AMP Power On\n", __func__, __LINE__, AUDIO_AMP_POWER);
                gpio_set_value(AUDIO_AMP_POWER, AUDIO_AMP_ACTIVE_LEVEL);
            }
#endif
            prtd->active = 1;
            prtd->hw_ptr = runtime->status->hw_ptr;
            dma_new_period(substream);
            dma_new_period(substream);
            if (substream->runtime->channels > 1)
                dma_new_period(substream);
            /* prepare second period ahead beacause polling method can't ensure second period is ready */
        }
        else{
            prtd->active = 1;
            dma_rx_new_period(substream);
            dma_rx_new_period(substream);
            if (substream->runtime->channels > 1)
                dma_rx_new_period(substream); 
            /* although rx has DMA interrupt, also need rx_new_period twice make record smooth somehow */
        }
        break;
    case SNDRV_PCM_TRIGGER_STOP:
        {
            /* requested stream shutdown */
            ret = audio_stop_dma(substream);
#ifdef CONFIG_PCMGPIOCONTROL
            pr_debug("%s:%d requested stream stutdown \n", __func__, __LINE__);
            prtd->pcm_event.action = 1;
            schedule_delayed_work(&prtd->pcm_event.gpioctrl, 1); 
#endif
#if defined (CFG_IO_AUDIO_AMP_POWER)
            if (!gpio_status) {
                pr_debug("%s:%d GPIO%d Audio AMP Power Off\n", __func__, __LINE__, AUDIO_AMP_POWER);
                gpio_set_value(AUDIO_AMP_POWER, !AUDIO_AMP_ACTIVE_LEVEL);
            }
#endif
            ret = audio_wait_dma(substream);
            break;
        }
    default:
        ret = -EINVAL;
    }
    spin_unlock_irq(&prtd->lock);
    return ret;
}

static snd_pcm_uframes_t
cta_pcm_pointer(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct cta_runtime_data *prtd = runtime->private_data;
    unsigned int offset = 0;
/*  --- cta_pcm_pointer called by snd_pcm_period_elapsed()
        periods and period are made for different purpose ---*/ 

    offset = (runtime->period_size * (prtd->periods));
    if (offset >= runtime->buffer_size)
        offset = 0;
#if 0
    printk(KERN_EMERG "%s:%d runtime:periods=%d period_size=%d buffer_size=%d \n", __func__, __LINE__, 
        runtime->periods, 
        (int)runtime->period_size, 
        (int)runtime->buffer_size);

    printk(KERN_EMERG "%s:%d prtd:periods=%d period=%d \n", __func__, __LINE__, 
        prtd->periods, 
        prtd->period);

    printk(KERN_EMERG "%s:%d pointer offset %d\n", __func__, __LINE__, offset);
#endif

    return offset;
}

static int cta_pcm_mmap(struct snd_pcm_substream *substream,
    struct vm_area_struct *vma)
{
/*--- enable MMAP chagnes the behavior which alsa lib wirtes dma buffer.
use MMAP could improve this driver's performance ---*/

#if 1
	return remap_pfn_range(vma, vma->vm_start,
		       substream->dma_buffer.addr >> PAGE_SHIFT,
		       vma->vm_end - vma->vm_start, vma->vm_page_prot);
#elif 1
    struct snd_pcm_runtime *runtime = substream->runtime;
    unsigned long off = vma->vm_pgoff;

#define	__phys_to_pfn(paddr)	((paddr) >> PAGE_SHIFT)
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
    return remap_pfn_range(vma, vma->vm_start,
        __phys_to_pfn(runtime->dma_addr) + off,
        vma->vm_end - vma->vm_start, vma->vm_page_prot);
#else
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct page *pg;
    unsigned long addr;
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
    addr = plat_dma_addr_to_phys(substream->pcm->card->dev, runtime->dma_addr);
    runtime->dma_area = phys_to_virt(addr);
    pg = virt_to_page(runtime->dma_area);

    return remap_pfn_range(vma, vma->vm_start, page_to_pfn(pg) + vma->vm_pgoff,
                    runtime->dma_bytes, vma->vm_page_prot);
#endif
}

static struct snd_pcm_ops cta_pcm_ops = {
    .open       = cta_pcm_open,
    .close      = cta_pcm_close,
    .ioctl      = snd_pcm_lib_ioctl,
    .hw_params  = cta_pcm_hw_params,
    .hw_free    = cta_pcm_hw_free,
    .prepare    = cta_pcm_prepare,
    .trigger    = cta_pcm_trigger,
    .pointer    = cta_pcm_pointer,
    .mmap       = cta_pcm_mmap,
};

static u64 cta_pcm_dmamask = DMA_BIT_MASK(32);

static int cta_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
    struct snd_pcm_substream *substream = pcm->streams[stream].substream;
    struct snd_dma_buffer *buf = &substream->dma_buffer;
    size_t size = cta_pcm_hardware.buffer_bytes_max;
    buf->dev.type = SNDRV_DMA_TYPE_DEV;
    buf->dev.dev = pcm->card->dev;
    buf->private_data = NULL;
    buf->area = dma_alloc_coherent(pcm->card->dev, size,
                       &buf->addr, GFP_KERNEL);
    pr_debug("%s:%d, substream->dma_buffer size=%d\n", __func__, __LINE__, size);
    pr_debug("%s:%d, buf->area=0x%x, buf->addr=0x%x\n", __func__, __LINE__, (unsigned int)buf->area, (unsigned int)buf->addr);
    if (unlikely(!buf->area))
        return -ENOMEM;
    buf->bytes = size;
    return 0;
}

static void cta_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
    struct snd_pcm_substream *substream;
    struct snd_dma_buffer *buf;
    int stream;

#if defined (CFG_IO_AUDIO_AMP_POWER)
    if (!gpio_status) {
	    gpio_set_value(AUDIO_AMP_POWER, !AUDIO_AMP_ACTIVE_LEVEL);
        gpio_free(AUDIO_AMP_POWER);
        gpio_status = -1;
    }
#endif

    for (stream = 0; stream < 2; stream++) { //release PLAYBACK and CAPTURE stream
        substream = pcm->streams[stream].substream;
        if (!substream)
            continue;
        buf = &substream->dma_buffer;
        if (!buf->area)
            continue;
        dma_free_coherent(pcm->card->dev, buf->bytes,
                      buf->area, buf->addr);
        buf->area = NULL;
    }
}

static int cta_soc_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
    struct snd_card *card = rtd->card->snd_card;
    struct snd_pcm *pcm = rtd->pcm;
    int ret = 0;

    if (!card->dev->dma_mask)
        card->dev->dma_mask = &cta_pcm_dmamask;
    if (!card->dev->coherent_dma_mask)
        card->dev->coherent_dma_mask = DMA_BIT_MASK(32);
    
    if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
        ret = cta_pcm_preallocate_dma_buffer(pcm,
            SNDRV_PCM_STREAM_PLAYBACK);
        if (unlikely(ret))
            goto out;
    }

    if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
        ret = cta_pcm_preallocate_dma_buffer(pcm,
            SNDRV_PCM_STREAM_CAPTURE);
        if (unlikely(ret))
            goto out;
    }
#if defined (CFG_IO_AUDIO_AMP_POWER)
    if (!(gpio_status = gpio_request(AUDIO_AMP_POWER, "cta_amp_en")))
        gpio_direction_output(AUDIO_AMP_POWER, !AUDIO_AMP_ACTIVE_LEVEL);
    else
        printk(KERN_ERR "ERR: request gpio[%d] for Audio AMP Power\n", AUDIO_AMP_POWER);
#endif
out:
    return ret;
}

static struct snd_soc_platform_driver cheetah_soc_platform = {
    .ops        = &cta_pcm_ops,
    .pcm_new    = cta_soc_pcm_new,
    .pcm_free   = cta_pcm_free_dma_buffers,
};

static int __devinit cheetah_soc_platform_probe(struct platform_device *pdev)
{
    return snd_soc_register_platform(&pdev->dev, &cheetah_soc_platform);
}

static int __devexit cheetah_soc_platform_remove(struct platform_device *pdev)
{
    snd_soc_unregister_platform(&pdev->dev);
    return 0;
}

static struct platform_driver cheetah_pcm_driver = {
    .driver = {
            .name = "cta-pcm-audio",
            .owner = THIS_MODULE,
    },
    .probe = cheetah_soc_platform_probe,
    .remove = __devexit_p(cheetah_soc_platform_remove),
};

module_platform_driver(cheetah_pcm_driver);

MODULE_AUTHOR("Nady Chen");
MODULE_DESCRIPTION("Montage Cheetah PCM DMA module");
MODULE_LICENSE("GPL");
