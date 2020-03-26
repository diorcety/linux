 /*
 * Copyright (c) 2012   Montage Inc.,  All rights reserved.
 * 
 * madc monitor
 */
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/synclink.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/idb.h>
#include "cta_madc.h"

#define hotplug_path	uevent_helper
#define VOLUME		"volume"
#define KEYPAD		"keypad"

#define DELAY		10
#define TICK_THD	0x50

#define FIXED 1
#define RANGE 2

#define NONE	0
#define INIT	1
#define START	2

#define STYPE_NONE        0 
#define STYPE_SHORT_TOUCH 1 
#define STYPE_LONG_TOUCH  2 

#define DRV_NAME "cheetah-madc"

#define HW_MAP(_name, _handle, _val)   \
{   \
    .name       = (_name),      \
	.handler    = (_handle),    \
    .val		= (_val),    \
}

extern int get_args (const char *string, char *argvs[]);
extern struct file *openfile(char *path,int flag,int mode);
extern int readfile(struct file *fp, char *buf, int readlen);
extern int closefile(struct file *fp);

static void madc_range_assign(struct madc_type *madct, char *val, unsigned num);
static void madc_func(struct madc_mon *madc, int argc, char *argv[]);

mm_segment_t fs;
static unsigned int DISABLE = 1;
static struct madc_setup madc_val[] = {
	HW_MAP("madc_val", madc_range_assign, "%d"),
};

#ifdef CONFIG_CHEETAH_MADC_USE_INPUT_DEV
static int cheeatch_input_keycode_map[] = {
    CTA_KEY_A,
    CTA_KEY_B,
    CTA_KEY_C,
    CTA_KEY_D,
    CTA_KEY_E,
    CTA_KEY_F,
    CTA_KEY_G,
    CTA_KEY_H,
    CTA_KEY_I,
    CTA_KEY_J,
    CTA_KEY_K,
    CTA_KEY_L,
    CTA_KEY_M,
    CTA_KEY_N,
    CTA_KEY_O,
    CTA_KEY_P,
    CTA_KEY_Q,
    CTA_KEY_R,
    CTA_KEY_S,
    CTA_KEY_T,
    CTA_KEY_U,
    CTA_KEY_V,
    CTA_KEY_W,
    CTA_KEY_X,
    CTA_KEY_Y,
    CTA_KEY_Z,
};
#endif

static void bsort(int *arr, int len)
{
	int swap=1;
	int cmptimes=len-1;
	int i=0;
	int tmp=0;
	
	while(swap)
	{
		swap=0;
		for(i=0;i<cmptimes;i++)
		{
			if(arr[i]>arr[i+1])
			{
				tmp=arr[i];
				arr[i]=arr[i+1];
				arr[i+1]=tmp;
				swap=1;
			}
		}
		cmptimes--;
	}
}

static unsigned int chk_range(struct madc_type *madct)
{
	unsigned int idx;

	for(idx=0; idx < MAXKEYNUM; idx++) {
		struct madc_range *range = (struct madc_range *)&madct->range[idx];
		if(madct->oval <= range->max && madct->oval >= range->min)
			return idx+1;
	}

	return 0;
}

static void kernelenv_t(int init)
{ 
    if (init) {
    	fs = get_fs(); 
    	set_fs(KERNEL_DS);
    }
    else {
	    set_fs(fs); 
    }
} 

static void get_battery_info(int *batt)
{
    char buf[1048] = {0};
    char sbuf[64] = {0};
    char mval[4] ={0};
	struct file *fp = NULL;
	char *token = NULL;
	char *p = NULL;
	char *s = NULL;
	char *pend = NULL;
    char *vp; 
	int val = 0;
	int total = 0;
	int i = 0;

    kernelenv_t(1);

    fp = openfile("/proc/bootvars", O_CREAT | O_RDONLY, 0); 
    if(fp == NULL)
        goto done;
        
    readfile(fp, buf, sizeof(buf));
        
    p = strstr(buf, "madc_val1");
    if(p != NULL)
	{
    	s = sbuf;
    	p = strchr(p, '=');
    	pend = strchr(p, 0xa);
    	p++;
    	memcpy(sbuf, p, pend-p);
    	sbuf[pend-p+1]='\0';

    	if(strchr(s, ',')) 
    	{   
        	while ((token = strsep(&s, ",")) != NULL)
        	{
            	if((vp = strchr(token, '=')) != 0)
            	{
                	vp++;
                	sscanf(vp, "%d", &val);
                	mval[i++] = val;
                	total++;
            	}
        	}
            if (batt) {
    			batt[0] = mval[0];
	    		batt[1] = mval[3];
            }
    	} 
    }   

done:   
    closefile(fp);
    kernelenv_t(0);
}

static void madc_range_assign(struct madc_type *madct, char *val, unsigned num)
{
    char default_val[MAXKEYNUM] = {3,9,12,16,20,23,26,30};
	unsigned int idx;
	char *ptr;
	
	//default val
	if(val == NULL)
		ptr = default_val;	
	else
		ptr = val;

	for(idx=0; idx < num; idx++) {
		struct madc_range *range = (struct madc_range *)&madct->range[idx];
		if(idx == 0) {
			range->max = (ptr[idx+1]+ptr[idx])/2;
			range->min = 1;
		}
		else if (idx == num -1) {
			range->max = 31;
			range->min = (ptr[idx-1]+ptr[idx])/2 + 1;
		}
		else {
			range->max = (ptr[idx+1]+ptr[idx])/2;
			range->min = (ptr[idx-1]+ptr[idx])/2 + 1;
		}
	}
		
	return;
}

static void parse_parm(char *p, unsigned int idx, struct madc_type *madct)
{
	char *token;
	unsigned int i=0,total=0,val;
	char buf[512] = {0};
	char *s;
	char *pend=NULL;
	char *vp;
	char measure_val[8] ={0};

	s = buf;
	p = strchr(p, '=');
	pend = strchr(p, 0xa);
	p += 1;
	memcpy(buf, p, pend-p);
	buf[pend-p+1]='\0';

	if(strlen(buf)==0) {
		madc_range_assign(madct, NULL, MAXKEYNUM); //default val
		return;
	}

	if(strchr(s, ',')) 
	{
		while ((token = strsep(&s, ",")) != NULL)
		{
			if((vp = strchr(token, '=')) != 0)
			{
				vp++;
				sscanf(vp, madc_val[idx].val, &val);
				measure_val[i++] = val;
				total++;
			}
		}
		madc_val[idx].handler(madct, measure_val,total);
	}
}

static void madc_range_init(struct madc_type *madct, unsigned int chan)
{
	struct file *fp =NULL;
	char *p=NULL;
	unsigned int i;
	char cmp[16];

	char read_buf[2048] = "";
	kernelenv_t(1);
	fp = openfile("/proc/bootvars", O_CREAT | O_RDONLY, 0);
	
	if(fp == NULL)
		goto done;
	
	readfile(fp, read_buf, sizeof(read_buf));
	
	for(i=0; i < ARRAY_SIZE(madc_val); i++) {
		sprintf(cmp, "%s%d",madc_val[i].name, chan);
		p =	strstr(read_buf, cmp);
		if(p != NULL) {
			parse_parm(p, i, madct);
		}
		else {
			madc_range_assign(madct, NULL, MAXKEYNUM); //default val
		}
	}
done:	
	closefile(fp);
	kernelenv_t(0);
	return;
}

#ifdef CONFIG_CHEETAH_MADC_USE_INPUT_DEV
static void input_send_key(struct input_dev *input, int key, int flag)
{
    input_report_key(input, key, 1);
    input_report_key(input, key, 0);
//    input_sync(input);
}

static void input_send_abs(struct input_dev *input, int key, int flag)
{
    input_report_abs(input, ABS_VOLUME, key);
    input_sync(input);
}

static int input_sort_key(struct madc_type *madct, int key, int flag)
{
	struct input_dev *input = madct->input;
    int keycode;
    int error = -ENODEV;
	if(input) {
		if (test_bit(EV_KEY, input->evbit)) {
            key += madct->misc;
            if (flag == STYPE_LONG_TOUCH) {
                key += MAXKEYNUM;
            }
            error = input_get_keycode(input, key, &keycode);
            if (!error) {
    			input_send_key(input, keycode, 0);
            }
		}
		else if (test_bit(EV_ABS, input->evbit)) {
            input_send_abs(input, key, flag);
		}
	}
    return error;
}
static void madc_event(struct work_struct *work)
{
// do nothing
    return;
}
#else
static void madc_event(struct work_struct *work)
{
	char *argv[5];
	char *envp[4];
	char val[4];
	struct madc_type *madct = (struct madc_type *)container_of(work, struct madc_type, queue_work.work);
	unsigned int value;

	value = madct->oval;

	argv[0] = hotplug_path;
	if(madct->type == FIXED)
		argv[1] = VOLUME;
	else if (madct->type == RANGE)
		argv[1] = KEYPAD;

	argv[2] = &val[0];
	sprintf(argv[2], "%d",value);

	if(madct->state == INIT)
		argv[3] = "1";
	else
		argv[3] = "0";
	
	argv[4] = NULL;

	/*env*/
	envp[0] = kmalloc(sizeof("CLICK=") + 4, GFP_ATOMIC);
	sprintf(envp[0], "CLICK=%d", madct->stype);
	
	envp[1] = kmalloc(sizeof("CHAN=") + 4, GFP_ATOMIC);
	sprintf(envp[1], "CHAN=%d", madct->chan);

	if(madct->type == RANGE) {
		envp[2] = kmalloc(sizeof("KEY=") + 4, GFP_ATOMIC);
		sprintf(envp[2], "KEY=%d", madct->key);
	}
	else
		envp[2] = NULL;

	envp[3]	= NULL; 

	call_usermodehelper(argv[0], argv, envp, UMH_WAIT_EXEC);

	kfree(envp[0]);
	kfree(envp[1]);
	
	if(madct->type == RANGE)
		kfree(envp[2]);

	return;
}
#endif

static void madc_func(struct madc_mon *madc, int argc, char *argv[])
{
	if (!strcmp(argv[0], "start"))
		DISABLE = 0;
	else if (!strcmp(argv[0], "stop"))
		DISABLE = 1;
	else if (!strcmp(argv[0], "reset")) {
		struct madc_type *madct;
		int i;
		for(i=0; i<MAX_CHAN_NUM; i++) {
			madct = &madc->madct[i];
			madct->type = 0;
		}
	}	
	else if (!strcmp(argv[0], "set")) {
		struct madc_type *madct;
		int chan, type;
		if(argc != 3) {
			return;
		}
		sscanf(argv[1],"%d",&chan);
		sscanf(argv[2],"%d",&type);
		madct = &madc->madct[chan];
		madct->type = type;
		if(madct->type == RANGE)
			madc_range_init(madct, chan);
	}
	else if (!strcmp(argv[0], "total")) {
		int total;
		madc->allchan=0;
		sscanf(argv[1],"%d",&total);
		madc->allchan = total;
	}
	else if (!strcmp(argv[0], "poll")) {
		int poll;
		if(argc != 2) {
			return;
		}
		sscanf(argv[1],"%d",&poll);
		madc->polling = poll;	
	}
	else if (!strcmp(argv[0], "show")) {
		int i;
		struct madc_type *madct;
		idb_print("Total channel:%d\n",madc->allchan);
		for(i=0; i<MAX_CHAN_NUM; i++) {
			madct = &madc->madct[i];
			idb_print("chan:%d type:%d oval:%d\n",i, madct->type, madct->oval);
		}
	}
	else if (!strcmp(argv[0], "state")) {
		int i;
		struct madc_type *madct;
		for(i=0; i<MAX_CHAN_NUM; i++) {
			madct = &madc->madct[i];
			if(madct->type == FIXED)
				idb_print("state:%d\n",madct->state);
		}
	}
	else if (!strcmp(argv[0], "dbg"))
		madc->dbg=!madc->dbg;
}

static int proc_madc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	struct madc_mon *madc = (struct madc_mon *)data;
	struct madc_type *madct;
	char *p = page;
	int len;
	int i;

	p += sprintf(p, "Monitor madc:[%s]\n", DISABLE ? "Disable" : "Enable");
	p += sprintf(p, "Total channel:%d\n", madc->allchan);
	for(i=0; i<MAX_CHAN_NUM; i++) {
		madct = &madc->madct[i];
		p += sprintf(p, "chan:%d type:%d oval:%d %s\n", i, madct->type, madct->oval, (madct->valid ? "valid" : "(invalid)"));
	}

	len = (p - page) - off;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}

static int proc_madc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	struct madc_mon *madc = (struct madc_mon *)data;
    char buf[300];
    int argc ;
    char * argv[8] ;

    if (count > 0 && count < 299) {
        if (copy_from_user(buf, buffer, count))
            return -EFAULT;
        buf[count-1]='\0';
        argc = get_args( (const char *)buf , argv );
        madc_func(madc, argc, argv);
    }
    return count;
    
}
 
static int proc_battery_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    static int batt[2] = {6, 26};
	struct madc_mon *madc = (struct madc_mon *)data;
	struct madc_type *madct = &madc->madct[1];
	int len = 0;

	get_battery_info(batt);
	len += sprintf(page+len, "%d %d %d\n", madct->oval, batt[0], batt[1]);

	return len;
}

#ifdef CONFIG_CHEETAH_MADC_USE_INPUT_DEV
static void madc_monitor(struct work_struct *work)
{
	unsigned int val = 0;
	unsigned int chan;
	struct madc_mon *madc = (struct madc_mon *)container_of(work, struct madc_mon, madc_work.work);
	struct madc_type *madct;
	int delay = 0;

	int i= 0;					
	int sampleTime= 1;			
	int sampleValues[4];		
	int aveVal= 0;					

	if(DISABLE)
		goto done;
	
	for(chan=0; chan<MAX_CHAN_NUM; chan++) {
		madct = &madc->madct[chan];
		
		if((madct->type == NONE) || (!madct->valid))
			continue;
		
		sampleTime = (madct->type == RANGE) ? 4 : 1 ;			
		
		// Sample more value.
		for(i=0;i<sampleTime;i++)
		{	
			ANAREG_UPDATE(MADCCTL, BIT0, BIT0);
			udelay(32);
			ANAREG_UPDATE(MADCCTL, (chan << 4), (BIT4 | BIT0));
			udelay(32);
			GPREG(MADCSAMP) = (TICK_THD << 16) | BIT0;
			while((GPREG(MADCSAMP) & BIT0))
				;
			val = (GPREG(MADCSAMP) >> 8) & 0xFF;
	
			if(val != 0 && chan == 0)
			{	
				sampleValues[i]= val; 	
			}
		}

		if(val != 0 && chan == 0)
		{
			bsort(sampleValues,4);	
			aveVal=(sampleValues[1] + sampleValues[2] )/2;
			val = aveVal; 
		}

		/*Depending madc type to define action*/
		if(madct->type == FIXED) {
			if(val == madct->oval) {
				if((madct->state != NONE)) {
					if(delayed_work_pending(&madct->queue_work)) {
						continue;
					}
					cancel_delayed_work(&madct->queue_work);
					madct->state = NONE;
					input_sort_key(madct, madct->oval, 0);
					schedule_delayed_work(&madct->queue_work, 0);
				}
				continue;
			}
			
			if((val != 0) && ((val-madct->oval==-1) || (val-madct->oval==1)))
				continue;
			
			if(madct->state == NONE) {
				madct->state = INIT;
				input_sort_key(madct, madct->oval, 0);
				schedule_delayed_work(&madct->queue_work, 0);
				madct->last = jiffies;
				continue;
			}
			
			if((long)jiffies - (long)madct->last < 50) {
				continue;
			}
			
			delay = 0;
			madct->state = START;
			madct->oval = val;
			madct->last = jiffies;
		}
		else if(madct->type == RANGE) {
			if(val == 0) {
                if (madct->state == NONE)
                    continue;
                else
                    madct->state = NONE;
			}
			else {
				if(madct->state == NONE) {
					madct->state = INIT;
					madct->oval = val;
					madct->last = jiffies;
				}
			}
			if((long)jiffies - (long)madct->last > 1) {
                madct->last = jiffies;
                input_sort_key(madct, chk_range(madct), STYPE_SHORT_TOUCH);
			}
			delay = 0;

			madct->key = chk_range(madct);
			if(madct->key == 0)
				continue;
			
			if(madc->dbg)
				idb_print("!!! KEY:%d chan:%d\n",madct->key,madct->chan);
		}

		if(delayed_work_pending(&madct->queue_work)) {
			continue;
		}
		cancel_delayed_work(&madct->queue_work);
		schedule_delayed_work(&madct->queue_work, delay);
	}

done:
	schedule_delayed_work(&madc->madc_work, madc->polling);
}
#else
static void madc_monitor(struct work_struct *work)
{
	unsigned int val = 0;
	unsigned int chan;
	struct madc_mon *madc = (struct madc_mon *)container_of(work, struct madc_mon, madc_work.work);
	struct madc_type *madct;
	int delay = 0;

	int i= 0;					
	int sampleTime= 1;			
	int sampleValues[4];		
	int aveVal= 0;					

	if(DISABLE)
		goto done;
	
	for(chan=0; chan<MAX_CHAN_NUM; chan++) {
		madct = &madc->madct[chan];
		
		if(madct->type == NONE)
			continue;
		
		sampleTime = (madct->type == RANGE) ? 4 : 1 ;			
		
		// Sample more value.
		for(i=0;i<sampleTime;i++)
		{	
			ANAREG_UPDATE(MADCCTL, BIT0, BIT0);
			udelay(32);
			ANAREG_UPDATE(MADCCTL, (chan << 4), (BIT4 | BIT0));
			udelay(32);
			GPREG(MADCSAMP) = (TICK_THD << 16) | BIT0;
			while((GPREG(MADCSAMP) & BIT0))
				;
			val = (GPREG(MADCSAMP) >> 8) & 0xFF;
	
			if(val != 0 && chan == 0)
			{	
				sampleValues[i]= val; 	
			}
		}

		if(val != 0 && chan == 0)
		{
			bsort(sampleValues,4);	
			aveVal=(sampleValues[1] + sampleValues[2] )/2;
			val = aveVal; 
		}

		/*Depending madc type to define action*/
		if(madct->type == FIXED) {
			if(val == madct->oval) {
				if((madct->state != NONE)) {
					if(delayed_work_pending(&madct->queue_work)) {
						continue;
					}
					cancel_delayed_work(&madct->queue_work);
					madct->state = NONE;
					schedule_delayed_work(&madct->queue_work, 0);
				}
				continue;
			}
			
			if((val != 0) && ((val-madct->oval==-1) || (val-madct->oval==1)))
				continue;
			
			if(madct->state == NONE) {
				madct->state = INIT;
				schedule_delayed_work(&madct->queue_work, 0);
				madct->last = jiffies;
				continue;
			}
			
			if((long)jiffies - (long)madct->last < 50) {
				continue;
			}
			
			delay = 0;
			madct->state = START;
			madct->oval = val;
			madct->last = jiffies;
		}
		else if(madct->type == RANGE) {
			if(val == 0) {
				switch (madct->state) {
					case START:
						madct->state = NONE;
					case NONE:
						continue;
					default:
						madct->state = NONE;
						break;
				}
			}
			else {
				if(madct->state == NONE) {
					madct->state = INIT;
					madct->oval = val;
					madct->last = jiffies;
					continue;
				}
				else if(madct->state == START) {
					// only report one shot for long touch 
					continue;
				}
			}
			if((long)jiffies - (long)madct->last < 100) {
				if(val != 0) {
					continue;
				}
				else {
					madct->stype = STYPE_SHORT_TOUCH;
				}
			}
			else {
	            madct->state = START;
	            madct->last = jiffies;
				madct->stype = STYPE_LONG_TOUCH;
			}
			delay = 0;

			madct->key = chk_range(madct);
			if(madct->key == 0)
				continue;
			
			if(madc->dbg)
				idb_print("!!! KEY:%d chan:%d\n",madct->key,madct->chan);
		}

		if(delayed_work_pending(&madct->queue_work)) {
			continue;
		}
		cancel_delayed_work(&madct->queue_work);
		schedule_delayed_work(&madct->queue_work, delay);
	}

done:
	schedule_delayed_work(&madc->madc_work, madc->polling);
}
#endif

#ifdef CONFIG_CHEETAH_MADC_USE_INPUT_DEV
static int __devinit madc_probe(struct platform_device *pdev)
{
	struct madc_platform_data *pdata = pdev->dev.platform_data;
	struct madc_mon *madc = NULL;
	struct proc_dir_entry *res = NULL;
	struct proc_dir_entry *battery = NULL;
	struct input_dev *input = NULL;
	struct madc_type *madct; 
	int err;
	int idx;
    int i;

    madc = kzalloc(sizeof(struct madc_mon), GFP_ATOMIC);

	INIT_DELAYED_WORK(&madc->madc_work, (work_func_t)madc_monitor);
	for(idx=0; idx<MAX_CHAN_NUM; idx++, pdata++) {
		madct = (struct madc_type *)&madc->madct[idx];
		madct->chan = idx;
		madct->misc = pdata->misc;
		madct->valid = pdata->valid;
		madct->madc = madc;
		INIT_DELAYED_WORK(&madct->queue_work, (work_func_t)madc_event);
		if (!pdata->valid)
			continue;

		input = input_allocate_device();
		if (!madc || !input) {
			dev_err(&pdev->dev, "failed to allocate memory for device\n");
			err = -ENOMEM;
			goto exit_free_mem;
		}

		/* create and register the input driver */
		input->name = pdata->name;
		input->id.bustype = BUS_HOST;
		input->dev.parent = &pdev->dev;

		if (pdata->type == EV_KEY) {
			set_bit(EV_KEY, input->evbit);
            input->keycodesize = sizeof(cheeatch_input_keycode_map[0]);
            input->keycodemax = ARRAY_SIZE(cheeatch_input_keycode_map);
            input->keycode = &cheeatch_input_keycode_map;
            for (i = 0; i < ARRAY_SIZE(cheeatch_input_keycode_map); i++) {
                if (cheeatch_input_keycode_map[i] != KEY_RESERVED) {
                    set_bit(cheeatch_input_keycode_map[i], input->keybit);
                }
            }
		}
		if (pdata->type == EV_ABS) {
			set_bit(EV_ABS, input->evbit);
            input_set_abs_params(input, ABS_VOLUME, 0, 31, 0, 0);
		}

		err = input_register_device(input);
		if (err) {
			dev_err(&pdev->dev, "failed to register input device\n");
			goto exit_free_mem;
		}

		madct->input = input;
	}

	madc->polling = DELAY;
	madc->allchan = MAX_CHAN_NUM;

	platform_set_drvdata(pdev, madc);

	res = create_proc_entry("madc", S_IWUSR | S_IRUGO, NULL);
	if (!res) {
		err = -ENOMEM;
		goto exit_unregister_input;
	}
	else {
		res->data = madc;
		res->read_proc = proc_madc_read;
		res->write_proc = proc_madc_write;
	}

	battery = create_proc_entry("battery", S_IWUSR | S_IRUGO, NULL);
	if (!battery) {
		err = -ENOMEM;
		goto exit_unregister_input;
	}
	else {
		battery->data = madc;
		battery->read_proc = proc_battery_read;
	}

	schedule_delayed_work(&madc->madc_work, madc->polling);
	
	return 0;

exit_unregister_input:
	for(idx=0; idx<MAX_CHAN_NUM; idx++) {
		madct = (struct madc_type *)&madc->madct[idx];
		if (madct->input) {
			input_unregister_device(madct->input);
			madct->input = NULL; /* so we don't try to free it */
		}
	}
exit_free_mem:
	if (!battery) {
        remove_proc_entry("battery", NULL);
    }
	if (!res) {
        remove_proc_entry("madc", NULL);
    }
	for(idx=0; idx<MAX_CHAN_NUM; idx++) {
		madct = (struct madc_type *)&madc->madct[idx];
		if (madct->input) {
			input_free_device(madct->input);
		}
	}
	kfree(madc);
	return err;
}

static int __devexit madc_remove(struct platform_device *pdev)
{
	struct madc_mon *madc = platform_get_drvdata(pdev);
	struct madc_type *madct; 
	int idx;

	for(idx=0; idx<MAX_CHAN_NUM; idx++) {
		madct = (struct madc_type *)&madc->madct[idx];
		if (madct->input) {
			input_unregister_device(madct->input);
		}
	}
	platform_set_drvdata(pdev, NULL);
	kfree(madc);

	return 0;
}
#else
static int __devinit madc_probe(struct platform_device *pdev)
{
	struct madc_platform_data *pdata = pdev->dev.platform_data;
	struct madc_mon *madc = NULL;
	struct proc_dir_entry *res = NULL;
	struct proc_dir_entry *battery = NULL;
	struct madc_type *madct; 
	int err;
	int idx;

    madc = kzalloc(sizeof(struct madc_mon), GFP_ATOMIC);

	INIT_DELAYED_WORK(&madc->madc_work, (work_func_t)madc_monitor);
	for(idx=0; idx<MAX_CHAN_NUM; idx++, pdata++) {
		madct = (struct madc_type *)&madc->madct[idx];
		madct->chan = idx;
		madct->madc = madc;
		INIT_DELAYED_WORK(&madct->queue_work, (work_func_t)madc_event);
		if (!pdata->valid)
			continue;
	}

	madc->polling = DELAY;
	madc->allchan = MAX_CHAN_NUM;

	platform_set_drvdata(pdev, madc);

	res = create_proc_entry("madc", S_IWUSR | S_IRUGO, NULL);
	if (!res) {
		err = -ENOMEM;
		goto exit_free_mem;
	}
	else {
		res->data = madc;
		res->read_proc = proc_madc_read;
		res->write_proc = proc_madc_write;
	}

	battery = create_proc_entry("battery", S_IWUSR | S_IRUGO, NULL);
	if (!battery) {
		err = -ENOMEM;
		goto exit_free_mem;
	}
	else {
		battery->data = madc;
		battery->read_proc = proc_battery_read;
	}

	schedule_delayed_work(&madc->madc_work, madc->polling);
	
	return 0;

exit_free_mem:
	if (!battery) {
        remove_proc_entry("battery", NULL);
    }
	if (!res) {
        remove_proc_entry("madc", NULL);
    }
	kfree(madc);
	return err;
}

static int __devexit madc_remove(struct platform_device *pdev)
{
	struct madc_mon *madc = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	kfree(madc);

	return 0;
}
#endif

static struct platform_driver madc_driver = {
	.probe		= madc_probe,
	.remove		= __devexit_p(madc_remove),
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	}
};

static int __init cta_madc_init(void)
{
	return platform_driver_register(&madc_driver);
}

static void __exit cta_madc_exit(void)
{
	platform_driver_unregister(&madc_driver);
}

module_init(cta_madc_init);
module_exit(cta_madc_exit);
