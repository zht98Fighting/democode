#include "L610_cldsd_extkb.h"

//************************************************************************************


static int32				*extkb_task = NULL;  /*数字键盘的串口处理线程*/
static uint32				extkb_backlight_time = 0;  /*数字键盘的背光定时器*/
static uint32				extkb_keylock_time = 0;  /*数字键盘的按键锁定时器*/
static int					extkb_keylock = 0;/*按键锁状态*/
static int					extkb_keysound_status = 0;/*按键音状态*/
static int                   extkb_keybacklight_status = 1;/*背光状态 0-长灭;2-常亮;1-按键定时亮*/
static UINT32                 extkb_keybacklight_time = 10*1000;/*背光状态为2时的超时时间*/
static int					extkb_device = EXTKB_TYPE_NO_DEVICE;/*数字键盘设备*/

static unsigned char kbbuf[128];
static int           kbbuf_in;
static int           kbbuf_out;
static UINT32		 extkb_kb_mutex;     /*用于通讯时，数据的互斥*/
void extkb_kbbacklight_timer(void *ctx){	
    if(1 == g_iUart0Status)
    {
        extkb_kbLightOff();
    }
}

void extkb_kbLockEnable(void *ctx){	
	extkb_keylock = 1;

	if(extkb_keylock_time != 0){
		fibo_timer_free(extkb_keylock_time);
		extkb_keylock_time = 0;
	}
}


int extkb_check_device(void)
{
	if(EXTKB_TYPE_NO_DEVICE == extkb_device){
		int ret;
		unsigned char ver[32];
		memset(ver, 0, sizeof(ver));
        ret = jk138_query_version(ver, sizeof(ver));
        DR_LOG_D("jk138 ret:%d ver:%s", ret, ver);
		if(0 > ret){
			memset(ver, 0, sizeof(ver));
			ret = mhp18st_query_version(ver, sizeof(ver));
	        DR_LOG_D("mhp18st ret:%d ver:%s", ret, ver);
			if(0 > ret){
				return EXTKB_TYPE_NO_DEVICE;
			}
			extkb_device = EXTKB_TYPE_MHP18ST;
		}else{
			extkb_device = EXTKB_TYPE_JK138;
		}
	}
	return ERR_KBD_OK;
}

int extkb_uart_pak_verify(unsigned char *data, int len)
{
	uint8 Lrc = 0 ;
	int nlen =0;;
	if(data == NULL) return -1;
	if(len<=0) return -2;
	if(data[0] == 0x02 && data[len-2] == 0X03){
		nlen = len - 1;
		do{
			Lrc ^=data[len-nlen];
			//DR_LOG_D("data[len-nlen]:%#x", data[len-nlen]);
		}while(--nlen>1);
		if (Lrc != data[len - 1]){
			//DR_LOG_D(" ");
			return 0;
		}else{
			//DR_LOG_D(" ");
			extkb_device = EXTKB_TYPE_JK138;
			return 1;
		}
	}else if(data[0] == 0xFE && data[1] == 0XDF){
		nlen = len;
		do{
			Lrc ^=data[len-nlen];
			//DR_LOG_D("data[len-nlen]:%#x", data[len-nlen]);
		}while(--nlen>1);
		if (Lrc != data[len - 1]){
			//DR_LOG_D(" ");
			return 0;
		}else{
			//DR_LOG_D(" ");
			extkb_device = EXTKB_TYPE_MHP18ST;
			return 1;
		}
	}else if(data[0] == 0x06){
		if(data[1] == 0x02 && data[len-2] == 0X03){
			nlen = len - 2;
			do{
				Lrc ^=data[len-nlen];
				//DR_LOG_D("data[len-nlen]:%#x", data[len-nlen]);
			}while(--nlen>1);
			if (Lrc != data[len - 1]){
				//DR_LOG_D(" ");
				return 0;
			}else{
				//DR_LOG_D(" ");
				extkb_device = EXTKB_TYPE_JK138;
				return 1;
			}
		}else{
			return 0;
		}
	}else{
		//DR_LOG_D(" ");
		return 0;
	}

}

void extkb_uart_rx(unsigned char *data, int len)
{
	if(extkb_keylock) return;
  
	if(extkb_device == EXTKB_TYPE_JK138){
		jk138_uart_rx(data, len);
	}else if(extkb_device == EXTKB_TYPE_MHP18ST){
		mhp18st_uart_rx(data, len);
	}
	
#if ONLY_EXTBK
	osiEvent_t extkb_event = {0};
    extkb_event.id = *extkb_task;
    osiEventSend(extkb_task, &extkb_event);
#endif
}

void extkb_do_uart_data(void){
	if(extkb_device == EXTKB_TYPE_JK138){
		jk138_do_uart_data();
	}else if(extkb_device == EXTKB_TYPE_MHP18ST){
		mhp18st_do_uart_data();
	}

}
								 
static void extkb_uart_notify_cb(hal_uart_port_t uart_port, INT8 *data, UINT16 len, void *arg)
{
	
	unsigned char datatmp[64];
    unsigned int real_size = 0;
    int read_len = 0;

    while(len > 0) {
        real_size = len > sizeof(datatmp) ? sizeof(datatmp) : len;
		memcpy(datatmp, data, real_size);
		read_len = real_size;
		//array2hexShow(datatmp, read_len);
		//if(extkb_uart_pak_verify(datatmp, read_len)) return;
        if((read_len > 0) && ((int)len >= read_len)) {
            len -= read_len;
            extkb_uart_rx(datatmp, read_len);
        } else {
            break;
        }
    }

}

static void extkb_thread(void *param)
{
    osiEvent_t event = {0};

    while (1) {
        event.id = 0;
        osiEventTryWait(extkb_task, &event, OSI_WAIT_FOREVER);
        if (*extkb_task == event.id) {
			extkb_do_uart_data();
        }
    }
    fibo_thread_delete();
}

int Extkb_Init(void)
{
    hal_uart_config_t uart0_drvcfg = {0};
    int ret;
	
#if ONLY_EXTBK
	
	extkb_task = osiThreadCreate("extkb_thread", extkb_thread, NULL, OSI_PRIORITY_NORMAL, 1024 * 8, 30);
	if (extkb_task == NULL) {
		  DR_LOG_D("ql_rtos_task_create err:%#x", ret);
	    return ERR_KBD_INIT;
	}
#endif

    extkb_kb_mutex = fibo_sem_new(1);
    if (extkb_kb_mutex == 0) {
        DR_LOG_D("crteate extkb_kb_mutex err:%#x", ret);
        return ERR_KBD_INIT;
    }
	
	ret = jk138_init();
    if (ret < 0) {
        DR_LOG_D("jk138_init err:%#x", ret);
        return ERR_KBD_INIT;
    }
	
	ret = mhp18st_init();
    if (ret < 0) {
        DR_LOG_D("mhp18st_init err:%#x", ret);
        return ERR_KBD_INIT;
    }
	
#if ONLY_EXTBK

	fibo_hal_uart_deinit(UART0_PORT);

    uart0_drvcfg.baud = 115200;
    uart0_drvcfg.parity = HAL_UART_NO_PARITY;
    uart0_drvcfg.data_bits = HAL_UART_DATA_BITS_8;
    uart0_drvcfg.stop_bits = HAL_UART_STOP_BITS_1;
    uart0_drvcfg.rx_buf_size = UART_RX_BUF_SIZE;
    uart0_drvcfg.tx_buf_size = UART_TX_BUF_SIZE;
	//uart0_drvcfg.recv_timeout = 10000;

    fibo_hal_uart_init(UART0_PORT, &uart0_drvcfg, extkb_uart_notify_cb, NULL);
#endif
	
    return ERR_KBD_OK;
}

int extkb_query_version(unsigned char *ver, int size)
{
    if ((NULL == ver) || (size <= 0)) {
        return ERR_KBD_PARAM;
    }
	
	if(EXTKB_TYPE_NO_DEVICE == extkb_device){
		int ret;
		memset(ver, 0, sizeof(ver));
        ret = jk138_query_version(ver, sizeof(ver));
        DR_LOG_D("jk138 ret:%d ver:%s", ret, ver);
		if(0 > ret){
			memset(ver, 0, sizeof(ver));
			ret = mhp18st_query_version(ver, sizeof(ver));
	        DR_LOG_D("mhp18st ret:%d ver:%s", ret, ver);
			if(0 > ret){
				return EXTKB_TYPE_NO_DEVICE;
			}
			return EXTKB_TYPE_MHP18ST;
		}else{
			return EXTKB_TYPE_JK138;
		}
	}
	
    if(extkb_device == EXTKB_TYPE_JK138){
		jk138_query_version( ver, size);
		return EXTKB_TYPE_JK138;
	}else if(extkb_device == EXTKB_TYPE_MHP18ST){
		mhp18st_query_version( ver, size);
		return EXTKB_TYPE_MHP18ST;
	}
	return EXTKB_TYPE_NO_DEVICE;
}


int extkb_display_flush(void)
{
    if(extkb_check_device() != ERR_KBD_OK) return EXTKB_TYPE_NO_DEVICE;
	
    if(extkb_device == EXTKB_TYPE_JK138){
		return jk138_display_flush();
	}else if(extkb_device == EXTKB_TYPE_MHP18ST){
		return mhp18st_display_flush();
	}
	return EXTKB_TYPE_NO_DEVICE;
}

int extkb_display(int num, enum segment_code code)
{
    if(extkb_check_device() != ERR_KBD_OK) return EXTKB_TYPE_NO_DEVICE;
	
    if(extkb_device == EXTKB_TYPE_JK138){
		return jk138_display(num, code);
	}else if(extkb_device == EXTKB_TYPE_MHP18ST){
		return mhp18st_display(num, code);
	}
	return EXTKB_TYPE_NO_DEVICE;
}

void extkb_inject_key(int value)
{
    int in = (kbbuf_in + 1) % sizeof(kbbuf);

    if (in != kbbuf_out) {
        kbbuf[kbbuf_in] = value;
        kbbuf_in = in;
		if(extkb_keybacklight_status == 1){
			fibo_timer_free(extkb_keylock_time);
			extkb_kbLightOn();
			extkb_keylock_time = fibo_timer_period_new(extkb_keybacklight_time, extkb_kbbacklight_timer, NULL);
		}
    } else {
        DR_LOG_D("kbbuf overflow, throw key:%#x", value);
    }
}

unsigned char extkb_kbHit(void)
{
	
	if(extkb_keylock) return ERR_KBD_LOCK;
    if (kbbuf_in == kbbuf_out) {
        /*按键缓存为空*/
        Sleep(2);
        return ERR_KBD_NO_KEY;
    }

    return ERR_KBD_OK;
}

void extkb_kbFlush(void)
{
	if(extkb_keylock) return;
    fibo_sem_wait(extkb_kb_mutex);
    kbbuf_out = kbbuf_in;
    fibo_sem_signal(extkb_kb_mutex);
}

unsigned char extkb_kbGetKey(void) 
{
	if(extkb_keylock) return KB_KEY_NULL;;
    unsigned char key;
	unsigned char ext_key;
	unsigned long long wait_key_outtime =0;
	
	extern unsigned long long DR_GetSystick_ms(void);
	wait_key_outtime = DR_GetSystick_ms();

    fibo_sem_wait(extkb_kb_mutex);
	
    while(1){
		if(DR_GetSystick_ms() - wait_key_outtime>500){
			key =KB_KEY_NULL;
			break;
		}
		if(!extkb_kbHit()){
			key = kbbuf[kbbuf_out];
			kbbuf_out = (kbbuf_out + 1) % sizeof(kbbuf);
			break;
		}
	};
    fibo_sem_signal(extkb_kb_mutex);

    return key; 
}

int extkb_kbSetSound(unsigned char flag)
{
	if(extkb_keylock) return ERR_KBD_LOCK;
    if(extkb_check_device() != ERR_KBD_OK) return EXTKB_TYPE_NO_DEVICE;
	
    if(extkb_device == EXTKB_TYPE_JK138){
		if(flag == 0){
			jk138_Sound(0);
		}else{
			jk138_Sound(1);
		}
	}else if(extkb_device == EXTKB_TYPE_MHP18ST){
		if(flag == 0){
			mhp18st_Sound(0);
		}else{
			mhp18st_Sound(0xFF);
		}
	}
	return ERR_KBD_OK;
}

int extkb_kbGetString(unsigned char *pucStr, unsigned char ucMode,
                                unsigned char ucMinLen, unsigned char ucMaxLen, 
                                unsigned short usTimeOutSec)
{
	if(extkb_keylock) return ERR_KBD_LOCK;
    int ret, i, code, string_num = 0, dot_ad = -1, str_len;
	int max_amount;
    if(extkb_device == EXTKB_TYPE_JK138){
		max_amount = 8;
	}else if(extkb_device == EXTKB_TYPE_MHP18ST){
		max_amount = 10;
	}

	while(*pucStr++=='\0'){
		str_len ++;
		if(pucStr[i] == '.'){
			dot_ad = str_len;
		}else{
        	string_num = pucStr[i]-'0' + string_num*10;
		}
	}
	if(string_num > (pow(10, max_amount)-1) || string_num < 0) {
        return ERR_KBD_PARAM;
    }

    for (i = 0; i < max_amount; i++, string_num /= 10) {
        if ((i >= 3) && (0 == string_num)) {
            /*前导0, 不显示*/
            code = SEG_CODE_OFF;
        } else {
            code = (string_num % 10) + SEG_CODE_0;
        }

        ret = extkb_display(i, code);
        if (ret) {
            return ret;
        }
    }
	if(!(dot_ad == -1 || dot_ad == 1 || dot_ad == str_len)){//无小数点或头尾不显示
    	ret = extkb_display(str_len-dot_ad-2, SEG_CODE_DOT);
	}
    if (ret) {
        return ret;
    }
    return extkb_display_flush();

}
                                
void extkb_kbLock(int mode)
{
	if(mode == 0){
		extkb_keylock = 0;
	}else if(mode == 1){
		extkb_keylock = 1;
	}if(mode == 2){
		extkb_keylock_time = fibo_timer_period_new(30*1000, extkb_kbLockEnable, NULL);
	}
}

int extkb_kbCheck(int iCmd)
{
	if(iCmd == 0) return extkb_keylock;
	else if(iCmd == 1) {
		if(kbbuf_in >= kbbuf_out)
			return kbbuf_in - kbbuf_out;
		else
			return 128 + kbbuf_in - kbbuf_out;
	}else if(iCmd == 2) {
		return extkb_keysound_status;
	}
}

unsigned char extkb_kbLightSetMode(unsigned char mode, unsigned int countms)
{
	if(extkb_keylock) return ERR_KBD_LOCK;
    if(extkb_check_device() != ERR_KBD_OK) return EXTKB_TYPE_NO_DEVICE;
	if(2 < mode) return ERR_KBD_PARAM1;
		
	extkb_keybacklight_status = mode;
	extkb_keybacklight_time = countms;
	fibo_timer_free(extkb_backlight_time);
	if(extkb_device == EXTKB_TYPE_JK138){
		if(mode == 0 || mode == 1){
			extkb_kbLightOff();
		}else if(mode == 2){
			extkb_kbLightOn();
		}
	}else if(extkb_device == EXTKB_TYPE_MHP18ST){

	}
	return ERR_KBD_OK;
}

void extkb_kbLightOff(void)
{
	if(extkb_keylock) return;
    if(extkb_check_device() != ERR_KBD_OK) return;
	
    if(extkb_device == EXTKB_TYPE_JK138){
		jk138_PWM(2,0X3F);
	}else if(extkb_device == EXTKB_TYPE_MHP18ST){
		
	}
}

void extkb_kbLightOn(void)
{
	if(extkb_keylock) return;
    if(extkb_check_device() != ERR_KBD_OK) return;
		
	if(extkb_device == EXTKB_TYPE_JK138){
		jk138_PWM(2,0);
	}else if(extkb_device == EXTKB_TYPE_MHP18ST){

	}
}

/**************************************************************************************/
//以下为断码显示代码

/*
*@Brief:		显示人民币金额
*@Param IN:		amount:金额值,单位：分。
*               显示屏固定会有两位小数，所以参数输入 125 时，显示 1.25
*@Return:		0-成功,其他-失败
*/
int extkb_DisplayAmount(int amount)
{
    int ret, i, code;
	int max_amount;
    if(extkb_device == EXTKB_TYPE_JK138){
		max_amount = 8;
	}else if(extkb_device == EXTKB_TYPE_MHP18ST){
		max_amount = 10;
	}
	if(amount > (pow(10, max_amount)-1) || amount < 0) {
        return ERR_KBD_PARAM;
    }

    for (i = 0; i < max_amount; i++, amount /= 10) {
        if ((i >= 3) && (0 == amount)) {
            /*前导0, 不显示*/
            code = SEG_CODE_OFF;
        } else {
            code = (amount % 10) + SEG_CODE_0;
        }

        ret = extkb_display(i, code);
        if (ret) {
            return ret;
        }
    }
    ret = extkb_display(1, SEG_CODE_DOT); //1 第二位显示小数点
    if (ret) {
        return ret;
    }
    return extkb_display_flush();
}

/*
*@Brief:		显示图标
*@Param IN:		mode:显示icon,1-wifi,2-gprs,3-bat,4-.,5-',(.与'，level最高位表显示，后七位表位置)
				mode		level
				1			0		wifi off
							1,2,3,4,5 wifi level,other param error
				2			0		gprs off
							1,2,3,4 gprs level,other param error
				3			0		bat off
							1,2,3,4 bat level,other param error
							jk138 unsupport
				4			.(点)符号，level >> 7 = 1(show),0(hide) 	level & 0x7F = 显示位（1-10）
							其中支持'元'功能的，第一位为元
				5			'(撇)符号，'level >> 7 = 1(show),0(hide) 	level & 0x7F = 显示位（1-7）
*@Param IN:		level:显示等级，0为关闭
*@Return:		0-成功,其他-失败
*/
int extkb_DisplayIcon(int mode, unsigned char level)
{
	int ret, modelevel,num = 0;
	switch(mode){
		case 1:
			switch(level){
				case 0:
					modelevel = SEG_CODE_ICON_WIFI_OFF;
					break;
				case 1:
					modelevel = SEG_CODE_ICON_SIGNAL_LEVEL_0;
					break;
				case 2:
					modelevel = SEG_CODE_ICON_SIGNAL_LEVEL_1;
					break;
				case 3:
					modelevel = SEG_CODE_ICON_SIGNAL_LEVEL_2;
					break;
				case 4:
					modelevel = SEG_CODE_ICON_SIGNAL_LEVEL_3;
					break;
				case 5:
					modelevel = SEG_CODE_ICON_SIGNAL_LEVEL_4;
					break;
				default:
					return ERR_KBD_PARAM;
			}
			break;
		case 2:
			switch(level){
				case 0:
					modelevel = SEG_CODE_GPRS_OFF;
					break;
				case 1:
					modelevel = SEG_CODE_GPRS_LEVEL_0;
					break;
				case 2:
					modelevel = SEG_CODE_GPRS_LEVEL_1;
					break;
				case 3:
					modelevel = SEG_CODE_GPRS_LEVEL_2;
					break;
				case 4:
					modelevel = SEG_CODE_GPRS_LEVEL_3;
					break;
				default:
					return ERR_KBD_PARAM;
			}
			if(extkb_device == EXTKB_TYPE_JK138){
				if(modelevel != SEG_CODE_GPRS_OFF)
					modelevel = SEG_CODE_GPRS_ON;
			}
			break;
		case 3:
			switch(level){
				case 0:
					modelevel = SEG_CODE_BAT_OFF;
					break;
				case 1:
					modelevel = SEG_CODE_BAT_LEVEL_0;
					break;
				case 2:
					modelevel = SEG_CODE_BAT_LEVEL_1;
					break;
				case 3:
					modelevel = SEG_CODE_BAT_LEVEL_2;
					break;
				case 4:
					modelevel = SEG_CODE_BAT_LEVEL_3;
					break;
				default:
					return ERR_KBD_PARAM;
			}
			if(extkb_device == EXTKB_TYPE_JK138){
				return ERR_KBD_UNSUPPORT;
			}
			break;
		case 4:
			modelevel = level>>7?SEG_CODE_DOT:SEG_CODE_DOT_OFF;
			num = level & 0x7F;
			break;
		case 5:
			modelevel = level>>7?SEG_CODE_PRIME:SEG_CODE_PRIME_OFF;
			num = level & 0x7F;
			break;
		default:
			return ERR_KBD_PARAM;
	}
	
	ret = extkb_display(num, modelevel);
	if (ret) {
		return ret;
	}

    return extkb_display_flush();
}

int extkb_DisplayShow(int num, enum segment_code code)
{
	int ret;
    ret = extkb_display(num, code);
	if (ret) {
		return ret;
	}

    return extkb_display_flush();
}

int extkb_DisplayCls(void)
{
	int ret;
    ret = extkb_display(0, SEG_CODE_ALL_OFF);
	if (ret) {
		return ret;
	}

    return extkb_display_flush();
}

static char *extkb_get_key_string(int key)
{
    char *str = NULL;

    switch (key) {
        case KB_KEY1:
            str = "1";
            break;
        case KB_KEY2:
            str = "2";
            break;
        case KB_KEY3:
            str = "3";
            break;
        case KB_KEY4:
            str = "4";
            break;
        case KB_KEY5:
            str = "5";
            break;
        case KB_KEY6:
            str = "6";
            break;
        case KB_KEY7:
            str = "7";
            break;
        case KB_KEY8:
            str = "8";
            break;
        case KB_KEY9:
            str = "9";
            break;
        case KB_KEY0:
            str = "0";
            break;
        case KB_KEYADD:
            str = "+";
            break;
        case KB_KEYSUB:
            str = "-";
            break;
        case KB_KEYMUL:
            str = "x";
            break;
        case KB_KEYDIV:
            str = "/";
            break;
        case KB_KEYDOT:
            str = ".";
            break;
        case KB_KEY_SET:
            str = "SET";//"设置";
            break;
        case KB_KEY_BILL:
            str = "BILL";//"账单";
            break;
        case KB_KEY_EQUAL:
            str = "=";
            break;
        case KB_KEYBACKSPACE:
            str = "BACKSPACE";//"退格";
            break;
        case KB_KEYREFUND:
            str = "REFUND";//"退款";
            break;
        case KB_KEYPAYEE:
            str = "PAYEE";//"收款";
            break;
        case KB_KEYCANCEL:
            str = "CANCEL"; /*取消*/
            break;
        case KB_KEYCLEAR:
            str = "CLEAR"; /*清除*/
            break;
        case KB_KEY_MENU:
            str = "MENU"; /*菜单*/
            break;
        case KB_KEYSOUND_VOL:
            str = "SOUND_VOL"; /*音量*/
            break;
        default:
            str = "unknow";
            break;
    }
    return str;
}

#if ENABLE_DEBUG_FUNCTION_EXTKB
static void Extkb_Wait_Key(void ){
	while(1){
		if (!extkb_kbHit()){
			break;
		}
		Sleep(10);
	}
}

static void Extkb_Test_Task(void *param)
{
    int ret, key;
    unsigned char ver[32];
    int num_indx = 0;
    int dot_indx = 0;

    while (1) {
		Extkb_Wait_Key();
		key = extkb_kbGetKey();
		DR_LOG_D("key:%s(%#x)", extkb_get_key_string(key), key);
		if (KB_KEY_BILL == key ||
			KB_KEY_MENU == key) {
			memset(ver, 0, sizeof(ver));
			ret = extkb_query_version(ver, sizeof(ver));
			DR_LOG_D("extkb ret:%d ver:%s", ret, ver);
		}else if (key >= KB_KEY0 && key <= KB_KEY9) {
			key = SEG_CODE_0 + key - KB_KEY0;
			ret = extkb_DisplayShow(num_indx++, key);
			DR_LOG_D("display n:%d c:%#x ret:%d ", num_indx, key, ret);
		} else if (KB_KEYDOT == key) {
			ret = extkb_DisplayShow(dot_indx++, SEG_CODE_DOT);
			DR_LOG_D("display n:%d c:%#x ret:%d ", dot_indx, SEG_CODE_DOT, ret);
		} else if (KB_KEYCANCEL == key){
			/*清屏*/
			ret = extkb_DisplayShow(0, SEG_CODE_ALL_OFF);
			DR_LOG_D("display n:%d c:%#x ret:%d ", 0, SEG_CODE_ALL_OFF, ret);
			num_indx = 0;
			dot_indx = 0;
		} else if (KB_KEY_SET == key){
			/*全点亮*/
			ret = extkb_DisplayShow(0, SEG_CODE_ALL_ON);
			DR_LOG_D("display n:%d c:%#x ret:%d ", 0, SEG_CODE_ALL_ON, ret);
			num_indx = 0;
			dot_indx = 0;
			} else if (KB_KEY_EQUAL == key){
			key = 10000;
			ret = extkb_DisplayAmount(key);
			DR_LOG_D("amount:%d  ret:%d ",  key, ret);
			Extkb_Wait_Key();
			extkb_kbGetKey();

			key = 10;
			ret = extkb_DisplayAmount(key);
			DR_LOG_D("amount:%d  ret:%d ",  key, ret);
			Extkb_Wait_Key();
			extkb_kbGetKey();

			key = 123;
			ret = extkb_DisplayAmount(key);
			DR_LOG_D("amount:%d  ret:%d ",  key, ret);
			Extkb_Wait_Key();
			extkb_kbGetKey();
		} else if (KB_KEYENTER == key){
			if(extkb_device == EXTKB_TYPE_JK138){
				ret = extkb_DisplayShow(0, SEG_CODE_ICON_WIFI_ON);
				DR_LOG_D("SEG_CODE_ICON_WIFI_ON ret:%d ",  ret);
				Extkb_Wait_Key();
				extkb_kbGetKey();
				
				ret = extkb_DisplayShow(0, SEG_CODE_ICON_WIFI_OFF);
				DR_LOG_D("SEG_CODE_ICON_WIFI_OFF ret:%d ",	ret);
				Extkb_Wait_Key();
				extkb_kbGetKey();
				
				ret = extkb_DisplayShow(0, SEG_CODE_GPRS_ON);
				DR_LOG_D("SEG_CODE_GPRS_ON ret:%d ",  ret);
				Extkb_Wait_Key();
				extkb_kbGetKey();
				
				ret = extkb_DisplayShow(0, SEG_CODE_GPRS_OFF);
				DR_LOG_D("SEG_CODE_GPRS_OFF ret:%d ",  ret);
				Extkb_Wait_Key();
				extkb_kbGetKey();
				
				ret = extkb_DisplayShow(0, SEG_CODE_ICON_SIGNAL_LEVEL_4);
				DR_LOG_D("SEG_CODE_ICON_SIGNAL_LEVEL_4 ret:%d ",  ret);
				Extkb_Wait_Key();
				extkb_kbGetKey();
				
				ret = extkb_DisplayShow(0, SEG_CODE_ICON_SIGNAL_LEVEL_3);
				DR_LOG_D("SEG_CODE_ICON_SIGNAL_LEVEL_3 ret:%d ",  ret);
				Extkb_Wait_Key();
				extkb_kbGetKey();
				
				ret = extkb_DisplayShow(0, SEG_CODE_ICON_SIGNAL_LEVEL_2);
				DR_LOG_D("SEG_CODE_ICON_SIGNAL_LEVEL_2 ret:%d ",  ret);
				Extkb_Wait_Key();
				extkb_kbGetKey();
				
				ret = extkb_DisplayShow(0, SEG_CODE_ICON_SIGNAL_LEVEL_1);
				DR_LOG_D("SEG_CODE_ICON_SIGNAL_LEVEL_1 ret:%d ",  ret);
				Extkb_Wait_Key();
				extkb_kbGetKey();
				
				ret = extkb_DisplayShow(0, SEG_CODE_ICON_SIGNAL_LEVEL_0);
				DR_LOG_D("SEG_CODE_ICON_SIGNAL_LEVEL_0 ret:%d ",  ret);
			}else if(extkb_device == EXTKB_TYPE_MHP18ST){
				ret = extkb_DisplayShow(0, SEG_CODE_ICON_WIFI_ON);
				DR_LOG_D("SEG_CODE_ICON_WIFI_ON ret:%d ",  ret);
				
				Extkb_Wait_Key();
				extkb_kbGetKey();
				ret = extkb_DisplayShow(0, SEG_CODE_GPRS_ON);
				DR_LOG_D("SEG_CODE_GPRS_ON ret:%d ",  ret);
				
				Extkb_Wait_Key();
				extkb_kbGetKey();
				ret = extkb_DisplayShow(0, SEG_CODE_BAT_ON);
				DR_LOG_D("SEG_CODE_BAT_ON ret:%d ",  ret);
				
				Extkb_Wait_Key();
				extkb_kbGetKey();
				ret = extkb_DisplayShow(0, SEG_CODE_ICON_WIFI_OFF);
				DR_LOG_D("SEG_CODE_ICON_WIFI_OFF ret:%d ",  ret);		
				ret = extkb_DisplayShow(0, SEG_CODE_GPRS_OFF);
				DR_LOG_D("SEG_CODE_GPRS_OFF ret:%d ",  ret);		
				ret = extkb_DisplayShow(0, SEG_CODE_BAT_OFF);
				DR_LOG_D("SEG_CODE_BAT_OFF ret:%d ",  ret);		
				
				Extkb_Wait_Key();
				extkb_kbGetKey();
				ret = extkb_DisplayShow(0, SEG_CODE_ICON_SIGNAL_LEVEL_0);
				DR_LOG_D("SEG_CODE_ICON_SIGNAL_LEVEL_0 ret:%d ",  ret);

				Extkb_Wait_Key();
				extkb_kbGetKey();
				ret = extkb_DisplayShow(0, SEG_CODE_ICON_SIGNAL_LEVEL_1);
				DR_LOG_D("SEG_CODE_ICON_SIGNAL_LEVEL_2 ret:%d ",	ret);

				Extkb_Wait_Key();
				extkb_kbGetKey();
				ret = extkb_DisplayShow(0, SEG_CODE_ICON_SIGNAL_LEVEL_2);
				DR_LOG_D("SEG_CODE_ICON_SIGNAL_LEVEL_3 ret:%d ",  ret);

				Extkb_Wait_Key();
				extkb_kbGetKey();
				ret = extkb_DisplayShow(0, SEG_CODE_ICON_SIGNAL_LEVEL_3);
				DR_LOG_D("SEG_CODE_ICON_SIGNAL_LEVEL_4 ret:%d ",  ret);

				Extkb_Wait_Key();
				extkb_kbGetKey();
				ret = extkb_DisplayShow(0, SEG_CODE_GPRS_LEVEL_0);
				DR_LOG_D("SEG_CODE_GPRS_LEVEL_0 ret:%d ",  ret);

				Extkb_Wait_Key();
				extkb_kbGetKey();
				ret = extkb_DisplayShow(0, SEG_CODE_GPRS_LEVEL_1);
				DR_LOG_D("SEG_CODE_GPRS_LEVEL_1 ret:%d ",  ret);

				Extkb_Wait_Key();
				extkb_kbGetKey();
				ret = extkb_DisplayShow(0, SEG_CODE_GPRS_LEVEL_2);
				DR_LOG_D("SEG_CODE_GPRS_LEVEL_2 ret:%d ",  ret);

				Extkb_Wait_Key();
				extkb_kbGetKey();
				ret = extkb_DisplayShow(0, SEG_CODE_GPRS_LEVEL_3);
				DR_LOG_D("SEG_CODE_GPRS_LEVEL_3 ret:%d ",  ret);

				Extkb_Wait_Key();
				extkb_kbGetKey();
				ret = extkb_DisplayShow(0, SEG_CODE_BAT_LEVEL_0);
				DR_LOG_D("SEG_CODE_BAT_LEVEL_0 ret:%d ",  ret);

				Extkb_Wait_Key();
				extkb_kbGetKey();
				ret = extkb_DisplayShow(0, SEG_CODE_BAT_LEVEL_1);
				DR_LOG_D("SEG_CODE_BAT_LEVEL_1 ret:%d ",  ret);

				Extkb_Wait_Key();
				extkb_kbGetKey();
				ret = extkb_DisplayShow(0, SEG_CODE_BAT_LEVEL_2);
				DR_LOG_D("SEG_CODE_BAT_LEVEL_2 ret:%d ",  ret);

				Extkb_Wait_Key();
				extkb_kbGetKey();
				ret = extkb_DisplayShow(0, SEG_CODE_BAT_LEVEL_3);
				DR_LOG_D("SEG_CODE_BAT_LEVEL_3 ret:%d ",  ret);

				Extkb_Wait_Key();
				extkb_kbGetKey();
				ret = extkb_DisplayShow(0, SEG_CODE_ICON_WIFI_OFF);
				DR_LOG_D("SEG_CODE_ICON_WIFI_OFF ret:%d ",  ret);		
				ret = extkb_DisplayShow(0, SEG_CODE_GPRS_OFF);
				DR_LOG_D("SEG_CODE_GPRS_OFF ret:%d ",  ret);		
				ret = extkb_DisplayShow(0, SEG_CODE_BAT_OFF);
				DR_LOG_D("SEG_CODE_BAT_OFF ret:%d ",  ret);		
			}

		} 
		else if (KB_KEYADD == key){
			ret = extkb_kbLightSetMode(2, 0);
			DR_LOG_D("extkb_kbLightOn on ret:%d ",  ret);

			Extkb_Wait_Key();
			extkb_kbGetKey();
			ret = extkb_kbLightSetMode(0, 0);
			DR_LOG_D("extkb_kbLightOn off ret:%d ",  ret);
		} else if (KB_KEYSUB == key) {
			ret = extkb_kbSetSound(1);
			DR_LOG_D("extkb_kbSetSound on ret:%d ",  ret);

			Extkb_Wait_Key();
			extkb_kbGetKey();
			ret = extkb_kbSetSound(0);
			DR_LOG_D("extkb_kbSetSound off ret:%d ",  ret);
		}
	}
	fibo_thread_delete();
}

void DR_Extkb_test(void)
{
	int ret;
	
	ret = fibo_thread_create(Extkb_Test_Task, "extkb_test", 1024*8*2, NULL, OSI_PRIORITY_NORMAL);
	if(ret < 0){
		DR_LOG_D("created task failed\r\n");
	}
}
#endif
