#include <stdint.h>
#include "CH58xBLE_LIB.h"
#include "CH583SFR.h"
#include "ISP583.h"


void compat_Init(void) {
    compat_SetSysClock();
		compat_LedInit();
		compat_LedToggle();
    compat_CH58X_BLEInit();
    compat_HAL_TimeInit();
    GAPRole_PeripheralInit();
}

#ifndef BLE_MEMHEAP_SIZE
#define BLE_MEMHEAP_SIZE                    (1024*6)
#endif
__attribute__((aligned(4))) u32 MEM_BUF[BLE_MEMHEAP_SIZE / 4];
#ifndef BLE_BUFF_MAX_LEN
#define BLE_BUFF_MAX_LEN                    27
#endif
#ifndef BLE_BUFF_NUM
#define BLE_BUFF_NUM                        5
#endif
#ifndef BLE_TX_NUM_EVENT
#define BLE_TX_NUM_EVENT                    1
#endif
#ifndef BLE_TX_POWER
#define BLE_TX_POWER                        LL_TX_POWEER_0_DBM
#endif
#ifndef PERIPHERAL_MAX_CONNECTION
#define PERIPHERAL_MAX_CONNECTION           1
#endif
#ifndef CENTRAL_MAX_CONNECTION
#define CENTRAL_MAX_CONNECTION              3
#endif
extern uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];

#ifndef BLE_SNV
#define BLE_SNV                             TRUE
#endif
#ifndef CLK_OSC32K
#define CLK_OSC32K                          1   // 该项请勿在此修改，必须在工程配置里的预处理中修改，如包含主机角色必须使用外部32K
#endif
#ifndef TEM_SAMPLE
#define TEM_SAMPLE                          FALSE
#endif
#ifndef HAL_SLEEP
#define HAL_SLEEP                           FALSE
#endif
#ifndef BLE_MAC
#define BLE_MAC                             FALSE
#endif
#ifndef BLE_SNV_ADDR
#define BLE_SNV_ADDR                        0x77E00-FLASH_ROM_MAX_SIZE
#endif
#ifdef CLK_OSC32K
#if ( CLK_OSC32K == 1 )
#define CAB_LSIFQ       32000
#else
#define CAB_LSIFQ       32768
#endif
#else
#define CAB_LSIFQ       32000
#endif

#define GPIO_Pin_4                 (0x00000010)  /*!< Pin 4 selected */
void compat_LedInit(void) {
	R32_PB_PD_DRV &= ~GPIO_Pin_4;
	R32_PB_DIR |= GPIO_Pin_4;
}
void compat_LedToggle(void) {
	R32_PB_OUT ^= GPIO_Pin_4;
}

#define __nop()                 __asm__ volatile("nop")
typedef struct
{
    volatile uint32_t CTLR;
    volatile uint32_t SR;
    volatile uint64_t CNT;
    volatile uint64_t CMP;
} SysTick_Type;
#define SysTick                 ((SysTick_Type *)0xE000F000)
#define SysTick_LOAD_RELOAD_Msk    (0xFFFFFFFFFFFFFFFF)
#define SysTick_CTLR_INIT          (1 << 5)
#define SysTick_CTLR_MODE          (1 << 4)
#define SysTick_CTLR_STRE          (1 << 3)
#define SysTick_CTLR_STCLK         (1 << 2)
#define SysTick_CTLR_STIE          (1 << 1)
#define SysTick_CTLR_STE           (1 << 0)
static inline uint32_t SysTick_Config(uint64_t ticks)
{
    if((ticks - 1) > SysTick_LOAD_RELOAD_Msk)
        return (1); /* Reload value impossible */

    SysTick->CMP = ticks - 1; /* set reload register */
    SysTick->CTLR = SysTick_CTLR_INIT |
                    SysTick_CTLR_STRE |
                    SysTick_CTLR_STCLK |
                    SysTick_CTLR_STIE |
                    SysTick_CTLR_STE; /* Enable SysTick IRQ and SysTick Timer */
    return (0);                       /* Function successful */
}

static uint32_t Lib_Read_Flash(uint32_t addr, uint32_t num, uint32_t *pBuf)
{
    EEPROM_READ(addr, pBuf, num * 4);
    return 0;
}
static uint32_t Lib_Write_Flash(uint32_t addr, uint32_t num, uint32_t *pBuf)
{
    EEPROM_ERASE(addr, num * 4);
    EEPROM_WRITE(addr, pBuf, num * 4);
    return 0;
}

#ifndef  SAFEOPERATE
#define  SAFEOPERATE   __nop();__nop()
#endif
 __attribute__((always_inline)) static inline void sys_safe_access_enable(void)
{
    R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG1;
    R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG2;
    SAFEOPERATE;
}

__attribute__((always_inline)) static inline void sys_safe_access_disable(void)
{
    R8_SAFE_ACCESS_SIG = 0;
}

typedef enum
{
    CLK_SOURCE_LSI = 0x00,
    CLK_SOURCE_LSE,

    CLK_SOURCE_HSE_16MHz = 0x22,
    CLK_SOURCE_HSE_8MHz = 0x24,
    CLK_SOURCE_HSE_6_4MHz = 0x25,
    CLK_SOURCE_HSE_4MHz = 0x28,
    CLK_SOURCE_HSE_2MHz = (0x20 | 16),
    CLK_SOURCE_HSE_1MHz = (0x20 | 0),

    CLK_SOURCE_PLL_80MHz = 0x46,
    CLK_SOURCE_PLL_60MHz = 0x48,
    CLK_SOURCE_PLL_48MHz = (0x40 | 10),
    CLK_SOURCE_PLL_40MHz = (0x40 | 12),
    CLK_SOURCE_PLL_36_9MHz = (0x40 | 13),
    CLK_SOURCE_PLL_32MHz = (0x40 | 15),
    CLK_SOURCE_PLL_30MHz = (0x40 | 16),
    CLK_SOURCE_PLL_24MHz = (0x40 | 20),
    CLK_SOURCE_PLL_20MHz = (0x40 | 24),
    CLK_SOURCE_PLL_15MHz = (0x40 | 0),
} compat_SYS_CLKTypeDef;

static void compat_SetSysClock2(compat_SYS_CLKTypeDef sc)
{
    uint32_t i;
    sys_safe_access_enable();
    R8_PLL_CONFIG &= ~(1 << 5); //
    sys_safe_access_disable();
    if(sc & 0x20)
    { // HSE div
        if(!(R8_HFCK_PWR_CTRL & RB_CLK_XT32M_PON))
        {
            sys_safe_access_enable();
            R8_HFCK_PWR_CTRL |= RB_CLK_XT32M_PON; // HSE power on
            for(i = 0; i < 1200; i++)
            {
                __nop();
                __nop();
            }
        }

        sys_safe_access_enable();
        R16_CLK_SYS_CFG = (0 << 6) | (sc & 0x1f);
        __nop();
        __nop();
        __nop();
        __nop();
        sys_safe_access_disable();
        sys_safe_access_enable();
        SAFEOPERATE;
        R8_FLASH_CFG = 0X51;
        sys_safe_access_disable();
    }

    else if(sc & 0x40)
    { // PLL div
        if(!(R8_HFCK_PWR_CTRL & RB_CLK_PLL_PON))
        {
            sys_safe_access_enable();
            R8_HFCK_PWR_CTRL |= RB_CLK_PLL_PON; // PLL power on
            for(i = 0; i < 2000; i++)
            {
                __nop();
                __nop();
            }
        }
        sys_safe_access_enable();
        R16_CLK_SYS_CFG = (1 << 6) | (sc & 0x1f);
        __nop();
        __nop();
        __nop();
        __nop();
        sys_safe_access_disable();
        if(sc == CLK_SOURCE_PLL_80MHz)
        {
            sys_safe_access_enable();
            R8_FLASH_CFG = 0X02;
            sys_safe_access_disable();
        }
        else
        {
            sys_safe_access_enable();
            R8_FLASH_CFG = 0X52;
            sys_safe_access_disable();
        }
    }
    else
    {
        sys_safe_access_enable();
        R16_CLK_SYS_CFG |= RB_CLK_SYS_MOD;
    }
    //更改FLASH clk的驱动能力
    sys_safe_access_enable();
    R8_PLL_CONFIG |= 1 << 7;
    sys_safe_access_disable();
}

void compat_SetSysClock(void)
{
	compat_SetSysClock2(CLK_SOURCE_PLL_60MHz);
} 

static uint32_t GetSysClock(void)
{
    uint16_t rev;

    rev = R16_CLK_SYS_CFG & 0xff;
    if((rev & 0x40) == (0 << 6))
    { // 32M进行分频
        return (32000000 / (rev & 0x1f));
    }
    else if((rev & RB_CLK_SYS_MOD) == (1 << 6))
    { // PLL进行分频
        return (480000000 / (rev & 0x1f));
    }
    else
    { // 32K做主频
        return (32000);
    }
}

typedef enum
{
    /* Ð£×¼¾«¶ÈÔ½¸ß£¬ºÄÊ±Ô½³¤ */
    Level_32 = 3, // ÓÃÊ± 1.2ms 1000ppm (32M Ö÷Æµ)  1100ppm (60M Ö÷Æµ)
    Level_64,     // ÓÃÊ± 2.2ms 800ppm  (32M Ö÷Æµ)  1000ppm (60M Ö÷Æµ)
    Level_128,    // ÓÃÊ± 4.2ms 600ppm  (32M Ö÷Æµ)  800ppm  (60M Ö÷Æµ)

} compat_Cali_LevelTypeDef;

static void compat_Calibration_LSI(compat_Cali_LevelTypeDef cali_Lv)
{
    UINT32 i;
    INT32  cnt_offset;
    UINT8  retry = 0;
    INT32  freq_sys;

    freq_sys = GetSysClock();

    sys_safe_access_enable();
    R8_CK32K_CONFIG |= RB_CLK_OSC32K_FILT;
    R8_CK32K_CONFIG &= ~RB_CLK_OSC32K_FILT;
    sys_safe_access_enable();
    R8_XT32K_TUNE &= ~3;
    R8_XT32K_TUNE |= 1;

    // ´Öµ÷
    sys_safe_access_enable();
    R8_OSC_CAL_CTRL &= ~RB_OSC_CNT_TOTAL;
    R8_OSC_CAL_CTRL |= 1;
    sys_safe_access_enable();
    R8_OSC_CAL_CTRL |= RB_OSC_CNT_EN;
    R16_OSC_CAL_CNT |= RB_OSC_CAL_OV_CLR;
    while( (R8_OSC_CAL_CTRL &RB_OSC_CNT_EN) == 0 )
    {
        sys_safe_access_enable();
        R8_OSC_CAL_CTRL |= RB_OSC_CNT_EN;
    }
    while(1)
    {
        while(!(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT));
        i = R16_OSC_CAL_CNT; // ÓÃÓÚ¶ªÆú
        while(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT);
        R16_OSC_CAL_CNT |= RB_OSC_CAL_OV_CLR;
        while(!(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT));
        i = R16_OSC_CAL_CNT; // ÊµÊ±Ð£×¼ºó²ÉÑùÖµ
        cnt_offset = (i & 0x3FFF) + R8_OSC_CAL_OV_CNT * 0x3FFF - 2000 * (freq_sys / 1000) / CAB_LSIFQ;
        if(((cnt_offset > -37 * (freq_sys / 1000) / CAB_LSIFQ) && (cnt_offset < 37 * (freq_sys / 1000) / CAB_LSIFQ)) || retry > 2)
        {
            if(retry)
                break;
        }
        retry++;
        cnt_offset = (cnt_offset > 0) ? (((cnt_offset * 2) / (74 * (freq_sys/1000) / 60000)) + 1) / 2 : (((cnt_offset * 2) / (74 * (freq_sys/1000) / 60000 )) - 1) / 2;
        sys_safe_access_enable();
        R16_INT32K_TUNE += cnt_offset;
    }

    // Ï¸µ÷
    // ÅäÖÃÏ¸µ÷²ÎÊýºó£¬¶ªÆú2´Î²¶»ñÖµ£¨Èí¼þÐÐÎª£©ÉÏÅÐ¶ÏÒÑÓÐÒ»´Î£¬ÕâÀïÖ»ÁôÒ»´Î
    while(!(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT));
    i = R16_OSC_CAL_CNT; // ÓÃÓÚ¶ªÆú
    R16_OSC_CAL_CNT |= RB_OSC_CAL_OV_CLR;
    sys_safe_access_enable();
    R8_OSC_CAL_CTRL &= ~RB_OSC_CNT_TOTAL;
    R8_OSC_CAL_CTRL |= cali_Lv;
    while( (R8_OSC_CAL_CTRL&0x07) != cali_Lv )
    {
        sys_safe_access_enable();
        R8_OSC_CAL_CTRL |= cali_Lv;
    }
    while(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT);
    while(!(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT));
    i = R16_OSC_CAL_CNT; // ÊµÊ±Ð£×¼ºó²ÉÑùÖµ

    cnt_offset = (i & 0x3FFF) + R8_OSC_CAL_OV_CNT * 0x3FFF - 4000 * (1 << cali_Lv) * (freq_sys / 1000000) / 256 * 1000/(CAB_LSIFQ/256) ;
    cnt_offset = (cnt_offset > 0) ? ((((cnt_offset * 2*(100 )) / (1366 * ((1 << cali_Lv)/8) * (freq_sys/1000) / 60000)) + 1) / 2)<<5 : ((((cnt_offset * 2*(100)) / (1366 * ((1 << cali_Lv)/8) * (freq_sys/1000) / 60000)) - 1) / 2)<<5;
    sys_safe_access_enable();
    R16_INT32K_TUNE += cnt_offset;
    R8_OSC_CAL_CTRL &= ~RB_OSC_CNT_EN;
}


static void Lib_Calibration_LSI(void) {
	compat_Calibration_LSI(Level_64);
}
static uint32_t SYS_GetSysTickCnt(void)
{
    uint32_t val;

    val = SysTick->CNT;
    return (val);
}

void compat_CH58X_BLEInit(void)
{
    uint8_t     i;
    bleConfig_t cfg;
    if(tmos_memcmp(VER_LIB, VER_FILE, strlen(VER_FILE)) == FALSE)
    {
        PRINT("head file error...\n");
        while(1);
    }
    SysTick_Config(SysTick_LOAD_RELOAD_Msk);

    tmos_memset(&cfg, 0, sizeof(bleConfig_t));
    cfg.MEMAddr = (uint32_t)MEM_BUF;
    cfg.MEMLen = (uint32_t)BLE_MEMHEAP_SIZE;
    cfg.BufMaxLen = (uint32_t)BLE_BUFF_MAX_LEN;
    cfg.BufNumber = (uint32_t)BLE_BUFF_NUM;
    cfg.TxNumEvent = (uint32_t)BLE_TX_NUM_EVENT;
    cfg.TxPower = (uint32_t)BLE_TX_POWER;
#if(defined(BLE_SNV)) && (BLE_SNV == TRUE)
    cfg.SNVAddr = (uint32_t)BLE_SNV_ADDR;
    cfg.readFlashCB = Lib_Read_Flash;
    cfg.writeFlashCB = Lib_Write_Flash;
#endif
#if(CLK_OSC32K)
    cfg.SelRTCClock = (uint32_t)CLK_OSC32K;
#endif
    cfg.ConnectNumber = (PERIPHERAL_MAX_CONNECTION & 3) | (CENTRAL_MAX_CONNECTION << 2);
    cfg.srandCB = SYS_GetSysTickCnt;
#if(defined TEM_SAMPLE) && (TEM_SAMPLE == TRUE)
    cfg.tsCB = HAL_GetInterTempValue; // ¸ù¾ÝÎÂ¶È±ä»¯Ð£×¼RFºÍÄÚ²¿RC( ´óÓÚ7ÉãÊÏ¶È )
  #if(CLK_OSC32K)
    cfg.rcCB = Lib_Calibration_LSI; // ÄÚ²¿32KÊ±ÖÓÐ£×¼
  #endif
#endif
#if(defined(HAL_SLEEP)) && (HAL_SLEEP == TRUE)
    cfg.WakeUpTime = WAKE_UP_RTC_MAX_TIME;
    cfg.sleepCB = CH58X_LowPower; // ÆôÓÃË¯Ãß
#endif
#if(defined(BLE_MAC)) && (BLE_MAC == TRUE)
    for(i = 0; i < 6; i++)
    {
        cfg.MacAddr[i] = MacAddr[5 - i];
    }
#else
    {
        uint8_t MacAddr[6];
        GetMACAddress(MacAddr);
        for(i = 0; i < 6; i++)
        {
            cfg.MacAddr[i] = MacAddr[i]; // Ê¹ÓÃÐ¾Æ¬macµØÖ·
        }
    }
#endif
    if(!cfg.MEMAddr || cfg.MEMLen < 4 * 1024)
    {
        while(1);
    }
    i = BLE_LibInit(&cfg);
    if(i)
    {
        PRINT("LIB init error code: %x ...\n", i);
        while(1);
    }
}

void compat_HAL_TimeInit(void)
{
#if(CLK_OSC32K)
    sys_safe_access_enable();
    R8_CK32K_CONFIG &= ~(RB_CLK_OSC32K_XT | RB_CLK_XT32K_PON);
    sys_safe_access_enable();
    R8_CK32K_CONFIG |= RB_CLK_INT32K_PON;
    sys_safe_access_disable();
    Lib_Calibration_LSI();
#else
    sys_safe_access_enable();
    R8_CK32K_CONFIG |= RB_CLK_OSC32K_XT | RB_CLK_INT32K_PON | RB_CLK_XT32K_PON;
    sys_safe_access_disable();
#endif
    //RTC_InitTime(2020, 1, 1, 0, 0, 0); //RTCÊ±ÖÓ³õÊ¼»¯µ±Ç°Ê±¼ä
    TMOS_TimerInit(0);
}

