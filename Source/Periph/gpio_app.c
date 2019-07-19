#include "gpio_app.h"
#include "gpio.h"
#include "stdint.h"
#include "soc_C6748.h"
#include "hw_types.h"
#include "hw_syscfg0_C6748.h"
#include "psc.h"
#include "canModule.h"

static void GPIOBankPinInterruputInit(void);
static void GPIOBankPinMuxSet(void);
static void GPIOBankPinInit(void);
static void GPIOBank6Pin0PinMuxSetup(void);
static void GPIOBank6Pin15PinMuxSetup(void);
static void GPIOBank6Pin14PinMuxSetup(void);
static void GPIOBank6Pin13PinMuxSetup(void);
static void GPIOBank6Pin10PinMuxSetup(void);
static void GPIOBank0Pin0PinMuxSetup(void);
static void GPIOBank0Pin1PinMuxSetup(void);
static void GPIOBank0Pin5PinMuxSetup(void);
static void GPIOBank2Pin15PinMuxSetup(void);
/* FPGA DONE信号*/
static void GPIOBank8Pin10PinMuxSetup(void);


static void rst_delay(uint32_t n)
{
    uint32_t i;

    for (i = 0; i < n; i++);
}


static void GPIOBankPinInterruputInit(void)
{
    // 配置UART为上升沿中断
    //GPIOIntTypeSet(SOC_GPIO_0_REGS, 111, GPIO_INT_TYPE_RISEDGE);
	GPIOIntTypeSet(SOC_GPIO_0_REGS, GPIO_MPU9250_INT, GPIO_INT_TYPE_FALLEDGE);
    GPIOIntTypeSet(SOC_GPIO_0_REGS, GPIO_UART_INT, GPIO_INT_TYPE_RISEDGE);
    GPIOIntTypeSet(SOC_GPIO_0_REGS, GPIO_CAN_INT, GPIO_INT_TYPE_RISEDGE);
    GPIOIntTypeSet(SOC_GPIO_0_REGS, GPIO_SIGNAL_INT, GPIO_INT_TYPE_RISEDGE);
    /* Enable interrupts for Bank*/
    GPIOBankIntEnable(SOC_GPIO_0_REGS, 6);
    GPIOBankIntEnable(SOC_GPIO_0_REGS, 0);
}


static void PSCInit(void)
{
    // 使能GPIO电源
    PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_GPIO, PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);
}
static void GPIOBankPinMuxSet(void)
{
    GPIOBank6Pin0PinMuxSetup();  //run led
    GPIOBank6Pin15PinMuxSetup(); //fpga rst
    //GPIOBank6Pin14PinMuxSetup(); //test int
    GPIOBank6Pin10PinMuxSetup(); //MPU9250 int
    GPIOBank0Pin0PinMuxSetup();  //uart int
    GPIOBank0Pin1PinMuxSetup();  //can int
    GPIOBank0Pin5PinMuxSetup();  //Signal int
    /* FPGA DONE信号 */
    GPIOBank8Pin10PinMuxSetup();
    //4G 模块复位 调试时为了restart后不用等4G上电，先屏蔽，等后面再加入
    GPIOBank2Pin15PinMuxSetup();  //LTE Reset
    GPIOBank6Pin13PinMuxSetup();    //EEPROM Write Protect
}

static void GPIOBankPinInit(void)
{
    GPIODirModeSet(SOC_GPIO_0_REGS, GPIO_RUN_LED, GPIO_DIR_OUTPUT);
    GPIODirModeSet(SOC_GPIO_0_REGS, GPIO_FPGA_RST, GPIO_DIR_OUTPUT);
    /*
     * 释放EEPROM的写保护
     */
    GPIODirModeSet(SOC_GPIO_0_REGS, GPIO_EEPROM_WP, GPIO_DIR_OUTPUT);
    GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_EEPROM_WP, GPIO_PIN_LOW);

    //GPIODirModeSet(SOC_GPIO_0_REGS, 111, GPIO_DIR_INPUT);  // GPIO6[14]
    GPIODirModeSet(SOC_GPIO_0_REGS, GPIO_MPU9250_INT, GPIO_DIR_INPUT);
    GPIODirModeSet(SOC_GPIO_0_REGS, GPIO_UART_INT, GPIO_DIR_INPUT);
    GPIODirModeSet(SOC_GPIO_0_REGS, GPIO_CAN_INT, GPIO_DIR_INPUT);
    GPIODirModeSet(SOC_GPIO_0_REGS, GPIO_SIGNAL_INT, GPIO_DIR_INPUT);
    GPIODirModeSet(SOC_GPIO_0_REGS, GPIO_FPGA_DONE, GPIO_DIR_INPUT);

    //4G 模块复位 调试时为了restart后不用等4G上电，先屏蔽，等后面再加入

    GPIODirModeSet(SOC_GPIO_0_REGS, GPIO_LTE_RESET, GPIO_DIR_OUTPUT);  // GPIO2[15]
    GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_LTE_RESET, GPIO_PIN_LOW);
    rst_delay(10000);
    GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_LTE_RESET, GPIO_PIN_HIGH);

}

static void GPIOBank6Pin0PinMuxSetup(void)  
{
     unsigned int savePinmux = 0;

     /*
     ** Clearing the bit in context and retaining the other bit values
     ** in PINMUX19 register.
     */
     savePinmux = (HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(19)) &
                  ~(SYSCFG_PINMUX19_PINMUX19_27_24));

     /* Setting the pins corresponding to GP6[0] in PINMUX19 register.*/
     HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(19)) =
          (PINMUX19_GPIO6_0_ENABLE | savePinmux);

}

static void GPIOBank2Pin15PinMuxSetup(void)
{
     unsigned int savePinmux = 0;

     /*
     ** Clearing the bit in context and retaining the other bit values
     ** in PINMUX19 register.
     */
     savePinmux = (HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(5)) &
                  ~(SYSCFG_PINMUX5_PINMUX5_3_0));

     /* Setting the pins corresponding to GP6[0] in PINMUX19 register.*/
     HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(5)) =
          (PINMUX19_GPIO2_15_ENABLE | savePinmux);

}


static void GPIOBank6Pin15PinMuxSetup(void)  
{
     unsigned int savePinmux = 0;

     /*
     ** Clearing the bit in context and retaining the other bit values
     ** in PINMUX13 register.
     */
     savePinmux = (HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(13)) &
                  ~(SYSCFG_PINMUX13_PINMUX13_3_0));

     /* Setting the pins corresponding to GP6[15] in PINMUX13 register.*/
     HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(13)) =
          (PINMUX13_GPIO6_15_ENABLE | savePinmux);

}

static void GPIOBank6Pin13PinMuxSetup(void)
{
    unsigned int savePinmux = 0;

    /*
     ** Clearing the bit in context and retaining the other bit values
     ** in PINMUX13 register.
     */
    savePinmux = (HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(13)) &
                  ~(SYSCFG_PINMUX13_PINMUX13_11_8));

    /* Setting the pins corresponding to GP6[14] in PINMUX13 register.*/
    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(13)) =
        (PINMUX13_GPIO6_13_ENABLE | savePinmux);
}

static void GPIOBank6Pin14PinMuxSetup(void)
{
    unsigned int savePinmux = 0;

    /*
     ** Clearing the bit in context and retaining the other bit values
     ** in PINMUX13 register.
     */
    savePinmux = (HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(13)) &
                  ~(SYSCFG_PINMUX13_PINMUX13_7_4));

    /* Setting the pins corresponding to GP6[14] in PINMUX13 register.*/
    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(13)) =
        (PINMUX13_GPIO6_14_ENABLE | savePinmux);
}

static void GPIOBank6Pin10PinMuxSetup(void)
{
    unsigned int savePinmux = 0;

    /*
     ** Clearing the bit in context and retaining the other bit values
     ** in PINMUX13 register.
     */
    savePinmux = (HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(13)) &
                  ~(SYSCFG_PINMUX13_PINMUX13_23_20));

    /* Setting the pins corresponding to GP6[14] in PINMUX13 register.*/
    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(13)) =
        (PINMUX13_GPIO6_10_ENABLE | savePinmux);
}

static void GPIOBank0Pin0PinMuxSetup(void)
{
    unsigned int savePinmux = 0;

    /*
     ** Clearing the bit in context and retaining the other bit values
     ** in PINMUX1 register.
     */
    savePinmux = (HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(1)) &
                  ~(SYSCFG_PINMUX1_PINMUX1_31_28));

    /* Setting the pins corresponding to GP0[0] in PINMUX1 register.*/
    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(1)) =
        (PINMUX1_GPIO0_0_ENABLE | savePinmux);
}

static void GPIOBank0Pin1PinMuxSetup(void)
{
    unsigned int savePinmux = 0;

    /*
     ** Clearing the bit in context and retaining the other bit values
     ** in PINMUX13 register.
     */
    savePinmux = (HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(1)) &
                  ~(SYSCFG_PINMUX1_PINMUX1_27_24));

    /* Setting the pins corresponding to GP0[1] in PINMUX1 register.*/
    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(1)) =
        (PINMUX1_GPIO0_1_ENABLE | savePinmux);
}

static void GPIOBank0Pin5PinMuxSetup(void)
{
    unsigned int savePinmux = 0;

    /*
     ** Clearing the bit in context and retaining the other bit values
     ** in PINMUX1 register.
     */
    savePinmux = (HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(1)) &
                  ~(SYSCFG_PINMUX1_PINMUX1_11_8));

    /* Setting the pins corresponding to GP0[0] in PINMUX1 register.*/
    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(1)) =
        (PINMUX1_GPIO0_5_ENABLE | savePinmux);
}

static void GPIOBank8Pin10PinMuxSetup(void)
{
    unsigned int savePinmux = 0;

    /*
     ** Clearing the bit in context and retaining the other bit values
     ** in PINMUX18 register.
     */
    savePinmux = (HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(18)) &
                  ~(SYSCFG_PINMUX18_PINMUX18_31_28));

    /* Setting the pins corresponding to GP6[14] in PINMUX13 register.*/
    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(18)) =
        (PINMUX13_GPIO8_10_ENABLE | savePinmux);
}

//接口==========================
void GPIOFpgaReset(void)
{
    GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_FPGA_RST, GPIO_PIN_LOW);
    rst_delay(0xfffff);
    GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_FPGA_RST, GPIO_PIN_HIGH);
    rst_delay(0xfffff);
}

void GPIOWaitFpgaDone(void)
{
	while(!GPIOPinRead(SOC_GPIO_0_REGS, GPIO_FPGA_DONE));
}

void GPIOToggleLed(void)
{
    static uint8_t led_sta=0;

    if(led_sta==0)
        GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_RUN_LED, GPIO_PIN_LOW);
    else
        GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_RUN_LED, GPIO_PIN_HIGH);

    led_sta=~led_sta;
}

void GPIOInit(void)
{
	PSCInit();
    GPIOBankPinMuxSet();
    GPIOBankPinInit();
    GPIOBankPinInterruputInit();
}
