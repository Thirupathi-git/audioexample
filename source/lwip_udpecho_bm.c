/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "lwip/opt.h"
#include "lwip/igmp.h"
#if LWIP_UDP

#include "udpecho_raw.h"
#include "lwip/timeouts.h"
#include "lwip/init.h"
#include "netif/ethernet.h"
#include "enet_ethernetif.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "netif/etharp.h"
#include "lwip/netif.h"
#include "lwip/udp.h"

#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* IP address configuration. */
#define configIP_ADDR0 192
#define configIP_ADDR1 168
#define configIP_ADDR2 0
#define configIP_ADDR3 102

/* Netmask configuration. */
#define configNET_MASK0 255
#define configNET_MASK1 255
#define configNET_MASK2 255
#define configNET_MASK3 0

/* Gateway address configuration. */
#define configGW_ADDR0 192
#define configGW_ADDR1 168
#define configGW_ADDR2 0
#define configGW_ADDR3 100

/* MAC address configuration. */
#define configMAC_ADDR                     \
    {                                      \
        0x02, 0x12, 0x13, 0x10, 0x15, 0x11 \
    }

/* Address of PHY interface. */
#define EXAMPLE_PHY_ADDRESS BOARD_ENET0_PHY_ADDRESS

/* System clock name. */
#define EXAMPLE_CLOCK_NAME kCLOCK_CoreSysClk


#ifndef EXAMPLE_NETIF_INIT_FN
/*! @brief Network interface initialization function. */
#define EXAMPLE_NETIF_INIT_FN ethernetif0_init
#endif /* EXAMPLE_NETIF_INIT_FN */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void BOARD_InitModuleClock(void)
{
    const clock_enet_pll_config_t config = {.enableClkOutput = true, .enableClkOutput25M = false, .loopDivider = 1};
    CLOCK_InitEnetPll(&config);
}

void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 1000000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}



/*!
 * @brief Interrupt service for SysTick timer.
 */
void SysTick_Handler(void)
{
    time_isr();
}
/****************************************************************/
/****************************************************************/
void UdpServerInit(void);
static struct udp_pcb *MyUDPServer;
#define MY_UDP_PORT 2099
static void receiveFunction(void *arg, struct udp_pcb *pcb, struct pbuf *p,
    const ip_addr_t *addr, u16_t port);
void SaiInterruptRedPlayInit(void);
void whileSaiInterrupt(void);
/*!
 * @brief Main function.
 */




/*###################################################################*/
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_sai.h"
#include "fsl_codec_common.h"

#include "fsl_wm8960.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_codec_adapter.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* SAI instance and clock */
#define DEMO_CODEC_WM8960
#define DEMO_SAI SAI1
#define DEMO_SAI_CHANNEL (0)
#define DEMO_SAI_BITWIDTH (kSAI_WordWidth16bits)
//#define DEMO_SAI_IRQ SAI1_IRQn
//#define SAI_TxIRQHandler SAI1_IRQHandler

/* Select Audio/Video PLL (786.48 MHz) as sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_SELECT (2U)
/* Clock pre divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER (0U)
/* Clock divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_DIVIDER (63U)
/* Get frequency of sai1 clock */
#define DEMO_SAI_CLK_FREQ                                                        \
    (CLOCK_GetFreq(kCLOCK_AudioPllClk) / (DEMO_SAI1_CLOCK_SOURCE_DIVIDER + 1U) / \
     (DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER + 1U))

/* I2C instance and clock */
#define DEMO_I2C LPI2C1

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define DEMO_LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define DEMO_LPI2C_CLOCK_SOURCE_DIVIDER (5U)
/* Get frequency of lpi2c clock */
#define DEMO_I2C_CLK_FREQ ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (DEMO_LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

#define OVER_SAMPLE_RATE (384U)
#define BUFFER_SIZE (1024U)
#define BUFFER_NUMBER (4U)
/* demo audio sample rate */
#define DEMO_AUDIO_SAMPLE_RATE (kSAI_SampleRate16KHz)
/* demo audio master clock */
#if (defined FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER && FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) || \
    (defined FSL_FEATURE_PCC_HAS_SAI_DIVIDER && FSL_FEATURE_PCC_HAS_SAI_DIVIDER)
#define DEMO_AUDIO_MASTER_CLOCK OVER_SAMPLE_RATE *DEMO_AUDIO_SAMPLE_RATE
#else
#define DEMO_AUDIO_MASTER_CLOCK DEMO_SAI_CLK_FREQ
#endif
/* demo audio data channel */
#define DEMO_AUDIO_DATA_CHANNEL (2U)
/* demo audio bit width */
#define DEMO_AUDIO_BIT_WIDTH kSAI_WordWidth16bits

#ifndef DEMO_SAI_TX_SYNC_MODE
#define DEMO_SAI_TX_SYNC_MODE kSAI_ModeAsync
#endif
#ifndef DEMO_SAI_RX_SYNC_MODE
#define DEMO_SAI_RX_SYNC_MODE kSAI_ModeSync
#endif
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
wm8960_config_t wm8960Config = {
    .i2cConfig = {.codecI2CInstance = BOARD_CODEC_I2C_INSTANCE, .codecI2CSourceClock = BOARD_CODEC_I2C_CLOCK_FREQ},
    .route     = kWM8960_RoutePlaybackandRecord,
    .rightInputSource = kWM8960_InputDifferentialMicInput2,
    .playSource       = kWM8960_PlaySourceDAC,
    .slaveAddress     = WM8960_I2C_ADDR,
    .bus              = kWM8960_BusI2S,
    .format = {.mclk_HZ = 6144000U, .sampleRate = kWM8960_AudioSampleRate16KHz, .bitWidth = kWM8960_AudioBitWidth16bit},
    .master_slave = false,
};
codec_config_t boardCodecConfig = {.codecDevType = kCODEC_WM8960, .codecDevConfig = &wm8960Config};

/*
 * AUDIO PLL setting: Frequency = Fref * (DIV_SELECT + NUM / DENOM)
 *                              = 24 * (32 + 77/100)
 *                              = 786.48 MHz
 */
const clock_audio_pll_config_t audioPllConfig = {
    .loopDivider = 32,  /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
    .postDivider = 1,   /* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
    .numerator   = 77,  /* 30 bit numerator of fractional loop divider. */
    .denominator = 100, /* 30 bit denominator of fractional loop divider */
};
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t Buffer[BUFFER_NUMBER * BUFFER_SIZE], 4);
sai_handle_t txHandle = {0}, rxHandle = {0};
static uint32_t tx_index = 0U, rx_index = 0U;
volatile uint32_t emptyBlock = BUFFER_NUMBER;
extern codec_config_t boardCodecConfig;
#if (defined(FSL_FEATURE_SAI_HAS_MCR) && (FSL_FEATURE_SAI_HAS_MCR)) || \
    (defined(FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) && (FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER))
sai_master_clock_t mclkConfig = {
#if defined(FSL_FEATURE_SAI_HAS_MCR) && (FSL_FEATURE_SAI_HAS_MCR)
    .mclkOutputEnable = true,
#if !(defined(FSL_FEATURE_SAI_HAS_NO_MCR_MICS) && (FSL_FEATURE_SAI_HAS_NO_MCR_MICS))
    .mclkSource = kSAI_MclkSourceSysclk,
#endif
#endif
};
#endif
codec_handle_t codecHandle;

/*******************************************************************************
 * Code
 ******************************************************************************/




uint8_t Mu8_TxIndex = 0;
void BOARD_EnableSaiMclkOutput(bool enable)
{
    if (enable)
    {
        IOMUXC_GPR->GPR1 |= IOMUXC_GPR_GPR1_SAI1_MCLK_DIR_MASK;
    }
    else
    {
        IOMUXC_GPR->GPR1 &= (~IOMUXC_GPR_GPR1_SAI1_MCLK_DIR_MASK);
    }
}

static void rx_callback(I2S_Type *base, sai_handle_t *handle, status_t status, void *userData)
{
    if (kStatus_SAI_RxError == status)
    {
        /* Handle the error. */
    }
    else
    {

    }
}

static void tx_callback(I2S_Type *base, sai_handle_t *handle, status_t status, void *userData)
{
    if (kStatus_SAI_TxError == status)
    {
        /* Handle the error. */
    }
    else
    {

    	tx_index = 0;

    }
}

/*!
 * @brief Main function
 */









sai_transfer_t xfer;
sai_transceiver_t config;
struct pbuf pbuffer;
static uint8_t SendingData[BUFFER_SIZE];



struct ip4_addr ipgroup, localIP;
struct udp_pcb *g_udppcb;

struct ip4_addr reciveIP;
static void UDP_Multicast_init(void );


int main(void)
{
    struct netif netif;
    ip4_addr_t netif_ipaddr, netif_netmask, netif_gw;
#if defined(FSL_FEATURE_SOC_LPC_ENET_COUNT) && (FSL_FEATURE_SOC_LPC_ENET_COUNT > 0)
    mem_range_t non_dma_memory[] = NON_DMA_MEMORY_ARRAY;
#endif /* FSL_FEATURE_SOC_LPC_ENET_COUNT */
    ethernetif_config_t enet_config = {
        .phyAddress = EXAMPLE_PHY_ADDRESS,
        .clockName  = EXAMPLE_CLOCK_NAME,
        .macAddress = configMAC_ADDR,
#if defined(FSL_FEATURE_SOC_LPC_ENET_COUNT) && (FSL_FEATURE_SOC_LPC_ENET_COUNT > 0)
        .non_dma_memory = non_dma_memory,
#endif /* FSL_FEATURE_SOC_LPC_ENET_COUNT */
    };

    gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    SaiInterruptRedPlayInit();
    BOARD_InitModuleClock();
    IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET1TxClkOutputDir, true);

    GPIO_PinInit(GPIO1, 9, &gpio_config);
    GPIO_PinInit(GPIO1, 10, &gpio_config);
    /* pull up the ENET_INT before RESET. */
    GPIO_WritePinOutput(GPIO1, 10, 1);
    GPIO_WritePinOutput(GPIO1, 9, 0);
    delay();
    GPIO_WritePinOutput(GPIO1, 9, 1);

    time_init();

    IP4_ADDR(&netif_ipaddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3);
    IP4_ADDR(&netif_netmask, configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3);
    IP4_ADDR(&netif_gw, configGW_ADDR0, configGW_ADDR1, configGW_ADDR2, configGW_ADDR3);

    lwip_init();

    netif_add(&netif, &netif_ipaddr, &netif_netmask, &netif_gw, &enet_config, EXAMPLE_NETIF_INIT_FN, ethernet_input);
    netif_set_default(&netif);
    netif_set_up(&netif);

    udpecho_raw_init();

    PRINTF("\r\n************************************************\r\n");
    PRINTF(" UDP Echo example\r\n");
    PRINTF("************************************************\r\n");
    PRINTF(" IPv4 Address     : %u.%u.%u.%u\r\n", ((u8_t *)&netif_ipaddr)[0], ((u8_t *)&netif_ipaddr)[1],
           ((u8_t *)&netif_ipaddr)[2], ((u8_t *)&netif_ipaddr)[3]);
    PRINTF(" IPv4 Subnet mask : %u.%u.%u.%u\r\n", ((u8_t *)&netif_netmask)[0], ((u8_t *)&netif_netmask)[1],
           ((u8_t *)&netif_netmask)[2], ((u8_t *)&netif_netmask)[3]);
    PRINTF(" IPv4 Gateway     : %u.%u.%u.%u\r\n", ((u8_t *)&netif_gw)[0], ((u8_t *)&netif_gw)[1],
           ((u8_t *)&netif_gw)[2], ((u8_t *)&netif_gw)[3]);
    PRINTF("************************************************\r\n");
    UdpServerInit();
    UDP_Multicast_init();


    while (1)
    {
        /* Poll the driver, get any outstanding frames */
        ethernetif_input(&netif);

        sys_check_timeouts(); /* Handle all system timeouts for all core protocols */
        whileSaiInterrupt();
    }
}
void UdpServerInit(void)
{
	MyUDPServer = udp_new();
	if(!MyUDPServer)
	{
		PRINTF("Pointer init failed\r\n");

	}
	err_t err = udp_bind(MyUDPServer,IP_ADDR_ANY,MY_UDP_PORT);
	if(err != ERR_OK)
	{
		PRINTF("Unable to bind\r\n");
	}
	udp_recv(MyUDPServer,receiveFunction,NULL);

}
static void receiveFunction(void *arg, struct udp_pcb *pcb, struct pbuf *p,
    const ip_addr_t *addr, u16_t port)
{
	PRINTF("Data received in user defined function\r\n");
	char *ptr;
	ptr = (char *)p->payload;
	PRINTF("Data received is %s",ptr);
	//udp_sendto(MyUDPServer,p,addr,port);


	/*################################*/
	struct pbuf *pBuffer;
	uint16_t length = BUFFER_SIZE;
	pBuffer = pbuf_alloc(PBUF_TRANSPORT,length,PBUF_POOL);
	if(!pBuffer)
	{
		PRINTF("memory allocation failed");
	}
    memcpy(pBuffer->payload,SendingData,length);

	udp_sendto(MyUDPServer,pBuffer,addr,port);


}

void SaiInterruptRedPlayInit()
{


	 //   BOARD_ConfigMPU();
	   // BOARD_InitPins();
	//    BOARD_BootClockRUN();
	    CLOCK_InitAudioPll(&audioPllConfig);
	 //   BOARD_InitDebugConsole();

	    /*Clock setting for LPI2C*/
	    CLOCK_SetMux(kCLOCK_Lpi2cMux, DEMO_LPI2C_CLOCK_SOURCE_SELECT);
	    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, DEMO_LPI2C_CLOCK_SOURCE_DIVIDER);

	    /*Clock setting for SAI1*/
	    CLOCK_SetMux(kCLOCK_Sai1Mux, DEMO_SAI1_CLOCK_SOURCE_SELECT);
	    CLOCK_SetDiv(kCLOCK_Sai1PreDiv, DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER);
	    CLOCK_SetDiv(kCLOCK_Sai1Div, DEMO_SAI1_CLOCK_SOURCE_DIVIDER);

	    /*Enable MCLK clock*/
	    BOARD_EnableSaiMclkOutput(true);

	    PRINTF("SAI example started!\n\r");

	    /* SAI init */
	    SAI_Init(DEMO_SAI);
	    SAI_TransferTxCreateHandle(DEMO_SAI, &txHandle, tx_callback, NULL);
	    SAI_TransferRxCreateHandle(DEMO_SAI, &rxHandle, rx_callback, NULL);

	    /* I2S mode configurations */
	    SAI_GetClassicI2SConfig(&config, DEMO_AUDIO_BIT_WIDTH, kSAI_Stereo, kSAI_Channel0Mask);
	    config.syncMode = DEMO_SAI_TX_SYNC_MODE;
	    SAI_TransferTxSetConfig(DEMO_SAI, &txHandle, &config);
	    config.syncMode = DEMO_SAI_RX_SYNC_MODE;
	    SAI_TransferRxSetConfig(DEMO_SAI, &rxHandle, &config);

	    /* set bit clock divider */
	    SAI_TxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, DEMO_AUDIO_SAMPLE_RATE, DEMO_AUDIO_BIT_WIDTH,
	                          DEMO_AUDIO_DATA_CHANNEL);
	    SAI_RxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, DEMO_AUDIO_SAMPLE_RATE, DEMO_AUDIO_BIT_WIDTH,
	                          DEMO_AUDIO_DATA_CHANNEL);

	    /* master clock configurations */
	#if (defined(FSL_FEATURE_SAI_HAS_MCR) && (FSL_FEATURE_SAI_HAS_MCR)) || \
	    (defined(FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) && (FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER))
	#if defined(FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) && (FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER)
	    mclkConfig.mclkHz          = DEMO_AUDIO_MASTER_CLOCK;
	    mclkConfig.mclkSourceClkHz = DEMO_SAI_CLK_FREQ;
	#endif
	    SAI_SetMasterClockConfig(DEMO_SAI, &mclkConfig);
	#endif

	    /* Use default setting to init codec */
	   CODEC_Init(&codecHandle, &boardCodecConfig);
}

AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t Mu8_tx_data[20 * BUFFER_SIZE], 4);


uint8_t Mu8_RxIndex = 0;
uint16_t Mu16_rxSize = 0;
static uint8_t Mu8_rx_data[BUFFER_SIZE];
void whileSaiInterrupt(void)
{




	   if ((Mu8_TxIndex !=  Mu8_RxIndex) )
	    {
			   sai_transfer_t transferLocal;
			   transferLocal.data = Mu8_tx_data + (Mu8_TxIndex * Mu16_rxSize);
			   transferLocal.dataSize = Mu16_rxSize;
			   if (kStatus_Success == SAI_TransferSendNonBlocking(DEMO_SAI, &txHandle, &transferLocal))
			   {

				    tx_index++;
					Mu8_TxIndex++;
					Mu8_TxIndex = Mu8_TxIndex % 20;

			   }


	    }





        xfer.data     = Mu8_rx_data;
        xfer.dataSize = BUFFER_SIZE;
        if (kStatus_Success == SAI_TransferReceiveNonBlocking(DEMO_SAI, &rxHandle, &xfer))
        {
            rx_index++;



            ////////////////////////////////////////

			struct pbuf *pBuffer;
			uint16_t length = BUFFER_SIZE;
			pBuffer = pbuf_alloc(PBUF_TRANSPORT,length,PBUF_POOL);
			if(!pBuffer)
			{
				PRINTF("memory allocation failed");
			}
			else
			{
				memcpy(pBuffer->payload,xfer.data,length);
#if 1
				udp_sendto(g_udppcb,pBuffer,&ipgroup,8080); //send a multicast packet
#endif
			}
			pbuf_free(pBuffer);
        }



}


void recCallBack(void *arg, struct udp_pcb *pcb, struct pbuf *p,
    const ip_addr_t *addr, u16_t port)
{
	static uint8_t s_allocate_memory = 1;
	//if(s_allocate_memory == 1)
	{
		Mu16_rxSize = p->len;
//		Mu8_tx_data = (uint8_t *)calloc(sizeof(uint8_t),Mu16_rxSize * 10);
		s_allocate_memory = 0;
	}
	memcpy(&Mu8_tx_data[Mu8_RxIndex * Mu16_rxSize],(uint8_t *)p->payload,p->len);
	Mu8_RxIndex ++;
	Mu8_RxIndex = Mu8_RxIndex % 20;
	pbuf_free(p);
}
static void UDP_Multicast_init(void )
{
   IP4_ADDR(&ipgroup, 224, 0, 1, 129 ); //Multicast IP address.
   IP4_ADDR(&localIP, 192, 168, 0, 102); //Interface IP address

   IP4_ADDR(&reciveIP, 224, 0, 1, 133 ); //Multicast IP address.
   #if LWIP_IGMP
      s8_t iret = igmp_joingroup((ip4_addr_t *)(&localIP),(ip4_addr_t *)(&ipgroup));
      s8_t iret1= igmp_joingroup((ip4_addr_t *)(&localIP),(ip4_addr_t *)(&reciveIP));
      PRINTF("IGMP ENABLED");
      PRINTF("%d",iret);
   #endif
   g_udppcb =( struct udp_pcb*) udp_new();
   udp_bind(g_udppcb, &reciveIP, 8080); //to allow receiving multicast
   udp_recv(g_udppcb, recCallBack,NULL); //recCallBack is the callback function that will be called every time you    receive multicast

}

#endif
