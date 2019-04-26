#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "math.h"
#include "string.h"
#include "DSP2833x_GlobalPrototypes.h"
#include "Example_freqcal.h"
#include "Example_posspeed.h"
#include "IQmathLib.h"

/* 自定义**/
#include "UART.h"


//###########################################################################
// $TI Release: DSP2833x Header Files V1.01 $
// $Release Date: September 26, 2007 $
//###########################################################################

#define T1PRD 0xFFFF      //
#define T2PRD 0xFFFF      //
#define T3PRD 0xEFFF       //
#define T6PRD 0x1FFF      //

//******Variables in different Programs******//
  //****** Main ******//
    int ProgStep = 1;   //ProgStep 1 位置回路  0 速度回路
    int Work_Mode = 0;
    int Rd_Mode_Select = 0;  //雷达工作模式   0为调试状态     1为工作状态
    int Set_Mode = 0;  //设备架设方式  00固定式   01移动式
    int Servo_Mode = 0; //伺服工作模式

    int Cmd_Query = 0;  //命令查询
    float PulseCounterOld = 0;
    float RevovorAngleOld = 0;
    int Flag_Fw_Zero = 0;
    int Counter_Fw_Zero = 0;
    float Fw_Zero = 0;
    float Cmd_Vel_Azi_From08 = 0;  //显控的方位速度指令
    float Cmd_Vel_Pit_From08 = 0;  //显控的俯仰速度指令
    float Cmd_Poi_Azi_Up = 0;          //方位扫描上限
    float Cmd_Poi_Azi_Bottom = 0;      //方位扫描下限
    float Cmd_Poi_Pit_Up = 0;          //俯仰扫描上限
    float Cmd_Poi_Pit_Bottom = 0;      //俯仰扫描下限

    int Flag_CPU_Timer0 = 0;
    int FrameStart = 0;


  //****** Main ******//

  //****** AD Program******// 
    int selector   = 0;      // Used to identify which signal to be collected                                                 
    int ADresult1  = 0;      // Channel 1 result    test                                         
    int ADresult2  = 0;      // Channel 2 result    test                                        
    int ADresult3  = 0;      // Channel 3 result    test                                        
    int ADresult4  = 0;      // Channel 4 result    test
    float Vely     = 0;
    float Velz     = 0;
    float Posiy    = 0;
    float Posiz    = 0;                                        
    unsigned int NumGyroInt = 0;
    unsigned int NumPosiInt = 0; 
    int RS422Length  = 16;

  //****** AD Program******// 

  //****** Scib Program******//
    float Deviationy = 0;       // y方向失调角增量
    float Deviationz = 0;       // z方向失调角增量
    int FramError    = 0;       // Frame Error Signals
    int ScibLoopNum  = 0;       // Scib Interrupt Number   Cancelled after Debug
    int Imagedata[256]; // 6字节图像原始数据，调试结束后改为内部变量
  //****** Scib Program******//

  //******  PWM1  ****** //
    int Pwm1IntNum    = 0;
    float Pwm1IntNum_fy = 0;
    float Pwm1IntNum_fw = 0;
    float angle_fy = 0;
    float angle_fw = 0;
    float direction_fy = -1;
    float direction_fw = -1;
  //******  PWM1  ****** //

  //******  PWM3 Posi ****** //
    int ModeAlter  = 1;       // Used to determin Image order or Sensor order
    float Zeroy    = 0;       // y direction zero inner
    float Zeroz    = 0;       // z direction zero outer
    int Pwm3IntNum = 0;       // PWM 中断次数 
	float Delta_Posiy_k = 0;
    float Delta_Posiz_k = 0;
  //******  PWM3 Posi ****** //

  //******  PWM6 Velo ****** //
	float Delta_Veloy_k = 0;
    float Delta_Veloz_k = 0;
	int Pwm6IntNum = 0;
  //******  PWM6 Velo ****** //

  //******  Posi_PID  ****** //
    float Upk_1_y = 0;
    float Epk_1_y = 0;
    float Upk_1_z = 0;
    float Epk_1_z = 0;
    float Upk_y   = 0;     // Velo_ordery
    float Upk_z   = 0;     // Velo_orderz
    float Epk_z   = 0;     // Delta_Posiy_k
    float Epk_y   = 0;     //  Delta_Posiz_k
  //******  Posi_PID  ****** //

  //******  Velo_PID  ****** //
    float Uvk_1_y = 0;
    float Evk_1_y = 0;
    float Uvk_1_z = 0;
    float Evk_1_z = 0;
    float Uvk_y   = 0;  
    float Uvk_z   = 0;  
    float Evk_z   = 0;  // Delta_Veloz_k;
    float Evk_y   = 0;  // Delta_Veloy_k;
  //******  Velo_PID  ****** //

 //   extern volatile Uint32 EvTimer2_InterruptCount = 0;

    void InitEPwmTimer(void);
    void InitScib(void);
    void scib_xmit(int DataImage);
    void InitFiFoScib(void);
    void InitInterrupt(void);
    void InitCan(void);
    void DelayAdc(volatile Uint16);
    void Posi_PID(void);
    void Velo_PID(void);
    void Duty(void);
    void SelfTest(void);
    void Track(void);
    void GpioInit(void);
    void POSSPEED_Init(void);
    interrupt void cpu_timer0_isr(void);					//定时器0中断服务程序
    extern void Timer0_work(void);			//主函数进程
    extern void Frame_Settle(void);

//    void POSSPEED_Init(void);
//    void POSSPEED_Calc(POSSPEED *p);
//void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);
//volatile void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);
              
    Uint32 Interrupt_Count = 0;



void main(void)
{

	int i = 0;


// Step 1. Initialize System Control:
   InitSysCtrl();

// Step 2. Initalize GPIO:

   InitXintf32Gpio();

   GpioInit();


// Step 3. Clear all interrupts and initialize PIE vector table:
   DINT;

// Initialize the PIE control registers to their default state.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

//	EALLOW;										//取消保护
//	PieVectTable.TINT0 = &cpu_timer0_isr;		//定位定时0中断入口
//	EDIS;										//恢复保护

	ConfigCpuTimer(&CpuTimer0, 150, 1000);		//设置定时器0，150Mhz，1000us



	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;			//打开PIE
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;			//Cpu_Timer0中断

  	IER |= M_INT1;								//打开第1组中断



// Initialize the PIE vector table with pointers to the shell Interrupt,中断服务函数入口，使用默认中断服务函数
   InitPieVectTable();
   //InitFlash();
   #ifdef FLASH
   MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
   InitFlash();
   #endif

// Step 4. Initialize all the Device Peripherals:
   InitEPwmTimer();    // For this example, only initialize the ePWM Timers
   InitScib();
   //InitFiFoScib();
   Init_revdata(&rev_data); //初始化串口接收buffer

  //????发送两次0xab ，0xAB
   scib_xmit(0xab);
   ScibRegs.SCITXBUF = 0xAB;

// Step 5. User specific code, enable interrupts:
   InitInterrupt();
// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

// Step 6. Self Testst   ProgStep = 1
   SelfTest();
   //POSSPEED_Init();

	#if (CPU_FRQ_150MHZ)
   	   EQep1Regs.QUPRD=1500000;			// Unit Timer for 100Hz at 150 MHz SYSCLKOUT
	#endif
	#if (CPU_FRQ_100MHZ)
   	   EQep1Regs.QUPRD=1000000;			// Unit Timer for 100Hz at 100 MHz SYSCLKOUT
	#endif

	EQep1Regs.QDECCTL.bit.QSRC=00;		// QEP quadrature count mode

	EQep1Regs.QEPCTL.bit.FREE_SOFT=2;
	EQep1Regs.QEPCTL.bit.PCRM=00;		// PCRM=00 mode - QPOSCNT reset on index event
	EQep1Regs.QEPCTL.bit.UTE=1; 		// Unit Timeout Enable
	EQep1Regs.QEPCTL.bit.QCLM=1; 		// Latch on unit time out
	EQep1Regs.QPOSMAX=0xffffffff;
	EQep1Regs.QEPCTL.bit.QPEN=1; 		// QEP enable

	EQep1Regs.QCAPCTL.bit.UPPS=5;   	// 1/32 for unit position
	EQep1Regs.QCAPCTL.bit.CCPS=7;		// 1/128 for CAP clock
	EQep1Regs.QCAPCTL.bit.CEN=1; 		// QEP Capture Enable


//	  for(i=0;i<256;i++)
//	    Imagedata[i] = 0x00;
	//

// Step 7. Track         ProgStep = 2

//   Track();

//	POSSPEED_Calc(POSSPEED *p);

	StartCpuTimer0();							//启动定时器0


   for(;;)
   {
	   if(Flag_CPU_Timer0 == 1)
	   {
		   Flag_CPU_Timer0 = 0;
		   Timer0_work();
	   }
	   if(rev_data.rev_Flag == REV_FRAM_ALL)
	   {
		   Frame_Settle();
	   }
//	   ScibRegs.SCITXBUF = 0xAB;
//	   qep_posspeed.calc(&qep_posspeed);
   }

}


void InitEPwmTimer()
{
 
//=====================================================================
// Initialization Time
//========================// EPWM Module 1 config
EPwm1Regs.TBPRD = 0x1FFF;  // Period = 0x1FFF(8191) * 0.0119ms =
EPwm1Regs.TBPHS.half.TBPHS = 0;       // Set Phase register to zero
EPwm1Regs.TBCTR = 0x0000;


EPwm1Regs.TBCTL.bit.CTRMODE   = 0; //Up Mode
EPwm1Regs.TBCTL.bit.PHSEN     = 0; //Master module
EPwm1Regs.TBCTL.bit.PRDLD     = 0; //Load when zero  1 dedicate load immediately
EPwm1Regs.TBCTL.bit.SYNCOSEL  = 3; //Disable the syncosel signals
EPwm1Regs.TBCTL.bit.HSPCLKDIV = 1; //5divisor is set to be 14
EPwm1Regs.TBCTL.bit.CLKDIV    = 1; //3divisor is set to be 128    TB Frequency is set to be 150M/(10*8) = 83.705kHz Period = 0.01195ms

EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;
EPwm1Regs.CMPCTL.bit.SHDWBMODE = 0;
EPwm1Regs.CMPCTL.bit.LOADAMODE = 0; // load on CTR=Zero
EPwm1Regs.CMPCTL.bit.LOADBMODE = 0; // load on CTR=Zero

  EPwm1Regs.AQCTLA.bit.CAU = 2;  //set actions for EPWM1A
  EPwm1Regs.AQCTLA.bit.CAD = 1;  //clear
  EPwm1Regs.AQCTLA.bit.PRD = 1;  //clear
  EPwm1Regs.AQCTLA.bit.ZRO = 1;  //clear  actions for 1A

  EPwm1Regs.AQCTLB.bit.CAU = 1;  //1   // clear actions for EPWM1A
  EPwm1Regs.AQCTLB.bit.CAD = 2;
  EPwm1Regs.AQCTLB.bit.PRD = 2;
  EPwm1Regs.AQCTLB.bit.ZRO = 2;  //2   // set actions for 1A

  EPwm1Regs.AQCTLB.bit.CAU = 2;  //2
  EPwm1Regs.AQCTLB.bit.ZRO = 1;  //1

EPwm1Regs.DBCTL.bit.IN_MODE  = 0; // enable Dead-band module
EPwm1Regs.DBCTL.bit.OUT_MODE = 3; 
EPwm1Regs.DBCTL.bit.POLSEL   = 2; // AH
EPwm1Regs.DBFED = 50; // FED = 50 TBCLKs
EPwm1Regs.DBRED = 50; // RED = 50 TBCLKs

// EPWM Module 2 config
EPwm2Regs.TBPRD = 0x1FFF;  // Period = 0x1FFF(8191) * 0.53us = 4.34ms
EPwm2Regs.TBPHS.half.TBPHS = 0;       // Set Phase register to zero

EPwm2Regs.TBCTL.bit.CTRMODE   = 0; //Up Mode
EPwm2Regs.TBCTL.bit.PHSEN     = 0; //Master module
EPwm2Regs.TBCTL.bit.PRDLD     = 0; //Load when zero  1 dedicate load immediately
EPwm2Regs.TBCTL.bit.SYNCOSEL  = 3; //Disable the syncosel signals
EPwm2Regs.TBCTL.bit.HSPCLKDIV = 1; //divisor is set to be 10
EPwm2Regs.TBCTL.bit.CLKDIV    = 1; //divisor is set to be 8    TB Frequency is set to be 150M/(10*8) = 1.875MHz Period = 0.53us

EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0;
EPwm2Regs.CMPCTL.bit.SHDWBMODE = 0;
EPwm2Regs.CMPCTL.bit.LOADAMODE = 0; // load on CTR=Zero
EPwm2Regs.CMPCTL.bit.LOADBMODE = 0; // load on CTR=Zero

EPwm2Regs.AQCTLA.bit.CAU = 2;  //set actions for EPWM1A
EPwm2Regs.AQCTLA.bit.CAD = 1;  //clear
EPwm2Regs.AQCTLA.bit.PRD = 1;  //clear
EPwm2Regs.AQCTLA.bit.ZRO = 1;  //clear  actions for 1A

EPwm2Regs.AQCTLB.bit.CAU = 1;  //1   // clear actions for EPWM1A
EPwm2Regs.AQCTLB.bit.CAD = 2;
EPwm2Regs.AQCTLB.bit.PRD = 2;
EPwm2Regs.AQCTLB.bit.ZRO = 2;  //2   // set actions for 1A

EPwm2Regs.DBCTL.bit.IN_MODE  = 0; // enable Dead-band module
EPwm2Regs.DBCTL.bit.OUT_MODE = 3; 
EPwm2Regs.DBCTL.bit.POLSEL   = 2; // AH
EPwm2Regs.DBFED = 50; // FED = 50 TBCLKs
EPwm2Regs.DBRED = 50; // RED = 50 TBCLKs

// EPWM Module 3 config
EPwm3Regs.TBPRD = T3PRD;  // Period = 0x1FFF(8191) * 0.53us = 4.34ms
EPwm3Regs.TBPHS.half.TBPHS = 0;       // Set Phase register to zero

EPwm3Regs.TBCTL.bit.CTRMODE   = 0; //Up Mode
EPwm3Regs.TBCTL.bit.PHSEN     = 0; //Master module
EPwm3Regs.TBCTL.bit.PRDLD     = 0; //Load when zero  1 dedicate load immediately
EPwm3Regs.TBCTL.bit.SYNCOSEL  = 3; //Disable the syncosel signals
EPwm3Regs.TBCTL.bit.HSPCLKDIV = 1; //divisor is set to be 10
EPwm3Regs.TBCTL.bit.CLKDIV    = 6; //divisor is set to be 8    TB Frequency is set to be 150M/(10*8) = 1.875MHz Period = 0.53us

EPwm3Regs.CMPCTL.bit.SHDWAMODE = 0;
EPwm3Regs.CMPCTL.bit.SHDWBMODE = 0;
EPwm3Regs.CMPCTL.bit.LOADAMODE = 0; // load on CTR=Zero
EPwm3Regs.CMPCTL.bit.LOADBMODE = 0; // load on CTR=Zero

EPwm3Regs.AQCTLA.bit.CAU = 1; // clear actions for EPWM1A
EPwm3Regs.AQCTLA.bit.ZRO = 2; // set actions for 1A
EPwm3Regs.AQCTLB.bit.CBU = 2;
EPwm3Regs.AQCTLB.bit.ZRO = 1;

EPwm3Regs.DBCTL.bit.IN_MODE  = 0; // enable Dead-band module
EPwm3Regs.DBCTL.bit.OUT_MODE = 3; 
EPwm3Regs.DBCTL.bit.POLSEL   = 2; // AH
EPwm3Regs.DBFED = 50; // FED = 50 TBCLKs
EPwm3Regs.DBRED = 50; // RED = 50 TBCLKs
EPwm3Regs.DBFED = 50; // FED = 50 TBCLKs
EPwm3Regs.DBRED = 50; // RED = 50 TBCLKs

// EPWM Module 4 config
EPwm4Regs.TBPRD = 0x1FFF;  // Period = 0x1FFF(8191) * 0.53us = 4.34ms
EPwm4Regs.TBPHS.half.TBPHS = 0;       // Set Phase register to zero

EPwm4Regs.TBCTL.bit.CTRMODE   = 0; //Up Mode
EPwm4Regs.TBCTL.bit.PHSEN     = 0; //Master module
EPwm4Regs.TBCTL.bit.PRDLD     = 0; //Load when zero  1 dedicate load immediately
EPwm4Regs.TBCTL.bit.SYNCOSEL  = 3; //Disable the syncosel signals
EPwm4Regs.TBCTL.bit.HSPCLKDIV = 5; //divisor is set to be 10
EPwm4Regs.TBCTL.bit.CLKDIV    = 3; //divisor is set to be 8    TB Frequency is set to be 150M/(10*8) = 1.875MHz Period = 0.53us

EPwm4Regs.CMPCTL.bit.SHDWAMODE = 0;
EPwm4Regs.CMPCTL.bit.SHDWBMODE = 0;
EPwm4Regs.CMPCTL.bit.LOADAMODE = 0; // load on CTR=Zero
EPwm4Regs.CMPCTL.bit.LOADBMODE = 0; // load on CTR=Zero

EPwm4Regs.AQCTLA.bit.CAU = 1; // clear actions for EPWM1A
EPwm4Regs.AQCTLA.bit.ZRO = 2; // set actions for 1A
EPwm4Regs.AQCTLB.bit.CBU = 2;
EPwm4Regs.AQCTLB.bit.ZRO = 1;

EPwm4Regs.DBCTL.bit.IN_MODE  = 0; // enable Dead-band module
EPwm4Regs.DBCTL.bit.OUT_MODE = 3; 
EPwm4Regs.DBCTL.bit.POLSEL   = 2; // AH
EPwm4Regs.DBFED = 50; // FED = 50 TBCLKs
EPwm4Regs.DBRED = 50; // RED = 50 TBCLKs
EPwm4Regs.DBFED = 50; // FED = 50 TBCLKs
EPwm4Regs.DBRED = 50; // RED = 50 TBCLKs

// EPWM Module 5 config
EPwm5Regs.TBPRD = 0x1FFF;  // Period = 0x1FFF(8191) * 0.53us = 4.34ms
EPwm5Regs.TBPHS.half.TBPHS = 0;       // Set Phase register to zero

EPwm5Regs.TBCTL.bit.CTRMODE   = 0; //Up Mode
EPwm5Regs.TBCTL.bit.PHSEN     = 0; //Master module
EPwm5Regs.TBCTL.bit.PRDLD     = 0; //Load when zero  1 dedicate load immediately
EPwm5Regs.TBCTL.bit.SYNCOSEL  = 3; //Disable the syncosel signals
EPwm5Regs.TBCTL.bit.HSPCLKDIV = 5; //divisor is set to be 10
EPwm5Regs.TBCTL.bit.CLKDIV    = 3; //divisor is set to be 8    TB Frequency is set to be 150M/(10*8) = 1.875MHz Period = 0.53us

EPwm5Regs.CMPCTL.bit.SHDWAMODE = 0;
EPwm5Regs.CMPCTL.bit.SHDWBMODE = 0;
EPwm5Regs.CMPCTL.bit.LOADAMODE = 0; // load on CTR=Zero
EPwm5Regs.CMPCTL.bit.LOADBMODE = 0; // load on CTR=Zero

EPwm5Regs.AQCTLA.bit.CAU = 2; // clear actions for EPWM1A
EPwm5Regs.AQCTLA.bit.ZRO = 1; // set actions for 1A
EPwm5Regs.AQCTLB.bit.CBU = 1;
EPwm5Regs.AQCTLB.bit.ZRO = 2;

EPwm5Regs.DBCTL.bit.IN_MODE  = 0; // enable Dead-band module
EPwm5Regs.DBCTL.bit.OUT_MODE = 3; 
EPwm5Regs.DBCTL.bit.POLSEL   = 2; // AH
EPwm5Regs.DBFED = 50; // FED = 50 TBCLKs
EPwm5Regs.DBRED = 50; // RED = 50 TBCLKs
EPwm5Regs.DBFED = 50; // FED = 50 TBCLKs
EPwm5Regs.DBRED = 50; // RED = 50 TBCLKs

// EPWM Module 6 config
EPwm6Regs.TBPRD = 0x075F;  // Period = 0x1FFF(8191) * 0.53us = 4.34ms
EPwm6Regs.TBPHS.half.TBPHS = 0;       // Set Phase register to zero

EPwm6Regs.TBCTL.bit.CTRMODE   = 0; //Up Mode
EPwm6Regs.TBCTL.bit.PHSEN     = 0; //Master module
EPwm6Regs.TBCTL.bit.PRDLD     = 0; //Load when zero  1 dedicate load immediately
EPwm6Regs.TBCTL.bit.SYNCOSEL  = 3; //Disable the syncosel signals
EPwm6Regs.TBCTL.bit.HSPCLKDIV = 5; //divisor is set to be 10
EPwm6Regs.TBCTL.bit.CLKDIV    = 3; //divisor is set to be 8    TB Frequency is set to be 150M/(10*8) = 1.875MHz Period = 0.53us

EPwm6Regs.CMPCTL.bit.SHDWAMODE = 0;
EPwm6Regs.CMPCTL.bit.SHDWBMODE = 0;
EPwm6Regs.CMPCTL.bit.LOADAMODE = 0; // load on CTR=Zero
EPwm6Regs.CMPCTL.bit.LOADBMODE = 0; // load on CTR=Zero

EPwm6Regs.AQCTLA.bit.CAU = 2; // clear actions for EPWM1A
EPwm6Regs.AQCTLA.bit.ZRO = 1; // set actions for 1A
EPwm6Regs.AQCTLB.bit.CBU = 1;
EPwm6Regs.AQCTLB.bit.ZRO = 2;

EPwm6Regs.DBCTL.bit.IN_MODE  = 0; // enable Dead-band module
EPwm6Regs.DBCTL.bit.OUT_MODE = 3; 
EPwm6Regs.DBCTL.bit.POLSEL   = 2; // AH
EPwm6Regs.DBFED = 50; // FED = 50 TBCLKs
EPwm6Regs.DBRED = 50; // RED = 50 TBCLKs
EPwm6Regs.DBFED = 50; // FED = 50 TBCLKs
EPwm6Regs.DBRED = 50; // RED = 50 TBCLKs

// Run Time (Note: Example execution of one run-time instant)
//=========================================================
EPwm1Regs.CMPA.half.CMPA = 0x0FFF; // adjust duty for output EPWM1A
EPwm1Regs.CMPB           = 0x0FFF;
EPwm2Regs.CMPA.half.CMPA = 0x0FFF;  //0xAAAA//0x7FFF adjust duty for output EPWM2A
//EPwm2Regs.CMPB           = 0x1CCC;  //0x5555 0x7FFF
EPwm3Regs.CMPA.half.CMPA = 0x0667;  // adjust duty for output EPWM3A
//EPwm3Regs.CMPB           = 0x0FFF;
EPwm4Regs.CMPA.half.CMPA = 0x09FF;  // adjust duty for output EPWM3A
//EPwm4Regs.CMPB           = 0x0FFF;
EPwm5Regs.CMPA.half.CMPA = 0x09FF;  // adjust duty for output EPWM3A
//EPwm5Regs.CMPB           = 0x0FFF;
EPwm6Regs.CMPA.half.CMPA = 0x03AF;  // adjust duty for output EPWM3A


// Init QEP
//EQep1Regs.QDECCTL.bit.QSRC=00;		// QEP quadrature count mode
//
//EQep1Regs.QEPCTL.bit.FREE_SOFT=2;
//EQep1Regs.QEPCTL.bit.PCRM=00;		// PCRM=00 mode - QPOSCNT reset on index event
//EQep1Regs.QEPCTL.bit.UTE=1; 		// Unit Timeout Enable
//EQep1Regs.QEPCTL.bit.QCLM=1; 		// Latch on unit time out
//EQep1Regs.QPOSMAX=0xffffffff;
//EQep1Regs.QEPCTL.bit.QPEN=1; 		// QEP enable
//
//EQep1Regs.QCAPCTL.bit.UPPS=5;   	// 1/32 for unit position
//EQep1Regs.QCAPCTL.bit.CCPS=7;		// 1/128 for CAP clock
//EQep1Regs.QCAPCTL.bit.CEN=1; 		// QEP Capture Enable
// Init QEP

}


void InitInterrupt()
{
    EALLOW;
//    XIntruptRegs.XINT1CR.bit.ENABLE   = 1;  // First Disable
//    XIntruptRegs.XINT1CR.bit.POLARITY = 1;  // low-to-high
	asm(" RPT #50 || NOP");
    EDIS;  

//*****PWM级中断设置
    EPwm1Regs.ETSEL.bit.INTEN  = 0;   // First Disable
    EPwm1Regs.ETSEL.bit.INTSEL = 2;   // Event when Time Base equal to Period
    EPwm1Regs.ETPS.bit.INTCNT = 1;    // one event
    EPwm1Regs.ETPS.bit.INTPRD = 1;    // occurs interrupt when one event happens
    EPwm1Regs.ETCLR.bit.INT = 1;      // 写 1 清 零
    
//    EPwm3Regs.ETSEL.bit.INTEN  = 1;   // First Disable
//    EPwm3Regs.ETSEL.bit.INTSEL = 2;   // Event when Time Base equal to Period
//    EPwm3Regs.ETPS.bit.INTCNT = 1;    // one event
//    EPwm3Regs.ETPS.bit.INTPRD = 1;    // occurs interrupt when one event happens
//    EPwm3Regs.ETCLR.bit.INT = 1;      // 写 1 清 零
//
//    EPwm6Regs.ETSEL.bit.INTEN  = 0;   // First Disable
//    EPwm6Regs.ETSEL.bit.INTSEL = 2;   // Event when Time Base equal to Period
//    EPwm6Regs.ETPS.bit.INTCNT = 1;    // one event
//    EPwm6Regs.ETPS.bit.INTPRD = 1;    // occurs interrupt when one event happens
//    EPwm6Regs.ETCLR.bit.INT = 1;      // 写 1 清 零
//*****PWM级中断设置

//*****Scib外设级中断设置
	ScibRegs.SCICTL2.bit.TXINTENA = 1;   // First Disable
    ScibRegs.SCICTL2.bit.RXBKINTENA = 1; // First Disable
//*****Scib外设级中断设置

	PieCtrlRegs.PIECTRL.bit.ENPIE  = 1;          // Enable the PIE block

	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;           // Enable PWM1 
//	PieCtrlRegs.PIEIER3.bit.INTx3 = 1;           // Enable PWM3
//	PieCtrlRegs.PIEIER3.bit.INTx6 = 1;           // Enable Pwm6
    PieCtrlRegs.PIEACK.bit.ACK3   = 1;
    PieCtrlRegs.PIEIFR3.bit.INTx1 = 0;           // 写 0 清 零
//    PieCtrlRegs.PIEIFR3.bit.INTx3 = 0;           // 写 0 清 零
//	PieCtrlRegs.PIEIFR3.bit.INTx6 = 0;           // 写 0 清 零
	 
//	PieCtrlRegs.PIEIER1.bit.INTx4  = 1;          // AD
//    PieCtrlRegs.PIEACK.bit.ACK1 = 1;             // AD
    
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;           // Scib Pie Enable
    PieCtrlRegs.PIEIFR9.bit.INTx3 = 1;           // Scib Pie Flag
    PieCtrlRegs.PIEACK.bit.ACK9   = 1;           // Scib Pie Ack

    EALLOW;
//	PieVectTable.XINT1 = &XINT1_ISR;        //AD7656 中断服务子程序
	PieVectTable.EPWM1_INT = &EPWM1_INT_ISR;
//	PieVectTable.EPWM3_INT = &EPWM3_INT_ISR;
//    PieVectTable.EPWM6_INT = &EPWM6_INT_ISR;
    PieVectTable.SCIRXINTB = &SCIRXINTB_ISR;
    EDIS;

   	IER |= 0x0105;                              // Enable CPU AD PWM3&6 Scib
}


void InitScib()
{
//ScibRegs.SCICCR.all =0x0007;       // 1 stop bit,  No loopback
//                                   // No parity,8 char bits,
//                                   // async mode, idle-line protocol
//ScibRegs.SCICTL1.all =0x0003;      // enable TX, RX, internal SCICLK,
//                                   // Disable RX ERR, SLEEP, TXWAKE
//ScibRegs.SCICTL2.all =0x0003;
//ScibRegs.SCICTL2.bit.TXINTENA = 0;
//ScibRegs.SCICTL2.bit.RXBKINTENA =0;
//	#if (CPU_FRQ_150MHZ)
//	      ScibRegs.SCIHBAUD    =0x0000;  // 115200 baud @LSPCLK = 37.5MHz.
//	      ScibRegs.SCILBAUD    =0x0028;
//	#endif
//	#if (CPU_FRQ_100MHZ)
//      ScibRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 20MHz.
//      ScibRegs.SCILBAUD    =0x0044;
//	#endif
//ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
//
//ScibRegs.SCICTL1.bit.TXENA = 1;       //使能发送
//ScibRegs.SCICTL1.bit.RXENA = 1;       //使能接收


/*
	ScibRegs.SCICTL1.bit.SWRESET = 0;

 	ScibRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
// baud = LSPCLK/8/((BRR+1)
// baud @LSPCLK = 37.5MHz (150 MHz SYSCLK)
	ScibRegs.SCIHBAUD    = 0x0000;
    ScibRegs.SCILBAUD    = 40;  //115200bps  0x28

    ScibRegs.SCICTL1.bit.SWRESET = 1;     // Relinquish SCI from Reset
    ScibRegs.SCIFFTX.bit.SCIRST=1;

	ScibRegs.SCIFFRX.bit.RXFFIL  = 9;  		//设置FIFO深度
	ScibRegs.SCICTL1.bit.TXENA   = 1;       //使能发送
	ScibRegs.SCICTL1.bit.RXENA   = 1;       //使能接收


	ScibRegs.SCICTL2.bit.RXBKINTENA = 1;    //使能接收中断

	//中断配置步骤-----1
	ScibRegs.SCIFFTX.bit.SCIFFENA = 1;      //使能FIFO中断
	ScibRegs.SCIFFRX.bit.RXFFIENA = 1;


	ScibRegs.SCIFFCT.all = 0x00;
	ScibRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
	ScibRegs.SCIFFRX.bit.RXFIFORESET  = 1;
*/



	ScibRegs.SCICTL1.bit.SWRESET = 0;

 	ScibRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
// baud = LSPCLK/8/((BRR+1) 
// baud @LSPCLK = 37.5MHz (150 MHz SYSCLK)
	ScibRegs.SCIHBAUD   =0x0000;
    ScibRegs.SCILBAUD    =40;  //115200bps

    ScibRegs.SCICTL1.bit.SWRESET = 1;     // Relinquish SCI from Reset
    ScibRegs.SCIFFTX.bit.SCIRST=1;

	ScibRegs.SCIFFRX.bit.RXFFIL  = 1;  //设置FIFO深度
	ScibRegs.SCICTL1.bit.TXENA = 1;       //使能发送
	ScibRegs.SCICTL1.bit.RXENA = 1;       //使能接收

//	SciaRegs.SCICTL2.bit.TXINTENA =1;
//	SciaRegs.SCICTL2.bit.RXBKINTENA =1;
//  SciaRegs.SCIFFTX.bit.TXFFIENA = 0; //禁止发送中断使能

	//中断配置步骤-----1
	ScibRegs.SCIFFTX.bit.SCIFFENA = 1;      //使能FIFO中断
	ScibRegs.SCIFFRX.bit.RXFFIENA=1;

	ScibRegs.SCIFFCT.all=0x00;
	ScibRegs.SCIFFTX.bit.TXFIFOXRESET=1;
	ScibRegs.SCIFFRX.bit.RXFIFORESET=1;


}

void InitCan()
{

}

void scib_xmit(int DataImage)
{
    while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {}
    ScibRegs.SCITXBUF = DataImage;
}
void UART_SEND (char *databuff,int len)
{
   int i=0;
	for(i=0;i<len;i++)
	{
		while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {}
		ScibRegs.SCITXBUF = databuff[i];
	}
}

void InitFiFoScib()
{
    ScibRegs.SCIFFTX.all=0xE041;  //0xE041;
    ScibRegs.SCIFFRX.all=0x4001;
    asm(" RPT #7 || NOP");
    ScibRegs.SCIFFRX.all=0x6061; //0x6069  09  禁止FF中断   69使能FF中断
    ScibRegs.SCIFFCT.all=0x0;

    ScibRegs.SCIFFTX.bit.SCIRST=1;
	ScibRegs.SCIFFCT.all=0x00;
	ScibRegs.SCIFFTX.bit.TXFIFOXRESET=1;
	ScibRegs.SCIFFRX.bit.RXFIFORESET=1;
}

void DelayAdc(volatile Uint16 Usec)                   
{                                                     
   while(Usec--)                                       
   {                                                   
    asm(" RPT #150 || NOP ");   
   }                                                   
}

void Posi_PID()
{
extern float Upk_1_y;
extern float Epk_1_y;
extern float Upk_1_z;
extern float Epk_1_z;
extern float Upk_y;     // Velo_ordery
extern float Upk_z;     // Velo_orderz
extern float Epk_z;     // Delta_Posiy_k
extern float Epk_y;     //  Delta_Posiz_k

float Coe_Upk_1_y = 1.0;
//float Coe_Upk_y   = 1.0;
float Coe_Epk_1_y = -0.0189;
float Coe_Epk_y   =  0.0219;

float Coe_Upk_1_z = 1.0;
//float Coe_Upk_z   = 1.0;
float Coe_Epk_1_z = -0.0189;
float Coe_Epk_z   =  0.0219;

   //Upk_y = Coe_Upk_1_y * Upk_1_y + Coe_Epk_y * Epk_y + Coe_Epk_1_y * Epk_1_y;
   Upk_y = Epk_y * 0.25/0.001;
   Upk_1_y = Upk_y;
   Epk_1_y = Epk_y;

   //Upk_z = Coe_Upk_1_z * Upk_1_z + Coe_Epk_z * Epk_z + Coe_Epk_1_z * Epk_1_z;  //俯仰 FY
   Upk_z = Epk_z * 0.5/0.001;
   Upk_1_z = Upk_z;
   Epk_1_z = Epk_z;       
}


void Velo_PID()                   
{
extern float Uvk_1_y;
extern float Evk_1_y;
extern float Uvk_1_z;
extern float Evk_1_z;
extern float Uvk_y;  
extern float Uvk_z;  
extern float Evk_z;  // Delta_Veloz_k;
extern float Evk_y;  // Delta_Veloy_k;

float Coe_Uvk_1_y = 1.0;
//float Coe_Uvk_y   = 1.0;
float Coe_Evk_1_y = -0.0189;
float Coe_Evk_y   =  0.0219;

float Coe_Uvk_1_z = 1.0;
//float Coe_Uvk_z   = 1.0;
float Coe_Evk_1_z = -0.0189;
float Coe_Evk_z   =  0.0219;

   //Uvk_y = Coe_Uvk_1_y * Uvk_1_y + Coe_Evk_y * Evk_y + Coe_Evk_1_y * Evk_1_y;
   Uvk_y = Evk_y * 0.5/0.001;
   Uvk_1_y = Uvk_y;
   Evk_1_y = Evk_y;
   
   //Uvk_z = Coe_Uvk_1_z * Uvk_1_z + Coe_Evk_z * Evk_z + Coe_Evk_1_z * Evk_1_z;
   Uvk_z = Evk_z * 0.3/0.001;
   Uvk_1_z = Uvk_z;
   Evk_1_z = Evk_z;       

}                                                    

void Duty()   
{
  extern float Uvk_y;  
  extern float Uvk_z;

  extern Uint32 Interrupt_Count;



  if(Upk_z >= 4095)
  { Upk_z = 4095;}
  else if(Upk_z <= -4095)
  {
	  Upk_z = -4095;
  }


  if(Uvk_z >= 4095)
  { Uvk_z = 4095;}
  else if(Uvk_z <= -4095)
  {
	  Uvk_z = -4095;
  }



  if(Upk_y >= 4095)
  {
	  Upk_y = 4095;
  }
  else if(Upk_y <= -4095)
  {
	  Upk_y = -4095;
  }

  if(Uvk_y >= 4095)
  {
	  Uvk_y = 4095;
  }
  else if(Uvk_y <= -4095)
  {
	  Uvk_y = -4095;
  }

  if(Interrupt_Count <= 10000)
  {
		EPwm1Regs.CMPA.half.CMPA = 0x0FFF; 	// adjust duty for output EPWM1A
	    EPwm2Regs.CMPA.half.CMPA = 0x1999;//;  	// adjust duty for output EPWM2A
  }
  else if (Interrupt_Count > 10000)
  {
		if(ProgStep == 1)  // 位置回路
		{
			EPwm1Regs.CMPA.half.CMPA = (Uint16)(int16)(Upk_z + 0x0FFF); 	// adjust duty for output EPWM1A
		    EPwm2Regs.CMPA.half.CMPA = (Uint16)(int16)(Upk_y + 0x0FFF);		//;  	// adjust duty for output EPWM2A
		}
		else if(ProgStep == 0)  //速度
		{
			EPwm1Regs.CMPA.half.CMPA = (Uint16)(int16)(Uvk_z + 0x0FFF); 	// adjust duty for output EPWM1A
		    EPwm2Regs.CMPA.half.CMPA = (Uint16)(int16)(Uvk_y + 0x0FFF);		//;  	// adjust duty for output EPWM2A
		}
  }



}   

void SelfTest()
{
	  extern int ProgStep;
	  extern int ModeAlter;
	  //ProgStep = 1;
	  ModeAlter = 1;
	  ScibRegs.SCICTL2.bit.TXINTENA     = 0;       // Enable
	  ScibRegs.SCICTL2.bit.RXBKINTENA   = 0;       // Enable
//	  XIntruptRegs.XINT1CR.bit.ENABLE   = 1;       // Enable
//	  EPwm3Regs.ETSEL.bit.INTEN  = 0;              // Enable
//	  EPwm6Regs.ETSEL.bit.INTEN  = 0;              // Enable
	  GpioDataRegs.GPCSET.bit.GPIO87 = 1;          //
	  GpioDataRegs.GPBSET.bit.GPIO47 = 1;          //
	  GpioDataRegs.GPCSET.bit.GPIO86 = 1;          //
	  GpioDataRegs.GPBSET.bit.GPIO46 = 1;          //
}                


void Track()
{
  extern int ProgStep;
  extern int ModeAlter;
  //ProgStep = 2;
  ModeAlter = 2;
  ScibRegs.SCICTL2.bit.TXINTENA   = 0;       // Enable
  ScibRegs.SCICTL2.bit.RXBKINTENA = 0;       // Enable
}  

void GpioInit()
{

	 EALLOW;

     GpioDataRegs.GPBSET.bit.GPIO40 = 1;  // A0

     GpioDataRegs.GPBSET.bit.GPIO41 = 1;  // A1     M1
     GpioDataRegs.GPBSET.bit.GPIO42 = 1;  // A2     M2
     GpioDataRegs.GPBSET.bit.GPIO43 = 1;  // A3   M3

     GpioDataRegs.GPBSET.bit.GPIO44 = 1; // A4


     GpioDataRegs.GPBSET.bit.GPIO45 = 1; //   direction of step motor
	 GpioDataRegs.GPBSET.bit.GPIO46 = 1;  // 
     GpioDataRegs.GPBSET.bit.GPIO47 = 1;  // 

	 GpioDataRegs.GPCSET.bit.GPIO80 = 1;  // B0

     GpioDataRegs.GPCSET.bit.GPIO81 = 1;  // B1    M1
     GpioDataRegs.GPCSET.bit.GPIO82 = 1;  // B2    M2
     GpioDataRegs.GPCSET.bit.GPIO83 = 1;  // B3  M3

     GpioDataRegs.GPCSET.bit.GPIO84 = 1;  // B4

     GpioDataRegs.GPCSET.bit.GPIO85 = 1;  //  direction of step motor 
     GpioDataRegs.GPCSET.bit.GPIO86 = 1;  // 
	 GpioDataRegs.GPCSET.bit.GPIO87 = 1;  // 

	 /* Enable internal pull-up for the selected pins */
	 // Pull-ups can be enabled or disabled by the user.
	 // This will enable the pullups for the specified pins.
	 // Comment out other unwanted lines.

	 //     GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pull-up on GPIO20 (EQEP1A)
	 //    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;   // Enable pull-up on GPIO21 (EQEP1B)
	 //    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   // Enable pull-up on GPIO22 (EQEP1S)
	 //    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;   // Enable pull-up on GPIO23 (EQEP1I)

	     GpioCtrlRegs.GPBPUD.bit.GPIO50 = 0;   // Enable pull-up on GPIO50 (EQEP1A)
	     GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0;   // Enable pull-up on GPIO51 (EQEP1B)
	     GpioCtrlRegs.GPBPUD.bit.GPIO52 = 0;   // Enable pull-up on GPIO52 (EQEP1S)
	     GpioCtrlRegs.GPBPUD.bit.GPIO53 = 0;   // Enable pull-up on GPIO53 (EQEP1I)


	 // Inputs are synchronized to SYSCLKOUT by default.
	 // Comment out other unwanted lines.

	 //    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 0;   // Sync to SYSCLKOUT GPIO20 (EQEP1A)
	 //    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 0;   // Sync to SYSCLKOUT GPIO21 (EQEP1B)
	 //    GpioCtrlRegs.GPAQSEL2.bit.GPIO22 = 0;   // Sync to SYSCLKOUT GPIO22 (EQEP1S)
	 //    GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 0;   // Sync to SYSCLKOUT GPIO23 (EQEP1I)

	     GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 0;   // Sync to SYSCLKOUT GPIO50 (EQEP1A)
	     GpioCtrlRegs.GPBQSEL2.bit.GPIO51 = 0;   // Sync to SYSCLKOUT GPIO51 (EQEP1B)
	     GpioCtrlRegs.GPBQSEL2.bit.GPIO52 = 0;   // Sync to SYSCLKOUT GPIO52 (EQEP1S)
	     GpioCtrlRegs.GPBQSEL2.bit.GPIO53 = 0;   // Sync to SYSCLKOUT GPIO53 (EQEP1I)

	 //	GpioCtrlRegs.GPBCTRL.bit.QUALPRD2 = 100;

	 /* Configure eQEP-1 pins using GPIO regs*/
	 // This specifies which of the possible GPIO pins will be eQEP1 functional pins.
	 // Comment out other unwanted lines.

	 //     GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;   // Configure GPIO20 as EQEP1A
	 //    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;   // Configure GPIO21 as EQEP1B
	 //    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 1;   // Configure GPIO22 as EQEP1S
	 //    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1;   // Configure GPIO23 as EQEP1I

	    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 1;   // Configure GPIO50 as EQEP1A
	    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 1;   // Configure GPIO51 as EQEP1B
	    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 1;   // Configure GPIO52 as EQEP1S
	    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 1;   // Configure GPIO53 as EQEP1I

	    GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;    // Enable pull-up on GPIO24 (EQEP2A)
	    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;    // Enable pull-up on GPIO25 (EQEP2B)
	    GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;    // Enable pull-up on GPIO26 (EQEP2I)
	    GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;    // Enable pull-up on GPIO27 (EQEP2S)

	// Inputs are synchronized to SYSCLKOUT by default.
	// Comment out other unwanted lines.

	    GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 0;  // Sync to SYSCLKOUT GPIO24 (EQEP2A)
	    GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 0;  // Sync to SYSCLKOUT GPIO25 (EQEP2B)
	    GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 0;  // Sync to SYSCLKOUT GPIO26 (EQEP2I)
	    GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 0;  // Sync to SYSCLKOUT GPIO27 (EQEP2S)

	/* Configure eQEP-2 pins using GPIO regs*/
	// This specifies which of the possible GPIO pins will be eQEP2 functional pins.
	// Comment out other unwanted lines.

	    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 2;   // Configure GPIO24 as EQEP2A
	    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 2;   // Configure GPIO25 as EQEP2B
	    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 2;   // Configure GPIO26 as EQEP2I
	    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 2;   // Configure GPIO27 as EQEP2S

	    EDIS;
}  


void  POSSPEED_Init(void)
{
//  #if (CPU_FRQ_150MHZ)
	  EQep1Regs.QUPRD=1500000;			// Unit Timer for 100Hz at 150 MHz SYSCLKOUT
//	#endif
//    #if (CPU_FRQ_100MHZ)
//	  EQep1Regs.QUPRD=1000000;			// Unit Timer for 100Hz at 100 MHz SYSCLKOUT
//	#endif

	EQep1Regs.QDECCTL.bit.QSRC=00;		// QEP quadrature count mode

	EQep1Regs.QEPCTL.bit.FREE_SOFT=2;
	EQep1Regs.QEPCTL.bit.PCRM=00;		// PCRM=00 mode - QPOSCNT reset on index event
	EQep1Regs.QEPCTL.bit.UTE=1; 		// Unit Timeout Enable
	EQep1Regs.QEPCTL.bit.QCLM=1; 		// Latch on unit time out
	EQep1Regs.QPOSMAX=0xffffffff;
	EQep1Regs.QEPCTL.bit.QPEN=1; 		// QEP enable

	EQep1Regs.QCAPCTL.bit.UPPS=5;   	// 1/32 for unit position
	EQep1Regs.QCAPCTL.bit.CCPS=7;		// 1/128 for CAP clock
	EQep1Regs.QCAPCTL.bit.CEN=1; 		// QEP Capture Enable
}


// TI File $Revision: /main/9 $
// Checkin $Date: April 21, 2008   15:42:23 $
//###########################################################################
//
// FILE:	Example_posspeed.c
//
// TITLE:	Pos/speed measurement using EQEP peripheral
//
// DESCRIPTION:
//
// This file includes the EQEP initialization and position and speed calcuation
// functions called by Example_2833xEqep_posspeed.c.  The position and
// speed calculation steps performed by POSSPEED_Calc() at  SYSCLKOUT = 150 MHz
// and 100 MHz are described in detail below:
//
// For 150 MHz Operation:
// ----------------------
//
// 1. This program calculates: **theta_mech**
//
//    theta_mech = QPOSCNT/mech_Scaler = QPOSCNT/4000, where 4000 is the number of
//                 counts in 1 revolution.(4000/4 = 1000 line/rev. quadrature encoder)
//
// 2. This program calculates: **theta_elec**
//
//    theta_elec = (# pole pairs) * theta_mech = 2*QPOSCNT/4000 for this example
//
// 3. This program calculates: **SpeedRpm_fr**
//
//    SpeedRpm_fr = [(x2-x1)/4000]/T                                             - Equation 1
//    Note (x2-x1) = difference in number of QPOSCNT counts. Dividing (x2-x1) by
//    4000 gives position relative to Index in one revolution.
// If base RPM  = 6000 rpm:   6000 rpm = [(x2-x1)/4000]/10ms                     - Equation 2
//                                     = [(x2-x1)/4000]/(.01s*1 min/60 sec)
//                                     = [(x2-x1)/4000]/(1/6000) min
//                         max (x2-x1) = 4000 counts, or 1 revolution in 10 ms
//
//
// If both sides of Equation 2 are divided by 6000 rpm, then:
//                            1 = [(x2-x1)/4000] rev./[(1/6000) min * 6000rpm]
//							Because (x2-x1) must be <4000 (max) for QPOSCNT increment,
//						    (x2-x1)/4000 < 1 for CW rotation
//                          And because (x2-x1) must be >-4000 for QPOSCNT decrement,
//                          (x2-x1)/4000>-1  for CCW rotation
//						    speed_fr = [(x2-x1)/4000]/[(1/6000) min * 6000rpm]
//                                   = (x2-x1)/4000                              - Equation 3
//
// To convert speed_fr to RPM, multiply Equation 3 by 6000 rpm
//                           SpeedRpm_fr = 6000rpm *(x2-x1)/4000                 - Final Equation
//
//
// 2. **min rpm ** = selected at 10 rpm based on CCPS prescaler options available (128 is greatest)
//
// 3. **SpeedRpm_pr**
//    SpeedRpm_pr = X/(t2-t1)                                                    - Equation 4
//    where X = QCAPCTL [UPPS]/4000 rev. (position relative to Index in 1 revolution)
// If max/base speed = 6000 rpm:  6000 = (32/4000)/[(t2-t1)/(150MHz/128)]
//    where 32 = QCAPCTL [UPPS] (Unit timeout - once every 32 edges)
//          32/4000 = position in 1 revolution (position as a fraction of 1 revolution)
//          t2-t1/(150MHz/128),  t2-t1= # of QCAPCLK cycles, and
//		                  1 QCAPCLK cycle = 1/(150MHz/128)
//										  = QCPRDLAT
//
//		        So: 6000 rpm = [32(150MHz/128)*60s/min]/[4000(t2-t1)]
//		             t2-t1 = [32(150MHz/128)*60 s/min]/(4000*6000rpm)           - Equation 5
//		                   = 94 CAPCLK cycles = maximum (t2-t1) = SpeedScaler
//
// Divide both sides by (t2-t1), and:
//                   1 = 94/(t2-t1) = [32(150MHz/128)*60 s/min]/(4000*6000rpm)]/(t2-t1)
//				     Because (t2-t1) must be < 94 for QPOSCNT increment:
//				     94/(t2-t1) < 1 for CW rotation
//                   And because (t2-t1) must be >-94 for QPOSCNT decrement:
//				     94/(t2-t1)> -1 for CCW rotation
//
//					 speed_pr = 94/(t2-t1)
//                      or [32(150MHz/128)*60 s/min]/(4000*6000rpm)]/(t2-t1)  - Equation 6
//
// To convert speed_pr to RPM:
// Multiply Equation 6 by 6000rpm:
//                  SpeedRpm_fr  = 6000rpm * [32(150MHz/128)*60 s/min]/[4000*6000rpm*(t2-t1)]
//							                = [32(150MHz/128)*60 s/min]/[4000*(t2-t1)]
//                                        or [(32/4000)rev * 60 s/min]/[(t2-t1)(QCPRDLAT)]- Final Equation



interrupt void cpu_timer0_isr()			//定时中断0服务程序，1ms周期
{
//	ReloadCpuTimer0();					//重置定时器
//	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;		//重置允许其他中断
//
//	EvTimer2_InterruptCount++;//事件定时器2计数器,EvTimer2_InterruptCount为32位计数，1ms周期，可以计数49.7天

}

//===========================================================================
// No more.
//===========================================================================
