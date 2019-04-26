// TI File $Revision: /main/1 $
// Checkin $Date: August 18, 2006   13:46:06 $
//###########################################################################
//
// FILE:	DSP2833x_DefaultIsr.c
//
// TITLE:	DSP2833x Device Default Interrupt Service Routines.
//
// This file contains shell ISR routines for the 2833x PIE vector table.
// Typically these shell ISR routines can be used to populate the entire PIE 
// vector table during device debug.  In this manner if an interrupt is taken
// during firmware development, there will always be an ISR to catch it.  
//
// As develpment progresses, these ISR rotuines can be eliminated and replaced
// with the user's own ISR routines for each interrupt.  Since these shell ISRs
// include infinite loops they will typically not be included as-is in the final
// production firmware. 
//
//###########################################################################
// $TI Release: DSP2833x Header Files V1.01 $
// $Release Date: September 26, 2007 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "math.h"
#include "string.h"
#include "UART.h"
#include <stdint.h>

void Duty(void); 
void DelayAdc(volatile Uint16);
void Posi_PID(void);
void Velo_PID(void);
void scib_xmit(int DataImage);
void POSSPEED_Calc(POSSPEED *p);
void POSSPEED_Init(void);
void Timer0_work(void);
void Frame_Settle(void);
extern void UART_SEND (char *databuff,int len);


volatile Uint32 EvTimer2_InterruptCount = 0;

float Angle_Fw_EncoderAAA = 0;

extern int RS422Length;

int Rate_Counter_FW = 0;
int Rate_Counter_FY = 0;


int FW_ZERO_ON = 15000;

float Angle_Fw_Encoder = 0;
float Angle_Fy_Revover = 0;
float Rate_Fw_Encoder = 0;
float Rate_Fy_Revover = 0;
float Cmd_Angle_y = -20.0;  //��λ�Ƕ�
float Cmd_Angle_z = 0.0;  //�����Ƕ�
float Cmd_Rate_y = -5.0;  //��λ��su��
float Cmd_Rate_z = 10.0;  //������su��

float IMU_Rate_x = 0;	//	imu ���ٶ� x����
float IMU_Rate_y = 0;	//	imu ���ٶ� y����
float IMU_Rate_z = 0;	//	imu ���ٶ� z����

float IMU_Angle_x = 0;	//	imu �Ƕ� x����
float IMU_Angle_y = 0;	//	imu �Ƕ� y����
float IMU_Angle_z = 0;	//	imu �Ƕ� z����

extern float Epk_y;     //  Delta_Posiz_k
extern float Epk_z;
extern float Evk_y;     //  Delta_Posiz_k
extern float Evk_z;
extern float PulseCounterOld;
extern float RevovorAngleOld;
//extern int RS422Length;
extern int Flag_Fw_Zero;
extern float Fw_Zero;
extern int Counter_Fw_Zero;

extern int Image[16];
char sbuff_tp[249];   //����֡�����ݲ���
char Send_buff[256];  //����buffer
extern int Work_Mode;


POSSPEED qep_posspeed = POSSPEED_DEFAULTS;


// Connected to INT13 of CPU (use MINT13 mask):
// Note CPU-Timer1 is reserved for TI use, however XINT13
// ISR can be used by the user. 
interrupt void INT13_ISR(void)     // INT13 or CPU-Timer1
{
  // Insert ISR Code here
  
  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// Note CPU-Timer2 is reserved for TI use.
interrupt void INT14_ISR(void)     // CPU-Timer2
{
  // Insert ISR Code here
  
  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

interrupt void DATALOG_ISR(void)   // Datalogging interrupt
{
   // Insert ISR Code here
 
   // Next two lines for debug only to halt the processor here
   // Remove after inserting ISR Code
   asm ("      ESTOP0");
   for(;;);
}

interrupt void RTOSINT_ISR(void)   // RTOS interrupt
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

interrupt void EMUINT_ISR(void)    // Emulation interrupt
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

interrupt void NMI_ISR(void)       // Non-maskable interrupt
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

interrupt void ILLEGAL_ISR(void)   // Illegal operation TRAP
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm("          ESTOP0");
  for(;;);

}


interrupt void USER1_ISR(void)     // User Defined trap 1
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}

interrupt void USER2_ISR(void)     // User Defined trap 2
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);


}

interrupt void USER3_ISR(void)     // User Defined trap 3
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

interrupt void USER4_ISR(void)     // User Defined trap 4
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

interrupt void USER5_ISR(void)     // User Defined trap 5
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

interrupt void USER6_ISR(void)     // User Defined trap 6
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

interrupt void USER7_ISR(void)     // User Defined trap 7
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

interrupt void USER8_ISR(void)     // User Defined trap 8
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

interrupt void USER9_ISR(void)     // User Defined trap 9
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

interrupt void USER10_ISR(void)    // User Defined trap 10
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

interrupt void USER11_ISR(void)    // User Defined trap 11
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

interrupt void USER12_ISR(void)     // User Defined trap 12
{
 // Insert ISR Code here
 
  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}

// -----------------------------------------------------------
// PIE Group 1 - MUXed into CPU INT1
// -----------------------------------------------------------

// INT1.1 
interrupt void SEQ1INT_ISR(void)   //SEQ1 ADC
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code

  asm ("      ESTOP0");
  for(;;);

}     

// INT1.2 
interrupt void SEQ2INT_ISR(void)  //SEQ2 ADC
{

  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  
  asm("	  ESTOP0");
  for(;;);
	
}
// INT1.3 - Reserved

// INT1.4
interrupt void  XINT1_ISR(void)
{

    extern int ADresult1;     // Channel 1 result    after test cancelled                                         
    extern int ADresult2;     // Channel 2 result    after test cancelled                                       
    extern int ADresult3;     // Channel 3 result    after test cancelled                                        
    extern int ADresult4;     // Channel 4 result    after test cancelled
    extern unsigned int NumGyroInt;  // after test cancelled
	extern unsigned int NumPosiInt;

    extern int selector;      // Used to identify which signal to be collected                                                 
    extern float Vely;
    extern float Velz;
    extern float Posiy;
    extern float Posiz;                                        
    
    float BiasRatey   = 0.002;
    float BiasRatez   = 0.002;
    float BiasPosiy   = 0.003;
    float BiasPosiz   = 0.003;
	float ScalarRatey = 0.1;
	float ScalarRatez = 0.1;
	float ScalarPosiy = 0.22;
	float ScalarPosiz = 0.22;

    if (selector == 1)        // Gyro information collection                                            
    {                                                                    
      ADresult1 = *(unsigned int *)0x004000;                              
      ADresult2 = *(unsigned int *)0x004000;  
     // ADresult3 = *(unsigned int *)0x004000;                              
     // ADresult4 = *(unsigned int *)0x004000;                              
      Vely      = ( ADresult1 * 0.305 / 1000 - BiasRatey ) / ScalarRatey;     // Velocity                          
      Velz      = ( ADresult2 * 0.305 / 1000 - BiasRatez ) / ScalarRatez;     // Velocity                         
      NumGyroInt++;                                                       
    }                                                                    
                                                                          
    else if (selector == 2)                                               
    {                                                                    
      //ADresult1 = *(unsigned int *)0x004000;                              
      //ADresult2 = *(unsigned int *)0x004000;                              
      ADresult3 = *(unsigned int *)0x004000;                              
      ADresult4 = *(unsigned int *)0x004000;                              
      Posiy     = ((ADresult3 * 0.305/1000) - BiasPosiy) / ScalarPosiy; 
      Posiz     = ((ADresult4 * 0.305/1000) - BiasPosiz) / ScalarPosiz;

      Angle_Fy_Revover  =   Posiz; //�����λ�ƽǶ�

      Rate_Counter_FY++;

      if(Rate_Counter_FY >= 9)
      {
         Rate_Fy_Revover = (Angle_Fy_Revover - RevovorAngleOld)/0.01;

      }

      RevovorAngleOld =  Angle_Fy_Revover;


      NumPosiInt++; 
    }                                                                    
                                                                          
    else                                                                  
     {                                                                    
      asm(" RPT #100 || NOP");                                            
     }                                                                    
                                                                          
    PieCtrlRegs.PIEACK.bit.ACK1 = 1; 
    

}     

// INT1.5
interrupt void  XINT2_ISR(void)
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}

// INT1.6
interrupt void  ADCINT_ISR(void)     // ADC
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; 
  
  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT1.7
interrupt void  TINT0_ISR(void)      // CPU-Timer 0
{
//  extern  int ProgStep;
//
//  extern int ADresult1;     // Channel 1 result    after test cancelled
//  extern int ADresult2;     // Channel 2 result    after test cancelled
//  extern int ADresult3;     // Channel 3 result    after test cancelled
//  extern int ADresult4;     // Channel 4 result    after test cancelled
//  extern unsigned int NumGyroInt;  // after test cancelled
//	extern unsigned int NumPosiInt;
//
//  extern int selector;      // Used to identify which signal to be collected
//  extern float Vely;
//  extern float Velz;
//  extern float Posiy;
//  extern float Posiz;
//
//  extern int Servo_Mode;

  extern Uint32 Interrupt_Count;
  extern int Flag_CPU_Timer0;

//  float BiasRatey   = 0.002;
//  float BiasRatez   = 0.002;
//  float BiasPosiy   = 3.467545;
//  float BiasPosiz   = 0.003;
//	float ScalarRatey = 0.1;
//	float ScalarRatez = 0.1;
//	float ScalarPosiy = 14.0 * 1.046;  //70deg  5V
//	float ScalarPosiz = 0.22;
//
// 	static float64 LastYPos = 0,LastZPos = 0;
// 	static float64 y_1[2] = {0,0},x_1[2] = {0,0};
// 	float64 temp,temp2;

 	Interrupt_Count++;
 	Flag_CPU_Timer0 = 1;

//    ScibRegs.SCITXBUF = 0xAB;


  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; 
  
  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  //for(;;);

	ReloadCpuTimer0();					//���ö�ʱ��
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;		//�������������ж�

	EvTimer2_InterruptCount++;//�¼���ʱ��2������,EvTimer2_InterruptCountΪ32λ������1ms���ڣ����Լ���49.7��

}


// INT1.8
interrupt void  WAKEINT_ISR(void)    // WD, LOW Power
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; 
  
  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}


// -----------------------------------------------------------
// PIE Group 2 - MUXed into CPU INT2
// -----------------------------------------------------------

// INT2.1
interrupt void EPWM1_TZINT_ISR(void)    // EPWM-1
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT2.2
interrupt void EPWM2_TZINT_ISR(void)    // EPWM-2
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT2.3
interrupt void EPWM3_TZINT_ISR(void)    // EPWM-3
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
  
  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}


// INT2.4
interrupt void EPWM4_TZINT_ISR(void)    // EPWM-4
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
  
  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}


// INT2.5
interrupt void EPWM5_TZINT_ISR(void)    // EPWM-5
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT2.6
interrupt void EPWM6_TZINT_ISR(void)   // EPWM-6
{
  // Insert ISR Code here

   
  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
  
  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT2.7 - Reserved
// INT2.8 - Reserved

// -----------------------------------------------------------
// PIE Group 3 - MUXed into CPU INT3
// -----------------------------------------------------------
   
// INT 3.1         
interrupt void EPWM1_INT_ISR(void)     // EPWM-1
{
     
  asm ("      ESTOP0");
  for(;;);
}

// INT3.2
interrupt void EPWM2_INT_ISR(void)     // EPWM-2
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT3.3
interrupt void EPWM3_INT_ISR(void)    // EPWM-3
{
  
  extern float Pwm1IntNum_fy;
  extern float Pwm1IntNum_fw;
  extern float angle_fy;
  extern float angle_fw;
  extern float direction_fy;
  extern float direction_fw;
  extern int Pwm1IntNum;


  float step_fy = 0.225;
  float step_fw = 0.9;

  Pwm1IntNum_fy++;
  Pwm1IntNum_fw++;
  Pwm1IntNum++;

  angle_fy = angle_fy + Pwm1IntNum_fy * step_fy * direction_fy;
  angle_fw = angle_fw + Pwm1IntNum_fw * step_fw * direction_fw;

  if (angle_fy > 10.0)
  {
	   Pwm1IntNum_fy = 5;
	   direction_fy  = direction_fy * -1;
	   GpioDataRegs.GPBSET.bit.GPIO45 = 1;
  }
  else if (angle_fy < -10.0)
  {
	   Pwm1IntNum_fy = 5;
	   direction_fy  = direction_fy * -1;
	   GpioDataRegs.GPBSET.bit.GPIO45 = 1;
  }
  else
  {
	  Pwm1IntNum_fy = Pwm1IntNum_fy;
  }

  if (angle_fw > 15.0)
  {
	   Pwm1IntNum_fw = 5;
	   direction_fw  = direction_fw * -1;
	   GpioDataRegs.GPCSET.bit.GPIO85 = 1;
  }
  else if (angle_fw < -15.0)
  {
	   Pwm1IntNum_fw = 5;
	   direction_fw  = direction_fw * -1;
	   GpioDataRegs.GPCCLEAR.bit.GPIO85 = 1;
  }
  else
  {
   asm(" RPT #10 || NOP"); 
  }
     
     
     //****** ���ж� ******//     
     EPwm3Regs.ETCLR.bit.INT       = 1;    // д 1 �� ��
     PieCtrlRegs.PIEACK.bit.ACK3   = 1;

}

// INT3.4
interrupt void EPWM4_INT_ISR(void)    // EPWM-4
{
  // Insert ISR Code here
  
  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;  

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT3.5
interrupt void EPWM5_INT_ISR(void)    // EPWM-5
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;  

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT3.6
interrupt void EPWM6_INT_ISR(void)    // EPWM-6
{
  extern float Upk_y;     // Velo_ordery
  extern float Upk_z;     // Velo_orderz
  extern float Evk_z;  // Delta_Veloz_k;
  extern float Evk_y;  // Delta_Veloy_k;
  extern float Vely;
  extern float Velz;
  extern int selector;
  extern int Pwm6IntNum;

  selector = 1;                            // channel A to get Velocity informations
  GpioDataRegs.GPASET.bit.GPIO15   = 1;    // ����ת������ָ��  A
  DelayAdc(5);                             // �ȴ�ADת����� 
  GpioDataRegs.GPACLEAR.bit.GPIO15 = 1;    // ����ֹͣת��ָ��  A
  //DelayAdc(5);

  Evk_y = Upk_y - Vely;
  Evk_z = Upk_z - Velz;

  //Velo_PID();

  //Duty();

  Pwm6IntNum++;

  //*****����жϱ�־
  EPwm6Regs.ETCLR.bit.INT = 1;      // д 1 �� ��
  PieCtrlRegs.PIEACK.bit.ACK3   = 1;
  //*****����жϱ�־
}

// INT3.7 - Reserved
// INT3.8 - Reserved


// -----------------------------------------------------------
// PIE Group 4 - MUXed into CPU INT4
// -----------------------------------------------------------

// INT 4.1
interrupt void ECAP1_INT_ISR(void)    // ECAP-1
{
  // Insert ISR Code here
  
  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;  

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT4.2
interrupt void ECAP2_INT_ISR(void)    // ECAP-2
{

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT4.3
interrupt void ECAP3_INT_ISR(void)    // ECAP-3
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
  
  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT4.4
interrupt void ECAP4_INT_ISR(void)     // ECAP-4
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT4.5
interrupt void ECAP5_INT_ISR(void)     // ECAP-5
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}
// INT4.6
interrupt void ECAP6_INT_ISR(void)     // ECAP-6
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}
// INT4.7 - Reserved
// INT4.8 - Reserved

// -----------------------------------------------------------
// PIE Group 5 - MUXed into CPU INT5
// -----------------------------------------------------------

// INT 5.1
interrupt void EQEP1_INT_ISR(void)    // EQEP-1
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT5.2
interrupt void EQEP2_INT_ISR(void)    // EQEP-2
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
 
  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT5.3 - Reserved
// INT5.4 - Reserved
// INT5.5 - Reserved
// INT5.6 - Reserved
// INT5.7 - Reserved
// INT5.8 - Reserved

// -----------------------------------------------------------
// PIE Group 6 - MUXed into CPU INT6
// -----------------------------------------------------------

// INT6.1
interrupt void SPIRXINTA_ISR(void)    // SPI-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
   asm ("      ESTOP0");
   for(;;);
}

// INT6.2
interrupt void SPITXINTA_ISR(void)     // SPI-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
   asm ("      ESTOP0");
   for(;;);
}

// INT6.3 
interrupt void MRINTB_ISR(void)     // McBSP-B
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT6.4
interrupt void MXINTB_ISR(void)     // McBSP-B
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT6.5
interrupt void MRINTA_ISR(void)     // McBSP-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
   asm ("      ESTOP0");
   for(;;);
}

// INT6.6
interrupt void MXINTA_ISR(void)     // McBSP-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
   asm ("      ESTOP0");
   for(;;);
}

// INT6.7 - Reserved
// INT6.8 - Reserved




// -----------------------------------------------------------
// PIE Group 7 - MUXed into CPU INT7
// -----------------------------------------------------------

// INT7.1
interrupt void DINTCH1_ISR(void)     // DMA
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
   asm ("      ESTOP0");
   for(;;);
}

// INT7.2
interrupt void DINTCH2_ISR(void)     // DMA
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
   asm ("      ESTOP0");
   for(;;);
}

// INT7.3
interrupt void DINTCH3_ISR(void)     // DMA
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
   asm ("      ESTOP0");
   for(;;);
}

// INT7.4
interrupt void DINTCH4_ISR(void)     // DMA
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
   asm ("      ESTOP0");
   for(;;);
}

// INT7.5
interrupt void DINTCH5_ISR(void)     // DMA
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
   asm ("      ESTOP0");
   for(;;);
}

// INT7.6
interrupt void DINTCH6_ISR(void)     // DMA
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
   asm ("      ESTOP0");
   for(;;);
}

// INT7.7 - Reserved
// INT7.8 - Reserved

// -----------------------------------------------------------
// PIE Group 8 - MUXed into CPU INT8
// -----------------------------------------------------------

// INT8.1
interrupt void I2CINT1A_ISR(void)     // I2C-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT8.2
interrupt void I2CINT2A_ISR(void)     // I2C-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);
}

// INT8.3 - Reserved
// INT8.4 - Reserved

// INT8.5
interrupt void SCIRXINTC_ISR(void)     // SCI-C
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}

// INT8.6
interrupt void SCITXINTC_ISR(void)     // SCI-C
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}

// INT8.7 - Reserved
// INT8.8 - Reserved


// -----------------------------------------------------------
// PIE Group 9 - MUXed into CPU INT9
// -----------------------------------------------------------

// INT9.1
interrupt void SCIRXINTA_ISR(void)     // SCI-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}

// INT9.2
interrupt void SCITXINTA_ISR(void)     // SCI-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}


// INT9.3
interrupt void SCIRXINTB_ISR(void)     // SCI-B
{

	extern int ScibLoopNum;        // Scib Interrupt Number   Cancelled after Debug
//	extern int Imagedata[40];       // 6�ֽ�ͼ��ԭʼ���ݣ����Խ������Ϊ�ڲ�����

  //float Pixel = 15/1000;           // ͼ����Ԫ�ߴ�
//  int i = 0;
  //int f = 90;                    // ����

//  for(i=0;i<15;i++)
//  Imagedata[i] = 0x0000;


//  for(i=0;i<RS422Length;i++)
//  {
//	  Imagedata[i] = ScibRegs.SCIRXBUF.all;
//  }
//

    if(ScibRegs.SCIFFRX.bit.RXFFST == 1)
    {
		  char temp = ScibRegs.SCIRXBUF.all;
		  uart_rdataproc(&rev_data,&temp);
    }

    ScibLoopNum++;

//    if(ScibLoopNum >= 9)
//    {
//    	ScibLoopNum = 0;
//    }

//
//     if(ScibLoopNum >= 15)
//     {
//    	 ScibLoopNum = 0;
//     }
//

  //******Clear Interrupt Flag******//
     //ScibLoopNum++;
     ScibRegs.SCIFFRX.bit.RXFFOVRCLR  = 1;
     ScibRegs.SCIFFRX.bit.RXFFINTCLR  = 1;
     ScibRegs.SCIFFTX.bit.TXFFINTCLR  = 1;
     ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;
     asm(" RPT #7 || NOP");
     ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
     PieCtrlRegs.PIEACK.bit.ACK9      = 1;
  //******Clear Interrupt Flag******//



}

// INT9.4
interrupt void SCITXINTB_ISR(void)     // SCI-B
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}

// INT9.5
interrupt void ECAN0INTA_ISR(void)  // eCAN-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}

// INT9.6
interrupt void ECAN1INTA_ISR(void)  // eCAN-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}

// INT9.7
interrupt void ECAN0INTB_ISR(void)  // eCAN-B
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}

// INT9.8
interrupt void ECAN1INTB_ISR(void)  // eCAN-B
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}

// -----------------------------------------------------------
// PIE Group 10 - MUXed into CPU INT10
// -----------------------------------------------------------

// INT10.1 - Reserved
// INT10.2 - Reserved
// INT10.3 - Reserved
// INT10.4 - Reserved
// INT10.5 - Reserved
// INT10.6 - Reserved
// INT10.7 - Reserved
// INT10.8 - Reserved


// -----------------------------------------------------------
// PIE Group 11 - MUXed into CPU INT11
// -----------------------------------------------------------

// INT11.1 - Reserved
// INT11.2 - Reserved
// INT11.3 - Reserved
// INT11.4 - Reserved
// INT11.5 - Reserved
// INT11.6 - Reserved
// INT11.7 - Reserved
// INT11.8 - Reserved

// -----------------------------------------------------------
// PIE Group 12 - MUXed into CPU INT12
// -----------------------------------------------------------

// INT12.1
interrupt void XINT3_ISR(void)  // External Interrupt
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}

// INT12.2
interrupt void XINT4_ISR(void)  // External Interrupt
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}

// INT12.3
interrupt void XINT5_ISR(void)  // External Interrupt
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}
// INT12.4
interrupt void XINT6_ISR(void)  // External Interrupt
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}

// INT12.5
interrupt void XINT7_ISR(void)  // External Interrupt
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}

// INT12.6 - Reserved
// INT12.7
interrupt void LVF_ISR(void)  // Latched overflow
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}
// INT12.8
interrupt void LUF_ISR(void)  // Latched underflow
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt 
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  asm ("      ESTOP0");
  for(;;);

}

//---------------------------------------------------------------------------
// Catch All Default ISRs: 
//

interrupt void EMPTY_ISR(void)  // Empty ISR - only does a return.
{

}

interrupt void PIE_RESERVED(void)  // Reserved space.  For test.
{
  asm ("      ESTOP0");
  for(;;);
}

interrupt void rsvd_ISR(void)      // For test
{
  asm ("      ESTOP0");
  for(;;);
}



void POSSPEED_Calc(POSSPEED *p)
{
     long tmp;
     unsigned int pos16bval,temp1;
   	 _iq Tmp1,newp,oldp;

 	static float64 LastYPos = 0,LastZPos = 0;
 	static float64 y_1[2] = {0,0},x_1[2] = {0,0};
 	float64 temp,temp2;


//**** Position calculation - mechanical and electrical motor angle  ****//
     p->DirectionQep = EQep1Regs.QEPSTS.bit.QDF;    // Motor direction: 0=CCW/reverse, 1=CW/forward


	 pos16bval=(unsigned int)EQep1Regs.QPOSCNT;     // capture position once per QA/QB period

     //Angle_Fw_Encoder = 360 * (float)EQep1Regs.QPOSCNT/65536;


     p->theta_raw = pos16bval+ p->cal_angle;        // raw theta = current pos. + ang. offset from QA

     Angle_Fw_Encoder = 360 * (float)(p->theta_raw)/65536;

     Angle_Fw_Encoder = Angle_Fw_Encoder - Fw_Zero;

     Rate_Counter_FW++;

	 if(Rate_Counter_FW >= 9)
	 {
		 	Rate_Fw_Encoder = (Angle_Fw_Encoder - PulseCounterOld )/0.01;
	    //	temp = Angle_Fw_Encoder - LastZPos;  //��λ
	    	temp2 = Rate_Fw_Encoder;//Rate_Fw_Encoder;//360*temp;  //360??
	    	temp = 0.8182*y_1[1] + 0.09091*(temp2 + x_1[1]);
	    	x_1[1] = temp2;
	    	y_1[1] = temp;
	    	Rate_Fw_Encoder = temp;

	    	Rate_Counter_FW = 0;
	    	PulseCounterOld = Angle_Fw_Encoder;
	 }



	 // The following lines calculate p->theta_mech ~= QPOSCNT/mech_scaler [current cnt/(total cnt in 1 rev.)]
	 // where mech_scaler = 4000 cnts/revolution
     tmp = (long)((long)p->theta_raw*(long)p->mech_scaler);  	// Q0*Q26 = Q26
     tmp &= 0x03FFF000;
     p->theta_mech = (int)(tmp>>11);         		// Q26 -> Q15
     p->theta_mech &= 0x7FFF;

	 // The following lines calculate p->elec_mech
     p->theta_elec = p->pole_pairs*p->theta_mech;  // Q0*Q15 = Q15
     p->theta_elec &= 0x7FFF;

// Check an index occurrence
     if (EQep1Regs.QFLG.bit.IEL == 1)
     {
    	p->index_sync_flag = 0x00F0;
    	EQep1Regs.QCLR.bit.IEL=1;					// Clear interrupt flag
     }



//**** High Speed Calcultation using QEP Position counter ****//
// Check unit Time out-event for speed calculation:
// Unit Timer is configured for 100Hz in INIT function

	if(EQep1Regs.QFLG.bit.UTO==1)                    // If unit timeout (one 100Hz period)
	{
		/** Differentiator	**/
		// The following lines calculate position = (x2-x1)/4000 (position in 1 revolution)
	 	pos16bval=(unsigned int)EQep1Regs.QPOSLAT;	              // Latched POSCNT value
     	tmp = (long)((long)pos16bval*(long)p->mech_scaler);  	  // Q0*Q26 = Q26
     	tmp &= 0x03FFF000;
     	tmp = (int)(tmp>>11);         			                  // Q26 -> Q15
     	tmp &= 0x7FFF;
		newp=_IQ15toIQ(tmp);
		oldp=p->oldpos;

   		if (p->DirectionQep==0)      				// POSCNT is counting down
   		{
    		if (newp>oldp)
      			Tmp1 = - (_IQ(1) - newp + oldp);    // x2-x1 should be negative
    		else
      		Tmp1 = newp -oldp;
   		}
   		else if (p->DirectionQep==1)      			// POSCNT is counting up
   		{
    		if (newp<oldp)
      		Tmp1 = _IQ(1) + newp - oldp;
    		else
      		Tmp1 = newp - oldp;                     // x2-x1 should be positive
   		}

	   	if (Tmp1>_IQ(1))
	     	p->Speed_fr = _IQ(1);
	   	else if (Tmp1<_IQ(-1))
	     	p->Speed_fr = _IQ(-1);
	   	else
	     	p->Speed_fr = Tmp1;

		// Update the electrical angle
    	p->oldpos = newp;

		// Change motor speed from pu value to rpm value (Q15 -> Q0)
		// Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
   		p->SpeedRpm_fr = _IQmpy(p->BaseRpm,p->Speed_fr);
		//=======================================


		EQep1Regs.QCLR.bit.UTO=1;					// Clear interrupt flag
	}

//**** Low-speed computation using QEP capture counter ****//
	if(EQep1Regs.QEPSTS.bit.UPEVNT==1)                 // Unit position event
	{
		if(EQep1Regs.QEPSTS.bit.COEF==0)               // No Capture overflow
			temp1=(unsigned long)EQep1Regs.QCPRDLAT;   // temp1 = t2-t1
		else							               // Capture overflow, saturate the result
			temp1=0xFFFF;

		p->Speed_pr = p->SpeedScaler/temp1;//;_IQdiv(p->SpeedScaler,temp1);    // p->Speed_pr = p->SpeedScaler/temp1
		Tmp1=p->Speed_pr;

		if (Tmp1>_IQ(1))
	 		p->Speed_pr = _IQ(1);
		else
	 		p->Speed_pr = Tmp1;

	    // Convert p->Speed_pr to RPM
		if (p->DirectionQep==0)                                 // Reverse direction = negative
			p->SpeedRpm_pr = -_IQmpy(p->BaseRpm,p->Speed_pr); 	// Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
		else                                                    // Forward direction = positive
			p->SpeedRpm_pr = _IQmpy(p->BaseRpm,p->Speed_pr); 	// Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q

   		Angle_Fw_EncoderAAA = p->SpeedRpm_fr * 360/60;

		EQep1Regs.QEPSTS.all=0x88;					// Clear Unit position event flag
													// Clear overflow error flag
	}
}

void Timer0_work(void)
{

		  extern  int ProgStep;

		  extern int ADresult1;     // Channel 1 result    after test cancelled
		  extern int ADresult2;     // Channel 2 result    after test cancelled
		  extern int ADresult3;     // Channel 3 result    after test cancelled
		  extern int ADresult4;     // Channel 4 result    after test cancelled
		  extern unsigned int NumGyroInt;  // after test cancelled
		  extern unsigned int NumPosiInt;

		  extern int selector;      // Used to identify which signal to be collected
		  extern float Vely;
		  extern float Velz;
		  extern float Posiy;
		  extern float Posiz;

		  extern int Servo_Mode;

		  extern Uint32 Interrupt_Count;
		  extern int Flag_CPU_Timer0;

//		  float BiasRatey   = 0.002;
//		  float BiasRatez   = 0.002;
		  float BiasPosiy   = 3.467545;
//		  float BiasPosiz   = 0.003;
//			float ScalarRatey = 0.1;
//			float ScalarRatez = 0.1;
			float ScalarPosiy = 14.0 * 1.046;  //70deg  5V
//			float ScalarPosiz = 0.22;

//		 	static float64 LastYPos = 0,LastZPos = 0;
		 	static float64 y_1[2] = {0,0},x_1[2] = {0,0};
		 	float64 temp,temp2;



   	qep_posspeed.calc(&qep_posspeed);//��ȡ������Ϣ,ִ��POSSPEED_Calc(POSSPEED *p)�����������������eqep�Ĳ�����������ת���ǶȺͽ��ٶȻ�ȡ;
   	/***/
   	GpioDataRegs.GPASET.bit.GPIO15   = 1;    // ����ת������ָ��  A��A��ӦVIN1��VIN2������VIN1�ӷ�λ����λ���أ�VIN2δ���κδ�����
   	DelayAdc(5);                             // �ȴ�ADת�����
   	GpioDataRegs.GPACLEAR.bit.GPIO15 = 1;    // ����ֹͣת��ָ��  A

   	DelayAdc(100);

       ADresult1 = *(unsigned int *)0x004000;
       ADresult2 = *(unsigned int *)0x004000;

       //�ϵ�����λ
       if(Flag_Fw_Zero == 0)
       {
		   if(ADresult1 > FW_ZERO_ON)
		   {
				Flag_Fw_Zero = 1;
				DelayAdc(10);
				Fw_Zero = Angle_Fw_Encoder;
		   }
       }
   //    else if (Interrupt_Count > 10000)
   //    {
   //    	Fw_Zero = Fw_Zero/(float)(Counter_Fw_Zero);
   //    }


   	DelayAdc(300);

   	GpioDataRegs.GPASET.bit.GPIO17   = 1;    // ����ת������ָ��  C��������λ�Ƶĳ�ͷ�ӵ���AD��VIN5��
   	DelayAdc(5);                             // �ȴ�ADת�����
   	GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;    // ����ֹͣת��ָ��  C

   	DelayAdc(100);

       ADresult3 = *(unsigned int *)0x004000;
       ADresult4 = *(unsigned int *)0x004000;
       Posiy     = ((ADresult3 * 0.305/1000) - BiasPosiy) * ScalarPosiy;
       //Posiz     = ((ADresult4 * 0.305/1000) - BiasPosiz) / ScalarPosiz;

       Angle_Fy_Revover  =  Posiy; //�����λ�ƽǶ�

       Rate_Counter_FY++;

       if(Rate_Counter_FY >= 9)
       {
			Rate_Fy_Revover = (Angle_Fy_Revover - RevovorAngleOld)/0.01;

			temp2 = Rate_Fy_Revover;//360*temp;
			temp = 0.8182*y_1[0] + 0.09091*(temp2 + x_1[0]);
			x_1[0] = temp2;
			y_1[0] = temp;
			Rate_Fy_Revover = temp;

			Rate_Counter_FY = 0;
		    RevovorAngleOld =  Angle_Fy_Revover;
       }

   	/*����ת̨�ĽǶȺ��ٶ�
   	 * ת̨ת�ٷ��͸��Կأ�0x08
   	 * ת̨�Ƕȷ��͸�FPGA��0x00*/
   	 if(Rate_Counter_FY >= 100)
   	 {
   		 char temp_08[6],temp_00[6];
   		 char buf_trs[20];
   		 int k=0;
   		 int16_t FW_temp,FY_temp;

   		 FW_temp = (int16_t)(Rate_Fw_Encoder*10);
   		 FY_temp = (int16_t)(Rate_Fy_Revover*10);
   		 temp_08[k++] = 0x09;temp_08[k++] = (FW_temp>>8)&0x00FF;temp_08[k++] = FW_temp&0x00FF;//��λת̨ת��
   		 temp_08[k++] = 0x0A;temp_08[k++] = (FY_temp>>8)&0x00FF;temp_08[k++] = FY_temp&0x00FF;//����ת̨ת��
   		 int len = Fram_Make(buf_trs,temp_08,0x08,k);
   		 UART_SEND(buf_trs,len);

   		 FW_temp = (int16_t)(Angle_Fw_Encoder*100);
   		 FY_temp = (int16_t)(Angle_Fy_Revover*100);
   		 k=0;
   		 temp_00[k++] = 0x0B;temp_00[k++] = (FW_temp>>8)&0x00FF;temp_00[k++] = FW_temp&0x00FF;//��λת̨�Ƕ�
   		 temp_00[k++] = 0x0C;temp_00[k++] = (FY_temp>>8)&0x00FF;temp_00[k++] = FY_temp&0x00FF;//����ת̨�Ƕ�
   		 len = 0;
   		 len = Fram_Make(buf_trs,temp_00,0x00,k);
   		 UART_SEND(buf_trs,len);
   	 }


       //�ڴ˴�����Ƕȿ�����  ʹ�� �ŷ�����ģʽ�����ж�

       if((Servo_Mode == 0) || (Servo_Mode == 5))  // ֹͣ ��  ����ģʽ
       {
			Cmd_Angle_y = 0;  //FW
			Cmd_Angle_z = 0;  //FY
			ProgStep = 1;
       }
       else if(Servo_Mode == 1)  //  ָ��ģʽ  ����ָ����Դ
       {
          	Cmd_Angle_y = 0;  //FW
           Cmd_Angle_z = 0;  //FY
           ProgStep = 1;
       }
       else if(Servo_Mode == 2)  //  ��ɨģʽ
       {
          	Cmd_Angle_y = 0;  //FW
           Cmd_Angle_z = 0;  //FY
           ProgStep = 1;
       }
       else if(Servo_Mode == 3)  //  ��ɨģʽ
       {
           Cmd_Angle_y = 0;  //FW
           Cmd_Angle_z = 0;  //FY
           ProgStep = 1;
       }
       else if(Servo_Mode == 4)  //  ����ģʽ
       {
           Cmd_Angle_y = 0;  //FW
           Cmd_Angle_z = 0;  //FY
           Cmd_Rate_y = 0; //FW
           Cmd_Rate_z = 0; //FY
           ProgStep = 0;
       }

       //�ڴ˴�����Ƕȿ�����  ʹ�� �ŷ�����ģʽ�����ж�


   	Epk_y = Cmd_Angle_y - Angle_Fw_Encoder;  //FW
   	Epk_z = Cmd_Angle_z - Angle_Fy_Revover;  //FY

   	Evk_y = Cmd_Rate_y - Rate_Fw_Encoder;
   	Evk_z = Cmd_Rate_z - Rate_Fy_Revover;

   	if(ProgStep == 1)  // λ�û�·
   	{
   		Posi_PID();
   	}
   	else if (ProgStep == 0)  //�ٶȻ�·
   	{
   		Velo_PID();
   	}

   	Duty();

}

void Frame_Settle(void)
{

	  extern float Deviationy;       // y����ʧ��������
	  extern float Deviationz;       // z����ʧ��������
	  extern float Cmd_Vel_Azi_From08;  //�Կصķ�λ�ٶ�ָ��
	  extern float Cmd_Vel_Pit_From08;  //�Կصĸ����ٶ�ָ��

	  extern float Cmd_Poi_Azi_Up;          //��λɨ������
	  extern float Cmd_Poi_Azi_Bottom;      //��λɨ������
	  extern float Cmd_Poi_Pit_Up;          //����ɨ������
	  extern float Cmd_Poi_Pit_Bottom;      //����ɨ������


	  extern int FramError;          // Frame Error Signals
//	  extern int RS422Length;
	  extern int  ProgStep;    //�ٶȻ�·
	  extern int Work_Mode;
	  extern int Cmd_Query;
	  extern int Servo_Mode;         //�ŷ�����ģʽ
	  extern int ScibLoopNum;        // Scib Interrupt Number   Cancelled after Debug
	  extern int Imagedata[256];       // 6�ֽ�ͼ��ԭʼ���ݣ����Խ������Ϊ�ڲ�����
	  extern int FrameStart;

	  extern int Rd_Mode_Select;
	  extern int Set_Mode;            //�豸���跽ʽ  00�̶�ʽ   01�ƶ�ʽ
	//
//	  int CheckSum = 0;
	  short fram_len;
	  float Angle_Scale = 0.01;
	  float ImuRate_Scale = 0.01;
	  float ImuAngle_Scale = 0.01;

//	  int i = 0;

	  //0: ֡ͷ  0xff  1�� ֡��-1   2��֡ID 0  3��ԴID  4��Ŀ�� ID  5�� ֡����  6:֡ͷУ���
	                                     //ԴID 0 1 2 3
	  //7 ������· 0x00 Ĭ��Ԥ��  0x01 �ٶȻ�·  0x02
	  //8 ɨ���ٶ� 0x00 Ĭ������ 0deg/s  0x01 5deg/s 0x02 10deg/s 0x03 15deg/s 0x04 20deg/s 0x05 25deg/s 0x06 30deg/s 0x07 35deg/s
	  //9 10 �����Ƕ�ָ��  �ȵͺ��    z ����
	  //11 12 ��λ�Ƕ�ָ�� �ȵͺ��   y ��λ
	  //13 14 15 ����
	/**********************
	 * ��������
	 * by shaobr
	 * 2019-4-18
	 **************************/
	  memcpy(Imagedata,rev_data.rev_buff,rev_data.rev_p);
	  fram_len = rev_data.rev_p;
	  Init_revdata(&rev_data);

	 if(Imagedata[4]==0x00)    //Դ��FPGA
	 {
		 if(fram_len <= 13)    //����֡������֡�Ϊ3��ָ����Ϊ1����û��ָ���Ϊ2�ֽڣ��ʿ���֡����<=7(֡ͷ)+6�������򣩣�
		 {
			 int i=0;
			 for(i=0;i<(fram_len-7)/2;i++)   //(fram_len-7)/2   ��ʾ����ָ��ĸ�������С1�����3
			 {
				 switch (Imagedata[7+2*i])
				 {
					 case 0x00:					//�豸���跽ʽ
					 {
						 switch (Imagedata[7+2*i+1]&0x01)
						 {
//							 case 0x00:	Set_Mode = 0x00;break;//�̶�ʽ��ȱʡֵ
							 case 0x01:	Set_Mode = 0x01; break;	//�ƶ�ʽ
							 default:	Set_Mode = 0x00;break;
						 }
						 break;
					 }
					 case 0x01:					//����ģʽ
					 {
						 Work_Mode = Imagedata[7+2*i+1]&0x0F;
						 switch (Work_Mode)
						 {
//						     case 0x00:		//���� ȱʡֵ
//						 	 case 0x01:			//�Ե�����
//						 	 case 0x03:			//�Կ�����
//						 	 case 0x05:			//�Ժ�����
//						 	 case 0x07:			//����ģʽ
//						 		 ProgStep = 1;  break;// λ�û�·
						 	 case 0x02:				//�Եظ���
						 	 case 0x04:				//�Կո���
						 	 case 0x06:				//�Ժ�����
						 		ProgStep = 0;  break; // �ٶȻ�·
						 	 default: ProgStep = 1;  break;// λ�û�·
						 }
						 break;
					 }
					 case 0x02:			//���Կ��ƿ���
					 {
						 switch (Imagedata[7+2*i+1]&0x01)
						 {
							 case 0x00: Rd_Mode_Select = 0;break;  //�״����״̬��ȡ���Բ���
							 case 0x01:	Rd_Mode_Select = 1; break;	//�״�����������ȡ�̶�����   ȱʡֵ
							 default:	Rd_Mode_Select = 1;break;
						 }
						 break;
					 }
				 }
			 }
		 }
		 else if(fram_len > 13 && Servo_Mode == 4)	//����֡�����ڸ���ģʽ����Ч�����ո���Ŀ�����Ϣ��������£�������ĳ��ȴ���6byte��
		 {
			 float delta_dis = 0.0; //У׼����ֵ��Ĭ��Ϊ0
			 int n=0;
			  n++;   // ����������ĵ�һ���ֽ�0x00��
			 /*���´���Ϊ����FPGA���͵ĸ���Ŀ����Ϣ*/
			 uint16_t Target_Amp =(uint16_t)(Imagedata[7+n]<<8 | Imagedata[7+n+1]);//Ŀ����� LSB��0.01db����ת��Ϊdb
			 n=n+2;
			 float Target_Dis = ((float)((uint16_t)(Imagedata[7+n]<<8 | Imagedata[7+n+1])))*0.5 + delta_dis;//Ŀ����� delta_disΪУ׼����ֵ��Ĭ��Ϊ0
			 n=n+2;
			 float Target_Vel = ((float)((int16_t)(Imagedata[7+n]<<8 | Imagedata[7+n+1])))*0.1;//Ŀ���ٶȣ�����ΪĿ�꿿���״��֮Զ��
			 n=n+2;
			 float Target_Azi = (float)((int16_t)(Imagedata[7+n]<<8 | Imagedata[7+n+1]))*0.01;//Ŀ�귽λ��
			 n=n+2;
			 float Target_Pit = (float)((int16_t)(Imagedata[7+n]<<8 | Imagedata[7+n+1]))*0.01;//Ŀ�긩����
		 }
	 }
	 else if(Imagedata[4]==0x08) //Դ���Կ� ID=0x08,������� 2019-04-24
	 {
		 int i = 0,k=0;
		 for(;i<fram_len-7;)
		 {
			 switch (Imagedata[7+i])
			 {
			 	 case 0x17:			//�ŷ�����ģʽ
				 {
				 	 i=i+1;	 //ָ��ָ���ĵ�һ���ֽ�
					 switch(Imagedata[7+i]&0x07)
					 {
					 	 case 0x00:	Servo_Mode = 0;		break;			//ֹͣģʽ��ȱʡֵ,λ�û�·��ֹͣ����λ
					 	 case 0x01:	Servo_Mode = 1;		break;			//ָ��ģʽ,λ�û�·��ָ��̶�λ��
					 	 case 0x02:	Servo_Mode = 2;		break;			//����ɨ��ģʽ��λ�û�·������ɨ��
					 	 case 0x03:	Servo_Mode = 3;		break;			//����ɨ��ģʽ��λ�û�·������ɨ��
					 	 case 0x04:	Servo_Mode = 4;		break;			//����ģʽ���ٶȻ�·
					 	 case 0x05:	Servo_Mode = 5;		break;			//����ģʽ �ݵ�ͬ��ֹͣģʽ ����λ
					 	 default:	Servo_Mode = 0;		break;
					 }
				 	 i=i+1;		//ָ����һ��ָ����ƴ���
					 break;
				 }
				 case 0x18:			//��λת̨ת��
				 {
				 	 i=i+1;	 //ָ��ָ���ĵ�һ���ֽ�
					 Cmd_Vel_Azi_From08 =  ((float)((Imagedata[7+i]<<8)|(Imagedata[7+i+1]&0x00FF))) * 0.1;
					 i = i + 2 ;
					 break;
				 }
				 case 0x19:			//����ת̨ת��
				 {
				 	 i=i+1;	 //ָ��ָ���ĵ�һ���ֽ�
					 Cmd_Vel_Pit_From08 = ((float)((Imagedata[7+i]<<8)|(Imagedata[7+i+1]&0x00FF))) * 0.1;
					 i = i+2;
					 break;
				 }
				 case 0x1A:			//��λת̨ɨ������
				 {
				 	 i=i+1;	 //ָ��ָ���ĵ�һ���ֽ�
					 Cmd_Poi_Azi_Bottom =  ((float)((Imagedata[7+i]<<8)|(Imagedata[7+i+1]&0x00FF))) * 0.01;
					 i = i +2;
					 break;
				 }
				 case 0x1B:			//��λת̨ɨ������
				 {
				 	 i=i+1;	 //ָ��ָ���ĵ�һ���ֽ�
					 Cmd_Poi_Azi_Up =  ((float)((Imagedata[7+i]<<8)|(Imagedata[7+i+1]&0x00FF))) * 0.01;
					 i = i + 2;
					 break;
				 }
				 case 0x1C:			//����ת̨ɨ������
				 {
				 	 i=i+1;	 //ָ��ָ���ĵ�һ���ֽ�
					 Cmd_Poi_Pit_Bottom =  ((float)((Imagedata[7+i]<<8)|(Imagedata[7+i+1]&0x00FF))) * 0.01;
					 i = i+2;
					 break;
				 }
				 case 0x1D:			//����ת̨ɨ������
				 {
				 	 i=i+1;	 //ָ��ָ���ĵ�һ���ֽ�
					 Cmd_Poi_Pit_Up =  ((float)((Imagedata[7+i]<<8)|(Imagedata[7+i+1]&0x00FF))) * 0.01;
					 i = i+2;
					 break;
				 }
				 case 0x1E:	//00 bit��ѯ  01�豸���跽ʽ  02 �ŷ�����ģʽ  03 ��λת̨����  04 ��λת̨����  05 ����ת̨���� 06 ����ת̨���� 07 ����ģʽ 08 ���Կ���
				 {
				 	 i=i+1;	 //ָ��ָ���ĵ�һ���ֽ�
					 switch(Imagedata[7+i]&0x0F)
					 {
					 	 case 0x00:{sbuff_tp[k++]=0x00;	sbuff_tp[k++]=0x01;break;	}		//00bit��ѯ
					 	 case 0x01:	{sbuff_tp[k++]=0x01;sbuff_tp[k++]=Set_Mode;break;}			//�豸���跽ʽ
					 	 case 0x02:	{sbuff_tp[k++]=0x02;sbuff_tp[k++]=Servo_Mode;break;	}		//�����ŷ�����ģʽ
					 	 case 0x03:																//���ط�λת̨ɨ������
					 	 {
					 		 sbuff_tp[k++]=0x03;
					 		 int16_t temp0 =(int16_t)(Cmd_Poi_Azi_Bottom*100);
					 		 sbuff_tp[k++]=(temp0>>8)&0x00ff;
					 		 sbuff_tp[k++]=temp0&0x00FF;
					 		 break;
					 	 }
					 	 case 0x04:	//���ط�λת̨ɨ������
					 	 {
					 		 sbuff_tp[k++]=0x04;
					 		 int16_t temp1 =(int16_t)(Cmd_Poi_Azi_Up*100);
					 		 sbuff_tp[k++]=(temp1>>8)&0x00ff;
					 		 sbuff_tp[k++]=temp1&0x00FF;
					 		 break;
					 	 }
					 	 case 0x05:	//���ظ���ת̨ɨ������
					 	 {
					 		 sbuff_tp[k++]=0x05;
							 int16_t temp2 =(int16_t)(Cmd_Poi_Pit_Bottom*100);
							 sbuff_tp[k++]=(temp2>>8)&0x00ff;
							 sbuff_tp[k++]=temp2&0x00FF;
							 break;
					 	 }
					 	 case 0x06:		//���ظ���ת̨ɨ������
					 	 {
					 		 sbuff_tp[k++]=0x06;
							 int16_t temp3 =(int16_t)(Cmd_Poi_Pit_Up*100);
							 sbuff_tp[k++]=(temp3>>8)&0x00ff;
							 sbuff_tp[k++]=temp3&0x00FF;
							 break;
					 	 }
					 	 case 0x07://���ع���ģʽ
					 	 {
					 		 sbuff_tp[k++]=0x07;
					 	     sbuff_tp[k++]=Work_Mode;
					 	     break;
					 	 }
					 	 case 0x08: //���ص��Կ���
					 	 {
					 		 sbuff_tp[k++]=0x08;
					 		 sbuff_tp[k++]=Rd_Mode_Select;
					 		 break;
					 	 }
					 	 default:break;
					 }
//					 Cmd_Query = Imagedata[7+i]&0x00000000F;
					 i = i+1;
					 if(i==(fram_len-7))
					 {
						 int len =Fram_Make(Send_buff,sbuff_tp,0x08,k); //��������֡
						 UART_SEND(Send_buff,len);
					 }
					 break;
				 }
			 }
		 }
	 }
	 else if(Imagedata[4]==0x04 && Servo_Mode == 4) //Դ��DSP0 ID=0x04,���ڸ���ģʽ����Ч
	 {
		 int i = 1;//����������Ϣ���ݱ���
		 float Target_Azi_Bias = (float)((short)(Imagedata[7+i]<<8+Imagedata[7+i+1]))*0.01;//Ŀ�귽λƫ���
		 i=i+2;
		 float Target_Pit_Bias = (float)((short)(Imagedata[7+i]<<8+Imagedata[7+i+1]))*0.01;//Ŀ�긩��ƫ���
	 }
	 else if(Imagedata[4]==0x07 && Set_Mode == 0x01)	//Դ��IMU ID=0x07,�����ƶ����跽ʽ����Ч
	 {
		int i = 0;
		if(Imagedata[7+i]==0x00)   //����ת�����ٶ���Ϣ
		{
			i++;
			float Course_Omega = (float)((short)(Imagedata[7+i]<<8+Imagedata[7+i+1]))*0.01;//����ת�����ٶ�
			i=i+2;
			float Pit_Omega = (float)((short)(Imagedata[7+i]<<8+Imagedata[7+i+1]))*0.01;//����ת�����ٶ�
			i=i+2;
			float Roll_Omega = (float)((short)(Imagedata[7+i]<<8+Imagedata[7+i+1]))*0.01;//����ת�����ٶ�

/*			  IMU_Rate_x = ( (Imagedata[9+FrameStart] & 0x00FF) + ((Imagedata[10+FrameStart] & 0x00FF) << 8) ) * ImuRate_Scale;	//	imu ���ٶ� x����
			  IMU_Rate_y = ( (Imagedata[9+FrameStart] & 0x00FF) + ((Imagedata[10+FrameStart] & 0x00FF) << 8) ) * ImuRate_Scale;	//	imu ���ٶ� y����
			  IMU_Rate_z = ( (Imagedata[9+FrameStart] & 0x00FF) + ((Imagedata[10+FrameStart] & 0x00FF) << 8) ) * ImuRate_Scale;	//	imu ���ٶ� z����

			  IMU_Angle_x = ( (Imagedata[9+FrameStart] & 0x00FF) + ((Imagedata[10+FrameStart] & 0x00FF) << 8) ) * ImuAngle_Scale;	//	imu �Ƕ� x����
			  IMU_Angle_y = ( (Imagedata[9+FrameStart] & 0x00FF) + ((Imagedata[10+FrameStart] & 0x00FF) << 8) ) * ImuAngle_Scale;	//	imu �Ƕ� y����
			  IMU_Angle_z = ( (Imagedata[9+FrameStart] & 0x00FF) + ((Imagedata[10+FrameStart] & 0x00FF) << 8) ) * ImuAngle_Scale;	//	imu �Ƕ� z����
*/
		}
		else
		{
			  IMU_Rate_x = 0;	//	imu ���ٶ� x����
			  IMU_Rate_y = 0;	//	imu ���ٶ� y����
			  IMU_Rate_z = 0;	//	imu ���ٶ� z����

			  IMU_Angle_x = 0;	//	imu �Ƕ� x����
			  IMU_Angle_y = 0;	//	imu �Ƕ� y����
			  IMU_Angle_z = 0;	//	imu �Ƕ� z����
		}
	 }

/*	  FrameStart = 0;

	  for(i=0;i<5;i++)
	  {
		  if(Imagedata[i] == 0xEB)
		  {FrameStart = i;}
			  break;
	  }

	  CheckSum = 0;

	  for(i=FrameStart;i<(6+FrameStart);i++)
	  {
		  CheckSum = CheckSum + Imagedata[i];
	  }

	  CheckSum = CheckSum & 0x00FF;

	  if((Imagedata[0+FrameStart] == 0xEB) && (Imagedata[1+FrameStart] == 0x90)
		&& (Imagedata[4+FrameStart] == 0x00) && (Imagedata[5+FrameStart] == 0x03)
		&& (Imagedata[6+FrameStart] == CheckSum))   //BF:Begin of Frame  EF:End of Frame
	  // Դ��FPGA  ID ��00
	  {
		   FramError  = 0;
		   if(Imagedata[7+FrameStart] == 0x01 )     //����ָ��1  ����ģʽ
		   {
			 Work_Mode = Imagedata[8+FrameStart]&0x0F;
			 if(Work_Mode == 0x00)	//����ģʽ   ������  λ�û�·
			 {
				 ProgStep = 1;  // λ�û�·
			 }
			 else if(Work_Mode == 0x01)  //�Ե�����ģʽ
			 {
				 ProgStep = 1;  // λ�û�·
			 }
			 else if(Work_Mode == 0x02)  //�Եظ���ģʽ   ����ģʽ   �ٶȻ�·
			 {
				 ProgStep = 0;  // �ٶȻ�·
			 }
			 else if(Work_Mode == 0x03)  //�Կ�����ģʽ
			 {
				 ProgStep = 1;  // λ�û�·
			 }
			 else if(Work_Mode == 0x04)  //�Կո���ģʽ
			 {
				 ProgStep = 0;  //�ٶȻ�·   ����ģʽ   �ٶȻ�·
			 }
			 else if(Work_Mode == 0x05)  //�Ժ�����ģʽ
			 {
				 ProgStep = 1;  // λ�û�·
			 }
			 else if(Work_Mode == 0x06)  //�Ժ�����ģʽ
			 {
				 ProgStep = 0;  //�ٶȻ�·   ����ģʽ   �ٶȻ�·
			 }
			 else if(Work_Mode == 0x07)  //����ģʽ
			 {
				 ProgStep = 1;  // λ�û�·
			 }
		   }
		   else if(Imagedata[9+FrameStart] == 0x02) //����ָ��2  ����ģʽ/����ģʽ����
		   {

			 if(Imagedata[10+FrameStart] == 0x00)  //�״����״̬
			 {
				 Rd_Mode_Select = 0;
			 }
			 else if(Imagedata[10+FrameStart] == 0x01)  //�״﹤��״̬  Ĭ��ȱʡֵ
			 {
				 Rd_Mode_Select = 1;
			 }
			 //Cmd_Angle_z = ( (Imagedata[9] & 0x00FF) + ((Imagedata[10] & 0x00FF) << 8) ) * Angle_Scale;
			 //Cmd_Angle_y = ( (Imagedata[11] & 0x00FF) +((Imagedata[12] & 0x00FF) << 8) ) * Angle_Scale;
			 //ProgStep = 1; //λ�û�·
		   }
		   else if(Imagedata[11+FrameStart]==0x00)    //�豸���跽ʽ
		   {
			   if(Imagedata[12+FrameStart]==0x00)  //�̶�ʽ��Ĭ��ȱ��ָ
			   {
				   Set_Mode = 0x00;
			   }
			   else if(Imagedata[12+FrameStart]==0x01)//�ƶ�ʽ
			   {
				   Set_Mode = 0x01;
			   }
		   }
	  }
	  else
	  {
		  FramError  = 1;
		  Deviationy = 0;
		  Deviationz = 0;
	  }*/


/*	  if((Imagedata[0+FrameStart] == 0x90) && (Imagedata[1+FrameStart] == 0xEB)
			  && (Imagedata[4+FrameStart] == 0x01) && (Imagedata[5+FrameStart] == 0x03)
			  && (Imagedata[6+FrameStart] == CheckSum))   //BF:Begin of Frame  EF:End of Frame
	  // Դ�ǵ�Դ    ����ģʽ    ID��01
	  {
		  FramError  = 0;
		  //
		  if(Imagedata[7+FrameStart] == 0x00) // ���Ե�Դ��ָ��
		  {
			  if(Imagedata[8] == 0x00)
			  {
				  Set_Mode = 0;
			  }
			  else if(Imagedata[8+FrameStart] == 0x01)
			  {
				  Set_Mode = 1;
			  }
	//	   Deviationz = ( (Imagedata[9] & 0x00FF) + ((Imagedata[10] & 0x00FF) << 8) ) * Angle_Scale;
	//	   Deviationy = ( (Imagedata[11] & 0x00FF) +((Imagedata[12] & 0x00FF) << 8) ) * Angle_Scale;
	//	   Cmd_Rate_y = Deviationy * 10;  //��ƫ��ת��Ϊ�ٶ�ָ��
	//	   Cmd_Rate_z = Deviationz * 10;  //��ƫ��ת��Ϊ�ٶ�ָ��
	//	   ProgStep = 0; //�ٶȻ�·
		  }
		  else
		  {
	//	   Deviationz = 0;
	//	   Deviationy = 0;
	//	   Cmd_Rate_y = Deviationy * 10;  //��ƫ��ת��Ϊ�ٶ�ָ��
	//	   Cmd_Rate_z = Deviationz * 10;  //��ƫ��ת��Ϊ�ٶ�ָ��
	//	   ProgStep = 0; //�ٶȻ�·
		  }
	  }
	  else
	  {
		  FramError  = 1;
		  Deviationy = 0;
		  Deviationz = 0;
	  }*/

	  // Դ���Կ�    ����ģʽ    ID��08
/*	  if((Imagedata[0+FrameStart] == 0x90) && (Imagedata[1+FrameStart] == 0xEB)
			  && (Imagedata[4+FrameStart] == 0x08) && (Imagedata[5+FrameStart] == 0x03)
			  && (Imagedata[6+FrameStart] == CheckSum))   //BF:Begin of Frame  EF:End of Frame
	  {
		  if(Imagedata[7+FrameStart] == 0x17)
		  {
			  if(Imagedata[8+FrameStart] == 0x00)  //ֹͣģʽ   λ�û�·  ֹͣ����λ
			  {
				 Servo_Mode = 0;
			  }
			  else if(Imagedata[8+FrameStart] == 0x01)  //ָ��ģʽ   λ�û�·  ָ��̶�λ��
			  {
				  Servo_Mode = 1;
			  }
			  else if(Imagedata[8+FrameStart] == 0x02)  //��ɨģʽ   λ�û�·  ����ɨ��
			  {
				  Servo_Mode = 2;
			  }
			  else if(Imagedata[8+FrameStart] == 0x03)  //��ɨģʽ   λ�û�·  ����ɨ��
			  {
				  Servo_Mode = 3;
			  }
			  else if(Imagedata[8+FrameStart] == 0x04)  //����ģʽ   �ٶȻ�·
			  {
				  Servo_Mode = 4;
			  }
			  else if(Imagedata[8+FrameStart] == 0x05)  //����ģʽ  �ݵ�ͬ��ֹͣģʽ ����λ
			  {
				  Servo_Mode = 5;
			  }
		  }

		  else if(Imagedata[7+FrameStart] == 0x18)
		  {
			  Cmd_Vel_Azi_From08 =  (float)(((Uint16)(Imagedata[9+FrameStart]))<<8 + (Uint16)(Imagedata[8+FrameStart])) * 0.1;
		  }
		  else if(Imagedata[7+FrameStart] == 0x19)
		  {
			  Cmd_Vel_Pit_From08 =  (float)(((Uint16)(Imagedata[9+FrameStart]))<<8 + (Uint16)(Imagedata[8+FrameStart])) * 0.1;
		  }
		  else if(Imagedata[7+FrameStart] == 0x1A)
		  {
			  Cmd_Poi_Azi_Bottom =  (float)(((Imagedata[9+FrameStart]))<<8 + (Imagedata[8+FrameStart])) * 0.01;
		  }
		  else if(Imagedata[7+FrameStart] == 0x1B)
		  {
			  Cmd_Poi_Azi_Up =  (float)(((Imagedata[9+FrameStart]))<<8 + (Imagedata[8+FrameStart])) * 0.01;
		  }
		  else if(Imagedata[7+FrameStart] == 0x1C)
		  {
			  Cmd_Poi_Pit_Bottom =  (float)(((Imagedata[9+FrameStart]))<<8 + (Imagedata[8+FrameStart])) * 0.01;
		  }
		  else if(Imagedata[7+FrameStart] == 0x1D)
		  {
			  Cmd_Poi_Pit_Up =  (float)(((Imagedata[9+FrameStart]))<<8 + (Imagedata[8+FrameStart])) * 0.01;
		  }
		  else if(Imagedata[7+FrameStart] == 0x1E)  //00 bit��ѯ  01�豸���跽ʽ  02 �ŷ�����ģʽ  03 ��λת̨����  04 ��λת̨����  05 ����ת̨���� 06 ����ת̨���� 07 ����ģʽ 08 ���Կ���
		  {
			  if(Imagedata[8+FrameStart] == 0x00)
			  {
				  Cmd_Query = 0;
			  }
			  else if(Imagedata[8+FrameStart] == 0x01)
			  {
				  Cmd_Query = 1;
			  }
			  else if(Imagedata[8+FrameStart] == 0x02)
			  {
				  Cmd_Query = 2;
			  }
			  else if(Imagedata[8+FrameStart] == 0x03)
			  {
				  Cmd_Query = 3;
			  }
			  else if(Imagedata[8+FrameStart] == 0x04)
			  {
				  Cmd_Query = 4;
			  }
			  else if(Imagedata[8+FrameStart] == 0x05)
			  {
				  Cmd_Query = 5;
			  }
			  else if(Imagedata[8+FrameStart] == 0x06)
			  {
				  Cmd_Query = 6;
			  }
			  else if(Imagedata[8+FrameStart] == 0x07)
			  {
				  Cmd_Query = 7;
			  }
			  else if(Imagedata[8+FrameStart] == 0x08)
			  {
				  Cmd_Query = 8;
			  }
		  }

	  }*/


/*	  if((Imagedata[0+FrameStart] == 0x90) && (Imagedata[1+FrameStart] == 0xEB)
			  && (Imagedata[4+FrameStart] == 0x07) && (Imagedata[5+FrameStart] == 0x03)
			  && (Imagedata[6+FrameStart] == CheckSum))   //BF:Begin of Frame  EF:End of Frame
	  // Դ�� IMU  ��ȡIMU������  ID Ϊ07
	  {
		  FramError  = 0;
		  IMU_Rate_x = ( (Imagedata[9+FrameStart] & 0x00FF) + ((Imagedata[10+FrameStart] & 0x00FF) << 8) ) * ImuRate_Scale;	//	imu ���ٶ� x����
		  IMU_Rate_y = ( (Imagedata[9+FrameStart] & 0x00FF) + ((Imagedata[10+FrameStart] & 0x00FF) << 8) ) * ImuRate_Scale;	//	imu ���ٶ� y����
		  IMU_Rate_z = ( (Imagedata[9+FrameStart] & 0x00FF) + ((Imagedata[10+FrameStart] & 0x00FF) << 8) ) * ImuRate_Scale;	//	imu ���ٶ� z����

		  IMU_Angle_x = ( (Imagedata[9+FrameStart] & 0x00FF) + ((Imagedata[10+FrameStart] & 0x00FF) << 8) ) * ImuAngle_Scale;	//	imu �Ƕ� x����
		  IMU_Angle_y = ( (Imagedata[9+FrameStart] & 0x00FF) + ((Imagedata[10+FrameStart] & 0x00FF) << 8) ) * ImuAngle_Scale;	//	imu �Ƕ� y����
		  IMU_Angle_z = ( (Imagedata[9+FrameStart] & 0x00FF) + ((Imagedata[10+FrameStart] & 0x00FF) << 8) ) * ImuAngle_Scale;	//	imu �Ƕ� z����
	  }
	  else
	  {
		  FramError  = 1;
		  IMU_Rate_x = 0;	//	imu ���ٶ� x����
		  IMU_Rate_y = 0;	//	imu ���ٶ� y����
		  IMU_Rate_z = 0;	//	imu ���ٶ� z����

		  IMU_Angle_x = 0;	//	imu �Ƕ� x����
		  IMU_Angle_y = 0;	//	imu �Ƕ� y����
		  IMU_Angle_z = 0;	//	imu �Ƕ� z����
	  }*/

}


//===========================================================================
// End of file.
//===========================================================================

