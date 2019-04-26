// TI File $Revision: /main/5 $
// Checkin $Date: August 16, 2007   11:06:26 $
//###########################################################################
//
// FILE:   DSP2833x_Xintf.c
//
// TITLE:   DSP2833x Device External Interface Init & Support Functions.
//
// DESCRIPTION:
//
//          Example initialization function for the external interface (XINTF).
//          This example configures the XINTF to its default state.  For an
//          example of how this function being used refer to the
//          examples/run_from_xintf project.
//
//###########################################################################
// $TI Release: DSP2833x Header Files V1.01 $
// $Release Date: September 26, 2007 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

//---------------------------------------------------------------------------
// InitXINTF:
//---------------------------------------------------------------------------
// This function initializes the External Interface the default reset state.
//
// Do not modify the timings of the XINTF while running from the XINTF.  Doing
// so can yield unpredictable results


void InitXintf(void)
{
    // This shows how to write to the XINTF registers.  The
    // values used here are the default state after reset.
    // Different hardware will require a different configuration.

    // For an example of an XINTF configuration used with the
    // F28335 eZdsp, refer to the examples/run_from_xintf project.

    // Any changes to XINTF timing should only be made by code
    // running outside of the XINTF.

    // All Zones---------------------------------
    // Timing for all zones based on XTIMCLK = 1/2 SYSCLKOUT
    EALLOW;
    XintfRegs.XINTCNF2.bit.XTIMCLK = 1;
    // No write buffering
    XintfRegs.XINTCNF2.bit.WRBUFF = 0;
    // XCLKOUT is enabled
    XintfRegs.XINTCNF2.bit.CLKOFF = 0;
    // XCLKOUT = XTIMCLK/2
    XintfRegs.XINTCNF2.bit.CLKMODE = 1;


    // Zone 0------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead must always be 1 or greater
    // Zone write timing
    XintfRegs.XTIMING0.bit.XWRLEAD = 3;
    XintfRegs.XTIMING0.bit.XWRACTIVE = 7;
    XintfRegs.XTIMING0.bit.XWRTRAIL = 3;
    // Zone read timing
    XintfRegs.XTIMING0.bit.XRDLEAD = 3;
    XintfRegs.XTIMING0.bit.XRDACTIVE = 7;
    XintfRegs.XTIMING0.bit.XRDTRAIL = 3;

    // double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING0.bit.X2TIMING = 1;

    // Zone will sample XREADY signal
    XintfRegs.XTIMING0.bit.USEREADY = 1;
    XintfRegs.XTIMING0.bit.READYMODE = 1;  // sample asynchronous

    // Size must be either:
    // 0,1 = x32 or
    // 1,1 = x16 other values are reserved
    XintfRegs.XTIMING0.bit.XSIZE = 3;

    // Zone 6------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead must always be 1 or greater
    // Zone write timing
    XintfRegs.XTIMING6.bit.XWRLEAD = 3;
    XintfRegs.XTIMING6.bit.XWRACTIVE = 7;
    XintfRegs.XTIMING6.bit.XWRTRAIL = 3;
    // Zone read timing
    XintfRegs.XTIMING6.bit.XRDLEAD = 3;
    XintfRegs.XTIMING6.bit.XRDACTIVE = 7;
    XintfRegs.XTIMING6.bit.XRDTRAIL = 3;

    // double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING6.bit.X2TIMING = 1;

    // Zone will sample XREADY signal
    XintfRegs.XTIMING6.bit.USEREADY = 1;
    XintfRegs.XTIMING6.bit.READYMODE = 1;  // sample asynchronous

    // Size must be either:
    // 0,1 = x32 or
    // 1,1 = x16 other values are reserved
    XintfRegs.XTIMING6.bit.XSIZE = 3;


    // Zone 7------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead must always be 1 or greater
    // Zone write timing
    XintfRegs.XTIMING7.bit.XWRLEAD = 3;
    XintfRegs.XTIMING7.bit.XWRACTIVE = 7;
    XintfRegs.XTIMING7.bit.XWRTRAIL = 3;
    // Zone read timing
    XintfRegs.XTIMING7.bit.XRDLEAD = 3;
    XintfRegs.XTIMING7.bit.XRDACTIVE = 7;
    XintfRegs.XTIMING7.bit.XRDTRAIL = 3;

    // double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING7.bit.X2TIMING = 1;

    // Zone will sample XREADY signal
    XintfRegs.XTIMING7.bit.USEREADY = 1;
    XintfRegs.XTIMING7.bit.READYMODE = 1;  // sample asynchronous

    // Size must be either:
    // 0,1 = x32 or
    // 1,1 = x16 other values are reserved
    XintfRegs.XTIMING7.bit.XSIZE = 3;

    // Bank switching
    // Assume Zone 7 is slow, so add additional BCYC cycles
    // when ever switching from Zone 7 to another Zone.
    // This will help avoid bus contention.
    XintfRegs.XBANK.bit.BANK = 7;
    XintfRegs.XBANK.bit.BCYC = 7;
    EDIS;
   //Force a pipeline flush to ensure that the write to
   //the last register configured occurs before returning.

   InitXintf16Gpio();
// InitXintf32Gpio();

   asm(" RPT #7 || NOP");

}

void InitXintf32Gpio()
{
     EALLOW;
     GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 3;  // XD31
     GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 3;  // XD30

     GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 1;  // QEP1A
     GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 1;  // QEP1B
     GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 1;  // QEP1S

     GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 3;  // XD26
     GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 3;  // XD25
     GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 3;  // XD24
     GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 3;  // XD23
     GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 3;  // XD22
     GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3;  // XD21
     GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 3;  // XD20
     GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 3;  // XD19
     GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 3;  // XD18
     GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 3;  // XD17
     GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 3;  // XD16

     GpioCtrlRegs.GPBQSEL2.bit.GPIO48 = 3;  // XD31 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO49 = 3;  // XD30 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 3;  // XD29 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO51 = 3;  // XD28 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO52 = 3;  // XD27 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO53 = 3;  // XD26 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 3;  // XD25 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 3;  // XD24 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO56 = 3;  // XD23 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO57 = 3;  // XD22 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3;  // XD21 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 3;  // XD20 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 3;  // XD19 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 3;  // XD18 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // XD17 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3;  // XD16 asynchronous input


     InitXintf16Gpio();
}

void InitXintf16Gpio()
{
     EALLOW;

     GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1; //1A DC MOTOR
     GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1; //1B DC MOTOR
     GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1; //2A DC MOTOR
     GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1; //2B DC MOTOR
     GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0; //3A STEP MOTOR
     GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0; //3B STEP MOTOR
     GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0; //4A DC MOTOR
     GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0; //4B DC MOTOR
     GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0; //5A DC MOTOR
     GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0; //5B DC MOTOR
     GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1; //6A DC MOTOR
     GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1; //6B DC MOTOR

     GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;  //BUSY
     GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;  //CONTA
     GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;  //CONTB
     GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;  //CONTC

     GpioCtrlRegs.GPADIR.bit.GPIO14  = 0;  //0 input
     GpioCtrlRegs.GPADIR.bit.GPIO15  = 1;  //1 output
     GpioCtrlRegs.GPADIR.bit.GPIO16  = 1;  //1 output
     GpioCtrlRegs.GPADIR.bit.GPIO17  = 1;  //1 output

     GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 0x0E;
     asm(" RPT #50 || NOP");


     GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;	    // Enable pull-up for GPIO18 (CANRXA)
	 GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;	    // Enable pull-up for GPIO19 (CANTXA)
     GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;   // Asynch qual for GPIO18 (CANRXA)
     GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3;	// Configure GPIO18 for CANRXA operation
     GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3;	// Configure GPIO18 for CANRXA operation

     GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pull-up for GPIO20 (CANTXB)
     GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;   // Enable pull-up for GPIO21 (CANRXB)
     GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 3; // Asynch qual for GPIO21 (CANRXB)
     GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 3;  // Configure GPIO20 for CANTXB operation
     GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 3;  // Configure GPIO21 for CANRXB operation

 
     GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up for GPIO28 (SCIRXDA)
     GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;    // Enable pull-up for GPIO29 (SCITXDA)
     GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)
     GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO28 for SCIRXDA operation
     GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 for SCITXDA operation

     GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;    // Enable pull-up for GPIO22 (SCITXDB)
     GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;    // Enable pull-up for GPIO23 (SCIRXDB)
     GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3;  // Asynch input GPIO23 (SCIRXDB)
     GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3;   // Configure GPIO22 for SCITXDB operation
     GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;   // Configure GPIO23 for SCIRXDB operation



     GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 2;   // EQEP2A
     GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 2;   // EQEP2B
     GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 2;   // EQEP2S

	 GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;    // Enable pull-up for GPIO62 (SCIRXDC)
	 GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;	   // Enable pull-up for GPIO63 (SCITXDC)
     GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO62 (SCIRXDC)
	 GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO62 for SCIRXDC operation
	 GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   // Configure GPIO63 for SCITXDC operation

     GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3;  // XD15
     GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3;  // XD14
     GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 3;  // XD13
     GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 3;  // XD12
     GpioCtrlRegs.GPCMUX1.bit.GPIO68 = 3;  // XD11
     GpioCtrlRegs.GPCMUX1.bit.GPIO69 = 3;  // XD10
     GpioCtrlRegs.GPCMUX1.bit.GPIO70 = 3;  // XD19
     GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 3;  // XD8
     GpioCtrlRegs.GPCMUX1.bit.GPIO72 = 3;  // XD7
     GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 3;  // XD6
     GpioCtrlRegs.GPCMUX1.bit.GPIO74 = 3;  // XD5
     GpioCtrlRegs.GPCMUX1.bit.GPIO75 = 3;  // XD4
     GpioCtrlRegs.GPCMUX1.bit.GPIO76 = 3;  // XD3
     GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 3;  // XD2
     GpioCtrlRegs.GPCMUX1.bit.GPIO78 = 3;  // XD1
     GpioCtrlRegs.GPCMUX1.bit.GPIO79 = 3;  // XD0

     GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0;  // A0
     GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0;  // A1
     GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 0;  // A2
     GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 0;  // A3
     GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 0;  // A4

     GpioCtrlRegs.GPBMUX1.bit.GPIO45 = 0;  // A_Decay
     GpioCtrlRegs.GPBMUX1.bit.GPIO46 = 0;  // A_Reset
     GpioCtrlRegs.GPBMUX1.bit.GPIO47 = 0;  // A_Sleep

     GpioCtrlRegs.GPCMUX2.bit.GPIO80 = 0;  // B0
     GpioCtrlRegs.GPCMUX2.bit.GPIO81 = 0;  // B1
     GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 0;  // B2
     GpioCtrlRegs.GPCMUX2.bit.GPIO83 = 0;  // B3
     GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 0;  // B4

     GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 0;  // B_Decay
     GpioCtrlRegs.GPCMUX2.bit.GPIO86 = 0;  // B_Reset
     GpioCtrlRegs.GPCMUX2.bit.GPIO87 = 0;  // B_Sleep

     GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1;  // A0
     GpioCtrlRegs.GPBDIR.bit.GPIO41 = 1;  // A1
     GpioCtrlRegs.GPBDIR.bit.GPIO42 = 1;  // A2
     GpioCtrlRegs.GPBDIR.bit.GPIO43 = 1;  // A3
     GpioCtrlRegs.GPBDIR.bit.GPIO44 = 1;  // A4

     GpioCtrlRegs.GPBDIR.bit.GPIO45 = 1;  // A_Decay
     GpioCtrlRegs.GPBDIR.bit.GPIO46 = 1;  // A_Reset
     GpioCtrlRegs.GPBDIR.bit.GPIO47 = 1;  // A_Sleep

     GpioCtrlRegs.GPCDIR.bit.GPIO80 = 1;  // B0
     GpioCtrlRegs.GPCDIR.bit.GPIO81 = 1;  // B1
     GpioCtrlRegs.GPCDIR.bit.GPIO82 = 1;  // B2
     GpioCtrlRegs.GPCDIR.bit.GPIO83 = 1;  // B3
     GpioCtrlRegs.GPCDIR.bit.GPIO84 = 1;  // B4

     GpioCtrlRegs.GPCDIR.bit.GPIO85 = 1;  // B_Decay
     GpioCtrlRegs.GPCDIR.bit.GPIO86 = 1;  // B_Reset
     GpioCtrlRegs.GPCDIR.bit.GPIO87 = 1;  // B_Sleep


     GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 3;  // XA16
     GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 3;  // XA17
     GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 3;  // XA18
     GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 3;  // XA19

     GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 3;  // XREADY
	 GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 3;  // XRNW
     GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 3;  // XWE0

     GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 3;  // XZCS0
     GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 3;  // XZCS7
     GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 3;  // XZCS6

     EDIS;
}

//===========================================================================
// No more.
//===========================================================================
