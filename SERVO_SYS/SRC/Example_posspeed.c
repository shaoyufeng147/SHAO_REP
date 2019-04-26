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
//
//
// For 100 MHz Operation:
// ----------------------
//
// The same calculations as above are performed, but with 100 MHz
// instead of 150MHz when calculating SpeedRpm_pr.
// The value for freqScaler_pr becomes: [32*(100MHz/128)*60s/min]/(4000*6000rpm) = 63
// More detailed calculation results can be found in the Example_freqcal.xls
// spreadsheet included in the example folder.
//
//
//
// This file contains source for the posspeed module
//
//###########################################################################
// Original Author: SD
//
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

#include "DSP2833x_Project.h"     // Device Headerfile and Examples Include File
#include "Example_posspeed.h"   // Example specific Include file
#include "DSP2833x_Device.h"



