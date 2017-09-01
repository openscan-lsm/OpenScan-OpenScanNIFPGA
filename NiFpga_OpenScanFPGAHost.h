/*
 * Generated with the FPGA Interface C API Generator 15.0.0
 * for NI-RIO 15.0.0 or later.
 */

#ifndef __NiFpga_OpenScanFPGAHost_h__
#define __NiFpga_OpenScanFPGAHost_h__

#ifndef NiFpga_Version
   #define NiFpga_Version 1500
#endif

#include "NiFpga.h"

/**
 * The filename of the FPGA bitfile.
 *
 * This is a #define to allow for string literal concatenation. For example:
 *
 *    static const char* const Bitfile = "C:\\" NiFpga_OpenScanFPGAHost_Bitfile;
 */
#define NiFpga_OpenScanFPGAHost_Bitfile "NiFpga_OpenScanFPGAHost.lvbitx"

/**
 * The signature of the FPGA bitfile.
 */
static const char* const NiFpga_OpenScanFPGAHost_Signature = "1B9F10A03F4A31562B1AF5E292E145E9";

typedef enum
{
   NiFpga_OpenScanFPGAHost_IndicatorBool_Averagedimagedisplayed = 0x10032,
   NiFpga_OpenScanFPGAHost_IndicatorBool_FrameGalvosignalwritedone = 0x10086,
   NiFpga_OpenScanFPGAHost_IndicatorBool_Frameacquisitionfinish = 0x1001E,
   NiFpga_OpenScanFPGAHost_IndicatorBool_Frameimagingdataaveraged = 0x1002E,
   NiFpga_OpenScanFPGAHost_IndicatorBool_Framewaveformoutputfinish = 0x1005A,
   NiFpga_OpenScanFPGAHost_IndicatorBool_Imageaveragingdone = 0x10036,
   NiFpga_OpenScanFPGAHost_IndicatorBool_Lineclock = 0x10046,
   NiFpga_OpenScanFPGAHost_IndicatorBool_PixelclockIndicator = 0x1004A,
   NiFpga_OpenScanFPGAHost_IndicatorBool_WriteDRAMdone = 0x1008A,
} NiFpga_OpenScanFPGAHost_IndicatorBool;

typedef enum
{
   NiFpga_OpenScanFPGAHost_IndicatorI32_Display_framesdone = 0x10024,
   NiFpga_OpenScanFPGAHost_IndicatorI32_loopcount = 0x10020,
} NiFpga_OpenScanFPGAHost_IndicatorI32;

typedef enum
{
   NiFpga_OpenScanFPGAHost_IndicatorU32_Averageddataaddress = 0x10028,
   NiFpga_OpenScanFPGAHost_IndicatorU32_Data = 0x1004C,
   NiFpga_OpenScanFPGAHost_IndicatorU32_Galvosignalreaded = 0x10050,
   NiFpga_OpenScanFPGAHost_IndicatorU32_Requestaddress = 0x10054,
} NiFpga_OpenScanFPGAHost_IndicatorU32;

typedef enum
{
   NiFpga_OpenScanFPGAHost_ControlBool_Done = 0x1000E,
   NiFpga_OpenScanFPGAHost_ControlBool_ReadytoScan = 0x1001A,
   NiFpga_OpenScanFPGAHost_ControlBool_WriteDRAMenable = 0x10092,
   NiFpga_OpenScanFPGAHost_ControlBool_WriteFrameGalvosignal = 0x1009A,
} NiFpga_OpenScanFPGAHost_ControlBool;

typedef enum
{
   NiFpga_OpenScanFPGAHost_ControlU16_Current = 0x10012,
   NiFpga_OpenScanFPGAHost_ControlU16_Filtergain = 0x1000A,
   NiFpga_OpenScanFPGAHost_ControlU16_Kalmangain = 0x10006,
} NiFpga_OpenScanFPGAHost_ControlU16;

typedef enum
{
   NiFpga_OpenScanFPGAHost_ControlI32_Elementsperline = 0x10064,
   NiFpga_OpenScanFPGAHost_ControlI32_Framenumber = 0x10040,
   NiFpga_OpenScanFPGAHost_ControlI32_Numberofframes = 0x10080,
   NiFpga_OpenScanFPGAHost_ControlI32_Numofundershoot = 0x10070,
   NiFpga_OpenScanFPGAHost_ControlI32_Pixelclock_pulsewidthtick = 0x1005C,
   NiFpga_OpenScanFPGAHost_ControlI32_Pixelpulse_initialdelay = 0x10074,
   NiFpga_OpenScanFPGAHost_ControlI32_Pixeltimetick = 0x1006C,
   NiFpga_OpenScanFPGAHost_ControlI32_Resolution = 0x10060,
} NiFpga_OpenScanFPGAHost_ControlI32;

typedef enum
{
   NiFpga_OpenScanFPGAHost_ControlU32_Frameretracetime = 0x10078,
   NiFpga_OpenScanFPGAHost_ControlU32_Galvosignal = 0x10014,
   NiFpga_OpenScanFPGAHost_ControlU32_MaxDRAMaddress = 0x10000,
   NiFpga_OpenScanFPGAHost_ControlU32_Numofelements = 0x10068,
   NiFpga_OpenScanFPGAHost_ControlU32_Samplesperframecontrol = 0x1007C,
   NiFpga_OpenScanFPGAHost_ControlU32_Totalelements = 0x1008C,
   NiFpga_OpenScanFPGAHost_ControlU32_maxaddr = 0x10094,
} NiFpga_OpenScanFPGAHost_ControlU32;

typedef enum
{
   NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettohostFIFO = 0,
} NiFpga_OpenScanFPGAHost_TargetToHostFifoU32;

typedef enum
{
   NiFpga_OpenScanFPGAHost_HostToTargetFifoU32_HosttotargetFIFO = 1,
} NiFpga_OpenScanFPGAHost_HostToTargetFifoU32;

#endif
