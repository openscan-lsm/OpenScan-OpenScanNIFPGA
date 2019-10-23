#pragma once

#include "OScNIFPGADevice.h"

#include "OpenScanDeviceLib.h"

#include <NiFpga.h>

#include <Windows.h>


enum
{
	FPGA_STATE_IDLE,
	FPGA_STATE_INIT,
	FPGA_STATE_WRITE,
	FPGA_STATE_SCAN,
	FPGA_STATE_BLANK,
	FPGA_STATE_DONE,
	FPGA_STATE_STOP,
};

#define OSc_DEFAULT_RESOLUTION 512
#define OSc_DEFAULT_ZOOM 1.0

struct OScNIFPGAPrivateData
{
	char rioResourceName[OScDev_MAX_STR_LEN + 1];
	NiFpga_Session niFpgaSession;
	char bitfile[OScDev_MAX_STR_LEN + 1];

	// Remember last used to avoid unnecessary waveform reloads
	double lastAcquisitionPixelRateHz;
	uint32_t lastAcquisitionResolution;
	double lastAcquisitionZoomFactor;
	bool settingsChanged;
	bool reloadWaveformRequired;

	bool scannerEnabled;
	bool detectorEnabled;

	// counted as number of pixels. 
    // to adjust for the lag between the mirror control signal and the actual position of the mirror
	// scan phase (uSec) = line delay * bin factor / scan rate
	uint32_t lineDelay;
	double offsetXY[2];

	enum {
		CHANNELS_1_,
		CHANNELS_2_,
		CHANNELS_3_,
		CHANNELS_4_,

		CHANNELS_NUM_VALUES
	} channels;

	bool kalmanProgressive;
	uint16_t filterGain;
	uint32_t kalmanFrames;

	struct
	{
		CRITICAL_SECTION mutex;
		HANDLE thread;
		CONDITION_VARIABLE acquisitionFinishCondition;
		bool running;
		bool armed; // Valid when running == true
		bool started; // Valid when running == true
		bool stopRequested; // Valid when running == true
		OScDev_Acquisition *acquisition;
	} acquisition;
};


static inline struct OScNIFPGAPrivateData *GetData(OScDev_Device *device)
{
	return (struct OScNIFPGAPrivateData *)OScDev_Device_GetImplData(device);
}


OScDev_Error MakeSettings(OScDev_Device *device, OScDev_PtrArray **settings);