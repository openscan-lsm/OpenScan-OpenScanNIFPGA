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

	OScDev_Setting **settings;
	size_t settingCount;

	bool settingsChanged;
	bool reloadWaveformRequired;
	bool scannerEnabled;
	bool detectorEnabled;

	double scanRate;
	uint32_t resolution;
	double zoom;
	double magnification; // = (resolution/512) * (zoom/1)
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
	uint32_t nFrames;

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


OScDev_Error PrepareSettings(OScDev_Device *device);