#pragma once

#include "OScNIFPGADevice.h"

#include "OpenScanLibPrivate.h"
#include "OpenScanDeviceImpl.h"

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


struct OScNIFPGAPrivateData
{
	char rioResourceName[OSc_MAX_STR_LEN + 1];
	NiFpga_Session niFpgaSession;
	char bitfile[OSc_MAX_STR_LEN + 1];

	OSc_Setting **settings;
	size_t settingCount;

	bool settingsChanged;

	double scanRate;
	uint32_t resolution;
	double zoom;
	double offsetXY[2];

	enum {
		CHANNELS_RAW_IMAGE,
		CHANNELS_KALMAN_AVERAGED,
		CHANNELS_RAW_AND_KALMAN,

		CHANNELS_NUM_VALUES
	} channels;

	bool kalmanProgressive;
	double filterGain;
	uint32_t kalmanFrames;

	struct
	{
		CRITICAL_SECTION mutex;
		HANDLE thread;
		bool running;
		bool armed; // Valid when running == true
		bool started; // Valid when running == true
		bool stopRequested; // Valid when running == true
		OSc_Acquisition *acquisition;
	} acquisition;
};


static inline struct OScNIFPGAPrivateData *GetData(OSc_Device *device)
{
	return (struct OScNIFPGAPrivateData *)(device->implData);
}


OSc_Error PrepareSettings(OSc_Device *device);