#include "OScNIFPGA.h"
#include "Waveform.h"

#include "NiFpga_OpenScanFPGAHost.h"
#include <NiFpga.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <Windows.h>


static bool g_NiFpga_initialized = false;
static size_t g_openDeviceCount = 0;


static inline uint16_t DoubleToFixed16(double d, int intBits)
{
	int fracBits = 16 - intBits;
	return (uint16_t)round(d * (1 << fracBits));
}


static OScDev_Error EnsureNiFpgaInitialized(void)
{
	if (g_NiFpga_initialized)
		return OScDev_OK;
	NiFpga_Status stat = NiFpga_Initialize();
	if (NiFpga_IsError(stat))
	{
		OScDev_Log_Error(NULL, "Cannot access NI FPGA C API");
		return OScDev_Error_Driver_Not_Available;
	}
	g_NiFpga_initialized = true;
	return OScDev_OK;
}


static OScDev_Error DeinitializeNiFpga(void)
{
	if (!g_NiFpga_initialized)
		return OScDev_OK;
	NiFpga_Status stat = NiFpga_Finalize();
	if (NiFpga_IsError(stat))
		return stat; // TODO
	g_NiFpga_initialized = false;
	return OScDev_OK;
}


static void PopulateDefaultParameters(struct OScNIFPGAPrivateData *data)
{
	strncpy(data->bitfile, NiFpga_OpenScanFPGAHost_Bitfile, OScDev_MAX_STR_LEN);

	data->settingsChanged = true;
	data->reloadWaveformRequired = true;
	data->scanRate = 0.2;
	data->resolution = 512;
	data->zoom = 1.25;
	data->magnification = 1.0;
	data->offsetXY[0] = data->offsetXY[1] = 0.0;
	data->channels = CHANNELS_1_;
	data->kalmanProgressive = true;
	data->detectorEnabled = true;
	data->scannerEnabled = true;
	data->filterGain = 0.99; // TODO This is probably wrong; see also SetScanParameters
	data->kalmanFrames = 1;
	data->nFrames = 1;

	InitializeCriticalSection(&(data->acquisition.mutex));
	data->acquisition.thread = NULL;
	InitializeConditionVariable(&(data->acquisition.acquisitionFinishCondition));
	data->acquisition.running = false;
	data->acquisition.armed = false;
	data->acquisition.started = false;
	data->acquisition.stopRequested = false;
	data->acquisition.acquisition = NULL;
}


OScDev_Error EnumerateInstances(OScDev_Device ***devices, size_t *deviceCount)
{
	OScDev_Error err;
	if (OScDev_CHECK(err, EnsureNiFpgaInitialized()))
		return err;

	// The first FPGA board on the system always has the RIO Resource Name
	// "RIO0" (as far as I know). For now, only support this one.

	struct OScNIFPGAPrivateData *data = calloc(1, sizeof(struct OScNIFPGAPrivateData));
	strncpy(data->rioResourceName, "RIO0", OScDev_MAX_STR_LEN);

	OScDev_Device *device;
	if (OScDev_CHECK(err, OScDev_Device_Create(&device, &OpenScan_NIFPGA_Device_Impl, data)))
	{
		char msg[OScDev_MAX_STR_LEN + 1] = "Failed to create device ";
		strcat(msg, data->rioResourceName);
		OScDev_Log_Error(NULL, msg);
		return err; // TODO
	}

	PopulateDefaultParameters(GetData(device));

	*devices = malloc(sizeof(OScDev_Device *));
	*deviceCount = 1;
	(*devices)[0] = device;

	return OScDev_OK;
}


OScDev_Error OpenFPGA(OScDev_Device *device)
{
	OScDev_Error err;
	if (OScDev_CHECK(err, EnsureNiFpgaInitialized()))
		return err;

	NiFpga_Status stat = NiFpga_Open(
		GetData(device)->bitfile,
		NiFpga_OpenScanFPGAHost_Signature,
		GetData(device)->rioResourceName,
		NiFpga_OpenAttribute_NoRun,
		&(GetData(device)->niFpgaSession));
	if (NiFpga_IsError(stat))
		return stat; // TODO

	++g_openDeviceCount;

	return OScDev_OK;
}


OScDev_Error CloseFPGA(OScDev_Device *device)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	if (session)
	{
		// Reset FPGA to close shutter (temporary workaround)
		StartFPGA(device);

		NiFpga_Status stat;
		stat = NiFpga_WriteU16(session,
			NiFpga_OpenScanFPGAHost_ControlU16_Current,
			FPGA_STATE_STOP);
		if (NiFpga_IsError(stat))
			return stat; // TODO Wrap

		stat = NiFpga_Close(session, 0);
		if (NiFpga_IsError(stat))
			return stat; // TODO Wrap
	}

	--g_openDeviceCount;
	if (g_openDeviceCount == 0) {
		OScDev_Error err;
		if (OScDev_CHECK(err, DeinitializeNiFpga()))
			return err;
	}

	return OScDev_OK;
}


OScDev_Error StartFPGA(OScDev_Device *device)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;
	NiFpga_Status stat;
	OScDev_Log_Debug(device, "Resetting FPGA...");
	stat = NiFpga_Reset(session);
	if (NiFpga_IsError(stat))
		return stat;
	OScDev_Log_Debug(device, "Starting FPGA...");
	stat = NiFpga_Run(session, 0);
	if (NiFpga_IsError(stat))
		return stat;

	uint16_t currentState;
	stat = NiFpga_ReadU16(session, NiFpga_OpenScanFPGAHost_ControlU16_Current,
		&currentState);
	if (NiFpga_IsError(stat))
		return stat;
	if (currentState != FPGA_STATE_IDLE)
	{
		OScDev_Log_Error(device, "Unexpected state after FPGA reset");
		return OScDev_Error_Unknown;
	}

	return OScDev_OK;
}


static OScDev_Error SendParameters(OScDev_Device *device)
{
	OScDev_Error err;
	if (OScDev_CHECK(err, StartFPGA(device)))
		return err;

	NiFpga_Session session = GetData(device)->niFpgaSession;

	NiFpga_Status stat;
	stat = NiFpga_WriteI32(session,
		NiFpga_OpenScanFPGAHost_ControlI32_Numofundershoot, X_UNDERSHOOT);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteI32(session,
		NiFpga_OpenScanFPGAHost_ControlI32_Pixelpulse_initialdelay, 1);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteU32(session,
		NiFpga_OpenScanFPGAHost_ControlU32_Frameretracetime, 100);
	if (NiFpga_IsError(stat))
		return stat;

	return OScDev_OK;
}



static OScDev_Error SetScanRate(OScDev_Device *device, double scanRate)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	double pixelTime = 40.0 / scanRate;
	int32_t pixelTimeTicks = (int32_t)round(pixelTime);
	NiFpga_Status stat = NiFpga_WriteI32(session,
		NiFpga_OpenScanFPGAHost_ControlI32_Pixeltimetick, pixelTimeTicks);
	if (NiFpga_IsError(stat))
		return stat;
	int32_t pulseWidthTicks = pixelTimeTicks - 4;
	stat = NiFpga_WriteI32(session,
		NiFpga_OpenScanFPGAHost_ControlI32_Pixelclock_pulsewidthtick, pulseWidthTicks);
	if (NiFpga_IsError(stat))
		return stat;

	return OScDev_OK;
}


static OScDev_Error SetResolution(OScDev_Device *device, uint32_t resolution)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	int32_t elementsPerLine = X_UNDERSHOOT + resolution + X_RETRACE_LEN;

	NiFpga_Status stat = NiFpga_WriteI32(session,
		NiFpga_OpenScanFPGAHost_ControlI32_Resolution, resolution);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteI32(session,
		NiFpga_OpenScanFPGAHost_ControlI32_Elementsperline, elementsPerLine);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteU32(session,
		NiFpga_OpenScanFPGAHost_ControlU32_maxaddr, (uint32_t)elementsPerLine);
	if (NiFpga_IsError(stat))
		return stat;

	uint32_t totalElements = elementsPerLine * resolution;
	stat = NiFpga_WriteU32(session,
		NiFpga_OpenScanFPGAHost_ControlU32_Totalelements, totalElements);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteU32(session,
		NiFpga_OpenScanFPGAHost_ControlU32_Numofelements, totalElements);
	if (NiFpga_IsError(stat))
		return stat;

	uint32_t totalPixels = resolution * resolution;
	stat = NiFpga_WriteU32(session,
		NiFpga_OpenScanFPGAHost_ControlU32_Samplesperframecontrol, totalPixels);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteU32(session,
		NiFpga_OpenScanFPGAHost_ControlU32_MaxDRAMaddress, totalPixels / 16);
	if (NiFpga_IsError(stat))
		return stat;

	return OScDev_OK;
}


OScDev_Error InitScan(OScDev_Device *device)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	NiFpga_Status stat;

	stat = NiFpga_WriteU16(session, NiFpga_OpenScanFPGAHost_ControlU16_Current, FPGA_STATE_INIT);
	if (NiFpga_IsError(stat))
		return stat;

	return OScDev_OK;
}


static OScDev_Error SetKalmanGain(OScDev_Device *device, double kg)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;
	return OScDev_OK;
}


OScDev_Error SetScanParameters(OScDev_Device *device)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	NiFpga_Status stat = NiFpga_WriteI32(session,
		NiFpga_OpenScanFPGAHost_ControlI32_Numberofframes, GetData(device)->nFrames);
	if (NiFpga_IsError(stat))
		return stat;
	//need to set kalman factor
	stat = NiFpga_WriteU16(session,
		NiFpga_OpenScanFPGAHost_ControlU16_Kalmanfactor, GetData(device)->kalmanFrames);
	if (NiFpga_IsError(stat))
		return stat;

	GetData(device)->filterGain = 65534; // TODO See also default parameters
	stat = NiFpga_WriteU16(session,
		NiFpga_OpenScanFPGAHost_ControlU16_Filtergain,
		GetData(device)->filterGain);
	if (NiFpga_IsError(stat))
		return stat;
	int err = SetKalmanGain(device, 1.0);
	if (err != OScDev_OK)
		return err;

	return OScDev_OK;
}


static OScDev_Error WriteWaveforms(OScDev_Device *device, uint16_t *firstX, uint16_t *firstY)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	uint32_t resolution = GetData(device)->resolution;
	double zoom = GetData(device)->zoom;
	double offsetX = GetData(device)->offsetXY[0];
	double offsetY = GetData(device)->offsetXY[1];

	uint32_t elementsPerLine =
		X_UNDERSHOOT + resolution + X_RETRACE_LEN;
	uint32_t elementsPerRow = resolution + Y_RETRACE_LEN;
	uint16_t *xScaled = (uint16_t *)malloc(sizeof(uint16_t) * elementsPerLine);
	uint16_t *yScaled = (uint16_t *)malloc(sizeof(uint16_t) * elementsPerRow);

	OScDev_Error err;
	if (OScDev_CHECK(err, GenerateScaledWaveforms(resolution, 0.25 * zoom,
		xScaled, yScaled, offsetX, offsetY)))
		return OScDev_Error_Waveform_Out_Of_Range;

	NiFpga_Status stat;

	stat = NiFpga_WriteBool(session,
		NiFpga_OpenScanFPGAHost_ControlBool_WriteDRAMenable, true);
	if (NiFpga_IsError(stat))
		goto error;
	stat = NiFpga_WriteBool(session,
		NiFpga_OpenScanFPGAHost_ControlBool_WriteFrameGalvosignal, true);
	if (NiFpga_IsError(stat))
		goto error;

	stat = NiFpga_WriteU16(session,
		NiFpga_OpenScanFPGAHost_ControlU16_Current, FPGA_STATE_WRITE);
	if (NiFpga_IsError(stat))
		goto error;

	size_t fifoSize = 0;
	stat = NiFpga_WriteFifoU32(session,
		NiFpga_OpenScanFPGAHost_HostToTargetFifoU32_HosttotargetFIFO,
		0, 0, 10000, &fifoSize);

	uint32_t *xy = (uint32_t *)malloc(sizeof(uint32_t) * elementsPerLine);
	for (unsigned j = 0; j < elementsPerRow; ++j)
	{
		for (unsigned i = 0; i < elementsPerLine; ++i)
		{
			xy[i] = ((uint32_t)xScaled[i] << 16) | yScaled[j];
		}

		/*NiFpga_Bool dramFull = false;
		stat = NiFpga_ReadBool(session, NiFpga_OpenScanFPGAHost_IndicatorBool_WriteDRAMdone, &dramFull);
		if (NiFpga_IsError(stat))
			goto error;
		if (dramFull)
		{
			char msg[OScDev_MAX_STR_LEN + 1];
			snprintf(msg, OScDev_MAX_STR_LEN, "FPGA DRAM full; wrote %u lines of %d",
				j, (int)resolution);
			OScDev_Log_Error(device, msg);
			stat = OScDev_Error_Waveform_Memory_Size_Mismatch;
			goto error;
		}*/

		size_t remaining;
		stat = NiFpga_WriteFifoU32(session,
			NiFpga_OpenScanFPGAHost_HostToTargetFifoU32_HosttotargetFIFO,
			xy, elementsPerLine, 10000, &remaining);
		if (NiFpga_IsError(stat))
			goto error;

		/*bool done = false;
		while (!done)
		{
			stat = NiFpga_WriteFifoU32(session,
				NiFpga_OpenScanFPGAHost_HostToTargetFifoU32_HosttotargetFIFO,
				0, 0, 10000, &remaining);
			if (remaining == fifoSize)
				done = true;
		}*/
		Sleep(1);
	}



	/*NiFpga_Bool dramFull = false;
	stat = NiFpga_ReadBool(session, NiFpga_OpenScanFPGAHost_IndicatorBool_WriteDRAMdone, &dramFull);
	if (NiFpga_IsError(stat))
		goto error;
	if (!dramFull)
	{
		stat = OScDev_Error_Waveform_Memory_Size_Mismatch;
		OScDev_Log_Error(device, "FPGA DRAM not full after writing all lines");
		goto error;
	}*/

	*firstX = xScaled[0];
	*firstY = yScaled[0];

	free(xScaled);
	free(yScaled);

	return OScDev_OK;

error:
	free(xScaled);
	free(yScaled);
	return stat;
}


static OScDev_Error MoveGalvosTo(OScDev_Device *device, uint16_t x, uint16_t y)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	uint32_t xy = (uint32_t)x << 16 | y;
	NiFpga_Status stat = NiFpga_WriteU32(session,
		NiFpga_OpenScanFPGAHost_ControlU32_Galvosignal, xy);
	if (NiFpga_IsError(stat))
		return stat;
	return OScDev_OK;
}


OScDev_Error ReloadWaveform(OScDev_Device *device)
{
	OScDev_Error err;

	uint16_t firstX, firstY;
	OScDev_Log_Debug(device, "Writing waveform...");
	if (OScDev_CHECK(err, WriteWaveforms(device, &firstX, &firstY)))
		return err;

	OScDev_Log_Debug(device, "Moving galvos to start position...");
	if (OScDev_CHECK(err, MoveGalvosTo(device, firstX, firstY)))
		return err;

	return OScDev_OK;
}

OScDev_Error WaitTillIdle(OScDev_Device *device)
{
	OScDev_Log_Debug(device, "Please wait...");
	NiFpga_Session session = GetData(device)->niFpgaSession;
	NiFpga_Status stat;
	uint16_t currentState;
	do {
		stat = NiFpga_ReadU16(session, NiFpga_OpenScanFPGAHost_ControlU16_Current,
			&currentState);
		if (NiFpga_IsError(stat))
			return stat;
	} while (currentState != FPGA_STATE_IDLE);

	return OScDev_OK;

}

OScDev_Error SetBuildInParameters(OScDev_Device *device)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;
	NiFpga_Status stat;
	stat = NiFpga_WriteU32(session,
		NiFpga_OpenScanFPGAHost_ControlU32_Frameretracetime, 50);
	if (NiFpga_IsError(stat))
		return stat;

	return OScDev_OK;
}

OScDev_Error SetPixelParameters(OScDev_Device *device, double scanRate)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;
	double pixelTime = 40.0 / scanRate;
	int32_t pixelTimeTicks = (int32_t)round(pixelTime);
	NiFpga_Status stat = NiFpga_WriteI32(session,
		NiFpga_OpenScanFPGAHost_ControlI32_Pixeltimetick, pixelTimeTicks);
	if (NiFpga_IsError(stat))
		return stat;
	int32_t pulseWidthTicks = pixelTimeTicks - 4;
	stat = NiFpga_WriteI32(session,
		NiFpga_OpenScanFPGAHost_ControlI32_Pixelclock_pulsewidthtick, pulseWidthTicks);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteI32(session,
		NiFpga_OpenScanFPGAHost_ControlI32_Pixelpulse_initialdelay, 1);
	if (NiFpga_IsError(stat))
		return stat;

	return OScDev_OK;
}

OScDev_Error SetResolutionParameters(OScDev_Device *device, uint32_t resolution) 
{
	NiFpga_Session session = GetData(device)->niFpgaSession;
	int32_t elementsPerLine = X_UNDERSHOOT + resolution + X_RETRACE_LEN;
	uint32_t elementsPerRow = resolution + Y_RETRACE_LEN;

	NiFpga_Status stat = NiFpga_WriteI32(session,
		NiFpga_OpenScanFPGAHost_ControlI32_Resolution, resolution);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteI32(session,
		NiFpga_OpenScanFPGAHost_ControlI32_Elementsperline, elementsPerLine);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteU32(session,
		NiFpga_OpenScanFPGAHost_ControlU32_maxaddr, (uint32_t)elementsPerLine);
	if (NiFpga_IsError(stat))
		return stat;

	uint32_t totalElements = elementsPerLine * elementsPerRow;
	stat = NiFpga_WriteU32(session,
		NiFpga_OpenScanFPGAHost_ControlU32_Totalelements, totalElements);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteU32(session,
		NiFpga_OpenScanFPGAHost_ControlU32_Numofelements, totalElements);
	if (NiFpga_IsError(stat))
		return stat;

	uint32_t totalPixels = resolution * resolution;
	stat = NiFpga_WriteU32(session,
		NiFpga_OpenScanFPGAHost_ControlU32_Samplesperframecontrol, totalPixels);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteU32(session,
		NiFpga_OpenScanFPGAHost_ControlU32_MaxDRAMaddress, totalPixels / 16);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteI32(session,
		NiFpga_OpenScanFPGAHost_ControlI32_Numofundershoot, X_UNDERSHOOT);
	if (NiFpga_IsError(stat))
		return stat;

	return OScDev_OK;
}

OScDev_Error SetTaskParameters(OScDev_Device *device, uint32_t nf)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;
	uint16_t filtergain_ = 65534;
	NiFpga_Status stat = NiFpga_WriteI32(session,
		NiFpga_OpenScanFPGAHost_ControlI32_Numberofframes, nf);
	if (NiFpga_IsError(stat))
		return stat;
	//need to set kalman factor
	stat = NiFpga_WriteU16(session,
		NiFpga_OpenScanFPGAHost_ControlU16_Kalmanfactor, GetData(device)->kalmanFrames);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteU16(session,
		NiFpga_OpenScanFPGAHost_ControlU16_Filtergain, filtergain_);
	if (NiFpga_IsError(stat))
		return stat;

	stat = NiFpga_WriteU16(session,
		NiFpga_OpenScanFPGAHost_ControlBool_Enablescanner, GetData(device)->scannerEnabled);
	if (NiFpga_IsError(stat))
		return stat;

	stat = NiFpga_WriteU16(session,
		NiFpga_OpenScanFPGAHost_ControlBool_Enabledetector, GetData(device)->detectorEnabled);
	if (NiFpga_IsError(stat))
		return stat;

	return OScDev_OK;
}

OScDev_Error Cleanflags(OScDev_Device *device)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	NiFpga_Status stat;
	stat = NiFpga_WriteBool(session, NiFpga_OpenScanFPGAHost_IndicatorBool_Imageaveragingdone, false);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteBool(session, NiFpga_OpenScanFPGAHost_IndicatorBool_Averagedimagedisplayed, false);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteBool(session, NiFpga_OpenScanFPGAHost_ControlBool_WriteFrameGalvosignal, false);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteBool(session, NiFpga_OpenScanFPGAHost_ControlBool_WriteDRAMenable, false);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteBool(session, NiFpga_OpenScanFPGAHost_IndicatorBool_FrameGalvosignalwritedone, false);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteBool(session, NiFpga_OpenScanFPGAHost_IndicatorBool_Framewaveformoutputfinish, false);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteBool(session, NiFpga_OpenScanFPGAHost_IndicatorBool_Frameacquisitionfinish, false);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteBool(session, NiFpga_OpenScanFPGAHost_ControlBool_Done, false);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteBool(session, NiFpga_OpenScanFPGAHost_IndicatorBool_WriteDRAMdone, false);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteBool(session, NiFpga_OpenScanFPGAHost_ControlBool_CustomizedKalmangain, false);
	if (NiFpga_IsError(stat))
		return stat;

	return OScDev_OK;
}


static OScDev_Error StartScan(OScDev_Device *device)
{
	OScDev_Log_Debug(device, "Starting scanning...");
	NiFpga_Session session = GetData(device)->niFpgaSession;

	// Workaround: Set ReadytoScan to false to acquire only one image
	NiFpga_Status stat = NiFpga_WriteBool(session,
		NiFpga_OpenScanFPGAHost_ControlBool_ReadytoScan, true); // bug fixed
	if (NiFpga_IsError(stat))
		return stat;

	stat = NiFpga_WriteU16(session,
		NiFpga_OpenScanFPGAHost_ControlU16_Current, FPGA_STATE_SCAN);
	if (NiFpga_IsError(stat))
		return stat;

	return OScDev_OK;
}


static OScDev_Error StopScan(OScDev_Device *device)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;
	
	OScDev_Log_Debug(device, "Stopping Scanning...");
	NiFpga_Status stat = NiFpga_WriteBool(session,
		NiFpga_OpenScanFPGAHost_ControlBool_ReadytoScan, false);
	if (NiFpga_IsError(stat))
		return stat;
	OScDev_Error err;
	if (OScDev_CHECK(err, WaitTillIdle(device)))
		return err;

	/*OScDev_Log_Debug(device, "Cleaning Fifo...");
	if (OScDev_CHECK(err, CleanFifo(device))) {TODO}
	if (OScDev_CHECK(err, WaitTillIdle(device))) {TODO}*/

	return OScDev_OK;
}

static OScDev_Error CleanFifo(OScDev_Device *device)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	uint32_t resolution = GetData(device)->resolution;
	size_t nPixels = resolution * resolution;

	uint32_t *clean = malloc(sizeof(uint32_t) * nPixels * 4);
	size_t readSoFar = 0;
	size_t available = 0;
	size_t remaining = 0;

	do
	{
		NiFpga_Status stat = NiFpga_ReadFifoU32(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettohostFIFO1,
			clean + readSoFar, 0, -1, &available);
		if (NiFpga_IsError(stat))
			return stat;
		stat = NiFpga_ReadFifoU32(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettohostFIFO1,
			clean + readSoFar, available, -1, &remaining);
		if (NiFpga_IsError(stat))
			return stat;
		readSoFar += available;
	} while (available != 0);

	clean = 0;
	readSoFar = 0;
	available = 0;
	remaining = 0;
	do
	{
		NiFpga_Status stat = NiFpga_ReadFifoU32(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO2,
			clean + readSoFar, 0, -1, &available);
		if (NiFpga_IsError(stat))
			return stat;
		stat = NiFpga_ReadFifoU32(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO2,
			clean + readSoFar, available, -1, &remaining);
		if (NiFpga_IsError(stat))
			return stat;
		readSoFar += available;
	} while (available != 0);

	clean = 0;
	readSoFar = 0;
	available = 0;
	remaining = 0;
	do
	{
		NiFpga_Status stat = NiFpga_ReadFifoU32(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO3,
			clean + readSoFar, 0, -1, &available);
		if (NiFpga_IsError(stat))
			return stat;
		stat = NiFpga_ReadFifoU32(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO3,
			clean + readSoFar, available, -1, &remaining);
		if (NiFpga_IsError(stat))
			return stat;
		readSoFar += available;
	} while (available != 0);

	clean = 0;
	readSoFar = 0;
	available = 0;
	remaining = 0;
	do
	{
		NiFpga_Status stat = NiFpga_ReadFifoU32(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO4,
			clean + readSoFar, 0, -1, &available);
		if (NiFpga_IsError(stat))
			return stat;
		stat = NiFpga_ReadFifoU32(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO4,
			clean + readSoFar, available, -1, &remaining);
		if (NiFpga_IsError(stat))
			return stat;
		readSoFar += available;
	} while (available != 0);

	return OScDev_OK;
}


static OScDev_Error ReadImage(OScDev_Device *device, OScDev_Acquisition *acq, bool discard)
{
	uint32_t resolution = GetData(device)->resolution;
	size_t nPixels = resolution * resolution;
	uint32_t* rawAndAveraged = malloc(sizeof(uint32_t) * nPixels);
	uint32_t* rawAndAveraged2 = malloc(sizeof(uint32_t) * nPixels);
	uint32_t* rawAndAveraged3 = malloc(sizeof(uint32_t) * nPixels);
	uint32_t* rawAndAveraged4 = malloc(sizeof(uint32_t) * nPixels);

	if (GetData(device)->detectorEnabled == true)
	{
		OScDev_Log_Debug(device, "Reading image...");
		NiFpga_Session session = GetData(device)->niFpgaSession;

		NiFpga_Status stat = NiFpga_StartFifo(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettohostFIFO1);
		if (NiFpga_IsError(stat))
			return stat;

		stat = NiFpga_StartFifo(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO2);
		if (NiFpga_IsError(stat))
			return stat;

		stat = NiFpga_StartFifo(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO3);
		if (NiFpga_IsError(stat))
			return stat;

		stat = NiFpga_StartFifo(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO4);
		if (NiFpga_IsError(stat))
			return stat;


		size_t readSoFar = 0;
		int32_t prevPercentRead = -1;
		size_t available = 0;
		size_t remaining = 0;

		size_t readSoFar2 = 0;
		int32_t prevPercentRead2 = -1;
		size_t available2 = 0;
		size_t remaining2 = 0;

		size_t readSoFar3 = 0;
		int32_t prevPercentRead3 = -1;
		size_t available3 = 0;
		size_t remaining3 = 0;

		size_t readSoFar4 = 0;
		int32_t prevPercentRead4 = -1;
		size_t available4 = 0;
		size_t remaining4 = 0;

		int32_t loopcount = 0;


		//Begin reading only when there is data input into FIFO
		while (!(available && available2 && available3 && available4))
		{
			loopcount++;
			if (loopcount > 5000)
			{
				char msg[OScDev_MAX_STR_LEN + 1];
				snprintf(msg, OScDev_MAX_STR_LEN, "Scan timeout");
				OScDev_Log_Debug(device, msg);
				break;
			}

			stat = NiFpga_ReadFifoU32(session,
				NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettohostFIFO1,
				rawAndAveraged + readSoFar, 0, -1, &available);
			if (NiFpga_IsError(stat))
				return stat;

			stat = NiFpga_ReadFifoU32(session,
				NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO2,
				rawAndAveraged2 + readSoFar2, 0, -1, &available2);
			if (NiFpga_IsError(stat))
				return stat;

			stat = NiFpga_ReadFifoU32(session,
				NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO3,
				rawAndAveraged3 + readSoFar3, 0, -1, &available3);
			if (NiFpga_IsError(stat))
				return stat;

			stat = NiFpga_ReadFifoU32(session,
				NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO4,
				rawAndAveraged4 + readSoFar4, 0, -1, &available4);
			if (NiFpga_IsError(stat))
				return stat;

			Sleep(5);
		}

		loopcount = 0;

		while ((readSoFar < nPixels) || (readSoFar2 < nPixels) || (readSoFar3 < nPixels) || (readSoFar4 < nPixels))
		{
			loopcount++;
			if (loopcount > 1000)
			{
				char msg[OScDev_MAX_STR_LEN + 1];
				snprintf(msg, OScDev_MAX_STR_LEN, "Read image timeout");
				OScDev_Log_Debug(device, msg);
				break;
			}
			int32_t percentRead = (int32_t)(readSoFar * 100 / nPixels);
			int32_t percentRead2 = (int32_t)(readSoFar2 * 100 / nPixels);
			int32_t percentRead3 = (int32_t)(readSoFar3 * 100 / nPixels);
			int32_t percentRead4 = (int32_t)(readSoFar4 * 100 / nPixels);

			if (percentRead > prevPercentRead)
			{
				if (percentRead % 1 == 0)
				{
					char msg[OScDev_MAX_STR_LEN + 1];
					snprintf(msg, OScDev_MAX_STR_LEN, "Read channel 1 %d %%", percentRead);
					OScDev_Log_Debug(device, msg);
				}
				prevPercentRead = percentRead;
			}

			if (percentRead2 > prevPercentRead2)
			{
				if (percentRead2 % 1 == 0)
				{
					char msg[OScDev_MAX_STR_LEN + 1];
					snprintf(msg, OScDev_MAX_STR_LEN, "Read channel 2 %d %%", percentRead2);
					OScDev_Log_Debug(device, msg);
				}
				prevPercentRead2 = percentRead2;
			}

			if (percentRead3 > prevPercentRead3)
			{
				if (percentRead3 % 1 == 0)
				{
					char msg[OScDev_MAX_STR_LEN + 1];
					snprintf(msg, OScDev_MAX_STR_LEN, "Read channel 3 %d %%", percentRead3);
					OScDev_Log_Debug(device, msg);
				}
				prevPercentRead3 = percentRead3;
			}

			if (percentRead4 > prevPercentRead4)
			{
				if (percentRead4 % 1 == 0)
				{
					char msg[OScDev_MAX_STR_LEN + 1];
					snprintf(msg, OScDev_MAX_STR_LEN, "Read channel 4 %d %%", percentRead4);
					OScDev_Log_Debug(device, msg);
				}
				prevPercentRead4 = percentRead4;
			}

			if (readSoFar < nPixels)
			{
				stat = NiFpga_ReadFifoU32(session,
					NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettohostFIFO1,
					rawAndAveraged + readSoFar, 0, 3000, &available);
				if (NiFpga_IsError(stat))
					return stat;

				stat = NiFpga_ReadFifoU32(session,
					NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettohostFIFO1,
					rawAndAveraged + readSoFar, available, 3000, &remaining);
				if (NiFpga_IsError(stat))
					return stat;

				readSoFar += available;
			}

			if (readSoFar2 < nPixels)
			{
				stat = NiFpga_ReadFifoU32(session,
					NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO2,
					rawAndAveraged2 + readSoFar2, 0, 3000, &available2);
				if (NiFpga_IsError(stat))
					return stat;

				stat = NiFpga_ReadFifoU32(session,
					NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO2,
					rawAndAveraged2 + readSoFar2, available2, 3000, &remaining2);
				if (NiFpga_IsError(stat))
					return stat;

				readSoFar2 += available2;
			}

			if (readSoFar3 < nPixels)
			{
				stat = NiFpga_ReadFifoU32(session,
					NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO3,
					rawAndAveraged3 + readSoFar3, 0, 3000, &available3);
				if (NiFpga_IsError(stat))
					return stat;

				stat = NiFpga_ReadFifoU32(session,
					NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO3,
					rawAndAveraged3 + readSoFar3, available3, 3000, &remaining3);
				if (NiFpga_IsError(stat))
					return stat;

				readSoFar3 += available3;
			}

			if (readSoFar4 < nPixels)
			{
				stat = NiFpga_ReadFifoU32(session,
					NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO4,
					rawAndAveraged4 + readSoFar4, 0, 3000, &available4);
				if (NiFpga_IsError(stat))
					return stat;

				stat = NiFpga_ReadFifoU32(session,
					NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO4,
					rawAndAveraged4 + readSoFar4, available4, 3000, &remaining4);
				if (NiFpga_IsError(stat))
					return stat;

				readSoFar4 += available4;
			}


			Sleep(5);
		}

		Sleep(10);
		if (remaining > 0)
		{
			return OScDev_Error_Data_Left_In_Fifo_After_Reading_Image;
		}

		stat = NiFpga_StopFifo(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettohostFIFO1);
		if (NiFpga_IsError(stat))
			return stat;

		stat = NiFpga_StopFifo(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO2);
		if (NiFpga_IsError(stat))
			return stat;

		stat = NiFpga_StopFifo(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO3);
		if (NiFpga_IsError(stat))
			return stat;

		stat = NiFpga_StopFifo(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettoHostFIFO4);
		if (NiFpga_IsError(stat))
			return stat;
	}

		if (!discard)
		{
			uint16_t *imageBuffer = malloc(sizeof(uint16_t) * nPixels);
			uint16_t *kalmanBuffer = malloc(sizeof(uint16_t) * nPixels);
			uint16_t *imageBuffer2 = malloc(sizeof(uint16_t) * nPixels);
			uint16_t *kalmanBuffer2 = malloc(sizeof(uint16_t) * nPixels);
			uint16_t *imageBuffer3 = malloc(sizeof(uint16_t) * nPixels);
			uint16_t *kalmanBuffer3 = malloc(sizeof(uint16_t) * nPixels);
			uint16_t *imageBuffer4 = malloc(sizeof(uint16_t) * nPixels);
			uint16_t *kalmanBuffer4 = malloc(sizeof(uint16_t) * nPixels);

			for (size_t i = 0; i < nPixels; ++i) {
				imageBuffer[i] = (uint16_t)(rawAndAveraged[i]);
				kalmanBuffer[i] = (uint16_t)(rawAndAveraged[i] >> 16);
				imageBuffer2[i] = (uint16_t)(rawAndAveraged2[i]);
				kalmanBuffer2[i] = (uint16_t)(rawAndAveraged2[i] >> 16);
				imageBuffer3[i] = (uint16_t)(rawAndAveraged3[i]);
				kalmanBuffer3[i] = (uint16_t)(rawAndAveraged3[i] >> 16);
				imageBuffer4[i] = (uint16_t)(rawAndAveraged4[i]);
				kalmanBuffer4[i] = (uint16_t)(rawAndAveraged4[i] >> 16);
			}

			bool shouldContinue;
			switch (GetData(device)->channels)
			{
			case CHANNELS_1_:
				shouldContinue = OScDev_Acquisition_CallFrameCallback(acq, 0, kalmanBuffer);
				break;

			case CHANNELS_2_:
				shouldContinue = OScDev_Acquisition_CallFrameCallback(acq, 0, kalmanBuffer) &&
					OScDev_Acquisition_CallFrameCallback(acq, 1, kalmanBuffer2);
				break;
			
			case CHANNELS_3_:
				shouldContinue = OScDev_Acquisition_CallFrameCallback(acq, 0, kalmanBuffer) &&
					OScDev_Acquisition_CallFrameCallback(acq, 1, kalmanBuffer2) &&
					OScDev_Acquisition_CallFrameCallback(acq, 2, kalmanBuffer3);
				break;
			
			case CHANNELS_4_:
			default: // TODO Why is default 4?
				shouldContinue = OScDev_Acquisition_CallFrameCallback(acq, 0, kalmanBuffer) &&
					OScDev_Acquisition_CallFrameCallback(acq, 1, kalmanBuffer2) &&
					OScDev_Acquisition_CallFrameCallback(acq, 2, kalmanBuffer3) &&
					OScDev_Acquisition_CallFrameCallback(acq, 3, kalmanBuffer4);
				break;
			}

			if (!shouldContinue) {
				// TODO We should use the return value of the frame callback to halt acquisition
			}
		}

		return OScDev_OK;


	
}


static OScDev_Error AcquireFrame(OScDev_Device *device, OScDev_Acquisition *acq, unsigned kalmanCounter)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;
	// if (OScDev_CHECK(err, StartScan(device))) {TODO}

	bool lastOfKalmanAveraging = GetData(device)->kalmanProgressive ||
		kalmanCounter + 1 == GetData(device)->kalmanFrames;

	int thisImage;
	thisImage = 1;

	for (unsigned i = 0; i < GetData(device)->kalmanFrames; ++i)
	{
		char msg[OScDev_MAX_STR_LEN + 1];
		snprintf(msg, OScDev_MAX_STR_LEN, "Image %d", thisImage);
		OScDev_Log_Debug(device, msg);
		thisImage++;

		OScDev_Error err;
		if (OScDev_CHECK(err, ReadImage(device, acq, !lastOfKalmanAveraging)))
			return err;
		OScDev_Log_Debug(device, "Finished reading image");

	}


	return OScDev_OK;
}


static void FinishAcquisition(OScDev_Device *device)
{
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	GetData(device)->acquisition.running = false;
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
	CONDITION_VARIABLE *cv = &(GetData(device)->acquisition.acquisitionFinishCondition);
	WakeAllConditionVariable(cv);
}


static DWORD WINAPI AcquisitionLoop(void *param)
{
	OScDev_Device *device = (OScDev_Device *)param;
	OScDev_Acquisition *acq = GetData(device)->acquisition.acquisition;

	OScDev_Error err;
	uint32_t acqNumFrames;
	if (OScDev_CHECK(err, OScDev_Acquisition_GetNumberOfFrames(acq, &acqNumFrames)))
		return err;

	int totalFrames;
	int thisFrame;
	if (acqNumFrames == INT32_MAX)
		totalFrames = INT32_MAX;
	//else if (GetData(device)->kalmanProgressive)
	// 	totalFrames = acq->numberOfFrames * GetData(device)->kalmanFrames;
	else
		totalFrames = acqNumFrames * GetData(device)->kalmanFrames;

	if (OScDev_CHECK(err, SetTaskParameters(device, totalFrames)))
		return err;
	if (OScDev_CHECK(err, WaitTillIdle(device)))
		return err;

	char msg[OScDev_MAX_STR_LEN + 1];
	snprintf(msg, OScDev_MAX_STR_LEN, "%d Kalman images", GetData(device)->kalmanFrames);
	OScDev_Log_Debug(device, msg);

	snprintf(msg, OScDev_MAX_STR_LEN, "%d number of frames", acqNumFrames);
	OScDev_Log_Debug(device, msg);

	snprintf(msg, OScDev_MAX_STR_LEN, "%d total images", totalFrames);
	OScDev_Log_Debug(device, msg);
	
	OScDev_Log_Debug(device, "Starting acquisition loop...");
	if (OScDev_CHECK(err, StartScan(device)))
		return err;

	thisFrame = 1;

	for (uint32_t frame = 0; frame < acqNumFrames; ++frame)
	{
		char msg[OScDev_MAX_STR_LEN + 1];
		snprintf(msg, OScDev_MAX_STR_LEN, "Start frame %d", thisFrame);
		OScDev_Log_Debug(device, msg);
		thisFrame++;

		bool stopRequested;
		EnterCriticalSection(&(GetData(device)->acquisition.mutex));
		stopRequested = GetData(device)->acquisition.stopRequested;
		LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
		if (stopRequested)
		{
			OScDev_Log_Debug(device, "User interruption...");
			if (OScDev_CHECK(err, StopScan(device)))
				return err;
			GetData(device)->settingsChanged = true;
			GetData(device)->reloadWaveformRequired = true;
			break;
		}


		OScDev_Error err;
		if (OScDev_CHECK(err,
			AcquireFrame(device, acq, frame % GetData(device)->kalmanFrames)))
		{
			char msg[OScDev_MAX_STR_LEN + 1];
			snprintf(msg, OScDev_MAX_STR_LEN,
				"Error during sequence acquisition: %d", (int)err);
			OScDev_Log_Error(device, msg);
			FinishAcquisition(device);
			return 0;
		}
	}

	// GetData(device)->settingsChanged = true;
	// GetData(device)->reloadWaveformRequired = true;
	FinishAcquisition(device);
	return 0;
}


OScDev_Error RunAcquisitionLoop(OScDev_Device *device)
{
	DWORD id;
	GetData(device)->acquisition.thread =
		CreateThread(NULL, 0, AcquisitionLoop, device, 0, &id);
	return OScDev_OK;
}


OScDev_Error StopAcquisitionAndWait(OScDev_Device *device)
{
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		if (!GetData(device)->acquisition.running)
		{
			LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
			return OScDev_OK;
		}

		GetData(device)->acquisition.stopRequested = true;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
	return WaitForAcquisitionToFinish(device);
}


OScDev_Error IsAcquisitionRunning(OScDev_Device *device, bool *isRunning)
{
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	*isRunning = GetData(device)->acquisition.running;
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
	return OScDev_OK;
}


OScDev_Error WaitForAcquisitionToFinish(OScDev_Device *device)
{
	OScDev_Error err = OScDev_OK;
	CONDITION_VARIABLE *cv = &(GetData(device)->acquisition.acquisitionFinishCondition);

	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	while (GetData(device)->acquisition.running)
	{
		SleepConditionVariableCS(cv, &(GetData(device)->acquisition.mutex), INFINITE);
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
	return err;
}
