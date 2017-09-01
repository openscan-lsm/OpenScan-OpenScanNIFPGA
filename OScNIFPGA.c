#include "OScNIFPGA.h"
#include "Waveform.h"

#include "NiFpga_OpenScanFPGAHost.h"
#include <NiFpga.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <Windows.h>


static bool g_NiFpga_initialized = false;


static inline uint16_t DoubleToFixed16(double d, int intBits)
{
	int fracBits = 16 - intBits;
	return (uint16_t)round(d * (1 << fracBits));
}


static OSc_Error EnsureNiFpgaInitialized(void)
{
	if (g_NiFpga_initialized)
		return OSc_Error_OK;
	NiFpga_Status stat = NiFpga_Initialize();
	if (NiFpga_IsError(stat))
	{
		OSc_Log_Error(NULL, "Cannot access NI FPGA C API");
		return OSc_Error_Driver_Not_Available;
	}
	g_NiFpga_initialized = true;
	return OSc_Error_OK;
}


static OSc_Error DeinitializeNiFpga(void)
{
	if (!g_NiFpga_initialized)
		return OSc_Error_OK;
	NiFpga_Status stat = NiFpga_Finalize();
	if (NiFpga_IsError(stat))
		return stat; // TODO
	return OSc_Error_OK;
}


static void PopulateDefaultParameters(struct OScNIFPGAPrivateData *data)
{
	strncpy(data->bitfile, NiFpga_OpenScanFPGAHost_Bitfile, OSc_MAX_STR_LEN);

	data->settingsChanged = true;
	data->scanRate = 0.2;
	data->resolution = 512;
	data->zoom = 1.0;
	data->offsetXY[0] = data->offsetXY[1] = 0.0;
	data->channels = CHANNELS_RAW_IMAGE;
	data->kalmanProgressive = false;
	data->filterGain = 0.99;
	data->kalmanFrames = 1;

	InitializeCriticalSection(&(data->acquisition.mutex));
	data->acquisition.thread = NULL;
	data->acquisition.running = false;
	data->acquisition.armed = false;
	data->acquisition.started = false;
	data->acquisition.stopRequested = false;
	data->acquisition.acquisition = NULL;
}


OSc_Error EnumerateInstances(OSc_Device ***devices, size_t *deviceCount)
{
	OSc_Return_If_Error(EnsureNiFpgaInitialized());

	// The first FPGA board on the system always has the RIO Resource Name
	// "RIO0" (as far as I know). For now, only support this one.

	struct OScNIFPGAPrivateData *data = calloc(1, sizeof(struct OScNIFPGAPrivateData));
	strncpy(data->rioResourceName, "RIO0", OSc_MAX_STR_LEN);

	OSc_Device *device;
	OSc_Error err;
	if (OSc_Check_Error(err, OSc_Device_Create(&device, &OpenScan_NIFPGA_Device_Impl, data)))
	{
		char msg[OSc_MAX_STR_LEN + 1] = "Failed to create device ";
		strcat(msg, data->rioResourceName);
		OSc_Log_Error(NULL, msg);
		return err; // TODO
	}

	PopulateDefaultParameters(GetData(device));

	*devices = malloc(sizeof(OSc_Device *));
	*deviceCount = 1;
	(*devices)[0] = device;

	return OSc_Error_OK;
}


OSc_Error OpenFPGA(OSc_Device *device)
{
	OSc_Return_If_Error(EnsureNiFpgaInitialized());

	NiFpga_Status stat = NiFpga_Open(
		GetData(device)->bitfile,
		NiFpga_OpenScanFPGAHost_Signature,
		GetData(device)->rioResourceName,
		NiFpga_OpenAttribute_NoRun,
		&(GetData(device)->niFpgaSession));
	if (NiFpga_IsError(stat))
		return stat; // TODO

	return OSc_Error_OK;
}


OSc_Error CloseFPGA(OSc_Device *device)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	// TODO Stop acquisition
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

	// TODO Only do this if this is the last open device
	OSc_Return_If_Error(DeinitializeNiFpga());

	return OSc_Error_OK;
}


OSc_Error StartFPGA(OSc_Device *device)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;
	NiFpga_Status stat;
	OSc_Log_Debug(device, "Resetting FPGA...");
	stat = NiFpga_Reset(session);
	if (NiFpga_IsError(stat))
		return stat;
	OSc_Log_Debug(device, "Starting FPGA...");
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
		OSc_Log_Error(device, "Unexpected state after FPGA reset");
		return OSc_Error_Unknown;
	}

	return OSc_Error_OK;
}


static OSc_Error SendParameters(OSc_Device *device)
{
	OSc_Return_If_Error(StartFPGA(device));

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

	return OSc_Error_OK;
}


static OSc_Error SetScanRate(OSc_Device *device, double scanRate)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	double pixelTime = 40.0 / scanRate;
	int32_t pixelTimeTicks = (int32_t)round(pixelTime);
	NiFpga_Status stat = NiFpga_WriteI32(session,
		NiFpga_OpenScanFPGAHost_ControlI32_Pixeltimetick, pixelTimeTicks);
	if (NiFpga_IsError(stat))
		return stat;
	int32_t pulseWidthTicks = pixelTimeTicks - 21;
	stat = NiFpga_WriteI32(session,
		NiFpga_OpenScanFPGAHost_ControlI32_Pixelclock_pulsewidthtick, pulseWidthTicks);
	if (NiFpga_IsError(stat))
		return stat;

	return OSc_Error_OK;
}


static OSc_Error SetResolution(OSc_Device *device, uint32_t resolution)
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

	return OSc_Error_OK;
}


static OSc_Error InitScan(OSc_Device *device)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	NiFpga_Status stat;
	stat = NiFpga_WriteBool(session, NiFpga_OpenScanFPGAHost_IndicatorBool_Imageaveragingdone, false);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteBool(session, NiFpga_OpenScanFPGAHost_IndicatorBool_Averagedimagedisplayed, false);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteBool(session, NiFpga_OpenScanFPGAHost_IndicatorBool_Frameimagingdataaveraged, false);
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

	stat = NiFpga_WriteU16(session, NiFpga_OpenScanFPGAHost_ControlU16_Current, FPGA_STATE_INIT);
	if (NiFpga_IsError(stat))
		return stat;

	return OSc_Error_OK;
}


static OSc_Error SetKalmanGain(OSc_Device *device, double kg)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	NiFpga_Status stat = NiFpga_WriteU16(session,
		NiFpga_OpenScanFPGAHost_ControlU16_Kalmangain, DoubleToFixed16(kg, 1));
	if (NiFpga_IsError(stat))
		return stat;
	return OSc_Error_OK;
}


OSc_Error SetScanParameters(OSc_Device *device)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	NiFpga_Status stat = NiFpga_WriteI32(session,
		NiFpga_OpenScanFPGAHost_ControlI32_Numberofframes, 1);
	if (NiFpga_IsError(stat))
		return stat;
	stat = NiFpga_WriteI32(session, NiFpga_OpenScanFPGAHost_ControlI32_Framenumber, 0);
	if (NiFpga_IsError(stat))
		return stat;

	stat = NiFpga_WriteU16(session,
		NiFpga_OpenScanFPGAHost_ControlU16_Filtergain,
		DoubleToFixed16(GetData(device)->filterGain, 0));
	if (NiFpga_IsError(stat))
		return stat;
	int err = SetKalmanGain(device, 1.0);
	if (err != OSc_Error_OK)
		return err;

	return OSc_Error_OK;
}


static OSc_Error WriteWaveforms(OSc_Device *device, uint16_t *firstX, uint16_t *firstY)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	uint32_t resolution = GetData(device)->resolution;
	double zoom = GetData(device)->zoom;
	double offsetX = GetData(device)->offsetXY[0];
	double offsetY = GetData(device)->offsetXY[1];

	uint32_t elementsPerLine =
		X_UNDERSHOOT + resolution + X_RETRACE_LEN;
	uint16_t *xScaled = (uint16_t *)malloc(sizeof(uint16_t) * elementsPerLine);
	uint16_t *yScaled = (uint16_t *)malloc(sizeof(uint16_t) * resolution);

	OSc_Error err;
	if (OSc_Check_Error(err, GenerateScaledWaveforms(resolution, 0.25 * zoom,
		xScaled, yScaled, offsetX, offsetY)))
		return OSc_Error_Waveform_Out_Of_Range;

	NiFpga_Status stat;

	stat = NiFpga_WriteU16(session,
		NiFpga_OpenScanFPGAHost_ControlU16_Current, FPGA_STATE_WRITE);
	if (NiFpga_IsError(stat))
		goto error;

	stat = NiFpga_WriteBool(session,
		NiFpga_OpenScanFPGAHost_ControlBool_WriteDRAMenable, true);
	if (NiFpga_IsError(stat))
		goto error;
	stat = NiFpga_WriteBool(session,
		NiFpga_OpenScanFPGAHost_ControlBool_WriteFrameGalvosignal, true);
	if (NiFpga_IsError(stat))
		goto error;

	size_t fifoSize = 0;
	stat = NiFpga_WriteFifoU32(session,
		NiFpga_OpenScanFPGAHost_HostToTargetFifoU32_HosttotargetFIFO,
		0, 0, 10000, &fifoSize);

	uint32_t *xy = (uint32_t *)malloc(sizeof(uint32_t) * elementsPerLine);
	for (unsigned j = 0; j < resolution; ++j)
	{
		for (unsigned i = 0; i < elementsPerLine; ++i)
		{
			xy[i] = ((uint32_t)xScaled[i] << 16) | yScaled[j];
		}

		NiFpga_Bool dramFull = false;
		stat = NiFpga_ReadBool(session, NiFpga_OpenScanFPGAHost_IndicatorBool_WriteDRAMdone, &dramFull);
		if (NiFpga_IsError(stat))
			goto error;
		if (dramFull)
		{
			char msg[OSc_MAX_STR_LEN + 1];
			snprintf(msg, OSc_MAX_STR_LEN, "FPGA DRAM full; wrote %u lines of %d",
				j, (int)resolution);
			OSc_Log_Error(device, msg);
			stat = OSc_Error_Waveform_Memory_Size_Mismatch;
			goto error;
		}

		size_t remaining;
		stat = NiFpga_WriteFifoU32(session,
			NiFpga_OpenScanFPGAHost_HostToTargetFifoU32_HosttotargetFIFO,
			xy, elementsPerLine, 10000, &remaining);
		if (NiFpga_IsError(stat))
			goto error;

		bool done = false;
		while (!done)
		{
			stat = NiFpga_WriteFifoU32(session,
				NiFpga_OpenScanFPGAHost_HostToTargetFifoU32_HosttotargetFIFO,
				0, 0, 10000, &remaining);
			if (remaining == fifoSize)
				done = true;
		}
	}

	Sleep(10);

	NiFpga_Bool dramFull = false;
	stat = NiFpga_ReadBool(session, NiFpga_OpenScanFPGAHost_IndicatorBool_WriteDRAMdone, &dramFull);
	if (NiFpga_IsError(stat))
		goto error;
	if (!dramFull)
	{
		stat = OSc_Error_Waveform_Memory_Size_Mismatch;
		OSc_Log_Error(device, "FPGA DRAM not full after writing all lines");
		goto error;
	}

	*firstX = xScaled[0];
	*firstY = yScaled[0];

	free(xScaled);
	free(yScaled);

	return OSc_Error_OK;

error:
	free(xScaled);
	free(yScaled);
	return stat;
}


static OSc_Error MoveGalvosTo(OSc_Device *device, uint16_t x, uint16_t y)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	uint32_t xy = (uint32_t)x << 16 | y;
	NiFpga_Status stat = NiFpga_WriteU32(session,
		NiFpga_OpenScanFPGAHost_ControlU32_Galvosignal, xy);
	if (NiFpga_IsError(stat))
		return stat;
	return OSc_Error_OK;
}


OSc_Error ReloadWaveform(OSc_Device *device)
{
	OSc_Log_Debug(device, "Sending parameters...");
	OSc_Return_If_Error(SendParameters(device));

	OSc_Log_Debug(device, "Setting resolution...");
	OSc_Return_If_Error(SetResolution(device,
		GetData(device)->resolution));

	OSc_Log_Debug(device, "Setting up scan...");
	OSc_Return_If_Error(InitScan(device));

	uint16_t firstX, firstY;
	OSc_Log_Debug(device, "Writing waveform...");
	OSc_Return_If_Error(WriteWaveforms(device, &firstX, &firstY));

	OSc_Log_Debug(device, "Moving galvos to start position...");
	OSc_Return_If_Error(MoveGalvosTo(device, firstX, firstY));

	return OSc_Error_OK;
}


static OSc_Error StartScan(OSc_Device *device)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	double scanRate = GetData(device)->scanRate;

	int err = SetScanRate(device, scanRate);
	if (err != OSc_Error_OK)
		return err;

	// Workaround: Set ReadytoScan to false to acquire only one image
	NiFpga_Status stat = NiFpga_WriteBool(session,
		NiFpga_OpenScanFPGAHost_ControlBool_ReadytoScan, false);
	if (NiFpga_IsError(stat))
		return stat;

	stat = NiFpga_WriteU16(session,
		NiFpga_OpenScanFPGAHost_ControlU16_Current, FPGA_STATE_SCAN);
	if (NiFpga_IsError(stat))
		return stat;

	return OSc_Error_OK;
}


static OSc_Error StopScan(OSc_Device *device)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	NiFpga_Status stat = NiFpga_WriteBool(session,
		NiFpga_OpenScanFPGAHost_ControlBool_ReadytoScan, false);
	if (NiFpga_IsError(stat))
		return stat;

	return OSc_Error_OK;
}


static OSc_Error ReadImage(OSc_Device *device, OSc_Acquisition *acq, bool discard)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	uint32_t resolution = GetData(device)->resolution;
	size_t nPixels = resolution * resolution;

	NiFpga_Status stat = NiFpga_StartFifo(session,
		NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettohostFIFO);
	if (NiFpga_IsError(stat))
		return stat;

	uint32_t* rawAndAveraged = malloc(sizeof(uint32_t) * nPixels);
	size_t readSoFar = 0;
	int32_t prevPercentRead = -1;
	size_t available = 0;
	size_t remaining = 0;
	while (readSoFar < nPixels)
	{
		int32_t percentRead = (int32_t)(readSoFar * 100 / nPixels);
		if (percentRead > prevPercentRead)
		{
			if (percentRead % 1 == 0)
			{
				char msg[OSc_MAX_STR_LEN + 1];
				snprintf(msg, OSc_MAX_STR_LEN, "Read %d %%", percentRead);
				OSc_Log_Debug(device, msg);
			}
			prevPercentRead = percentRead;
		}

		stat = NiFpga_ReadFifoU32(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettohostFIFO,
			rawAndAveraged + readSoFar, 0, 3000, &available);
		if (NiFpga_IsError(stat))
			return stat;

		stat = NiFpga_ReadFifoU32(session,
			NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettohostFIFO,
			rawAndAveraged + readSoFar, available, 3000, &remaining);
		if (NiFpga_IsError(stat))
			return stat;

		readSoFar += available;

		Sleep(5);
	}

	Sleep(10);
	if (remaining > 0)
	{
		return OSc_Error_Data_Left_In_Fifo_After_Reading_Image;
	}

	stat = NiFpga_StopFifo(session,
		NiFpga_OpenScanFPGAHost_TargetToHostFifoU32_TargettohostFIFO);
	if (NiFpga_IsError(stat))
		return stat;

	if (!discard)
	{
		uint16_t *imageBuffer = malloc(sizeof(uint16_t) * nPixels);
		uint16_t *kalmanBuffer = malloc(sizeof(uint16_t) * nPixels);
		for (size_t i = 0; i < nPixels; ++i) {
			imageBuffer[i] = (uint16_t)(rawAndAveraged[i]);
			kalmanBuffer[i] = (uint16_t)(rawAndAveraged[i] >> 16);
		}

		acq->frameCallback(acq, 0, imageBuffer);
		acq->frameCallback(acq, 1, kalmanBuffer);
	}

	return OSc_Error_OK;
}


static OSc_Error AcquireFrame(OSc_Device *device, OSc_Acquisition *acq, unsigned kalmanCounter)
{
	NiFpga_Session session = GetData(device)->niFpgaSession;

	OSc_Log_Debug(device, "Starting scan...");
	OSc_Return_If_Error(StartScan(device));

	OSc_Log_Debug(device, "Waiting for scan to finish...");
	NiFpga_Bool finished = false;
	while (!finished)
	{
		NiFpga_Status stat = NiFpga_ReadBool(session,
			NiFpga_OpenScanFPGAHost_IndicatorBool_Frameacquisitionfinish, &finished);
		if (NiFpga_IsError(stat))
			return stat;
		Sleep(10);
	}

	bool lastOfKalmanAveraging = GetData(device)->kalmanProgressive ||
		kalmanCounter + 1 == GetData(device)->kalmanFrames;

	OSc_Log_Debug(device, "Reading image...");
	OSc_Return_If_Error(ReadImage(device, acq, !lastOfKalmanAveraging));
	OSc_Return_If_Error(SetKalmanGain(device, 1.0 / (kalmanCounter + 2)));
	OSc_Log_Debug(device, "Finished reading image");

	return OSc_Error_OK;
}


static DWORD WINAPI AcquisitionLoop(void *param)
{
	OSc_Device *device = (OSc_Device *)param;
	OSc_Acquisition *acq = GetData(device)->acquisition.acquisition;

	NiFpga_Status stat = NiFpga_WriteI32(GetData(device)->niFpgaSession,
		NiFpga_OpenScanFPGAHost_ControlI32_Framenumber, 0);
	if (NiFpga_IsError(stat))
	{
		char msg[OSc_MAX_STR_LEN + 1];
		snprintf(msg, OSc_MAX_STR_LEN,
			"Error during sequence acquisition: %d", (int)stat);
		OSc_Log_Error(device, msg);
		StopAcquisition(device, acq, false);
		return 0;
	}

	int totalFrames;
	if (acq->numberOfFrames == INT32_MAX)
		totalFrames = INT32_MAX;
	else if (GetData(device)->kalmanProgressive)
		totalFrames = acq->numberOfFrames;
	else
		totalFrames = acq->numberOfFrames * GetData(device)->kalmanFrames;

	for (int frame = 0; frame < totalFrames; ++frame)
	{
		InitializeCriticalSection(&(GetData(device)->acquisition.mutex));
		{
			if (GetData(device)->acquisition.stopRequested)
				break;
		}
		DeleteCriticalSection(&(GetData(device)->acquisition.mutex));

		OSc_Error err;
		if (OSc_Check_Error(err,
			AcquireFrame(device, acq, frame % GetData(device)->kalmanFrames)))
		{
			char msg[OSc_MAX_STR_LEN + 1];
			snprintf(msg, OSc_MAX_STR_LEN,
				"Error during sequence acquisition: %d", (int)err);
			OSc_Log_Error(device, msg);
			StopAcquisition(device, acq, false);
			return 0;
		}

		NiFpga_Status stat = NiFpga_WriteI32(GetData(device)->niFpgaSession,
			NiFpga_OpenScanFPGAHost_ControlI32_Framenumber, frame + 1);
		if (NiFpga_IsError(stat))
		{
			char msg[OSc_MAX_STR_LEN + 1];
			snprintf(msg, OSc_MAX_STR_LEN,
				"Error during sequence acquisition: %d", (int)stat);
			OSc_Log_Error(device, msg);
			StopAcquisition(device, acq, false);
			return 0;
		}
	}

	StopAcquisition(device, acq, false);
	return 0;
}


OSc_Error RunAcquisitionLoop(OSc_Device *device, OSc_Acquisition *acq)
{
	GetData(device)->acquisition.acquisition = acq;
	DWORD id;
	HANDLE thread = CreateThread(NULL, 0, AcquisitionLoop, device, 0, &id);
	return OSc_Error_OK;
}


OSc_Error StopAcquisition(OSc_Device *device, OSc_Acquisition *acq, bool wait)
{	GetData(device)->acquisition.acquisition = acq;
	DWORD id;
	HANDLE thread = CreateThread(NULL, 0, AcquisitionLoop, device, 0, &id);
	return OSc_Error_OK;

	InitializeCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		if (!GetData(device)->acquisition.running)
			return OSc_Error_OK;

		GetData(device)->acquisition.stopRequested = true;
	}
	if (wait)
		WaitForSingleObject(GetData(device)->acquisition.thread, INFINITE);
	return OSc_Error_OK;
}