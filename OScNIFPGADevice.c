#include "OScNIFPGADevicePrivate.h"
#include "OScNIFPGA.h"



static OSc_Device **g_devices;
static size_t g_deviceCount;
static uint32_t n;


static OSc_Error NIFPGAGetModelName(const char **name)
{
	*name = "OpenScan-NIFPGA";
	return OSc_Error_OK;
}


static OSc_Error NIFPGAGetInstances(OSc_Device ***devices, size_t *count)
{
	if (!g_devices)
		OSc_Return_If_Error(EnumerateInstances(&g_devices, &g_deviceCount));
	*devices = g_devices;
	*count = g_deviceCount;
	return OSc_Error_OK;
}


static OSc_Error NIFPGAReleaseInstance(OSc_Device *device)
{
	free(GetData(device));
	return OSc_Error_OK;
}


static OSc_Error NIFPGAGetName(OSc_Device *device, char *name)
{
	strncpy(name, GetData(device)->rioResourceName, OSc_MAX_STR_LEN);
	return OSc_Error_OK;
}


static OSc_Error NIFPGAOpen(OSc_Device *device)
{
	OSc_Return_If_Error(OpenFPGA(device));
	OSc_Return_If_Error(StartFPGA(device));

	return OSc_Error_OK;
}


static OSc_Error NIFPGAClose(OSc_Device *device)
{
	StopAcquisitionAndWait(device, GetData(device)->acquisition.acquisition);
	OSc_Error err = CloseFPGA(device);
	return err;
}


static OSc_Error NIFPGAHasScanner(OSc_Device *device, bool *hasScanner)
{
	*hasScanner = true;
	return OSc_Error_OK;
}


static OSc_Error NIFPGAHasDetector(OSc_Device *device, bool *hasDetector)
{
	*hasDetector = true;
	return OSc_Error_OK;
}


static OSc_Error NIFPGAGetSettings(OSc_Device *device, OSc_Setting ***settings, size_t *count)
{
	OSc_Return_If_Error(PrepareSettings(device));
	*settings = GetData(device)->settings;
	*count = GetData(device)->settingCount;
	return OSc_Error_OK;
}


static OSc_Error NIFPGAGetAllowedResolutions(OSc_Device *device, size_t **widths, size_t **heights, size_t *count)
{
	static size_t resolutions[] = {
		256, 512, 1024, 2048,
	};
	*widths = *heights = resolutions;
	*count = sizeof(resolutions) / sizeof(size_t);
	return OSc_Error_OK;
}


static OSc_Error NIFPGAGetResolution(OSc_Device *device, size_t *width, size_t *height)
{
	*width = *height = GetData(device)->resolution;
	return OSc_Error_OK;
}


static OSc_Error NIFPGASetResolution(OSc_Device *device, size_t width, size_t height)
{
	if (width == GetData(device)->resolution)
		return OSc_Error_OK;
	GetData(device)->resolution = (uint32_t)width;
	GetData(device)->settingsChanged = true;
	GetData(device)->reloadWaveformRequired = true;

	GetData(device)->magnification = (double)width / OSc_DEFAULT_RESOLUTION * GetData(device)->zoom / OSc_DEFAULT_ZOOM;

	return OSc_Error_OK;
}

static OSc_Error NIFPGAGetMagnification(OSc_Device *device, double *magnification)
{
	*magnification = GetData(device)->magnification;
	return OSc_Error_OK;
}

static OSc_Error NIFPGASetMagnification(OSc_Device *device)
{
	size_t resolution = GetData(device)->resolution;
	double zoom = GetData(device)->zoom;
	GetData(device)->magnification = (double)(resolution / OSc_DEFAULT_RESOLUTION) * (zoom / OSc_DEFAULT_ZOOM);
	
	return OSc_Error_OK;
}



static OSc_Error NIFPGAGetImageSize(OSc_Device *device, uint32_t *width, uint32_t *height)
{
	*width = GetData(device)->resolution;
	*height = GetData(device)->resolution;
	return OSc_Error_OK;
}


static OSc_Error NIFPGAGetNumberOfChannels(OSc_Device *device, uint32_t *nChannels)
{
	switch (GetData(device)->channels)
	{
	case CHANNELS_1_:
		*nChannels = 1;
		break;
	case CHANNELS_2_:
		*nChannels = 2;
		break;
	case CHANNELS_3_:
		*nChannels = 3;
		break;
	case CHANNELS_4_:
		*nChannels = 4;
		break;
	}
	return OSc_Error_OK;
}


static OSc_Error NIFPGAGetBytesPerSample(OSc_Device *device, uint32_t *bytesPerSample)
{
	*bytesPerSample = 2; // chaneg from 2 (16 bit) to 1 (8 bit)
	return OSc_Error_OK;
}


static OSc_Error ArmImpl(OSc_Device *device, OSc_Acquisition *acq)
{
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		if (GetData(device)->acquisition.running &&
			GetData(device)->acquisition.armed)
		{
			LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
			if (GetData(device)->acquisition.started)
				return OSc_Error_Acquisition_Running;
			else
				return OSc_Error_OK;
		}
		GetData(device)->acquisition.stopRequested = false;
		GetData(device)->acquisition.running = true;
		GetData(device)->acquisition.armed = false;
		GetData(device)->acquisition.started = false;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));

	if (GetData(device)->settingsChanged)
	{
		OSc_Log_Debug(device, "Setting up new scan...");
		if (GetData(device)->reloadWaveformRequired)
		{
			OSc_Return_If_Error(StartFPGA(device));
			OSc_Return_If_Error(WaitTillIdle(device));
		}
		OSc_Log_Debug(device, "1. Setting parameters...");
		OSc_Return_If_Error(SetBuildInParameters(device));
		OSc_Return_If_Error(SetPixelParameters(device,GetData(device)->scanRate));

		OSc_Log_Debug(device, "2. Clean flags...");
		OSc_Return_If_Error(Cleanflags(device));
		OSc_Return_If_Error(SetResolutionParameters(device, GetData(device)->resolution));
		OSc_Return_If_Error(SetTaskParameters(device, GetData(device)->nFrames));

		OSc_Log_Debug(device, "3. Cleaning FPGA DRAM and initializing globals...");
		OSc_Return_If_Error(InitScan(device));
		OSc_Return_If_Error(WaitTillIdle(device));

		if (GetData(device)->reloadWaveformRequired)
		{
			OSc_Return_If_Error(ReloadWaveform(device));
			OSc_Return_If_Error(WaitTillIdle(device));
		}

		GetData(device)->settingsChanged = false;
		GetData(device)->reloadWaveformRequired = false;
	}

	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		GetData(device)->acquisition.armed = true;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));

	return OSc_Error_OK;
}


static OSc_Error NIFPGAArmScanner(OSc_Device *device, OSc_Acquisition *acq)
{
	return ArmImpl(device, acq);
}


static OSc_Error NIFPGAStartScanner(OSc_Device *device, OSc_Acquisition *acq)
{
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		if (!GetData(device)->acquisition.running ||
			!GetData(device)->acquisition.armed)
		{
			LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
			return OSc_Error_Not_Armed;
		}
		if (GetData(device)->acquisition.started)
		{
			LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
			return OSc_Error_Acquisition_Running;
		}

		GetData(device)->acquisition.started = true;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));

	return RunAcquisitionLoop(device, acq);
}


static OSc_Error NIFPGAStopScanner(OSc_Device *device, OSc_Acquisition *acq)
{
	return StopAcquisitionAndWait(device, acq);
}


static OSc_Error NIFPGAArmDetector(OSc_Device *device, OSc_Acquisition *acq)
{
	return ArmImpl(device, acq);
}


static OSc_Error NIFPGAStartDetector(OSc_Device *device, OSc_Acquisition *acq)
{
	// We don't yet support running detector as trigger source
	return OSc_Error_Unsupported_Operation;
}


static OSc_Error NIFPGAStopDetector(OSc_Device *device, OSc_Acquisition *acq)
{
	return StopAcquisitionAndWait(device, acq);
}


static OSc_Error NIFPGAIsRunning(OSc_Device *device, bool *isRunning)
{
	return IsAcquisitionRunning(device, isRunning);
}


static OSc_Error NIFPGAWait(OSc_Device *device)
{
	return WaitForAcquisitionToFinish(device);
}


struct OSc_Device_Impl OpenScan_NIFPGA_Device_Impl = {
	.GetModelName = NIFPGAGetModelName,
	.GetInstances = NIFPGAGetInstances,
	.ReleaseInstance = NIFPGAReleaseInstance,
	.GetName = NIFPGAGetName,
	.Open = NIFPGAOpen,
	.Close = NIFPGAClose,
	.HasScanner = NIFPGAHasScanner,
	.HasDetector = NIFPGAHasDetector,
	.GetSettings = NIFPGAGetSettings,
	.GetAllowedResolutions = NIFPGAGetAllowedResolutions,
	.GetResolution = NIFPGAGetResolution,
	.SetResolution = NIFPGASetResolution,
	.GetMagnification = NIFPGAGetMagnification,
	.SetMagnification = NIFPGASetMagnification,
	.GetImageSize = NIFPGAGetImageSize,
	.GetNumberOfChannels = NIFPGAGetNumberOfChannels,
	.GetBytesPerSample = NIFPGAGetBytesPerSample,
	.ArmScanner = NIFPGAArmScanner,
	.StartScanner = NIFPGAStartScanner,
	.StopScanner = NIFPGAStopScanner,
	.ArmDetector =  NIFPGAArmDetector,
	.StartDetector = NIFPGAStartDetector,
	.StopDetector = NIFPGAStopDetector,
	.IsRunning = NIFPGAIsRunning,
	.Wait = NIFPGAWait,
};



OSc_Error OSc_ENTRY_POINT_FUNC(struct OSc_Device_Impl **impls, size_t *implCount)
{
	if (*implCount < 1)
		return OSc_Error_OK;

	impls[0] = &OpenScan_NIFPGA_Device_Impl;
	*implCount = 1;
	return OSc_Error_OK;
}