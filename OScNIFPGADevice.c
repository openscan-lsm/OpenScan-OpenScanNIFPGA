#include "OScNIFPGADevicePrivate.h"
#include "OScNIFPGA.h"



static OScDev_Device **g_devices;
static size_t g_deviceCount;
static uint32_t n;


static OScDev_Error NIFPGAGetModelName(const char **name)
{
	*name = "OpenScan-NIFPGA";
	return OScDev_OK;
}


static OScDev_Error NIFPGAGetInstances(OScDev_Device ***devices, size_t *count)
{
	if (!g_devices)
	{
		OScDev_Error err;
		if (OScDev_CHECK(err, EnumerateInstances(&g_devices, &g_deviceCount)))
			return err;
	}
	*devices = g_devices;
	*count = g_deviceCount;
	return OScDev_OK;
}


static OScDev_Error NIFPGAReleaseInstance(OScDev_Device *device)
{
	free(GetData(device));
	return OScDev_OK;
}


static OScDev_Error NIFPGAGetName(OScDev_Device *device, char *name)
{
	strncpy(name, GetData(device)->rioResourceName, OScDev_MAX_STR_LEN);
	return OScDev_OK;
}


static OScDev_Error NIFPGAOpen(OScDev_Device *device)
{
	OScDev_Error err;
	if (OScDev_CHECK(err, OpenFPGA(device)))
		return err;
	if (OScDev_CHECK(err, StartFPGA(device)))
		return err;

	return OScDev_OK;
}


static OScDev_Error NIFPGAClose(OScDev_Device *device)
{
	StopAcquisitionAndWait(device);
	OScDev_Error err = CloseFPGA(device);
	return err;
}


static OScDev_Error NIFPGAHasClock(OScDev_Device *device, bool *hasClock)
{
	*hasClock = false;  // TODO: clock is not decoupled from scanner in current FPGA code
	return OScDev_OK;
}

static OScDev_Error NIFPGAHasScanner(OScDev_Device *device, bool *hasScanner)
{
	*hasScanner = true;
	return OScDev_OK;
}


static OScDev_Error NIFPGAHasDetector(OScDev_Device *device, bool *hasDetector)
{
	*hasDetector = true;
	return OScDev_OK;
}


static OScDev_Error NIFPGAGetSettings(OScDev_Device *device, OScDev_Setting ***settings, size_t *count)
{
	OScDev_Error err;
	if (OScDev_CHECK(err, PrepareSettings(device)))
		return err;
	*settings = GetData(device)->settings;
	*count = GetData(device)->settingCount;
	return OScDev_OK;
}


static OScDev_Error NIFPGAGetAllowedResolutions(OScDev_Device *device, size_t **widths, size_t **heights, size_t *count)
{
	static size_t resolutions[] = {
		256, 512, 1024, 2048,
	};
	*widths = *heights = resolutions;
	*count = sizeof(resolutions) / sizeof(size_t);
	return OScDev_OK;
}


static OScDev_Error NIFPGAGetResolution(OScDev_Device *device, size_t *width, size_t *height)
{
	*width = *height = GetData(device)->resolution;
	return OScDev_OK;
}


static OScDev_Error NIFPGASetResolution(OScDev_Device *device, size_t width, size_t height)
{
	if (width == GetData(device)->resolution)
		return OScDev_OK;
	GetData(device)->resolution = (uint32_t)width;
	GetData(device)->settingsChanged = true;
	GetData(device)->reloadWaveformRequired = true;

	GetData(device)->magnification = (double)width / OSc_DEFAULT_RESOLUTION * GetData(device)->zoom / OSc_DEFAULT_ZOOM;

	return OScDev_OK;
}

static OScDev_Error NIFPGAGetMagnification(OScDev_Device *device, double *magnification)
{
	*magnification = GetData(device)->magnification;
	return OScDev_OK;
}

static OScDev_Error NIFPGASetMagnification(OScDev_Device *device)
{
	size_t resolution = GetData(device)->resolution;
	double zoom = GetData(device)->zoom;
	GetData(device)->magnification = (double)(resolution / OSc_DEFAULT_RESOLUTION) * (zoom / OSc_DEFAULT_ZOOM);
	
	return OScDev_OK;
}



static OScDev_Error NIFPGAGetImageSize(OScDev_Device *device, uint32_t *width, uint32_t *height)
{
	*width = GetData(device)->resolution;
	*height = GetData(device)->resolution;
	return OScDev_OK;
}


static OScDev_Error NIFPGAGetNumberOfChannels(OScDev_Device *device, uint32_t *nChannels)
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
	return OScDev_OK;
}


static OScDev_Error NIFPGAGetBytesPerSample(OScDev_Device *device, uint32_t *bytesPerSample)
{
	*bytesPerSample = 2; // chaneg from 2 (16 bit) to 1 (8 bit)
	return OScDev_OK;
}


static OScDev_Error NIFPGAArm(OScDev_Device *device, OScDev_Acquisition *acq)
{
	bool useClock, useScanner, useDetector;
	OScDev_Acquisition_IsClockRequested(acq, &useClock);
	OScDev_Acquisition_IsScannerRequested(acq, &useScanner);
	OScDev_Acquisition_IsDetectorRequested(acq, &useDetector);

	// assume scanner is always enabled
	if (!useClock || !useScanner)
		return OScDev_Error_Unsupported_Operation;

	enum OScDev_TriggerSource clockStartTriggerSource;
	OScDev_Acquisition_GetClockStartTriggerSource(acq, &clockStartTriggerSource);
	if (clockStartTriggerSource != OScDev_TriggerSource_Software)
		return OScDev_Error_Unsupported_Operation;

	enum OScDev_ClockSource clockSource;
	OScDev_Acquisition_GetClockSource(acq, &clockSource);
	if (clockSource != OScDev_ClockSource_Internal)
		return OScDev_Error_Unsupported_Operation;
	// what if we use external line clock to trigger acquisition?

	if (useDetector)
	{
		// arm scanner/clock and detector
		GetData(device)->detectorEnabled = true;
	}
	else
	{
		// arm scanner and clock and disable detector
		GetData(device)->detectorEnabled = false;
	}

	if (useScanner)
	{
		GetData(device)->scannerEnabled = true;
	}
	else
		GetData(device)->scannerEnabled = false;
	
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		if (GetData(device)->acquisition.running &&
			GetData(device)->acquisition.armed)
		{
			LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
			if (GetData(device)->acquisition.started)
				return OScDev_Error_Acquisition_Running;
			else
				return OScDev_OK;
		}
		GetData(device)->acquisition.stopRequested = false;
		GetData(device)->acquisition.running = true;
		GetData(device)->acquisition.armed = false;
		GetData(device)->acquisition.started = false;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));

	if (GetData(device)->settingsChanged)
	{
		OScDev_Error err;

		OScDev_Log_Debug(device, "Setting up new scan...");
		if (GetData(device)->reloadWaveformRequired)
		{
			if (OScDev_CHECK(err, StartFPGA(device)))
				return err;
			if (OScDev_CHECK(err, WaitTillIdle(device)))
				return err;
		}
		OScDev_Log_Debug(device, "1. Setting parameters...");
		if (OScDev_CHECK(err, SetBuildInParameters(device)))
			return err;
		if (OScDev_CHECK(err, SetPixelParameters(device, GetData(device)->scanRate)))
			return err;

		OScDev_Log_Debug(device, "2. Clean flags...");
		if (OScDev_CHECK(err, Cleanflags(device)))
			return err;
		if (OScDev_CHECK(err, SetResolutionParameters(device, GetData(device)->resolution)))
			return err;
		if (OScDev_CHECK(err, SetTaskParameters(device, GetData(device)->nFrames)))
			return err;

		OScDev_Log_Debug(device, "3. Cleaning FPGA DRAM and initializing globals...");
		if (OScDev_CHECK(err, InitScan(device)))
			return err;
		if (OScDev_CHECK(err, WaitTillIdle(device)))
			return err;

		if (GetData(device)->reloadWaveformRequired)
		{
			if (OScDev_CHECK(err, ReloadWaveform(device)))
				return err;
			if (OScDev_CHECK(err, WaitTillIdle(device)))
				return err;
		}

		GetData(device)->settingsChanged = false;
		GetData(device)->reloadWaveformRequired = false;
	}

	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		GetData(device)->acquisition.armed = true;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));

	return OScDev_OK;

}


static OScDev_Error NIFPGAStart(OScDev_Device *device)
{
	// start scanner
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		if (!GetData(device)->acquisition.running ||
			!GetData(device)->acquisition.armed)
		{
			LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
			return OScDev_Error_Not_Armed;
		}
		if (GetData(device)->acquisition.started)
		{
			LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
			return OScDev_Error_Acquisition_Running;
		}

		GetData(device)->acquisition.started = true;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));

	// We don't yet support running detector as trigger source

	return RunAcquisitionLoop(device);
}


static OScDev_Error NIFPGAStop(OScDev_Device *device)
{
	return StopAcquisitionAndWait(device);
}


static OScDev_Error NIFPGAIsRunning(OScDev_Device *device, bool *isRunning)
{
	return IsAcquisitionRunning(device, isRunning);
}


static OScDev_Error NIFPGAWait(OScDev_Device *device)
{
	return WaitForAcquisitionToFinish(device);
}


struct OScDev_DeviceImpl OpenScan_NIFPGA_Device_Impl = {
	.GetModelName = NIFPGAGetModelName,
	.GetInstances = NIFPGAGetInstances,
	.ReleaseInstance = NIFPGAReleaseInstance,
	.GetName = NIFPGAGetName,
	.Open = NIFPGAOpen,
	.Close = NIFPGAClose,
	.HasClock = NIFPGAHasClock,
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
	.Arm = NIFPGAArm,
	.Start = NIFPGAStart,
	.Stop = NIFPGAStop,
	.IsRunning = NIFPGAIsRunning,
	.Wait = NIFPGAWait,
};


static OScDev_Error GetDeviceImpls(struct OScDev_DeviceImpl **impls, size_t *implCount)
{
	if (*implCount < 1)
		return OScDev_OK;

	impls[0] = &OpenScan_NIFPGA_Device_Impl;
	*implCount = 1;
	return OScDev_OK;
}


OScDev_MODULE_IMPL = {
	.displayName = "OpenScan NI-FPGA",
	.GetDeviceImpls = GetDeviceImpls,
};