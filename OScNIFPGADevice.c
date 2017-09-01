#include "OScNIFPGADevicePrivate.h"
#include "OScNIFPGA.h"
#include "OpenScanLibPrivate.h"


static OSc_Device **g_devices;
static size_t g_deviceCount;


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
	device->implData = NULL;
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
	return CloseFPGA(device);
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


static OSc_Error NIFPGAGetSettings(OSc_Device *device, OSc_Setting ***settings)
{
	OSc_Return_If_Error(PrepareSettings(device));
	*settings = GetData(device)->settings;
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
	*nChannels = GetData(device)->channels == CHANNELS_RAW_AND_KALMAN ? 2 : 1;
	return OSc_Error_OK;
}


static OSc_Error NIFPGAGetBytesPerSample(OSc_Device *device, uint32_t *bytesPerSample)
{
	*bytesPerSample = 2;
	return OSc_Error_OK;
}


static OSc_Error ArmImpl(OSc_Device *device, OSc_Acquisition *acq)
{
	InitializeCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		if (GetData(device)->acquisition.running &&
			GetData(device)->acquisition.armed)
		{
			DeleteCriticalSection(&(GetData(device)->acquisition.mutex));
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
	DeleteCriticalSection(&(GetData(device)->acquisition.mutex));

	if (GetData(device)->settingsChanged)
	{
		OSc_Return_If_Error(ReloadWaveform(device));
		GetData(device)->settingsChanged = false;
	}

	OSc_Return_If_Error(SetScanParameters(device));

	InitializeCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		GetData(device)->acquisition.armed = true;
	}
	DeleteCriticalSection(&(GetData(device)->acquisition.mutex));

	return OSc_Error_OK;
}


static OSc_Error NIFPGAArmScanner(OSc_Device *device, OSc_Acquisition *acq)
{
	return ArmImpl(device, acq);
}


static OSc_Error NIFPGAStartScanner(OSc_Device *device, OSc_Acquisition *acq)
{
	InitializeCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		if (!GetData(device)->acquisition.running ||
			!GetData(device)->acquisition.armed)
		{
			DeleteCriticalSection(&(GetData(device)->acquisition.mutex));
			return OSc_Error_Not_Armed;
		}
		if (GetData(device)->acquisition.started)
		{
			DeleteCriticalSection(&(GetData(device)->acquisition.mutex));
			return OSc_Error_Acquisition_Running;
		}

		GetData(device)->acquisition.started = true;
	}
	DeleteCriticalSection(&(GetData(device)->acquisition.mutex));

	return RunAcquisitionLoop(device, acq);
}


static OSc_Error NIFPGAStopScanner(OSc_Device *device, OSc_Acquisition *acq)
{
	return StopAcquisition(device, acq, true);
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
	return StopAcquisition(device, acq, true);
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
	.GetImageSize = NIFPGAGetImageSize,
	.GetNumberOfChannels = NIFPGAGetNumberOfChannels,
	.GetBytesPerSample = NIFPGAGetBytesPerSample,
	.ArmScanner = NIFPGAArmScanner,
	.StartScanner = NIFPGAStartScanner,
	.StopScanner = NIFPGAStopScanner,
	.ArmDetector =  NIFPGAArmDetector,
	.StartDetector = NIFPGAStartDetector,
	.StopDetector = NIFPGAStopDetector,
};