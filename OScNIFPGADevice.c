#include "OScNIFPGADevicePrivate.h"
#include "OScNIFPGA.h"

#include <math.h>


static OScDev_Error NIFPGAGetModelName(const char **name)
{
	*name = "OpenScan-NIFPGA";
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
	*hasClock = true;  // TODO: clock is not decoupled from scanner in current FPGA code
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


static OScDev_Error GetPixelRates(OScDev_Device *device, OScDev_NumRange **pixelRatesHz)
{
	// TODO There should be a comment here explaining how these values were chosen
	// (factors of 1.2 MHz???)
	static const double ratesHz[] = {
		100,
		200,
		250,
		500,
		1000,
		2500,
		5000,
		7500,
		10000,
		12500,
		15000,
		20000,
		25000,
		30000,
		40000,
		50000,
		60000,
		80000,
		100000,
		125000,
		150000,
		200000,
		250000,
		300000,
		350000,
		400000,
		500000,
		NAN // End mark
	};
	*pixelRatesHz = OScDev_NumRange_CreateDiscreteFromNaNTerminated(ratesHz);
	return OScDev_OK;
}


static OScDev_Error GetResolutions(OScDev_Device *device, OScDev_NumRange **resolutions)
{
	*resolutions = OScDev_NumRange_CreateDiscreteFromNaNTerminated((double[]) {
		256, 512, 1024, 2048, NAN
	});
	return OScDev_OK;
}


static OScDev_Error GetZoomFactors(OScDev_Device *device, OScDev_NumRange **zooms)
{
	*zooms = OScDev_NumRange_CreateContinuous(0.5, 40.0);
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
	*bytesPerSample = 2;
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

	OScDev_TriggerSource clockStartTriggerSource;
	OScDev_Acquisition_GetClockStartTriggerSource(acq, &clockStartTriggerSource);
	if (clockStartTriggerSource != OScDev_TriggerSource_Software)
		return OScDev_Error_Unsupported_Operation;

	OScDev_ClockSource clockSource;
	OScDev_Acquisition_GetClockSource(acq, &clockSource);
	if (clockSource != OScDev_ClockSource_Internal)
		return OScDev_Error_Unsupported_Operation;
	// what if we use external line clock to trigger acquisition?

	GetData(device)->detectorEnabled = useDetector;
	GetData(device)->scannerEnabled = useScanner;
	
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

		GetData(device)->acquisition.acquisition = acq;

		GetData(device)->acquisition.stopRequested = false;
		GetData(device)->acquisition.running = true;
		GetData(device)->acquisition.armed = false;
		GetData(device)->acquisition.started = false;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));

	double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
	uint32_t resolution = OScDev_Acquisition_GetResolution(acq);
	double zoomFactor = OScDev_Acquisition_GetZoomFactor(acq);
	if (pixelRateHz != GetData(device)->lastAcquisitionPixelRateHz ||
		resolution != GetData(device)->lastAcquisitionResolution ||
		zoomFactor != GetData(device)->lastAcquisitionZoomFactor) {
		GetData(device)->settingsChanged = true;
	}
	if (resolution != GetData(device)->lastAcquisitionResolution ||
		zoomFactor != GetData(device)->lastAcquisitionZoomFactor) {
		GetData(device)->reloadWaveformRequired = true;
	}

	uint32_t nFrames = OScDev_Acquisition_GetNumberOfFrames(acq);

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
		if (OScDev_CHECK(err, SetPixelParameters(device, pixelRateHz)))
			return err;

		OScDev_Log_Debug(device, "2. Clean flags...");
		if (OScDev_CHECK(err, Cleanflags(device)))
			return err;
		if (OScDev_CHECK(err, SetResolutionParameters(device, resolution)))
			return err;
		if (OScDev_CHECK(err, SetTaskParameters(device, nFrames)))
			return err;

		OScDev_Log_Debug(device, "3. Cleaning FPGA DRAM and initializing globals...");
		if (OScDev_CHECK(err, InitScan(device)))
			return err;
		if (OScDev_CHECK(err, WaitTillIdle(device)))
			return err;

		if (GetData(device)->reloadWaveformRequired)
		{
			if (OScDev_CHECK(err, ReloadWaveform(device, acq)))
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


OScDev_DeviceImpl OpenScan_NIFPGA_Device_Impl = {
	.GetModelName = NIFPGAGetModelName,
	.EnumerateInstances = EnumerateInstances,
	.ReleaseInstance = NIFPGAReleaseInstance,
	.GetName = NIFPGAGetName,
	.Open = NIFPGAOpen,
	.Close = NIFPGAClose,
	.HasClock = NIFPGAHasClock,
	.HasScanner = NIFPGAHasScanner,
	.HasDetector = NIFPGAHasDetector,
	.MakeSettings = MakeSettings,
	.GetPixelRates = GetPixelRates,
	.GetResolutions = GetResolutions,
	.GetZoomFactors = GetZoomFactors,
	.GetRasterWidths = GetResolutions, // No ROI support
	.GetRasterHeights = GetResolutions, // No ROI support
	.GetNumberOfChannels = NIFPGAGetNumberOfChannels,
	.GetBytesPerSample = NIFPGAGetBytesPerSample,
	.Arm = NIFPGAArm,
	.Start = NIFPGAStart,
	.Stop = NIFPGAStop,
	.IsRunning = NIFPGAIsRunning,
	.Wait = NIFPGAWait,
};


static OScDev_Error GetDeviceImpls(OScDev_PtrArray **deviceImpls)
{
	*deviceImpls = OScDev_PtrArray_Create();
	OScDev_PtrArray_Append(*deviceImpls, &OpenScan_NIFPGA_Device_Impl);
	return OScDev_OK;
}


OScDev_MODULE_IMPL = {
	.displayName = "OpenScan NI-FPGA",
	.GetDeviceImpls = GetDeviceImpls,
};