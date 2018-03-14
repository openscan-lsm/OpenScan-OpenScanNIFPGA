#pragma once

#include "OScNIFPGADevicePrivate.h"

OSc_Error EnumerateInstances(OSc_Device ***devices, size_t *deviceCount);
OSc_Error OpenFPGA(OSc_Device *device);
OSc_Error CloseFPGA(OSc_Device *device);
OSc_Error StartFPGA(OSc_Device *device);
OSc_Error SetScanParameters(OSc_Device *device);
OSc_Error ReloadWaveform(OSc_Device *device);
OSc_Error WaitTillIdle(OSc_Device *device);
OSc_Error SetBuildInParameters(OSc_Device *device);
OSc_Error SetPixelParameters(OSc_Device *device, double scanRate);
OSc_Error SetResolutionParameters(OSc_Device *device, uint32_t resolution);
OSc_Error SetTaskParameters(OSc_Device *device, uint32_t nf);
OSc_Error Cleanflags(OSc_Device *device);
OSc_Error InitScan(OSc_Device *device);
OSc_Error RunAcquisitionLoop(OSc_Device *device, OSc_Acquisition *acq);
OSc_Error StopAcquisitionAndWait(OSc_Device *device, OSc_Acquisition *acq);
OSc_Error IsAcquisitionRunning(OSc_Device *device, bool *isRunning);
OSc_Error WaitForAcquisitionToFinish(OSc_Device *device);