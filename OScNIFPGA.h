#pragma once

#include "OScNIFPGADevicePrivate.h"

OScDev_Error EnumerateInstances(OScDev_Device ***devices, size_t *deviceCount);
OScDev_Error OpenFPGA(OScDev_Device *device);
OScDev_Error CloseFPGA(OScDev_Device *device);
OScDev_Error StartFPGA(OScDev_Device *device);
OScDev_Error SetScanParameters(OScDev_Device *device);
OScDev_Error ReloadWaveform(OScDev_Device *device);
OScDev_Error WaitTillIdle(OScDev_Device *device);
OScDev_Error SetBuildInParameters(OScDev_Device *device);
OScDev_Error SetPixelParameters(OScDev_Device *device, double scanRate);
OScDev_Error SetResolutionParameters(OScDev_Device *device, uint32_t resolution);
OScDev_Error SetTaskParameters(OScDev_Device *device, uint32_t nf);
OScDev_Error Cleanflags(OScDev_Device *device);
OScDev_Error InitScan(OScDev_Device *device);
OScDev_Error RunAcquisitionLoop(OScDev_Device *device);
OScDev_Error StopAcquisitionAndWait(OScDev_Device *device);
OScDev_Error IsAcquisitionRunning(OScDev_Device *device, bool *isRunning);
OScDev_Error WaitForAcquisitionToFinish(OScDev_Device *device);
