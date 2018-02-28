#include "OScNIFPGADevicePrivate.h"
#include "OpenScanLibPrivate.h"

#include "NiFpga_OpenScanFPGAHost.h"

#include <NiFpga.h>

#include <string.h>


static OSc_Error GetScanRate(OSc_Setting *setting, double *value)
{
	*value = GetData(setting->device)->scanRate;
	return OSc_Error_OK;
}


static OSc_Error SetScanRate(OSc_Setting *setting, double value)
{
	GetData(setting->device)->scanRate = value;
	return OSc_Error_OK;
}


static OSc_Error GetScanRateValues(OSc_Setting *setting, double **values, size_t *count)
{
	static double v[] = {
		0.05,
		0.08,
		0.10,
		0.12,
		0.15,
		0.20,
		0.25,
		0.30,
		0.35,
		0.40,
		0.50,
	};
	*values = v;
	*count = sizeof(v) / sizeof(double);
	return OSc_Error_OK;
}


static struct OSc_Setting_Impl SettingImpl_ScanRate = {
	.GetFloat64 = GetScanRate,
	.SetFloat64 = SetScanRate,
	.GetNumericConstraintType = OSc_Setting_NumericConstraintDiscreteValues,
	.GetFloat64DiscreteValues = GetScanRateValues,
};


static OSc_Error GetZoom(OSc_Setting *setting, double *value)
{
	*value = GetData(setting->device)->zoom;
	return OSc_Error_OK;
}


static OSc_Error SetZoom(OSc_Setting *setting, double value)
{
	GetData(setting->device)->zoom = value;
	return OSc_Error_OK;
}


static OSc_Error GetZoomRange(OSc_Setting *setting, double *min, double *max)
{
	*min = 1.0;
	*max = 40.0;
	return OSc_Error_OK;
}


static struct OSc_Setting_Impl SettingImpl_Zoom = {
	.GetFloat64 = GetZoom,
	.SetFloat64 = SetZoom,
	.GetNumericConstraintType = OSc_Setting_NumericConstraintRange,
	.GetFloat64Range = GetZoomRange,
};


static OSc_Error GetOffset(OSc_Setting *setting, double *value)
{
	*value = GetData(setting->device)->offsetXY[(intptr_t)(setting->implData)];
	return OSc_Error_OK;
}


static OSc_Error SetOffset(OSc_Setting *setting, double value)
{
	GetData(setting->device)->offsetXY[(intptr_t)(setting->implData)] = value;
	return OSc_Error_OK;
}


static OSc_Error GetOffsetRange(OSc_Setting *setting, double *min, double *max)
{
	/*The galvoOffsetX and galvoOffsetY variables are expressed  in optical degrees
	This is a rough correspondence - it likely needs to be calibrated to the actual
	sensitivity of the galvos*/
	*min = -10.0;
	*max = +10.0;
	return OSc_Error_OK;
}


static struct OSc_Setting_Impl SettingImpl_Offset = {
	.GetFloat64 = GetOffset,
	.SetFloat64 = SetOffset,
	.GetNumericConstraintType = OSc_Setting_NumericConstraintRange,
	.GetFloat64Range = GetOffsetRange,
};


static OSc_Error GetChannels(OSc_Setting *setting, uint32_t *value)
{
	*value = GetData(setting->device)->channels;
	return OSc_Error_OK;
}


static OSc_Error SetChannels(OSc_Setting *setting, uint32_t value)
{
	GetData(setting->device)->channels = value;
	return OSc_Error_OK;
}


static OSc_Error GetChannelsNumValues(OSc_Setting *setting, uint32_t *count)
{
	*count = CHANNELS_NUM_VALUES;
	return OSc_Error_OK;
}


static OSc_Error GetChannelsNameForValue(OSc_Setting *setting, uint32_t value, char *name)
{
	switch (value)
	{
	case CHANNELS_1_:
		strcpy(name, "Channel 1");
		break;
	case CHANNELS_2_:
		strcpy(name, "Channel 1-2");
		break;
	case CHANNELS_3_:
		strcpy(name, "Channel 1-3");
		break;
	case CHANNELS_4_:
		strcpy(name, "Channel 1-4");
		break;
	default:
		strcpy(name, "");
		return OSc_Error_Unknown;
	}
	return OSc_Error_OK;
}


static OSc_Error GetChannelsValueForName(OSc_Setting *setting, uint32_t *value, const char *name)
{
	if (!strcmp(name, "Channel 1"))
		*value = CHANNELS_1_;
	else if (!strcmp(name, "Channel 1-2"))
		*value = CHANNELS_2_;
	else if (!strcmp(name, "Channel 1-3"))
		*value = CHANNELS_3_;
	else if (!strcmp(name, "Channel 1-4"))
		*value = CHANNELS_4_;
	else
		return OSc_Error_Unknown;
	return OSc_Error_OK;
}


static struct OSc_Setting_Impl SettingImpl_Channels = {
	.GetEnum = GetChannels,
	.SetEnum = SetChannels,
	.GetEnumNumValues = GetChannelsNumValues,
	.GetEnumNameForValue = GetChannelsNameForValue,
	.GetEnumValueForName = GetChannelsValueForName,
};


static OSc_Error GetKalmanProgressive(OSc_Setting *setting, bool *value)
{
	*value = GetData(setting->device)->kalmanProgressive;
	return OSc_Error_OK;
}


static OSc_Error SetKalmanProgressive(OSc_Setting *setting, bool value)
{
	GetData(setting->device)->kalmanProgressive = value;
	return OSc_Error_OK;
}


static struct OSc_Setting_Impl SettingImpl_KalmanProgressive = {
	.GetBool = GetKalmanProgressive,
	.SetBool = SetKalmanProgressive,
};


static OSc_Error GetFilterGain(OSc_Setting *setting, double *value)
{
	*value = GetData(setting->device)->filterGain;
	return OSc_Error_OK;
}


static OSc_Error SetFilterGain(OSc_Setting *setting, double value)
{
	GetData(setting->device)->filterGain = value;
	return OSc_Error_OK;
}


static OSc_Error GetFilterGainRange(OSc_Setting *setting, double *min, double *max)
{
	*min = 0.0;
	*max = 1.0;
	return OSc_Error_OK;
}


static struct OSc_Setting_Impl SettingImpl_FilterGain = {
	.GetFloat64 = GetFilterGain,
	.SetFloat64 = SetFilterGain,
	.GetNumericConstraintType = OSc_Setting_NumericConstraintRange,
	.GetFloat64Range = GetFilterGainRange,
};


static OSc_Error GetKalmanFrames(OSc_Setting *setting, int32_t *value)
{
	*value = GetData(setting->device)->kalmanFrames;
	return OSc_Error_OK;
}


static OSc_Error SetKalmanFrames(OSc_Setting *setting, int32_t value)
{
	GetData(setting->device)->kalmanFrames = value;
	return OSc_Error_OK;
}


static OSc_Error GetKalmanFramesRange(OSc_Setting *setting, int32_t *min, int32_t *max)
{
	*min = 1;
	*max = 100;
	return OSc_Error_OK;
}


static struct OSc_Setting_Impl SettingImpl_KalmanFrames = {
	.GetInt32 = GetKalmanFrames,
	.SetInt32 = SetKalmanFrames,
	.GetNumericConstraintType = OSc_Setting_NumericConstraintRange,
	.GetInt32Range = GetKalmanFramesRange,
};


OSc_Error PrepareSettings(OSc_Device *device)
{
	if (GetData(device)->settings)
		return OSc_Error_OK;

	OSc_Setting *scanRate;
	OSc_Return_If_Error(OSc_Setting_Create(&scanRate, device, "ScanRate", OSc_Value_Type_Float64,
		&SettingImpl_ScanRate, NULL));

	OSc_Setting *zoom;
	OSc_Return_If_Error(OSc_Setting_Create(&zoom, device, "Zoom", OSc_Value_Type_Float64,
		&SettingImpl_Zoom, NULL));

	OSc_Setting *offsetX;
	OSc_Return_If_Error(OSc_Setting_Create(&offsetX, device, "GalvoOffsetX", OSc_Value_Type_Float64,
		&SettingImpl_Offset, (void *)0));

	OSc_Setting *offsetY;
	OSc_Return_If_Error(OSc_Setting_Create(&offsetY, device, "GalvoOffsetY", OSc_Value_Type_Float64,
		&SettingImpl_Offset, (void *)1));

	OSc_Setting *channels;
	OSc_Return_If_Error(OSc_Setting_Create(&channels, device, "Channels", OSc_Value_Type_Enum,
		&SettingImpl_Channels, NULL));

	OSc_Setting *kalmanProgressive;
	OSc_Return_If_Error(OSc_Setting_Create(&kalmanProgressive, device, "KalmanAveragingProgressive", OSc_Value_Type_Bool,
		&SettingImpl_KalmanProgressive, NULL));

	OSc_Setting *filterGain;
	OSc_Return_If_Error(OSc_Setting_Create(&filterGain, device, "KalmanAveragingFilterGain", OSc_Value_Type_Float64,
		&SettingImpl_FilterGain, NULL));

	OSc_Setting *kalmanFrames;
	OSc_Return_If_Error(OSc_Setting_Create(&kalmanFrames, device, "KalmanAverageFrames", OSc_Value_Type_Int32,
		&SettingImpl_KalmanFrames, NULL));

	OSc_Setting *ss[] = {
		scanRate, zoom, offsetX, offsetY,
		channels, kalmanProgressive, filterGain, kalmanFrames,
	};
	size_t nSettings = sizeof(ss) / sizeof(OSc_Setting *);
	OSc_Setting **settings = malloc(sizeof(ss));
	memcpy(settings, ss, sizeof(ss));

	GetData(device)->settings = settings;
	GetData(device)->settingCount = nSettings;
	return OSc_Error_OK;
}