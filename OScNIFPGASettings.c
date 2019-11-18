#include "OScNIFPGADevicePrivate.h"

#include "NiFpga_OpenScanFPGAHost.h"

#include <NiFpga.h>

#include <string.h>


// For most cases, we set the setting's implData to the device.
// This function can then be used to retrieve the device implData.
static inline struct OScNIFPGAPrivateData *GetSettingDeviceData(OScDev_Setting *setting)
{
	return (struct OScNIFPGAPrivateData *)OScDev_Device_GetImplData((OScDev_Device *)OScDev_Setting_GetImplData(setting));
}


static OScDev_Error GetNumericConstraintTypeImpl_Range(OScDev_Setting *setting, OScDev_ValueConstraint *constraintType)
{
	*constraintType = OScDev_ValueConstraint_Range;
	return OScDev_OK;
}


static OScDev_Error GetLineDelay(OScDev_Setting *setting, int32_t *value)
{
	*value = GetSettingDeviceData(setting)->lineDelay;

	return OScDev_OK;
}


static OScDev_Error SetLineDelay(OScDev_Setting *setting, int32_t value)
{
	GetSettingDeviceData(setting)->lineDelay = value;

	GetSettingDeviceData(setting)->settingsChanged = true;
	GetSettingDeviceData(setting)->reloadWaveformRequired = true;

	return OScDev_OK;
}


static OScDev_Error GetLineDelayRange(OScDev_Setting *setting, int32_t *min, int32_t *max)
{
	*min = 1;
	*max = 200;
	return OScDev_OK;
}


static struct OScDev_SettingImpl SettingImpl_LineDelay = {
	.GetInt32 = GetLineDelay,
	.SetInt32 = SetLineDelay,
	.GetNumericConstraintType = GetNumericConstraintTypeImpl_Range,
	.GetInt32Range = GetLineDelayRange,
};


struct OffsetSettingData
{
	OScDev_Device *device;
	int axis; // 0 = x, 1 = y
};


static OScDev_Error GetOffset(OScDev_Setting *setting, double *value)
{
	struct OffsetSettingData *data = (struct OffsetSettingData *)OScDev_Setting_GetImplData(setting);
	*value = GetData(data->device)->offsetXY[data->axis];
	return OScDev_OK;
}


static OScDev_Error SetOffset(OScDev_Setting *setting, double value)
{
	struct OffsetSettingData *data = (struct OffsetSettingData *)OScDev_Setting_GetImplData(setting);
	GetData(data->device)->offsetXY[data->axis] = value;
	GetData(data->device)->settingsChanged = true;
	GetData(data->device)->reloadWaveformRequired = true;
	return OScDev_OK;
}


static OScDev_Error GetOffsetRange(OScDev_Setting *setting, double *min, double *max)
{
	/*The galvoOffsetX and galvoOffsetY variables are expressed  in optical degrees
	This is a rough correspondence - it likely needs to be calibrated to the actual
	sensitivity of the galvos*/
	*min = -10.0;
	*max = +10.0;
	return OScDev_OK;
}


static void ReleaseOffset(OScDev_Setting *setting)
{
	struct OffsetSettingData *data = OScDev_Setting_GetImplData(setting);
	free(data);
}


static OScDev_SettingImpl SettingImpl_Offset = {
	.GetFloat64 = GetOffset,
	.SetFloat64 = SetOffset,
	.GetNumericConstraintType = GetNumericConstraintTypeImpl_Range,
	.GetFloat64Range = GetOffsetRange,
	.Release = ReleaseOffset,
};


static OScDev_Error GetChannels(OScDev_Setting *setting, uint32_t *value)
{
	*value = GetSettingDeviceData(setting)->channels;
	return OScDev_OK;
}


static OScDev_Error SetChannels(OScDev_Setting *setting, uint32_t value)
{
	GetSettingDeviceData(setting)->channels = value;
	return OScDev_OK;
}


static OScDev_Error GetChannelsNumValues(OScDev_Setting *setting, uint32_t *count)
{
	*count = CHANNELS_NUM_VALUES;
	return OScDev_OK;
}


static OScDev_Error GetChannelsNameForValue(OScDev_Setting *setting, uint32_t value, char *name)
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
		return OScDev_Error_Unknown;
	}
	return OScDev_OK;
}


static OScDev_Error GetChannelsValueForName(OScDev_Setting *setting, uint32_t *value, const char *name)
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
		return OScDev_Error_Unknown;
	return OScDev_OK;
}


static OScDev_SettingImpl SettingImpl_Channels = {
	.GetEnum = GetChannels,
	.SetEnum = SetChannels,
	.GetEnumNumValues = GetChannelsNumValues,
	.GetEnumNameForValue = GetChannelsNameForValue,
	.GetEnumValueForName = GetChannelsValueForName,
};


static OScDev_Error GetKalmanProgressive(OScDev_Setting *setting, bool *value)
{
	*value = GetSettingDeviceData(setting)->kalmanProgressive;
	return OScDev_OK;
}


static OScDev_Error SetKalmanProgressive(OScDev_Setting *setting, bool value)
{
	GetSettingDeviceData(setting)->kalmanProgressive = value;
	return OScDev_OK;
}


static OScDev_SettingImpl SettingImpl_KalmanProgressive = {
	.GetBool = GetKalmanProgressive,
	.SetBool = SetKalmanProgressive,
};

static OScDev_Error GetScannerEnabled(OScDev_Setting *setting, bool *value)
{
	*value = GetSettingDeviceData(setting)->scannerEnabled;
	return OScDev_OK;
}


static OScDev_Error SetScannerEnabled(OScDev_Setting *setting, bool value)
{
	GetSettingDeviceData(setting)->scannerEnabled = value;
	GetSettingDeviceData(setting)->settingsChanged = true;
	GetSettingDeviceData(setting)->reloadWaveformRequired = true;
	return OScDev_OK;
}


static OScDev_SettingImpl SettingImpl_ScannerEnabled = {
	.GetBool = GetScannerEnabled,
	.SetBool = SetScannerEnabled,
};

static OScDev_Error GetDetectorEnabled(OScDev_Setting *setting, bool *value)
{
	*value = GetSettingDeviceData(setting)->detectorEnabled;
	return OScDev_OK;
}


static OScDev_Error SetDetectorEnabled(OScDev_Setting *setting, bool value)
{
	GetSettingDeviceData(setting)->detectorEnabled = value;
	GetSettingDeviceData(setting)->settingsChanged = true;
	GetSettingDeviceData(setting)->reloadWaveformRequired = true;
	return OScDev_OK;
}


static OScDev_SettingImpl SettingImpl_DetectorEnabled = {
	.GetBool = GetDetectorEnabled,
	.SetBool = SetDetectorEnabled,
};


static OScDev_Error GetFilterGain(OScDev_Setting *setting, double *value)
{
	*value = GetSettingDeviceData(setting)->filterGain;
	return OScDev_OK;
}


static OScDev_Error SetFilterGain(OScDev_Setting *setting, double value)
{
	GetSettingDeviceData(setting)->filterGain = (uint16_t) value;
	return OScDev_OK;
}


static OScDev_Error GetFilterGainRange(OScDev_Setting *setting, double *min, double *max)
{
	*min = 0.0;
	*max = 1.0;
	return OScDev_OK;
}


static OScDev_SettingImpl SettingImpl_FilterGain = {
	.GetFloat64 = GetFilterGain,
	.SetFloat64 = SetFilterGain,
	.GetNumericConstraintType = GetNumericConstraintTypeImpl_Range,
	.GetFloat64Range = GetFilterGainRange,
};


static OScDev_Error GetKalmanFrames(OScDev_Setting *setting, int32_t *value)
{
	*value = GetSettingDeviceData(setting)->kalmanFrames;
	return OScDev_OK;
}


static OScDev_Error SetKalmanFrames(OScDev_Setting *setting, int32_t value)
{
	GetSettingDeviceData(setting)->kalmanFrames = value;
	GetSettingDeviceData(setting)->settingsChanged = true;
	return OScDev_OK;
}


static OScDev_Error GetKalmanFramesRange(OScDev_Setting *setting, int32_t *min, int32_t *max)
{
	*min = 1;
	*max = 100;
	return OScDev_OK;
}


static OScDev_SettingImpl SettingImpl_KalmanFrames = {
	.GetInt32 = GetKalmanFrames,
	.SetInt32 = SetKalmanFrames,
	.GetNumericConstraintType = GetNumericConstraintTypeImpl_Range,
	.GetInt32Range = GetKalmanFramesRange,
};


OScDev_Error MakeSettings(OScDev_Device *device, OScDev_PtrArray **settings)
{
	OScDev_Error err;
	*settings = OScDev_PtrArray_Create();

	OScDev_Setting *lineDelay;
	if (OScDev_CHECK(err, OScDev_Setting_Create(&lineDelay, "Line Delay (pixels)", OScDev_ValueType_Int32,
		&SettingImpl_LineDelay, device)))
		goto error;
	OScDev_PtrArray_Append(*settings, lineDelay);

	for (int i = 0; i < 2; ++i)
	{
		OScDev_Setting *offset;
		struct OffsetSettingData *data = malloc(sizeof(struct OffsetSettingData));
		data->device = device;
		data->axis = i;
		const char *name = i == 0 ? "GalvoOffsetX" : "GalvoOffsetY";
		if (OScDev_CHECK(err, OScDev_Setting_Create(&offset, name,
			OScDev_ValueType_Float64, &SettingImpl_Offset, data)))
			goto error;
		OScDev_PtrArray_Append(*settings, offset);
	}

	OScDev_Setting *channels;
	if (OScDev_CHECK(err, OScDev_Setting_Create(&channels, "Channels",
		OScDev_ValueType_Enum, &SettingImpl_Channels, device)))
		goto error;
	OScDev_PtrArray_Append(*settings, channels);

	OScDev_Setting *kalmanProgressive;
	if (OScDev_CHECK(err, OScDev_Setting_Create(&kalmanProgressive,
		"KalmanAveragingProgressive", OScDev_ValueType_Bool, &SettingImpl_KalmanProgressive, device)))
		goto error;
	OScDev_PtrArray_Append(*settings, kalmanProgressive);

	OScDev_Setting *scannerEnabled;
	if (OScDev_CHECK(err, OScDev_Setting_Create(&scannerEnabled,
		"EnableScanner", OScDev_ValueType_Bool, &SettingImpl_ScannerEnabled, device)))
		goto error;
	OScDev_PtrArray_Append(*settings, scannerEnabled);

	OScDev_Setting *detectorEnabled;
	if (OScDev_CHECK(err, OScDev_Setting_Create(&detectorEnabled,
		"EnableDetector", OScDev_ValueType_Bool, &SettingImpl_DetectorEnabled, device)))
		goto error;
	OScDev_PtrArray_Append(*settings, detectorEnabled);

	OScDev_Setting *filterGain;
	if (OScDev_CHECK(err, OScDev_Setting_Create(&filterGain,
		"KalmanAveragingFilterGain", OScDev_ValueType_Float64, &SettingImpl_FilterGain, device)))
		goto error;
	OScDev_PtrArray_Append(*settings, filterGain);

	OScDev_Setting *kalmanFrames;
	if (OScDev_CHECK(err, OScDev_Setting_Create(&kalmanFrames,
		"KalmanAverageFrames", OScDev_ValueType_Int32, &SettingImpl_KalmanFrames, device)))
		goto error;
	OScDev_PtrArray_Append(*settings, kalmanFrames);

	return OScDev_OK;

error:
	for (size_t i = 0; i < OScDev_PtrArray_Size(*settings); ++i) {
		OScDev_Setting_Destroy(OScDev_PtrArray_At(*settings, i));
	}
	OScDev_PtrArray_Destroy(*settings);
	*settings = NULL;
	return err;
}
