#pragma once

#include "OpenScanLib.h"

#include <stdint.h>

static const uint32_t X_UNDERSHOOT = 50;
static const uint32_t X_RETRACE_LEN = 438;


OSc_Error GenerateScaledWaveforms(uint32_t resolution, double zoom, uint16_t *xScaled, uint16_t *yScaled,
	double galvoOffsetX, double galvoOffsetY);
void GenerateGalvoWaveform(int32_t effectiveScanLen, int32_t retraceLen,
	int32_t undershootLen, double scanStart, double scanEnd, double *waveform);
void SplineInterpolate(int32_t n, double yFirst, double yLast,
	double slopeFirst, double slopeLast, double* result);