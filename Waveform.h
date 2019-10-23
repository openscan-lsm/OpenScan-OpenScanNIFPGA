#pragma once

#include <stdint.h>

static const uint32_t X_RETRACE_LEN = 128;
static const uint32_t Y_RETRACE_LEN = 16;


int GenerateScaledWaveforms(uint32_t resolution, double zoom, uint32_t lineDelay, uint16_t *xScaled, uint16_t *yScaled,
	double galvoOffsetX, double galvoOffsetY);
void GenerateGalvoWaveform(int32_t effectiveScanLen, int32_t retraceLen,
	int32_t undershootLen, double scanStart, double scanEnd, double *waveform);
void SplineInterpolate(int32_t n, double yFirst, double yLast,
	double slopeFirst, double slopeLast, double* result);
// int SaveWaveformData(uint16_t *xScaled, uint16_t *yScaled, 
//	uint16_t elementsPerLine, uint16_t elementsPerRow);