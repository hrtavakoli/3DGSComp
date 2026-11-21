#pragma once

// Phase 2 of compression: spatial prediction before entropy coding.
//
// Step 1 — Morton sort: reorder Gaussians along the 3D Z-order curve so that
// spatially adjacent splats become consecutive in memory.
//
// Step 2 — Delta + zigzag: for each quantized channel, replace each integer
// code with zigzag(code[i] - code[i-1]).  Spatially-ordered splats have
// similar attribute values, so deltas cluster tightly around 0 — the resulting
// distribution is far more compressible than the raw codes.
//
// Zigzag mapping: d >= 0 → 2d,  d < 0 → -2d-1  (all non-negative output).
// Effective alphabet doubles: bits → bits+1.  The entropy coder (SCALE_BITS=12)
// therefore requires input quantization bits <= 11.

#include "quantization.h"
#include <vector>

// Sort cloud points in-place by 30-bit 3D Morton code (10 bits per axis).
// Returns the permutation: perm[i] = original index of the point now at i.
std::vector<uint32_t> mortonSort(GaussianCloud& cloud);

// Delta-encode all channels of attr across the point dimension.
// Output bits = attr.bits + 1 (zigzag expands the alphabet).
QuantizedAttribute deltaEncode(const QuantizedAttribute& attr, int32_t numPoints);

// Inverse: cumulative un-zigzag across points.
// Output bits = attr.bits - 1 (restores original quantization depth).
QuantizedAttribute deltaDecode(const QuantizedAttribute& attr, int32_t numPoints);

// Apply deltaEncode / deltaDecode to every attribute in a cloud.
QuantizedCloud deltaEncodeCloud(const QuantizedCloud& qc);
QuantizedCloud deltaDecodeCloud(const QuantizedCloud& qc);
