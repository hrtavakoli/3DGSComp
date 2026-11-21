#pragma once

// Uniform affine quantization for GaussianCloud attributes.
//
// Each floating-point channel is independently mapped to an N-bit integer:
//
//   encode:  q = clamp( round( (v - lo) / (hi - lo) * (2^bits - 1) ), 0, 2^bits-1 )
//   decode:  v = q / (2^bits - 1) * (hi - lo) + lo
//
// Ranges are tracked per channel (not per attribute) so that attributes with
// independent dynamic ranges — e.g. X vs Y vs Z positions in a non-cubic scene —
// each use their full code space.

#include "gaussian_cloud.h"
#include <vector>

struct ChannelRange {
    float lo = 0.0f;
    float hi = 0.0f;
};

// One quantized attribute.
// data layout: point-major interleaved — data[i * numChannels + c] for point i, channel c.
// Storage type is uint16_t regardless of bit depth; actual precision is capped at `bits`.
struct QuantizedAttribute {
    int numChannels = 0;
    int bits        = 8;
    std::vector<ChannelRange> ranges; // one per channel
    std::vector<uint16_t>     data;   // numPoints * numChannels entries
};

struct QuantizedCloud {
    int32_t numPoints = 0;
    int32_t shDegree  = 0;
    int     bits      = 8;

    QuantizedAttribute positions;  // 3 channels
    QuantizedAttribute normals;    // 3 channels
    QuantizedAttribute colors;     // 3 channels
    QuantizedAttribute sh;         // shCoeffsPerPoint() channels
    QuantizedAttribute opacities;  // 1 channel
    QuantizedAttribute scales;     // 3 channels
    QuantizedAttribute rotations;  // 4 channels
};

// Per-attribute bit depths. Each field is clamped to [1, 16] at call time.
// Defaults reflect the perceptual importance of each attribute in 3DGS rendering:
// positions and rotations warrant higher precision; SH rest coefficients least.
struct AttributeBits {
    int positions = 16;
    int normals   =  8;
    int colors    =  4;   // SH DC term
    int sh        =  4;   // SH rest coefficients
    int opacities =  8;
    int scales    = 12;
    int rotations = 12;
};

// Uniform quantization: all attributes use the same bit depth. bits in [1, 16].
QuantizedCloud quantize(const GaussianCloud& cloud, int bits = 8);

// Per-attribute quantization: each attribute uses its own bit depth.
QuantizedCloud quantize(const GaussianCloud& cloud, const AttributeBits& ab);

GaussianCloud  dequantize(const QuantizedCloud& qcloud);

// Bit-packed byte estimate summed across attributes using each attribute's own bits field.
// Smaller than the actual uint16_t allocation (which is always 2 bytes/value).
size_t quantizedDataBytes(const QuantizedCloud& qcloud);
