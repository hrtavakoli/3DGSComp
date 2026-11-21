#pragma once

// Entropy coding layer (rANS) over QuantizedAttribute / QuantizedCloud.
//
// Each channel is entropy-coded independently with its own frequency model.
// The model precision is chosen per attribute as scaleBits = min(bits + 4, 16),
// giving 2^scaleBits probability slots. This works for any quantization bit
// depth in [1, 16]; higher bit depths automatically get a larger model.
//
// The frequency tables stored in EntropyAttribute are all a decoder needs
// (along with the bitstream) to reconstruct the original integer codes exactly.
// Entropy coding is lossless over the quantized codes.

#include "quantization.h"
#include <vector>

// Per-attribute entropy-coded data.
// freqTables[c][s]  = normalized frequency of symbol s in channel c (sums to 2^scaleBits)
// cumulTables[c][s] = cumulative frequency strictly before symbol s
// bitstreams[c]     = rANS-compressed byte stream for channel c
struct EntropyAttribute {
    int numChannels = 0;
    int bits        = 0;
    int scaleBits   = 12; // model precision: min(bits + 4, 16); decoder must match encoder
    std::vector<ChannelRange>          ranges;
    std::vector<std::vector<uint32_t>> freqTables;
    std::vector<std::vector<uint32_t>> cumulTables;
    std::vector<std::vector<uint8_t>>  bitstreams;
};

struct EntropyCloud {
    int32_t numPoints = 0;
    int32_t shDegree  = 0;
    int     bits      = 0;

    EntropyAttribute positions, normals, colors, sh, opacities, scales, rotations;
};

// qc attribute bits must be in [1, 16].
EntropyCloud   entropyEncode(const QuantizedCloud& qc);
QuantizedCloud entropyDecode(const EntropyCloud& ec);

// Sum of all per-channel bitstream sizes in bytes (excludes frequency table metadata).
size_t entropyBytes(const EntropyCloud& ec);
