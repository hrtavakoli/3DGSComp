#pragma once

// Binary serialization of EntropyCloud + preprocessing state to/from .3dgc files.
//
// File layout (all values little-endian):
//
//   Header (14 bytes):
//     magic[4]        "3DGC"
//     version[1]      = 1
//     numPoints[4]    int32
//     shDegree[4]     int32
//     preprocFlags[1] uint8  — bit 0: smallestThree rotation encoding active
//
//   7 attribute blocks in order:
//     positions, normals, colors, sh, opacities, scales, rotations
//
//   Per attribute block:
//     bits[1]        uint8   — quantization bit depth
//     scaleBits[1]   uint8   — rANS model bits (min(bits+4, 16))
//     numChannels[1] uint8   — 3 for rotations when smallestThree is active
//     Per channel:
//       lo[4]         float32
//       hi[4]         float32
//       numActive[4]  uint32  — symbols with freq > 0
//       Per active symbol (in symbol order):
//         symbol[2]   uint16
//         freq[4]     uint32
//       bitstreamSize[4] uint32
//       bitstream[bitstreamSize] bytes
//
//   Preprocessing sidecar (present only when preprocFlags & 0x01):
//     rotDrop[numPoints] uint8  — per-point dropped quaternion index (0–3)

#include "entropy.h"
#include "preprocess.h"
#include <string>
#include <vector>

// Returned by readEntropyCloud; carries the entropy cloud plus any
// preprocessing state needed to call deprocess() after dequantize().
struct ReadResult {
    EntropyCloud         ec;
    PreprocessFlags      flags;
    std::vector<uint8_t> rotDrop;
};

// Write an entropy-coded cloud without preprocessing metadata.
void writeEntropyCloud(const EntropyCloud& ec, const std::string& path);

// Write an entropy-coded cloud with preprocessing metadata (rotDrop etc.).
void writeEntropyCloud(const EntropyCloud& ec,
                       const PreprocessedCloud& preproc,
                       const std::string& path);

// Read a .3dgc file. The returned ReadResult always contains a valid ec; the
// flags and rotDrop fields are populated only when the file was written with
// preprocessing active.
ReadResult readEntropyCloud(const std::string& path);
