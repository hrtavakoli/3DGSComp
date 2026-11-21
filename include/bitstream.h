#pragma once

// NAL-unit structured bitstream for 3DGS compressed scenes.
//
// File layout:
//
//   START unit   — file signature (magic + version), zero payload
//   SPS unit     — Splat Parameter Set: all codec configuration
//   CAU × 7      — Compressed Attribute Unit, one per attribute type
//   SIDECAR      — preprocessing sidecar data (optional)
//   END unit     — end of stream, zero payload
//
// Unit framing (all non-START units):
//   [1B] nal_type      uint8
//   [4B] payload_size  uint32   — bytes of payload that follow
//   [payload_size B]   payload
//
// Unknown unit types are skipped via payload_size — forward-compatible.

#include "entropy.h"
#include "preprocess.h"
#include <cstdint>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

enum class NalType : uint8_t {
    START   = 0x01,  // file signature; magic + version precede framing, zero payload
    SPS     = 0x02,  // Splat Parameter Set
    CAU     = 0x10,  // Compressed Attribute Unit
    SIDECAR = 0x20,  // preprocessing sidecar data
    END     = 0xFF,  // end of stream, zero payload
};

// Data type identifier carried in every CAU. First byte of CAU payload.
// The (CauType, sequence_num) pair uniquely addresses a CAU.
enum class CauType : uint8_t {
    POSITIONS = 0x00,
    NORMALS   = 0x01,
    COLORS    = 0x02,
    SH        = 0x03,
    OPACITIES = 0x04,
    SCALES    = 0x05,
    ROTATIONS = 0x06,
};

enum class SidecarType : uint8_t {
    ROT_DROP = 0x00,  // per-point dropped quaternion index (smallestThree)
};

enum class QuantScheme : uint8_t {
    NONE     = 0,
    UNIFORM  = 1,
    PER_ATTR = 2,
};

enum class EntropyScheme : uint8_t {
    NONE = 0,
    RANS = 1,
};

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

struct StreamReadResult {
    EntropyCloud         ec;
    PreprocessFlags      flags;
    std::vector<uint8_t> rotDrop;
};

// Write a NAL-structured bitstream. The preprocessing overload embeds the
// PreprocessedCloud flags and rotDrop sidecar in the file.
void writeStream(const EntropyCloud& ec,
                 const PreprocessedCloud& preproc,
                 const std::string& path);

void writeStream(const EntropyCloud& ec, const std::string& path);

// Parse a NAL-structured bitstream and return the entropy cloud with any
// preprocessing metadata needed for deprocess().
StreamReadResult readStream(const std::string& path);
