#pragma once

// Pre-quantization transforms with individual feature flags.
//
// Each flag is independently togglable. When a flag is on, the corresponding
// transform modifies the GaussianCloud representation before quantization and
// must be inverted after dequantization to recover the original float values.
//
// Current transforms:
//
//   smallestThree — rotation quaternion compression
//     A unit quaternion (w,x,y,z) has 4 components but only 3 degrees of
//     freedom. Drop the component with the largest absolute value (call its
//     index k), store only the remaining 3, and record k in a 2-bit sidecar.
//     The dropped component is always made positive (canonical form: q and -q
//     are the same rotation) and reconstructed at decode from the unit-norm
//     constraint: q_k = sqrt(1 - a² - b² - c²).
//
//     Benefit: rotations shrink from 4 channels to 3 channels (-25%), and the
//     stored values lie in [-1/sqrt(2), 1/sqrt(2)]; scaled by sqrt(2) before
//     quantization this maps to [-1,1], giving ~41% finer quantization steps
//     for the same bit depth vs the raw quaternion range.

#include "gaussian_cloud.h"
#include <cstdint>
#include <vector>

struct PreprocessFlags {
    bool smallestThree = false; // rotations: 4-ch quaternion → 3-ch smallest-3
};

// A GaussianCloud with any active transforms already applied, plus the sidecar
// metadata needed to invert them. Pass this to quantize() in place of the
// original cloud, then call deprocess() after dequantize().
struct PreprocessedCloud {
    GaussianCloud        cloud;    // rotations is 3×N when smallestThree, else 4×N
    std::vector<uint8_t> rotDrop;  // per-point dropped index 0–3; empty if !smallestThree
    PreprocessFlags      flags;
};

// Apply all active transforms to cloud. Returns the modified cloud + sidecar.
PreprocessedCloud preprocess(const GaussianCloud& cloud, const PreprocessFlags& flags);

// Invert all transforms, recovering the original float representation.
GaussianCloud deprocess(const PreprocessedCloud& pc);
