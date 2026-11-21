#include "quantization.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static QuantizedAttribute quantizeAttribute(
    const float* data,
    int32_t      numPoints,
    int          numChannels,
    int          bits)
{
    assert(bits >= 1 && bits <= 16);
    const float maxCode = static_cast<float>((1u << bits) - 1u);

    QuantizedAttribute attr;
    attr.numChannels = numChannels;
    attr.bits        = bits;
    attr.ranges.resize(numChannels);
    attr.data.resize(static_cast<size_t>(numPoints) * numChannels);

    // Per-channel ranges: each dimension gets its own [lo, hi] rather than sharing one
    // global range across all channels of the attribute. This matters for non-cubic scenes
    // where, e.g., the Z extent is much smaller than X/Y — a shared range would waste most
    // of the code space on the narrow axis.
    for (int c = 0; c < numChannels; ++c) {
        float lo =  std::numeric_limits<float>::max();
        float hi = -std::numeric_limits<float>::max();
        for (int32_t i = 0; i < numPoints; ++i) {
            float v = data[i * numChannels + c];
            lo = std::min(lo, v);
            hi = std::max(hi, v);
        }
        attr.ranges[c] = {lo, hi};
    }

    // Quantize.
    for (int c = 0; c < numChannels; ++c) {
        float lo    = attr.ranges[c].lo;
        float hi    = attr.ranges[c].hi;
        float range = hi - lo;
        float scale = (range > 0.0f) ? (maxCode / range) : 0.0f;

        for (int32_t i = 0; i < numPoints; ++i) {
            float v = data[i * numChannels + c];
            // range == 0 when all values in a channel are identical (e.g. all-zero normals);
            // clamp to q=0 so every point decodes back to lo without dividing by zero.
            float q = (range > 0.0f) ? std::round((v - lo) * scale) : 0.0f;
            q = std::max(0.0f, std::min(maxCode, q));
            attr.data[i * numChannels + c] = static_cast<uint16_t>(q);
        }
    }

    return attr;
}

static std::vector<float> dequantizeAttribute(const QuantizedAttribute& attr, int32_t numPoints)
{
    const float maxCode = static_cast<float>((1u << attr.bits) - 1u);
    const int   nc      = attr.numChannels;
    std::vector<float> out(static_cast<size_t>(numPoints) * nc);

    for (int c = 0; c < nc; ++c) {
        float lo    = attr.ranges[c].lo;
        float hi    = attr.ranges[c].hi;
        float range = hi - lo;

        for (int32_t i = 0; i < numPoints; ++i) {
            uint16_t q = attr.data[i * nc + c];
            out[i * nc + c] = (maxCode > 0.0f && range > 0.0f)
                ? (static_cast<float>(q) / maxCode) * range + lo
                : lo;
        }
    }

    return out;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

QuantizedCloud quantize(const GaussianCloud& cloud, int bits)
{
    if (bits < 1 || bits > 16)
        throw std::invalid_argument("bits must be in [1, 16]");

    const int32_t N  = cloud.numPoints;
    const int     sh = cloud.shCoeffsPerPoint();

    QuantizedCloud qc;
    qc.numPoints = N;
    qc.shDegree  = cloud.shDegree;
    qc.bits      = bits;

    qc.positions  = quantizeAttribute(cloud.positions.data(),  N, 3,  bits);
    qc.normals    = quantizeAttribute(cloud.normals.data(),    N, 3,  bits);
    qc.colors     = quantizeAttribute(cloud.colors.data(),     N, 3,  bits);
    qc.opacities  = quantizeAttribute(cloud.opacities.data(),  N, 1,  bits);
    qc.scales     = quantizeAttribute(cloud.scales.data(),     N, 3,  bits);
    const int rotCh1 = static_cast<int>(cloud.rotations.size() / static_cast<size_t>(N));
    qc.rotations  = quantizeAttribute(cloud.rotations.data(),  N, rotCh1, bits);

    if (sh > 0)
        qc.sh = quantizeAttribute(cloud.sh.data(), N, sh, bits);
    else
        qc.sh = QuantizedAttribute{sh, bits, {}, {}};

    return qc;
}

QuantizedCloud quantize(const GaussianCloud& cloud, const AttributeBits& ab)
{
    const int32_t N  = cloud.numPoints;
    const int     sh = cloud.shCoeffsPerPoint();

    QuantizedCloud qc;
    qc.numPoints = N;
    qc.shDegree  = cloud.shDegree;
    // Cloud-level bits is the maximum across all attributes (used as a fast-fail
    // hint by the entropy coder; actual per-attribute depths are in each QuantizedAttribute).
    qc.bits = std::max({ab.positions, ab.normals, ab.colors,
                        ab.sh, ab.opacities, ab.scales, ab.rotations});

    qc.positions = quantizeAttribute(cloud.positions.data(), N, 3, ab.positions);
    qc.normals   = quantizeAttribute(cloud.normals.data(),   N, 3, ab.normals);
    qc.colors    = quantizeAttribute(cloud.colors.data(),    N, 3, ab.colors);
    qc.opacities = quantizeAttribute(cloud.opacities.data(), N, 1, ab.opacities);
    qc.scales    = quantizeAttribute(cloud.scales.data(),    N, 3, ab.scales);
    const int rotCh2 = static_cast<int>(cloud.rotations.size() / static_cast<size_t>(N));
    qc.rotations = quantizeAttribute(cloud.rotations.data(), N, rotCh2, ab.rotations);

    if (sh > 0)
        qc.sh = quantizeAttribute(cloud.sh.data(), N, sh, ab.sh);
    else
        qc.sh = QuantizedAttribute{sh, ab.sh, {}, {}};

    return qc;
}

GaussianCloud dequantize(const QuantizedCloud& qc)
{
    GaussianCloud cloud;
    cloud.numPoints = qc.numPoints;
    cloud.shDegree  = qc.shDegree;

    cloud.positions = dequantizeAttribute(qc.positions, qc.numPoints);
    cloud.normals   = dequantizeAttribute(qc.normals,   qc.numPoints);
    cloud.colors    = dequantizeAttribute(qc.colors,    qc.numPoints);
    cloud.opacities = dequantizeAttribute(qc.opacities, qc.numPoints);
    cloud.scales    = dequantizeAttribute(qc.scales,    qc.numPoints);
    cloud.rotations = dequantizeAttribute(qc.rotations, qc.numPoints);

    if (qc.sh.numChannels > 0)
        cloud.sh = dequantizeAttribute(qc.sh, qc.numPoints);

    return cloud;
}

size_t quantizedDataBytes(const QuantizedCloud& qc)
{
    // Bit-packed estimate per attribute using each attribute's own bit depth.
    // Works correctly for both uniform and per-attribute quantization.
    const size_t N = static_cast<size_t>(qc.numPoints);
    auto attrBytes = [N](const QuantizedAttribute& a) {
        return N * static_cast<size_t>(a.numChannels) * static_cast<size_t>(a.bits) / 8;
    };
    return attrBytes(qc.positions) + attrBytes(qc.normals)  + attrBytes(qc.colors)
         + attrBytes(qc.sh)        + attrBytes(qc.opacities) + attrBytes(qc.scales)
         + attrBytes(qc.rotations);
}
