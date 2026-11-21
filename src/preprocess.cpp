#include "preprocess.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

// ---------------------------------------------------------------------------
// Smallest-three quaternion encoding
// ---------------------------------------------------------------------------

static void encodeSmallestThree(GaussianCloud& cloud, std::vector<uint8_t>& rotDrop)
{
    const int32_t N = cloud.numPoints;
    rotDrop.resize(static_cast<size_t>(N));

    // Output: 3 components per point instead of 4.
    std::vector<float> out(static_cast<size_t>(N) * 3);

    const float sqrt2 = std::sqrt(2.0f);

    for (int32_t i = 0; i < N; ++i) {
        float q[4];
        for (int j = 0; j < 4; ++j)
            q[j] = cloud.rotations[static_cast<size_t>(i) * 4 + j];

        // Find the component with the largest absolute value.
        int k = 0;
        for (int j = 1; j < 4; ++j)
            if (std::abs(q[j]) > std::abs(q[k])) k = j;

        // Canonical form: q and -q represent the same rotation. Always keep
        // the dropped component positive so the decoder can assume q_k >= 0.
        if (q[k] < 0.0f)
            for (int j = 0; j < 4; ++j) q[j] = -q[j];

        rotDrop[i] = static_cast<uint8_t>(k);

        // Store the 3 remaining components scaled by sqrt(2), mapping their
        // natural range [-1/sqrt(2), 1/sqrt(2)] to [-1, 1] for maximum
        // quantization code coverage.
        int c = 0;
        for (int j = 0; j < 4; ++j) {
            if (j == k) continue;
            out[static_cast<size_t>(i) * 3 + c++] = q[j] * sqrt2;
        }
    }

    cloud.rotations = std::move(out);
}

static void decodeSmallestThree(GaussianCloud& cloud, const std::vector<uint8_t>& rotDrop)
{
    const int32_t N = cloud.numPoints;
    std::vector<float> out(static_cast<size_t>(N) * 4);

    const float inv_sqrt2 = 1.0f / std::sqrt(2.0f);

    for (int32_t i = 0; i < N; ++i) {
        const int k = rotDrop[i];

        // Undo the sqrt(2) scaling to get back the raw quaternion components.
        float a[3];
        float sumSq = 0.0f;
        for (int c = 0; c < 3; ++c) {
            a[c]   = cloud.rotations[static_cast<size_t>(i) * 3 + c] * inv_sqrt2;
            sumSq += a[c] * a[c];
        }

        // Recover the dropped component (always non-negative by canonical form).
        const float qk = std::sqrt(std::max(0.0f, 1.0f - sumSq));

        // Reinsert at the correct position.
        int c = 0;
        for (int j = 0; j < 4; ++j)
            out[static_cast<size_t>(i) * 4 + j] = (j == k) ? qk : a[c++];
    }

    cloud.rotations = std::move(out);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

PreprocessedCloud preprocess(const GaussianCloud& cloud, const PreprocessFlags& flags)
{
    PreprocessedCloud pc;
    pc.cloud = cloud;
    pc.flags = flags;

    if (flags.smallestThree)
        encodeSmallestThree(pc.cloud, pc.rotDrop);

    return pc;
}

GaussianCloud deprocess(const PreprocessedCloud& pc)
{
    GaussianCloud cloud = pc.cloud;

    if (pc.flags.smallestThree) {
        if (pc.rotDrop.size() != static_cast<size_t>(cloud.numPoints))
            throw std::runtime_error("deprocess: rotDrop size mismatch");
        decodeSmallestThree(cloud, pc.rotDrop);
    }

    return cloud;
}
