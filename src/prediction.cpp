#include "prediction.h"

#include <algorithm>
#include <cassert>
#include <numeric>

// ---------------------------------------------------------------------------
// Morton encoding (Z-order curve)
// ---------------------------------------------------------------------------

// Spread a 10-bit integer so each bit occupies every third position (0,3,6,...,27).
// mortonCode(x,y,z) = spread10(x) | spread10(y)<<1 | spread10(z)<<2
// interleaves xyz bits to give the 3D Z-order index.
static uint32_t spread10(uint32_t x)
{
    x &= 0x000003FFu;
    x = (x | x << 16u) & 0x030000FFu;
    x = (x | x <<  8u) & 0x0300F00Fu;
    x = (x | x <<  4u) & 0x030C30C3u;
    x = (x | x <<  2u) & 0x09249249u;
    return x;
}

static uint32_t mortonCode(uint32_t x, uint32_t y, uint32_t z)
{
    return spread10(x) | (spread10(y) << 1u) | (spread10(z) << 2u);
}

std::vector<uint32_t> mortonSort(GaussianCloud& cloud)
{
    const int32_t N = cloud.numPoints;
    if (N == 0) return {};

    // Bounding box for normalizing positions to [0, 1023].
    float mnX = cloud.positions[0], mxX = mnX;
    float mnY = cloud.positions[1], mxY = mnY;
    float mnZ = cloud.positions[2], mxZ = mnZ;
    for (int32_t i = 0; i < N; ++i) {
        float x = cloud.positions[i*3+0];
        float y = cloud.positions[i*3+1];
        float z = cloud.positions[i*3+2];
        mnX = std::min(mnX, x); mxX = std::max(mxX, x);
        mnY = std::min(mnY, y); mxY = std::max(mxY, y);
        mnZ = std::min(mnZ, z); mxZ = std::max(mxZ, z);
    }
    float scX = (mxX > mnX) ? 1023.0f / (mxX - mnX) : 0.0f;
    float scY = (mxY > mnY) ? 1023.0f / (mxY - mnY) : 0.0f;
    float scZ = (mxZ > mnZ) ? 1023.0f / (mxZ - mnZ) : 0.0f;

    std::vector<uint32_t> codes(N);
    for (int32_t i = 0; i < N; ++i) {
        uint32_t xi = static_cast<uint32_t>((cloud.positions[i*3+0] - mnX) * scX);
        uint32_t yi = static_cast<uint32_t>((cloud.positions[i*3+1] - mnY) * scY);
        uint32_t zi = static_cast<uint32_t>((cloud.positions[i*3+2] - mnZ) * scZ);
        codes[i] = mortonCode(xi, yi, zi);
    }

    std::vector<uint32_t> perm(N);
    std::iota(perm.begin(), perm.end(), 0u);
    std::sort(perm.begin(), perm.end(),
              [&codes](uint32_t a, uint32_t b) { return codes[a] < codes[b]; });

    // Reorder every attribute array in-place.
    auto reorder = [&](std::vector<float>& v, int k) {
        if (v.empty()) return;
        std::vector<float> tmp(v.size());
        for (int32_t i = 0; i < N; ++i)
            for (int j = 0; j < k; ++j)
                tmp[static_cast<size_t>(i)*k + j] = v[static_cast<size_t>(perm[i])*k + j];
        v = std::move(tmp);
    };

    reorder(cloud.positions, 3);
    reorder(cloud.normals,   3);
    reorder(cloud.colors,    3);
    reorder(cloud.opacities, 1);
    reorder(cloud.scales,    3);
    reorder(cloud.rotations, 4);
    if (cloud.shCoeffsPerPoint() > 0)
        reorder(cloud.sh, cloud.shCoeffsPerPoint());

    return perm;
}

// ---------------------------------------------------------------------------
// Delta + zigzag coding
// ---------------------------------------------------------------------------

static inline uint16_t zigzag(int32_t d)
{
    return static_cast<uint16_t>(d >= 0 ? 2 * d : -2 * d - 1);
}

static inline int32_t unzigzag(uint16_t z)
{
    return (z & 1u) ? -(static_cast<int32_t>(z) + 1) / 2
                    :   static_cast<int32_t>(z)       / 2;
}

QuantizedAttribute deltaEncode(const QuantizedAttribute& attr, int32_t numPoints)
{
    const int nc = attr.numChannels;
    QuantizedAttribute out;
    out.numChannels = nc;
    out.bits        = attr.bits + 1; // zigzag range is [0, 2*(2^bits-1)]
    out.ranges      = attr.ranges;
    out.data.resize(attr.data.size());

    for (int c = 0; c < nc; ++c) {
        // First point: store raw code (fits in bits+1 range since raw < 2^bits < 2^(bits+1))
        out.data[0 * nc + c] = attr.data[0 * nc + c];
        for (int32_t i = 1; i < numPoints; ++i) {
            int32_t delta = static_cast<int32_t>(attr.data[i * nc + c])
                          - static_cast<int32_t>(attr.data[(i-1) * nc + c]);
            out.data[i * nc + c] = zigzag(delta);
        }
    }
    return out;
}

QuantizedAttribute deltaDecode(const QuantizedAttribute& attr, int32_t numPoints)
{
    const int nc = attr.numChannels;
    QuantizedAttribute out;
    out.numChannels = nc;
    out.bits        = attr.bits - 1;
    out.ranges      = attr.ranges;
    out.data.resize(attr.data.size());

    for (int c = 0; c < nc; ++c) {
        out.data[0 * nc + c] = attr.data[0 * nc + c];
        for (int32_t i = 1; i < numPoints; ++i) {
            int32_t prev = static_cast<int32_t>(out.data[(i-1) * nc + c]);
            out.data[i * nc + c] =
                static_cast<uint16_t>(prev + unzigzag(attr.data[i * nc + c]));
        }
    }
    return out;
}

QuantizedCloud deltaEncodeCloud(const QuantizedCloud& qc)
{
    QuantizedCloud dc;
    dc.numPoints = qc.numPoints;
    dc.shDegree  = qc.shDegree;
    dc.bits      = qc.bits + 1;

    dc.positions = deltaEncode(qc.positions, qc.numPoints);
    dc.normals   = deltaEncode(qc.normals,   qc.numPoints);
    dc.colors    = deltaEncode(qc.colors,    qc.numPoints);
    dc.opacities = deltaEncode(qc.opacities, qc.numPoints);
    dc.scales    = deltaEncode(qc.scales,    qc.numPoints);
    dc.rotations = deltaEncode(qc.rotations, qc.numPoints);

    if (qc.sh.numChannels > 0)
        dc.sh = deltaEncode(qc.sh, qc.numPoints);
    else
        dc.sh = QuantizedAttribute{0, qc.bits + 1, {}, {}};

    return dc;
}

QuantizedCloud deltaDecodeCloud(const QuantizedCloud& dc)
{
    QuantizedCloud qc;
    qc.numPoints = dc.numPoints;
    qc.shDegree  = dc.shDegree;
    qc.bits      = dc.bits - 1;

    qc.positions = deltaDecode(dc.positions, dc.numPoints);
    qc.normals   = deltaDecode(dc.normals,   dc.numPoints);
    qc.colors    = deltaDecode(dc.colors,    dc.numPoints);
    qc.opacities = deltaDecode(dc.opacities, dc.numPoints);
    qc.scales    = deltaDecode(dc.scales,    dc.numPoints);
    qc.rotations = deltaDecode(dc.rotations, dc.numPoints);

    if (dc.sh.numChannels > 0)
        qc.sh = deltaDecode(dc.sh, dc.numPoints);

    return qc;
}
