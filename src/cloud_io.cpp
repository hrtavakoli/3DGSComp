#include "cloud_io.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>

static constexpr char    MAGIC[4] = {'3', 'D', 'G', 'C'};
static constexpr uint8_t VERSION  = 1;

// Bit positions within the preprocFlags byte.
static constexpr uint8_t FLAG_SMALLEST_THREE = 0x01;

// ---------------------------------------------------------------------------
// Low-level I/O helpers
// ---------------------------------------------------------------------------

template<typename T>
static void wval(std::ofstream& f, T v) {
    f.write(reinterpret_cast<const char*>(&v), sizeof(T));
}

template<typename T>
static T rval(std::ifstream& f) {
    T v;
    if (!f.read(reinterpret_cast<char*>(&v), sizeof(T)))
        throw std::runtime_error("Unexpected end of .3dgc file");
    return v;
}

// ---------------------------------------------------------------------------
// Per-attribute serialization
// ---------------------------------------------------------------------------

static void writeAttribute(std::ofstream& f, const EntropyAttribute& attr)
{
    wval<uint8_t>(f, static_cast<uint8_t>(attr.bits));
    wval<uint8_t>(f, static_cast<uint8_t>(attr.scaleBits));
    wval<uint8_t>(f, static_cast<uint8_t>(attr.numChannels));

    for (int c = 0; c < attr.numChannels; ++c) {
        wval<float>(f, attr.ranges[c].lo);
        wval<float>(f, attr.ranges[c].hi);

        // Sparse freq table: only write (symbol, freq) for active symbols.
        const auto& freq = attr.freqTables[c];
        uint32_t numActive = 0;
        for (uint32_t v : freq) if (v > 0) ++numActive;
        wval<uint32_t>(f, numActive);
        for (size_t s = 0; s < freq.size(); ++s) {
            if (freq[s] > 0) {
                wval<uint16_t>(f, static_cast<uint16_t>(s));
                wval<uint32_t>(f, freq[s]);
            }
        }

        const auto& bs = attr.bitstreams[c];
        wval<uint32_t>(f, static_cast<uint32_t>(bs.size()));
        f.write(reinterpret_cast<const char*>(bs.data()),
                static_cast<std::streamsize>(bs.size()));
    }
}

static EntropyAttribute readAttribute(std::ifstream& f)
{
    EntropyAttribute attr;
    attr.bits        = static_cast<int>(rval<uint8_t>(f));
    attr.scaleBits   = static_cast<int>(rval<uint8_t>(f));
    attr.numChannels = static_cast<int>(rval<uint8_t>(f));

    const int      N            = attr.numChannels;
    const uint32_t alphabetSize = 1u << attr.bits;

    attr.ranges    .resize(N);
    attr.freqTables .resize(N);
    attr.cumulTables.resize(N);
    attr.bitstreams .resize(N);

    for (int c = 0; c < N; ++c) {
        attr.ranges[c].lo = rval<float>(f);
        attr.ranges[c].hi = rval<float>(f);

        // Rebuild dense freq table from sparse entries.
        attr.freqTables[c].assign(alphabetSize, 0);
        uint32_t numActive = rval<uint32_t>(f);
        for (uint32_t i = 0; i < numActive; ++i) {
            uint16_t sym  = rval<uint16_t>(f);
            uint32_t freq = rval<uint32_t>(f);
            attr.freqTables[c][sym] = freq;
        }

        // Reconstruct cumulative table required by decodeChannel.
        attr.cumulTables[c].resize(alphabetSize + 1);
        attr.cumulTables[c][0] = 0;
        for (uint32_t s = 0; s < alphabetSize; ++s)
            attr.cumulTables[c][s + 1] = attr.cumulTables[c][s] + attr.freqTables[c][s];

        uint32_t bsSize = rval<uint32_t>(f);
        attr.bitstreams[c].resize(bsSize);
        if (bsSize > 0 &&
            !f.read(reinterpret_cast<char*>(attr.bitstreams[c].data()), bsSize))
            throw std::runtime_error("Truncated bitstream in .3dgc file");
    }

    return attr;
}

// ---------------------------------------------------------------------------
// Shared write core
// ---------------------------------------------------------------------------

static void writeCore(std::ofstream& f,
                      const EntropyCloud& ec,
                      uint8_t preprocFlags,
                      const std::vector<uint8_t>& rotDrop)
{
    f.write(MAGIC, 4);
    wval<uint8_t> (f, VERSION);
    wval<int32_t> (f, ec.numPoints);
    wval<int32_t> (f, ec.shDegree);
    wval<uint8_t> (f, preprocFlags);

    writeAttribute(f, ec.positions);
    writeAttribute(f, ec.normals);
    writeAttribute(f, ec.colors);
    writeAttribute(f, ec.sh);
    writeAttribute(f, ec.opacities);
    writeAttribute(f, ec.scales);
    writeAttribute(f, ec.rotations);

    // Preprocessing sidecar data.
    if (preprocFlags & FLAG_SMALLEST_THREE) {
        f.write(reinterpret_cast<const char*>(rotDrop.data()),
                static_cast<std::streamsize>(rotDrop.size()));
    }

    if (!f) throw std::runtime_error("Write error");
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void writeEntropyCloud(const EntropyCloud& ec, const std::string& path)
{
    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open for writing: " + path);
    writeCore(f, ec, 0x00, {});
}

void writeEntropyCloud(const EntropyCloud& ec,
                       const PreprocessedCloud& preproc,
                       const std::string& path)
{
    uint8_t flags = 0x00;
    if (preproc.flags.smallestThree) flags |= FLAG_SMALLEST_THREE;

    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open for writing: " + path);
    writeCore(f, ec, flags, preproc.rotDrop);
}

ReadResult readEntropyCloud(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open for reading: " + path);

    char magic[4];
    f.read(magic, 4);
    if (std::memcmp(magic, MAGIC, 4) != 0)
        throw std::runtime_error("Not a .3dgc file: " + path);

    uint8_t version = rval<uint8_t>(f);
    if (version != VERSION)
        throw std::runtime_error("Unsupported .3dgc version " +
                                 std::to_string(version) + ": " + path);

    ReadResult result;
    EntropyCloud& ec = result.ec;

    ec.numPoints         = rval<int32_t>(f);
    ec.shDegree          = rval<int32_t>(f);
    const uint8_t pflags = rval<uint8_t>(f);

    result.flags.smallestThree = (pflags & FLAG_SMALLEST_THREE) != 0;

    ec.positions = readAttribute(f);
    ec.normals   = readAttribute(f);
    ec.colors    = readAttribute(f);
    ec.sh        = readAttribute(f);
    ec.opacities = readAttribute(f);
    ec.scales    = readAttribute(f);
    ec.rotations = readAttribute(f);

    ec.bits = std::max({ec.positions.bits, ec.normals.bits, ec.colors.bits,
                        ec.sh.bits,        ec.opacities.bits,
                        ec.scales.bits,    ec.rotations.bits});

    if (result.flags.smallestThree) {
        result.rotDrop.resize(static_cast<size_t>(ec.numPoints));
        if (!f.read(reinterpret_cast<char*>(result.rotDrop.data()),
                    static_cast<std::streamsize>(ec.numPoints)))
            throw std::runtime_error("Truncated rotDrop sidecar in .3dgc file");
    }

    return result;
}
