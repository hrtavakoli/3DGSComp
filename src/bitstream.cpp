#include "bitstream.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <vector>

static constexpr char    MAGIC[4] = {'3', 'D', 'G', 'S'};
static constexpr uint8_t VERSION  = 1;

// Number of recognised attribute types; maps CauType index → attrBits array index.
static constexpr int NUM_ATTR = 7;

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
        throw std::runtime_error("Unexpected end of bitstream");
    return v;
}

static void skipBytes(std::ifstream& f, uint32_t n) {
    f.seekg(static_cast<std::streamoff>(n), std::ios::cur);
}

// ---------------------------------------------------------------------------
// NAL unit framing
// ---------------------------------------------------------------------------

static void writeNalHeader(std::ofstream& f, NalType type, uint32_t payloadSize) {
    wval<uint8_t> (f, static_cast<uint8_t>(type));
    wval<uint32_t>(f, payloadSize);
}

// Returns false on clean EOF; throws on malformed input.
static bool readNalHeader(std::ifstream& f, NalType& type, uint32_t& payloadSize) {
    uint8_t t;
    if (!f.read(reinterpret_cast<char*>(&t), 1)) return false;
    type        = static_cast<NalType>(t);
    payloadSize = rval<uint32_t>(f);
    return true;
}

// ---------------------------------------------------------------------------
// START unit
// ---------------------------------------------------------------------------

static void writeStart(std::ofstream& f) {
    // Magic and version precede the common framing for this unit type.
    f.write(MAGIC, 4);
    wval<uint8_t>(f, VERSION);
    writeNalHeader(f, NalType::START, 0);
}

static void readStart(std::ifstream& f) {
    char magic[4];
    f.read(magic, 4);
    if (std::memcmp(magic, MAGIC, 4) != 0)
        throw std::runtime_error("Not a .3dgc bitstream: invalid magic");

    uint8_t version = rval<uint8_t>(f);
    if (version != VERSION)
        throw std::runtime_error("Unsupported bitstream version: " +
                                 std::to_string(version));

    NalType type; uint32_t size;
    if (!readNalHeader(f, type, size) || type != NalType::START || size != 0)
        throw std::runtime_error("Expected START NAL unit after magic");
}

// ---------------------------------------------------------------------------
// SPS unit
// ---------------------------------------------------------------------------

// SPS payload layout (23 bytes):
//   numPoints[4]       int32
//   shDegree[1]        uint8
//   quantScheme[1]     uint8
//   entropyScheme[1]   uint8
//   preprocFlags[1]    uint8   bit0=smallestThree
//   attrBits[7]        uint8[] pos,nrm,col,sh,opa,scl,rot
//   creation_ts[8]     uint64  ms since Unix epoch (UTC)
static constexpr uint32_t SPS_PAYLOAD_SIZE = 23;

static void writeSPS(std::ofstream& f,
                     const EntropyCloud& ec,
                     const PreprocessedCloud& preproc) {
    const uint8_t bits[NUM_ATTR] = {
        static_cast<uint8_t>(ec.positions.bits),
        static_cast<uint8_t>(ec.normals.bits),
        static_cast<uint8_t>(ec.colors.bits),
        static_cast<uint8_t>(ec.sh.bits),
        static_cast<uint8_t>(ec.opacities.bits),
        static_cast<uint8_t>(ec.scales.bits),
        static_cast<uint8_t>(ec.rotations.bits),
    };

    bool uniform = true;
    for (int i = 1; i < NUM_ATTR; ++i)
        if (bits[i] != bits[0]) { uniform = false; break; }

    uint8_t preprocFlags = 0;
    if (preproc.flags.smallestThree) preprocFlags |= 0x01;

    const uint64_t ts = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

    writeNalHeader(f, NalType::SPS, SPS_PAYLOAD_SIZE);
    wval<int32_t>(f, ec.numPoints);
    wval<uint8_t>(f, static_cast<uint8_t>(ec.shDegree));
    wval<uint8_t>(f, static_cast<uint8_t>(
        uniform ? QuantScheme::UNIFORM : QuantScheme::PER_ATTR));
    wval<uint8_t>(f, static_cast<uint8_t>(EntropyScheme::RANS));
    wval<uint8_t>(f, preprocFlags);
    f.write(reinterpret_cast<const char*>(bits), NUM_ATTR);
    wval<uint64_t>(f, ts);
}

struct SPSData {
    int32_t  numPoints  = 0;
    int32_t  shDegree   = 0;
    uint8_t  preprocFlags = 0;
    uint8_t  attrBits[NUM_ATTR] = {};
    uint64_t timestamp  = 0;
};

static SPSData readSPS(std::ifstream& f, uint32_t payloadSize) {
    if (payloadSize < SPS_PAYLOAD_SIZE)
        throw std::runtime_error("SPS payload smaller than expected");

    SPSData s;
    s.numPoints   = rval<int32_t>(f);
    s.shDegree    = static_cast<int32_t>(rval<uint8_t>(f));
    /* quantScheme  = */ rval<uint8_t>(f);   // informational; not used to drive decode
    /* entropyScheme= */ rval<uint8_t>(f);   // reserved for future schemes
    s.preprocFlags = rval<uint8_t>(f);
    f.read(reinterpret_cast<char*>(s.attrBits), NUM_ATTR);
    s.timestamp   = rval<uint64_t>(f);

    if (payloadSize > SPS_PAYLOAD_SIZE)
        skipBytes(f, payloadSize - SPS_PAYLOAD_SIZE); // forward-compat: ignore extra

    return s;
}

// ---------------------------------------------------------------------------
// CAU unit
// ---------------------------------------------------------------------------

// CAU payload layout:
//   cau_type[1]      CauType   — which attribute
//   sequence_num[2]  uint16    — order among CAUs of the same type, starting at 1
//   numChannels[1]   uint8
//   scaleBits[1]     uint8     — rANS model precision
//   Per channel:
//     lo[4]          float32
//     hi[4]          float32
//     numActive[4]   uint32    — symbols with freq > 0
//     numActive × { symbol[2] uint16, freq[4] uint32 }
//     bitstreamSize[4] uint32
//     bitstream[bitstreamSize] bytes

static void writeCAU(std::ofstream& f,
                     CauType cauType, uint16_t seqNum,
                     const EntropyAttribute& attr) {
    const int N = attr.numChannels;

    // Pre-compute numActive per channel (needed for payload size calculation).
    std::vector<uint32_t> numActive(N, 0);
    for (int c = 0; c < N; ++c)
        for (uint32_t v : attr.freqTables[c]) if (v > 0) ++numActive[c];

    // Payload size: fixed header + per-channel data.
    uint32_t payloadSize = 1 + 2 + 1 + 1; // cau_type + seq_num + numChannels + scaleBits
    for (int c = 0; c < N; ++c) {
        payloadSize += 4 + 4;                       // lo + hi
        payloadSize += 4;                           // numActive
        payloadSize += numActive[c] * (2 + 4);      // (symbol, freq) pairs
        payloadSize += 4;                           // bitstreamSize
        payloadSize += static_cast<uint32_t>(attr.bitstreams[c].size());
    }

    writeNalHeader(f, NalType::CAU, payloadSize);
    wval<uint8_t> (f, static_cast<uint8_t>(cauType));
    wval<uint16_t>(f, seqNum);
    wval<uint8_t> (f, static_cast<uint8_t>(N));
    wval<uint8_t> (f, static_cast<uint8_t>(attr.scaleBits));

    for (int c = 0; c < N; ++c) {
        wval<float>(f, attr.ranges[c].lo);
        wval<float>(f, attr.ranges[c].hi);

        wval<uint32_t>(f, numActive[c]);
        const auto& freq = attr.freqTables[c];
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

// Reads the channel data portion of a CAU (after cau_type and seq_num).
// bits = quantization bit depth sourced from SPS.attrBits for this attribute.
static EntropyAttribute readCAUData(std::ifstream& f, int bits) {
    EntropyAttribute attr;
    attr.bits        = bits;
    attr.numChannels = static_cast<int>(rval<uint8_t>(f));
    attr.scaleBits   = static_cast<int>(rval<uint8_t>(f));

    const int      N            = attr.numChannels;
    const uint32_t alphabetSize = 1u << bits;

    attr.ranges    .resize(N);
    attr.freqTables .resize(N);
    attr.cumulTables.resize(N);
    attr.bitstreams .resize(N);

    for (int c = 0; c < N; ++c) {
        attr.ranges[c].lo = rval<float>(f);
        attr.ranges[c].hi = rval<float>(f);

        attr.freqTables[c].assign(alphabetSize, 0);
        const uint32_t numActive = rval<uint32_t>(f);
        for (uint32_t i = 0; i < numActive; ++i) {
            const uint16_t sym  = rval<uint16_t>(f);
            const uint32_t freq = rval<uint32_t>(f);
            attr.freqTables[c][sym] = freq;
        }

        // Reconstruct cumulative table required by the rANS decoder.
        attr.cumulTables[c].resize(alphabetSize + 1);
        attr.cumulTables[c][0] = 0;
        for (uint32_t s = 0; s < alphabetSize; ++s)
            attr.cumulTables[c][s + 1] =
                attr.cumulTables[c][s] + attr.freqTables[c][s];

        const uint32_t bsSize = rval<uint32_t>(f);
        attr.bitstreams[c].resize(bsSize);
        if (bsSize > 0 &&
            !f.read(reinterpret_cast<char*>(attr.bitstreams[c].data()), bsSize))
            throw std::runtime_error("Truncated CAU bitstream");
    }

    return attr;
}

// ---------------------------------------------------------------------------
// SIDECAR unit
// ---------------------------------------------------------------------------

static void writeSidecar(std::ofstream& f,
                         SidecarType type,
                         const std::vector<uint8_t>& data) {
    writeNalHeader(f, NalType::SIDECAR,
                   1 + static_cast<uint32_t>(data.size()));
    wval<uint8_t>(f, static_cast<uint8_t>(type));
    f.write(reinterpret_cast<const char*>(data.data()),                                                                                                                                                                                                                         
            static_cast<std::streamsize>(data.size()));
}

// ---------------------------------------------------------------------------
// END unit
// ---------------------------------------------------------------------------

static void writeEnd(std::ofstream& f) {
    writeNalHeader(f, NalType::END, 0);
}

// ---------------------------------------------------------------------------
// Write core
// ---------------------------------------------------------------------------

static void writeStreamCore(std::ofstream& f,
                             const EntropyCloud& ec,
                             const PreprocessedCloud& preproc) {
    writeStart(f);
    writeSPS(f, ec, preproc);

    // CAUs in canonical order, sequence_num = 1 (single chunk per attribute).
    writeCAU(f, CauType::POSITIONS, 1, ec.positions);
    writeCAU(f, CauType::NORMALS,   1, ec.normals);
    writeCAU(f, CauType::COLORS,    1, ec.colors);
    writeCAU(f, CauType::SH,        1, ec.sh);
    writeCAU(f, CauType::OPACITIES, 1, ec.opacities);
    writeCAU(f, CauType::SCALES,    1, ec.scales);
    writeCAU(f, CauType::ROTATIONS, 1, ec.rotations);

    if (preproc.flags.smallestThree && !preproc.rotDrop.empty())
        writeSidecar(f, SidecarType::ROT_DROP, preproc.rotDrop);

    writeEnd(f);

    if (!f) throw std::runtime_error("Write error");
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void writeStream(const EntropyCloud& ec,
                 const PreprocessedCloud& preproc,
                 const std::string& path) {
    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open for writing: " + path);
    writeStreamCore(f, ec, preproc);
}

void writeStream(const EntropyCloud& ec, const std::string& path) {
    PreprocessedCloud none;
    writeStream(ec, none, path);
}

StreamReadResult readStream(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open for reading: " + path);

    readStart(f);

    StreamReadResult result;
    EntropyCloud& ec = result.ec;
    SPSData sps;
    bool hasSPS = false;

    // Map CauType index → pointer into ec for direct placement.
    EntropyAttribute* attrMap[NUM_ATTR] = {
        &ec.positions, &ec.normals, &ec.colors, &ec.sh,
        &ec.opacities, &ec.scales,  &ec.rotations,
    };

    NalType type; uint32_t size;
    while (readNalHeader(f, type, size)) {
        switch (type) {

        case NalType::SPS:
            sps    = readSPS(f, size);
            hasSPS = true;
            ec.numPoints = sps.numPoints;
            ec.shDegree  = sps.shDegree;
            result.flags.smallestThree = (sps.preprocFlags & 0x01) != 0;
            break;

        case NalType::CAU: {
            if (!hasSPS)
                throw std::runtime_error("CAU unit encountered before SPS");
            if (size < 4) // cau_type(1) + seq_num(2) + numChannels(1) minimum
                throw std::runtime_error("CAU payload too small");

            const CauType  cauType = static_cast<CauType>(rval<uint8_t>(f));
            const uint16_t seqNum  = rval<uint16_t>(f);
            (void)seqNum; // informational; future multi-chunk: sort by (type, seq_num)

            const int idx  = static_cast<int>(cauType);
            const int bits = (idx >= 0 && idx < NUM_ATTR)
                             ? static_cast<int>(sps.attrBits[idx]) : 8;

            EntropyAttribute attr = readCAUData(f, bits);

            if (idx >= 0 && idx < NUM_ATTR)
                *attrMap[idx] = std::move(attr);
            break;
        }

        case NalType::SIDECAR: {
            if (size < 1) { skipBytes(f, size); break; }
            const SidecarType st     = static_cast<SidecarType>(rval<uint8_t>(f));
            const uint32_t    dSize  = size - 1;
            if (st == SidecarType::ROT_DROP) {
                result.rotDrop.resize(dSize);
                if (dSize > 0 &&
                    !f.read(reinterpret_cast<char*>(result.rotDrop.data()), dSize))
                    throw std::runtime_error("Truncated ROT_DROP sidecar");
            } else {
                skipBytes(f, dSize); // unknown sidecar — skip
            }
            break;
        }

        case NalType::END:
            goto stream_done;

        default:
            skipBytes(f, size); // unknown NAL type — skip for forward compat
            break;
        }
    }
stream_done:
    ec.bits = std::max({ec.positions.bits, ec.normals.bits, ec.colors.bits,
                        ec.sh.bits,        ec.opacities.bits,
                        ec.scales.bits,    ec.rotations.bits});
    return result;
}
