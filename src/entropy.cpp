#include "entropy.h"
#include "rans_byte.h"

#include <algorithm>
#include <stdexcept>
#include <vector>

// ---------------------------------------------------------------------------
// Frequency table
// ---------------------------------------------------------------------------

// Normalize raw counts to sum to scaleM = 2^scaleBits, guaranteeing freq >= 1
// for every observed symbol. Uses a two-pass approach:
//   1. Reserve 1 slot per active symbol (prevents underflow when alphabetSize
//      equals scaleM, where the naive bump-to-1 strategy over-allocates).
//   2. Distribute the remaining (scaleM - activeCount) slots proportionally,
//      then give the fractional-part leftovers to the highest-shortfall symbols.
static void buildFreqTable(const std::vector<uint32_t>& rawFreq,
                           std::vector<uint32_t>&       freq,
                           std::vector<uint32_t>&       cumul,
                           int                          scaleBits)
{
    const uint32_t scaleM = 1u << scaleBits;
    const size_t   N      = rawFreq.size();
    freq.assign(N, 0);

    uint64_t total       = 0;
    uint32_t activeCount = 0;
    for (uint32_t f : rawFreq) { if (f > 0) { total += f; ++activeCount; } }

    if (total == 0) { // no data — assign all slots to symbol 0 (unused in practice)
        freq[0] = scaleM;
        cumul.assign(N + 1, 0);
        for (size_t s = 0; s < N; ++s) cumul[s + 1] = cumul[s] + freq[s];
        return;
    }

    if (activeCount > scaleM)
        throw std::runtime_error("entropyEncode: distinct symbol count exceeds scaleM");

    const uint32_t base = scaleM - activeCount; // slots to distribute beyond the 1-per-symbol floor

    struct SymFrac { size_t idx; double frac; };
    std::vector<SymFrac> fracs;
    fracs.reserve(activeCount);

    uint32_t bonusSum = 0;
    for (size_t s = 0; s < N; ++s) {
        if (rawFreq[s] == 0) continue;
        double exact     = static_cast<double>(rawFreq[s]) * base / total;
        uint32_t bonus   = static_cast<uint32_t>(exact);
        freq[s]          = 1 + bonus;
        bonusSum        += bonus;
        fracs.push_back({s, exact - bonus});
    }

    // Give remaining slots to symbols with the largest fractional shortfall.
    uint32_t remaining = base - bonusSum; // guaranteed >= 0 and <= activeCount
    std::partial_sort(fracs.begin(), fracs.begin() + remaining, fracs.end(),
                      [](const SymFrac& a, const SymFrac& b) { return a.frac > b.frac; });
    for (uint32_t i = 0; i < remaining; ++i)
        freq[fracs[i].idx]++;

    cumul.assign(N + 1, 0);
    for (size_t s = 0; s < N; ++s)
        cumul[s + 1] = cumul[s] + freq[s];
}

// ---------------------------------------------------------------------------
// Per-channel encode / decode
// ---------------------------------------------------------------------------

static std::vector<uint8_t> encodeChannel(
    const uint16_t* data, int32_t numPoints, int numCh, int ch,
    const std::vector<uint32_t>& freq,
    const std::vector<uint32_t>& cumul,
    int                          scaleBits)
{
    RansEnc enc;
    // Symbols fed in reverse so the decoder can read them in forward order.
    for (int32_t i = numPoints - 1; i >= 0; --i) {
        uint16_t s = data[static_cast<size_t>(i) * numCh + ch];
        rans_put(enc, freq[s], cumul[s], scaleBits);
    }
    return rans_flush(enc);
}

static void decodeChannel(
    const std::vector<uint8_t>& stream, int32_t numPoints, int numCh, int ch,
    uint16_t*                    out,
    const std::vector<uint32_t>& freq,
    const std::vector<uint32_t>& cumul,
    int                          alphabetSize,
    int                          scaleBits)
{
    // Flat decode table: slot -> symbol. Built once per channel, O(2^scaleBits).
    std::vector<uint16_t> decodeTable(1u << scaleBits, 0);
    for (int s = 0; s < alphabetSize; ++s)
        for (uint32_t slot = cumul[s]; slot < cumul[s + 1]; ++slot)
            decodeTable[slot] = static_cast<uint16_t>(s);

    RansDec dec;
    rans_init(dec, stream.data());
    for (int32_t i = 0; i < numPoints; ++i) {
        uint16_t s = decodeTable[rans_get_slot(dec, scaleBits)];
        out[static_cast<size_t>(i) * numCh + ch] = s;
        rans_advance(dec, freq[s], cumul[s], scaleBits);
    }
}

// ---------------------------------------------------------------------------
// Attribute encode / decode
// ---------------------------------------------------------------------------

static EntropyAttribute encodeAttribute(const QuantizedAttribute& attr, int32_t numPoints)
{
    const int      N            = attr.numChannels;
    const uint32_t alphabetSize = 1u << attr.bits;
    // Use more model precision than the symbol bits, capped at 16.
    // 8-bit → scaleBits=12 (4096 slots, 16 per symbol on average)
    // 12-bit → scaleBits=16 (65536 slots, 16 per symbol on average)
    // 16-bit → scaleBits=16 (65536 slots = alphabet size; 1 per symbol minimum)
    const int scaleBits = std::min(attr.bits + 4, 16);

    EntropyAttribute enc;
    enc.numChannels = N;
    enc.bits        = attr.bits;
    enc.scaleBits   = scaleBits;
    enc.ranges      = attr.ranges;
    enc.freqTables .resize(N);
    enc.cumulTables.resize(N);
    enc.bitstreams .resize(N);

    for (int c = 0; c < N; ++c) {
        std::vector<uint32_t> rawFreq(alphabetSize, 0);
        for (int32_t i = 0; i < numPoints; ++i)
            ++rawFreq[attr.data[static_cast<size_t>(i) * N + c]];

        buildFreqTable(rawFreq, enc.freqTables[c], enc.cumulTables[c], scaleBits);
        enc.bitstreams[c] = encodeChannel(
            attr.data.data(), numPoints, N, c,
            enc.freqTables[c], enc.cumulTables[c], scaleBits);
    }
    return enc;
}

static QuantizedAttribute decodeAttribute(const EntropyAttribute& enc, int32_t numPoints)
{
    const int N            = enc.numChannels;
    const int alphabetSize = 1 << enc.bits;

    QuantizedAttribute attr;
    attr.numChannels = N;
    attr.bits        = enc.bits;
    attr.ranges      = enc.ranges;
    attr.data.resize(static_cast<size_t>(numPoints) * N);

    for (int c = 0; c < N; ++c)
        decodeChannel(enc.bitstreams[c], numPoints, N, c,
                      attr.data.data(),
                      enc.freqTables[c], enc.cumulTables[c],
                      alphabetSize, enc.scaleBits);

    return attr;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

EntropyCloud entropyEncode(const QuantizedCloud& qc)
{
    EntropyCloud ec;
    ec.numPoints = qc.numPoints;
    ec.shDegree  = qc.shDegree;
    ec.bits      = qc.bits;

    ec.positions = encodeAttribute(qc.positions, qc.numPoints);
    ec.normals   = encodeAttribute(qc.normals,   qc.numPoints);
    ec.colors    = encodeAttribute(qc.colors,    qc.numPoints);
    ec.opacities = encodeAttribute(qc.opacities, qc.numPoints);
    ec.scales    = encodeAttribute(qc.scales,    qc.numPoints);
    ec.rotations = encodeAttribute(qc.rotations, qc.numPoints);

    if (qc.sh.numChannels > 0)
        ec.sh = encodeAttribute(qc.sh, qc.numPoints);
    else
        ec.sh = EntropyAttribute{0, qc.bits, qc.bits > 4 ? 12 : qc.bits + 4, {}, {}, {}, {}};

    return ec;
}

QuantizedCloud entropyDecode(const EntropyCloud& ec)
{
    QuantizedCloud qc;
    qc.numPoints = ec.numPoints;
    qc.shDegree  = ec.shDegree;
    qc.bits      = ec.bits;

    qc.positions = decodeAttribute(ec.positions, ec.numPoints);
    qc.normals   = decodeAttribute(ec.normals,   ec.numPoints);
    qc.colors    = decodeAttribute(ec.colors,    ec.numPoints);
    qc.opacities = decodeAttribute(ec.opacities, ec.numPoints);
    qc.scales    = decodeAttribute(ec.scales,    ec.numPoints);
    qc.rotations = decodeAttribute(ec.rotations, ec.numPoints);

    if (ec.sh.numChannels > 0)
        qc.sh = decodeAttribute(ec.sh, ec.numPoints);

    return qc;
}

size_t entropyBytes(const EntropyCloud& ec)
{
    size_t total = 0;
    auto sum = [&](const EntropyAttribute& a) {
        for (const auto& bs : a.bitstreams) total += bs.size();
    };
    sum(ec.positions);
    sum(ec.normals);
    sum(ec.colors);
    sum(ec.sh);
    sum(ec.opacities);
    sum(ec.scales);
    sum(ec.rotations);
    return total;
}
