#pragma once

// Byte-streaming rANS (range Asymmetric Numeral Systems).
// Based on Fabian Giesen's public-domain ryg_rans design.
//
// Encoding is done in REVERSE symbol order. rans_flush() reverses the internal
// buffer so the final stream is in forward decoder order: [4-byte state][norm bytes…].
// The decoder reads symbols in forward order by consuming bytes left to right.
//
// State x is always in [L, L*256) between symbol operations (the invariant that
// lets normalization flush/fill exactly one byte per step).

#include <algorithm>
#include <cstdint>
#include <vector>

static constexpr uint32_t RANS_BYTE_L = 1u << 23;

// ---------------------------------------------------------------------------
// Encoder
// ---------------------------------------------------------------------------

struct RansEnc {
    uint32_t             x   = RANS_BYTE_L;
    std::vector<uint8_t> buf; // bytes collected in reverse emission order
};

// Feed one symbol (in reverse order): cumulative freq bs, frequency fs,
// model precision scale_bits.  0 < fs, bs + fs <= 1 << scale_bits.
inline void rans_put(RansEnc& r, uint32_t fs, uint32_t bs, int scale_bits)
{
    uint32_t x_max = ((RANS_BYTE_L >> scale_bits) << 8) * fs;
    while (r.x >= x_max) {
        r.buf.push_back(static_cast<uint8_t>(r.x & 0xFF));
        r.x >>= 8;
    }
    r.x = (r.x / fs) * (1u << scale_bits) + bs + (r.x % fs);
}

// Emit the 4-byte ANS state in big-endian (so after reversal the stream
// starts with the state in little-endian, matching rans_init's read order).
inline std::vector<uint8_t> rans_flush(RansEnc& r)
{
    r.buf.push_back(static_cast<uint8_t>((r.x >> 24) & 0xFF));
    r.buf.push_back(static_cast<uint8_t>((r.x >> 16) & 0xFF));
    r.buf.push_back(static_cast<uint8_t>((r.x >>  8) & 0xFF));
    r.buf.push_back(static_cast<uint8_t>( r.x        & 0xFF));
    std::reverse(r.buf.begin(), r.buf.end());
    return std::move(r.buf);
}

// ---------------------------------------------------------------------------
// Decoder
// ---------------------------------------------------------------------------

struct RansDec {
    uint32_t       x   = 0;
    const uint8_t* ptr = nullptr;
};

// Initialize from the start of a stream produced by rans_flush.
inline void rans_init(RansDec& r, const uint8_t* data)
{
    r.x = static_cast<uint32_t>(data[0])
        | (static_cast<uint32_t>(data[1]) << 8)
        | (static_cast<uint32_t>(data[2]) << 16)
        | (static_cast<uint32_t>(data[3]) << 24);
    r.ptr = data + 4;
}

// Cumulative-frequency slot — index into the decode lookup table.
inline uint32_t rans_get_slot(const RansDec& r, int scale_bits)
{
    return r.x & ((1u << scale_bits) - 1u);
}

// Advance state after consuming symbol with frequency fs and cumulative freq bs.
inline void rans_advance(RansDec& r, uint32_t fs, uint32_t bs, int scale_bits)
{
    r.x = fs * (r.x >> scale_bits) + (r.x & ((1u << scale_bits) - 1u)) - bs;
    while (r.x < RANS_BYTE_L)
        r.x = (r.x << 8) | *r.ptr++;
}
