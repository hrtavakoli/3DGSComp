# 3DGSComp


A small, readable C++17 codec for [3D Gaussian Splatting](https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/) (3DGS) scenes. 
It is meant for **teaching how splat compression works**, using simple, well-known building blocks. 
It is deliberately free of royalty-bearing or patent-encumbered tools (e.g., the entropy coder is rANS).
The codec has a **NAL-unit structured bitstream**, modelled after the
High-Level Syntax layer found in video codecs such as H.264/H.265. Each unit carries a
typed, self-described payload and an explicit size field that allows unknown units to be
skipped — making the format forward-compatible by design.

---

## Pipeline

```
PLY ─► [smallest-three] ─► quantize ─► rANS (per channel) ─► NAL-unit bitstream (.3dgc)
        (optional)         (lossy)

.3dgc ─► parse NAL units ─► rANS decode ─► dequantize ─► [rebuild quaternions] ─► PLY
```

| Stage | What it does | Lossy? |
|-------|--------------|--------|
| Load | Reads a 3DGS PLY (SH degree 0–3, any splat count) into a structure-of-arrays `GaussianCloud` | no |
| Smallest-three (optional) | Drops the largest-magnitude quaternion component, stores the other three scaled by √2, and records the dropped index in a 1-byte-per-splat sidecar | no (float rounding only) |
| Quantize | Per-channel uniform affine quantization to 1–16 bits, with a separate bit depth per attribute | **yes**, error ≤ half a step |
| Entropy code | Static per-channel frequency table plus a byte-oriented rANS stream, model precision `min(bits + 4, 16)` | no |
| Pack | Writes START, SPS, 7 CAUs, optional SIDECAR and END as NAL units | no |

Quantization is the only lossy step. For a channel with range `[lo, hi]` and `b` bits the worst-case error is `(hi - lo) / (2 · (2^b - 1))`.

### Default bit depths (`AttributeBits`)

| Attribute | Bits |
|-----------|------|
| positions | 16 |
| normals | 8 |
| DC colour | 4 |
| SH rest | 4 |
| opacity (logit) | 8 |
| log-scale | 12 |
| rotation | 12 |

These are hand-chosen, not tuned against an image-quality metric.

### Experimental: Morton sort and delta coding

`prediction.cpp` sorts splats along a 30-bit Morton curve and codes the zigzag-mapped difference between consecutive splats. It is **benchmarked but not part of the file format**: the sort permutation and a delta flag are not written. It gains about 3% at 4–8 bits and loses at 2 bits, because a static per-channel model already captures much of the same redundancy. Delta coding grows the alphabet by one bit, so it only applies up to 15-bit input.

---

## Features

- Load and save 3DGS PLY files
- Per-channel quantization at 1–16 bits, uniform or per-attribute
- Optional smallest-three quaternion transform with canonical-sign handling
- Static per-channel rANS with sparse frequency tables (byte-wise renormalization, 32-bit state)
- NAL-unit container with skippable unknown units (forward compatible; see [HLS.md](HLS.md))
- End-to-end round trip in the CLI: write, read back, decode, dequantize and save a decompressed PLY

---

## Build

Requires CMake ≥ 3.18 and a C++17 compiler. [tinyply](https://github.com/ddiakopoulos/tinyply) is fetched automatically.

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

---

## Usage

```sh
./build/3dgscomp path/to/scene.ply [output.3dgc]
```

If the output path is omitted, files are written next to the input using its name without the extension (`<stem>`). One run writes:

| File | Content |
|------|---------|
| `<stem>.3dgc` | per-attribute quantization + rANS |
| `<stem>_st.3dgc` | the same plus smallest-three rotations and a ROT_DROP sidecar |
| `<stem>_decompressed.ply` | decoded from `<stem>.3dgc` |
| `<stem>_st_decompressed.ply` | decoded from `<stem>_st.3dgc` |

The tool also prints a benchmark of the pipeline variants. There is no standalone decoder executable yet; decoding is done through the library (`readStream`, `entropyDecode`, `dequantize`, `deprocess`) and exercised by `main.cpp` as a round-trip check.

Example output on the 139,410-splat cactus scene (SH degree 3). "MB" is MiB; "payload" counts rANS bytes only:

```
Gaussians   : 139410
Memory (raw): 32.972 MB

Quant 8-bit :  8.243 MB  (pos max-err 0.00496)
       +rANS :  6.348 MB  (lossless)
  +delta+rANS : 6.183 MB  (lossless)
Quant 12-bit : 12.36 MB  (pos max-err 0.000309)
       +rANS : 10.15 MB  (lossless)
Per-attr    :  5.916 MB  (pos 16-bit max-err 1.96e-05, sh 4-bit, scales/rots 12-bit)
Per-attr+rANS: 4.024 MB  (lossless)
  +SmallestThree+rANS: 3.881 MB  (lossless)
Written ST  : ..._st.3dgc  (4.99 MB on disk)
Written     : ....3dgc     (5.01 MB on disk)
```

| Pipeline | Size | vs raw | Position max error |
|----------|------|--------|--------------------|
| Raw float32 | 32.97 MB | 1.0× | n/a |
| Uniform 8-bit, packed | 8.24 MB | 4.0× | 4.96e-3 |
| Uniform 8-bit + rANS (payload) | 6.35 MB | 5.2× | 4.96e-3 |
| Uniform 8-bit + Morton + delta + rANS (payload) | 6.18 MB | 5.3× | 4.96e-3 |
| Per-attribute, packed | 5.92 MB | 5.6× | 1.96e-5 |
| Per-attribute + rANS (payload) | 4.02 MB | 8.2× | 1.96e-5 |
| Per-attribute + smallest-three + rANS (payload) | 3.88 MB | 8.5× | 1.96e-5 |
| Per-attribute + rANS, `.3dgc` file | 5.01 MB | 6.6× | 1.96e-5 |
| Per-attribute + smallest-three + rANS, `.3dgc` file | 4.99 MB | 6.6× | 1.96e-5 |


---

## `.3dgc` bitstream

The container follows the high-level-syntax idea of video codecs (H.264/HEVC): typed, size-prefixed NAL units. A reader skips any unit type it does not know using `payload_size`. All integers are little-endian with no padding.

```
START   magic "3DGS", version 1
SPS     23 B: numPoints, shDegree, quantScheme, entropyScheme, preprocFlags, 7 × bit depth, creation timestamp
CAU × 7 one per attribute: positions, normals, colours, SH, opacities, scales, rotations
SIDECAR optional: ROT_DROP, one byte per splat (present when preprocFlags bit 0 is set)
END     empty payload
```

Each CAU holds `cau_type`, `sequence_num`, `numChannels`, `scaleBits`, then per channel `lo`, `hi`, a sparse `(symbol, freq)` table and the rANS bytes. The cumulative table is rebuilt at load time.

File size is `10 + 28 + 5 + 7 × 10 + 16 × C + 6 × A + S + sidecar`, where `C` is the channel count, `A` the total number of distinct symbols over all channels and `S` the rANS bytes. The SPS timestamp means two encodes of the same scene are not byte-identical.

The full byte-level specification, decoder pseudocode and extensibility rules are in **[HLS.md](HLS.md)**.
`cloud_io.h` holds an older flat `.3dgc` writer. It remains in the build for reference but the pipeline uses `writeStream` and `readStream`.

---

## Project structure

```
3dgsComp/
├── include/
│   ├── gaussian_cloud.h   # GaussianCloud (SoA) + SH degree helpers
│   ├── ply_loader.h       # PLY load / save
│   ├── quantization.h     # QuantizedCloud, AttributeBits, quantize(), dequantize()
│   ├── preprocess.h       # smallest-three rotation transform (preprocess / deprocess)
│   ├── prediction.h       # Morton sort, delta + zigzag (experimental)
│   ├── entropy.h          # EntropyCloud, entropyEncode / entropyDecode
│   ├── rans_byte.h        # byte-oriented rANS (after Giesen's ryg_rans, public domain)
│   ├── bitstream.h        # NAL-unit writer / reader (writeStream, readStream)
│   └── cloud_io.h         # legacy flat .3dgc format
├── src/                   # implementations, plus main.cpp (CLI and benchmark)
├── HLS.md                 # bitstream specification
└── CMakeLists.txt
```

### Attribute layout (`GaussianCloud`)

All arrays are structure-of-arrays, `numPoints × k` floats each:

| Field | Channels | Interpretation |
|-------|----------|----------------|
| `positions` | 3 | world-space x, y, z |
| `normals` | 3 | usually zero in trained scenes |
| `colors` | 3 | SH DC term (apply the SH activation to shade) |
| `sh` | 0 / 9 / 24 / 45 | SH rest coefficients, interleaved R₀G₀B₀… |
| `opacities` | 1 | logit; `sigmoid(v)` gives alpha |
| `scales` | 3 | log-scale; `exp(v)` gives half-axes |
| `rotations` | 4 (3 with smallest-three) | unit quaternion (w, x, y, z) |

