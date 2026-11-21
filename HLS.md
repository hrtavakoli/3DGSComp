# High-Level Syntax (HLS) — 3DGS Compressed Bitstream

The `.3dgc` container follows a **NAL-unit structured bitstream**, modelled after the
High-Level Syntax layer found in video codecs such as H.264/H.265. Each unit carries a
typed, self-described payload and an explicit size field that allows unknown units to be
skipped — making the format forward-compatible by design.

---

## 1. File Structure

```
┌──────────────┐
│  START unit  │  magic "3DGS" + version byte + zero-payload NAL
├──────────────┤
│  SPS unit    │  Splat Parameter Set — all codec configuration (23 B fixed)
├──────────────┤
│  CAU #0      │  Compressed Attribute Unit — POSITIONS
│  CAU #1      │  Compressed Attribute Unit — NORMALS
│  CAU #2      │  Compressed Attribute Unit — COLORS
│  CAU #3      │  Compressed Attribute Unit — SH (spherical harmonics rest)
│  CAU #4      │  Compressed Attribute Unit — OPACITIES
│  CAU #5      │  Compressed Attribute Unit — SCALES
│  CAU #6      │  Compressed Attribute Unit — ROTATIONS
├──────────────┤
│  SIDECAR     │  (optional) preprocessing sidecar data
├──────────────┤
│  END unit    │  zero-payload terminal marker
└──────────────┘
```

A decoder processes units sequentially. The SPS must precede all CAUs. Any unknown
NAL type is **silently skipped** using its `payload_size` field.

---

## 2. Common NAL Unit Framing

Every unit except START uses this 5-byte header, immediately followed by `payload_size`
bytes of payload:

| Offset | Size | Type   | Field          | Description                              |
|--------|------|--------|----------------|------------------------------------------|
| 0      | 1    | uint8  | `nal_type`     | Unit type identifier (see §3)            |
| 1      | 4    | uint32 | `payload_size` | Bytes that follow; allows unknown-skip   |
| 5      | var  | bytes  | payload        | Unit-specific content                    |

All multi-byte integers are **little-endian**.

---

## 3. NAL Type Identifiers

| Value  | Name      | Enum `NalType`      |
|--------|-----------|---------------------|
| `0x01` | START     | `NalType::START`    |
| `0x02` | SPS       | `NalType::SPS`      |
| `0x10` | CAU       | `NalType::CAU`      |
| `0x20` | SIDECAR   | `NalType::SIDECAR`  |
| `0xFF` | END       | `NalType::END`      |

---

## 4. START Unit

The START unit uses a different framing: its 5-byte magic prefix comes **before** the
common NAL header, so the on-disk layout is:

| Offset | Size | Content                      |
|--------|------|------------------------------|
| 0      | 4    | Magic: `0x33 0x44 0x47 0x53` ("3DGS") |
| 4      | 1    | Version: `0x01`              |
| 5      | 1    | `nal_type` = `0x01` (START)  |
| 6      | 4    | `payload_size` = `0x00000000`|

The START unit has **zero payload**. Readers verify the magic and version before
processing any further units.

---

## 5. SPS — Splat Parameter Set

The SPS carries all configuration required to decode every subsequent CAU. Its payload
is **23 bytes fixed**; future versions may append trailing bytes, which are ignored.

### 5.1 Payload Layout

| Offset | Size | Type   | Field             | Description                                    |
|--------|------|--------|-------------------|------------------------------------------------|
| 0      | 4    | int32  | `numPoints`       | Total number of Gaussians                      |
| 4      | 1    | uint8  | `shDegree`        | Spherical-harmonics degree (0–3)               |
| 5      | 1    | uint8  | `quantScheme`     | `0`=NONE `1`=UNIFORM `2`=PER_ATTR             |
| 6      | 1    | uint8  | `entropyScheme`   | `0`=NONE `1`=RANS                             |
| 7      | 1    | uint8  | `preprocFlags`    | Bit flags: bit 0 = `smallestThree`             |
| 8      | 7    | uint8[] | `attrBits[7]`   | Quantisation depth per attribute (see §5.2)    |
| 15     | 8    | uint64 | `creation_ts`     | Milliseconds since Unix epoch (UTC) at encode  |

### 5.2 `attrBits` Index Map

| Index | `CauType`   | Attribute              | Default bits |
|-------|-------------|------------------------|-------------|
| 0     | POSITIONS   | x / y / z              | 16          |
| 1     | NORMALS     | nx / ny / nz           | 8           |
| 2     | COLORS      | f_dc_0/1/2             | 4           |
| 3     | SH          | f_rest_0 … f_rest_N    | 4           |
| 4     | OPACITIES   | opacity                | 8           |
| 5     | SCALES      | scale_0/1/2            | 12          |
| 6     | ROTATIONS   | rot_0/1/2(/3)          | 12          |

`attrBits[6]` is always the rotation precision; the number of channels
(3 or 4) is inferred at decode from `preprocFlags` bit 0.

### 5.3 `quantScheme` Semantics

`UNIFORM` is emitted when all seven `attrBits` values are equal. `PER_ATTR` is emitted
otherwise. The decoder does **not** branch on this field — it reads per-attribute bits
directly from `attrBits[7]` regardless.

---

## 6. CAU — Compressed Attribute Unit

Each CAU encodes one attribute (positions, SH coefficients, etc.) using rANS entropy
coding over the quantised symbol stream. (Morton sorting and delta + zigzag coding
exist in `prediction.cpp` as benchmark experiments; they are not signalled in the SPS
and are not applied to the stream written by `writeStream`.)

### 6.1 CAU Type Identifiers

| Value  | Name      | Attribute         |
|--------|-----------|-------------------|
| `0x00` | POSITIONS | xyz               |
| `0x01` | NORMALS   | nx ny nz          |
| `0x02` | COLORS    | f_dc_0/1/2        |
| `0x03` | SH        | f_rest_*          |
| `0x04` | OPACITIES | opacity           |
| `0x05` | SCALES    | scale_0/1/2       |
| `0x06` | ROTATIONS | rot_0/1/2(/3)     |

### 6.2 Payload Layout

```
[1B]  cau_type        uint8   — CauType identifier
[2B]  sequence_num    uint16  — order within this type, starts at 1
[1B]  numChannels     uint8   — channels in this attribute
[1B]  scaleBits       uint8   — rANS model precision = min(attrBits + 4, 16)
─── repeated numChannels times ────────────────────────────────────────────
[4B]  lo              float32 — dequantisation range minimum
[4B]  hi              float32 — dequantisation range maximum
[4B]  numActive       uint32  — symbols with freq > 0 in this channel
─── repeated numActive times ───────────────────────────────────────────────
  [2B]  symbol        uint16  — quantised symbol value
  [4B]  freq          uint32  — normalised frequency (sum = 1 << scaleBits)
────────────────────────────────────────────────────────────────────────────
[4B]  bitstreamSize   uint32  — rANS-coded bytes for this channel
[…]   bitstream       bytes
```

**Frequency table format.** Only `(symbol, freq)` pairs where `freq > 0` are stored
(sparse encoding). The decoder reconstructs the dense frequency array and cumulative
table by initialising all entries to zero and replaying the sparse pairs.

**`sequence_num` ordering.** CAUs are emitted in attribute type order:
POSITIONS seq=1, NORMALS seq=1, …, ROTATIONS seq=1. The `(CauType, sequence_num)`
pair uniquely addresses a chunk; a future tiling or LOD extension could emit multiple
chunks per type (seq=1, seq=2, …) without a format change.

### 6.3 rANS Model Precision

`scaleBits = min(attrBits + 4, 16)`

| Attribute bits | scaleBits | rANS scale M    |
|----------------|-----------|-----------------|
| 4              | 8         | 256             |
| 8              | 12        | 4 096           |
| 12             | 16        | 65 536          |
| 16             | 16        | 65 536          |

### 6.4 Size Accounting

Every unit except START has a 5-byte header; START is 10 bytes in total (6-byte
prefix plus a 4-byte zero size field). A CAU payload is `5 B + Σ channels (16 B +
6 B × numActive + bitstreamSize)`. For a file with `C` channels in total, `A` distinct
symbols summed over all channels and `S` rANS bytes in total:

```
file_bytes = 10 + 28 + 5 + 7 × 10 + 16 × C + 6 × A + S + sidecar_bytes
```

where 28 is the SPS unit (5 B header + 23 B payload), 5 the END unit, and `7 × 10` the
unit header plus CAU header of each of the seven CAUs. `sidecar_bytes` is `6 + numPoints`
for a ROT_DROP unit and 0 otherwise. A constant channel has `numActive = 1` and a 4-byte
stream (the final rANS state), which gives the 83-byte NORMALS payload in §11.

`creation_ts` is part of the SPS, so two encodes of the same scene differ in those
8 bytes; files are not bit-for-bit reproducible unless the timestamp is fixed.

---

## 7. SIDECAR Unit

SIDECAR units carry preprocessing metadata that is not part of the entropy-coded stream.

### 7.1 Payload Layout

```
[1B]  sidecar_type    uint8   — see §7.2
[…]   data            bytes   — type-specific raw data
```

### 7.2 Sidecar Type Identifiers

| Value  | Name      | Description                                       |
|--------|-----------|---------------------------------------------------|
| `0x00` | ROT_DROP  | Per-point dropped quaternion index (0–3); `numPoints` bytes |

**ROT_DROP** is present only when `preprocFlags & 0x01` (smallestThree) is set. It
stores one byte per Gaussian indicating which quaternion component was dropped before
encoding (the component with the largest absolute value).

---

## 8. END Unit

The END unit signals the logical end of the bitstream.

| Field          | Value    |
|----------------|----------|
| `nal_type`     | `0xFF`   |
| `payload_size` | `0`      |

No payload follows. A decoder stops reading after encountering END. Any bytes after
END in the file are ignored.

---

## 9. Decoder Algorithm

```
readStart(f)                       // verify magic + version

sps  ← (not yet read)
attrs ← {positions, normals, colors, sh, opacities, scales, rotations}

loop:
  (type, payloadSize) ← readNalHeader(f)

  switch type:
    SPS:
      sps ← readSPS(f, payloadSize)

    CAU:
      require hasSPS
      cauType  ← read uint8
      seqNum   ← read uint16          // future multi-chunk: sort by (cauType, seqNum)
      bits     ← sps.attrBits[cauType]
      attr     ← readCAUData(f, bits) // freq table + bitstreams
      attrs[cauType] ← attr

    SIDECAR:
      sidecarType ← read uint8
      if sidecarType == ROT_DROP:
        rotDrop ← read (payloadSize − 1) bytes
      else:
        skip (payloadSize − 1) bytes  // unknown sidecar — forward compat

    END:
      break loop

    default:
      skip payloadSize bytes          // unknown NAL type — forward compat

ec.bits ← max(attrBits[0..6])
return (ec, sps.preprocFlags, rotDrop)
```

After reading the file the caller drives the decode pipeline:

```
qc   ← entropyDecode(ec)
g    ← dequantize(qc)
if preprocFlags & smallestThree:
  g ← deprocess({cloud=g, rotDrop=rotDrop, flags=…})
```

---

## 10. Extensibility

| Mechanism                | How it works                                                         |
|--------------------------|----------------------------------------------------------------------|
| Unknown NAL type         | `payload_size` allows the reader to skip it unconditionally          |
| Unknown sidecar type     | Same skip-by-size within the SIDECAR unit                           |
| Larger SPS payload       | `payloadSize > 23` trailing bytes are consumed and discarded         |
| Additional CAU sequence  | `(CauType, sequence_num)` addressing allows tiling or LOD extensions |
| New `preprocFlags` bits  | Bits 1–7 are currently zero; unknown bits should be ignored          |
| New `attrBits` entries   | Adding an 8th attribute requires a new `CauType` value (unambiguous) |

---

## 11. Worked Example

Parsing `cactus_splat3_30kSteps_142k_splats.3dgc` (139 410 Gaussians, SH degree 3):

```
magic=b'3DGS' version=1
NAL 0x01  START    payload=0 B
NAL 0x02  SPS      payload=23 B
          numPoints=139410  shDeg=3  qScheme=PER_ATTR  eScheme=RANS
          preprocFlags=0x00
          attrBits=[16, 8, 4, 4, 8, 12, 12]
          timestamp=1793552485002 ms (UTC)
NAL 0x10  CAU      payload=1 692 921 B   type=POSITIONS  seq=1  numCh=3  scaleBits=16
NAL 0x10  CAU      payload=83 B          type=NORMALS    seq=1  numCh=3  scaleBits=12
NAL 0x10  CAU      payload=138 224 B     type=COLORS     seq=1  numCh=3  scaleBits=8
NAL 0x10  CAU      payload=1 831 657 B   type=SH         seq=1  numCh=45 scaleBits=8
NAL 0x10  CAU      payload=125 517 B     type=OPACITIES  seq=1  numCh=1  scaleBits=12
NAL 0x10  CAU      payload=631 708 B     type=SCALES     seq=1  numCh=3  scaleBits=16
NAL 0x10  CAU      payload=830 514 B     type=ROTATIONS  seq=1  numCh=4  scaleBits=16
NAL 0xFF  END      payload=0 B
```

The file is 5 250 702 bytes. The `_st.3dgc` variant adds `preprocFlags=0x01` and a
SIDECAR ROT_DROP unit (`payload=139 411 B`: one type byte plus `numPoints` index bytes)
between the last CAU and END. Its ROTATIONS unit has `numCh=3` and `payload=672 408 B`
(smallest-three encoding drops one component).

---

## 12. Implementation

| File                     | Role                                              |
|--------------------------|---------------------------------------------------|
| `include/bitstream.h`    | `NalType`, `CauType`, `SidecarType`, `QuantScheme`, `EntropyScheme` enums; `StreamReadResult`; `writeStream` / `readStream` API |
| `src/bitstream.cpp`      | Full encoder and decoder — `writeStart`, `writeSPS`, `writeCAU`, `writeSidecar`, `writeEnd`, `readStream` |
| `include/preprocess.h`   | `PreprocessFlags`, `PreprocessedCloud`; `preprocess` / `deprocess` API |
| `src/preprocess.cpp`     | Smallest-three quaternion encode/decode           |
| `include/entropy.h`      | `EntropyAttribute`, `EntropyCloud`; rANS API      |
| `src/entropy.cpp`        | rANS encode/decode with dynamic `scaleBits`       |
| `src/main.cpp`           | End-to-end pipeline: PLY → quantize → entropy → `writeStream` → `readStream` → `entropyDecode` → `dequantize` → PLY |

The old flat format (`cloud_io.h`) remains in the build for reference but is no longer
used by the main pipeline; `writeStream` / `readStream` are the canonical I/O path.
