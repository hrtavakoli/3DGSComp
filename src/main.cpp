#include "ply_loader.h"
#include "quantization.h"
#include "entropy.h"
#include "prediction.h"
#include "preprocess.h"
#include "cloud_io.h"
#include "bitstream.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: 3dgscomp <path/to/scene.ply> [output.3dgc]\n";
        return 1;
    }

    // Derive output paths: strip extension (if any) and append suffixes.
    std::string stem;
    if (argc >= 3) {
        stem = argv[2];
        auto dot = stem.rfind('.');
        if (dot != std::string::npos) stem = stem.substr(0, dot);
    } else {
        std::string src = argv[1];
        auto dot = src.rfind('.');
        stem = (dot != std::string::npos ? src.substr(0, dot) : src);
    }
    const std::string outPath   = stem + ".3dgc";
    const std::string outPathST = stem + "_st.3dgc";

    GaussianCloud cloud;
    try {
        cloud = loadGaussiansFromPly(argv[1]);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    // --- Bounding box ---
    float minX = std::numeric_limits<float>::max();
    float minY = minX, minZ = minX;
    float maxX = std::numeric_limits<float>::lowest();
    float maxY = maxX, maxZ = maxX;

    for (int32_t i = 0; i < cloud.numPoints; ++i) {
        float x = cloud.positions[i * 3 + 0];
        float y = cloud.positions[i * 3 + 1];
        float z = cloud.positions[i * 3 + 2];
        minX = std::min(minX, x); maxX = std::max(maxX, x);
        minY = std::min(minY, y); maxY = std::max(maxY, y);
        minZ = std::min(minZ, z); maxZ = std::max(maxZ, z);
    }

    // --- Memory footprint (raw floats only) ---
    size_t floatCount =
        cloud.positions.size() + cloud.normals.size() + cloud.colors.size() +
        cloud.sh.size() + cloud.opacities.size() + cloud.scales.size() +
        cloud.rotations.size();
    double mb = static_cast<double>(floatCount * sizeof(float)) / (1024.0 * 1024.0);

    std::cout << "File        : " << argv[1] << "\n"
              << "Gaussians   : " << cloud.numPoints << "\n"
              << "SH degree   : " << cloud.shDegree
              << "  (" << cloud.shCoeffsPerPoint() << " rest coeffs/point)\n"
              << "Bounding box:\n"
              << "  X  [" << minX << ", " << maxX << "]\n"
              << "  Y  [" << minY << ", " << maxY << "]\n"
              << "  Z  [" << minZ << ", " << maxZ << "]\n"
              << "Memory (raw): " << mb << " MB\n";

    // Morton-sorted copy used by Phase 2 (sort once, reuse across bit depths).
    GaussianCloud sortedCloud = cloud;
    mortonSort(sortedCloud);

    // --- Quantization + entropy coding ---
    for (int bits : {2, 4, 8, 12, 16}) {
        QuantizedCloud qc = quantize(cloud, bits);
        GaussianCloud  rc = dequantize(qc);

        float maxErr = 0.0f;
        for (size_t i = 0; i < cloud.positions.size(); ++i)
            maxErr = std::max(maxErr, std::abs(cloud.positions[i] - rc.positions[i]));

        double qmb = static_cast<double>(quantizedDataBytes(qc)) / (1024.0 * 1024.0);
        std::cout << "Quant " << bits << "-bit : " << qmb << " MB"
                  << "  (pos max-err " << maxErr << ")\n";

        if (bits <= 12) {
            // Phase 1: rANS entropy coding on raw quantized codes.
            EntropyCloud   ec1  = entropyEncode(qc);
            QuantizedCloud qcr1 = entropyDecode(ec1);
            bool lossless1 = (qc.positions.data == qcr1.positions.data);
            double emb1 = static_cast<double>(entropyBytes(ec1)) / (1024.0 * 1024.0);
            std::cout << "       +rANS : " << emb1 << " MB"
                      << "  (" << (lossless1 ? "lossless" : "ERROR") << ")\n";
        }

        // Phase 2: Morton sort → delta+zigzag → rANS.
        // Delta expands the alphabet by 1 bit (bits → bits+1), so we need
        // bits+1 <= 12 to stay within the entropy coder's SCALE_BITS limit.
        if (bits + 1 <= 12) {
            QuantizedCloud qcs  = quantize(sortedCloud, bits);
            QuantizedCloud dc   = deltaEncodeCloud(qcs);
            EntropyCloud   ec2  = entropyEncode(dc);
            QuantizedCloud dcr  = entropyDecode(ec2);
            QuantizedCloud qcr2 = deltaDecodeCloud(dcr);
            bool lossless2 = (qcs.positions.data == qcr2.positions.data);
            double emb2 = static_cast<double>(entropyBytes(ec2)) / (1024.0 * 1024.0);
            std::cout << "  +delta+rANS : " << emb2 << " MB"
                      << "  (" << (lossless2 ? "lossless" : "ERROR") << ")\n";
        }
    }

    // --- Per-attribute bit depths ---
    // Positions get 16-bit precision; SH rest coefficients (the bulk of the data)
    // get 8-bit; scales and rotations get 12-bit as a middle ground.
    AttributeBits ab;  // uses struct defaults: pos=16, scales/rots=12, rest=8
    QuantizedCloud qcab = quantize(cloud, ab);
    GaussianCloud  rcab = dequantize(qcab);

    float maxErrAb = 0.0f;
    for (size_t i = 0; i < cloud.positions.size(); ++i)
        maxErrAb = std::max(maxErrAb, std::abs(cloud.positions[i] - rcab.positions[i]));

    double mbAb = static_cast<double>(quantizedDataBytes(qcab)) / (1024.0 * 1024.0);
    std::cout << "Per-attr    : " << mbAb << " MB"
              << "  (pos " << ab.positions << "-bit max-err " << maxErrAb
              << ", sh " << ab.sh << "-bit"
              << ", scales/rots " << ab.scales << "-bit)\n";

    // Entropy-code the per-attribute cloud. Each attribute uses its own scaleBits
    // = min(bits + 4, 16), so 16-bit positions use a 65536-slot model while 8-bit
    // SH stays at 4096 slots. No delta is applied here because delta of 16-bit
    // positions would produce a 17-bit alphabet that exceeds the 16-bit model limit.
    EntropyCloud   ecab  = entropyEncode(qcab);
    QuantizedCloud qcabr = entropyDecode(ecab);
    bool losslessAb = (qcab.positions.data == qcabr.positions.data &&
                       qcab.sh.data        == qcabr.sh.data);
    double embAb = static_cast<double>(entropyBytes(ecab)) / (1024.0 * 1024.0);
    std::cout << "Per-attr+rANS: " << embAb << " MB"
              << "  (" << (losslessAb ? "lossless" : "ERROR") << ")\n";

    // --- Per-attribute + smallest-three preprocessing ---
    // Rotations are reduced from 4 channels to 3 (dropping the largest-magnitude
    // component) before quantization. The dropped index (0-3) is stored as a 1-byte
    // sidecar per point and appended to the .3dgc file.
    {
        PreprocessFlags pf;
        pf.smallestThree = true;
        PreprocessedCloud pc = preprocess(cloud, pf);

        // Same bit depths as the per-attribute block; rotations now have 3 channels.
        QuantizedCloud qcst = quantize(pc.cloud, ab);

        // Measure quantization error on positions (unchanged by smallestThree).
        GaussianCloud rcst_q = dequantize(qcst);
        float maxErrSt = 0.0f;
        for (size_t i = 0; i < cloud.positions.size(); ++i)
            maxErrSt = std::max(maxErrSt,
                                std::abs(cloud.positions[i] - rcst_q.positions[i]));

        double mbSt = static_cast<double>(quantizedDataBytes(qcst)) / (1024.0 * 1024.0);
        std::cout << "  +SmallestThree: " << mbSt << " MB"
                  << "  (pos max-err " << maxErrSt
                  << ", rot 3-ch " << ab.rotations << "-bit)\n";

        EntropyCloud   ecst  = entropyEncode(qcst);
        QuantizedCloud qcstr = entropyDecode(ecst);
        bool losslessSt = (qcst.positions.data == qcstr.positions.data &&
                           qcst.rotations.data == qcstr.rotations.data);
        double embSt = static_cast<double>(entropyBytes(ecst)) / (1024.0 * 1024.0);
        std::cout << "  +SmallestThree+rANS: " << embSt << " MB"
                  << "  (" << (losslessSt ? "lossless" : "ERROR") << ")\n";

        // Full round-trip: write file, read back, decode, deprocess, check error.
        try {
            writeStream(ecst, pc, outPathST);

            StreamReadResult rdST  = readStream(outPathST);
            QuantizedCloud   qcrtd = entropyDecode(rdST.ec);
            GaussianCloud  grtd  = dequantize(qcrtd);

            // Deprocess: reconstruct 4-channel quaternions from the 3-channel codes.
            PreprocessedCloud pcRt;
            pcRt.cloud   = std::move(grtd);
            pcRt.rotDrop = rdST.rotDrop;
            pcRt.flags   = rdST.flags;
            GaussianCloud gFull = deprocess(pcRt);

            float maxErrRt = 0.0f;
            for (size_t i = 0; i < cloud.positions.size(); ++i)
                maxErrRt = std::max(maxErrRt,
                                    std::abs(cloud.positions[i] - gFull.positions[i]));

            std::ifstream fs(outPathST, std::ios::binary | std::ios::ate);
            double fileMBSt = static_cast<double>(fs.tellg()) / (1024.0 * 1024.0);

            bool rtOkSt = (qcst.positions.data == qcrtd.positions.data &&
                           qcst.rotations.data == qcrtd.rotations.data);
            std::string decompPathST = stem + "_st_decompressed.ply";
            saveGaussiansToPly(gFull, decompPathST);
            std::cout << "  Written ST  : " << outPathST
                      << "  (" << fileMBSt << " MB on disk"
                      << ", pos max-err " << maxErrRt
                      << ", " << (rtOkSt ? "lossless" : "ERROR") << ")\n"
                      << "  Decomp  ST  : " << decompPathST << "\n";
        } catch (const std::exception& e) {
            std::cerr << "I/O error (ST): " << e.what() << "\n";
        }
    }

    // --- Write base per-attribute .3dgc file (no preprocessing) ---
    try {
        writeStream(ecab, outPath);

        StreamReadResult rd    = readStream(outPath);
        QuantizedCloud qcDisk = entropyDecode(rd.ec); // rd is StreamReadResult
        GaussianCloud  rcDisk = dequantize(qcDisk);

        float maxErrDisk = 0.0f;
        for (size_t i = 0; i < cloud.positions.size(); ++i)
            maxErrDisk = std::max(maxErrDisk,
                                  std::abs(cloud.positions[i] - rcDisk.positions[i]));

        std::ifstream fs(outPath, std::ios::binary | std::ios::ate);
        double fileMB = static_cast<double>(fs.tellg()) / (1024.0 * 1024.0);

        bool rtOk = (qcab.positions.data == qcDisk.positions.data &&
                     qcab.sh.data        == qcDisk.sh.data);
        std::string decompPath = stem + "_decompressed.ply";
        saveGaussiansToPly(rcDisk, decompPath);
        std::cout << "Written     : " << outPath << "  (" << fileMB << " MB on disk"
                  << ", pos max-err " << maxErrDisk
                  << ", " << (rtOk ? "lossless" : "ERROR") << ")\n"
                  << "Decomp      : " << decompPath << "\n";
    } catch (const std::exception& e) {
        std::cerr << "I/O error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
