#pragma once

#include <cstdint>
#include <vector>

// SH rest-coefficient counts per point (excluding DC term) for each degree:
//   degree 0 →  0
//   degree 1 →  9  (3 channels × 3 coeffs)
//   degree 2 → 24  (3 channels × 8 coeffs)
//   degree 3 → 45  (3 channels × 15 coeffs)

inline int restCountForDegree(int degree) {
    static const int counts[] = {0, 9, 24, 45};
    if (degree < 0 || degree > 3) return 0;
    return counts[degree];
}

inline int degreeForRestCount(int restCount) {
    if (restCount >= 45) return 3;
    if (restCount >= 24) return 2;
    if (restCount >= 9)  return 1;
    return 0;
}

// Structure-of-Arrays layout: each vector holds numPoints × k floats.
struct GaussianCloud {
    int32_t numPoints = 0;
    int32_t shDegree  = 0;

    std::vector<float> positions;   // 3 floats/point  x y z
    std::vector<float> normals;     // 3 floats/point  nx ny nz (typically zero in trained scenes)
    std::vector<float> colors;      // 3 floats/point  SH DC term — NOT gamma-corrected RGB; apply SH activation to shade
    std::vector<float> sh;          // shCoeffsPerPoint() floats/point  f_rest_* interleaved as R₀G₀B₀ R₁G₁B₁ …
    std::vector<float> opacities;   // 1 float/point   raw logit — apply sigmoid() to get [0, 1] alpha
    std::vector<float> scales;      // 3 floats/point  log-scale — apply exp() to get world-space half-axes
    std::vector<float> rotations;   // 4 floats/point  unit quaternion (w, x, y, z) — rot_0 is w

    int shCoeffsPerPoint() const { return restCountForDegree(shDegree); }
};
