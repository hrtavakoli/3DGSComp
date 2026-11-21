#pragma once

#include "gaussian_cloud.h"
#include <string>
#include <stdexcept>

// Load a 3D Gaussian Splatting PLY file into a GaussianCloud.
// Throws std::runtime_error on I/O or parse failures.
GaussianCloud loadGaussiansFromPly(const std::string& filepath);

// Write a GaussianCloud to a binary little-endian PLY file using the same
// property names and layout as the original 3DGS training code. The file is
// directly loadable by standard 3DGS viewers and render scripts.
// Throws std::runtime_error on I/O failures.
void saveGaussiansToPly(const GaussianCloud& cloud, const std::string& filepath);
