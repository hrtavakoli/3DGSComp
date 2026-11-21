#define TINYPLY_IMPLEMENTATION
#include "tinyply.h"

#include "ply_loader.h"

#include <fstream>
#include <cstring>
#include <memory>
#include <stdexcept>

// Helper: copy a tinyply float buffer into a std::vector<float>.
static void copyBuffer(const std::shared_ptr<tinyply::PlyData>& data,
                       std::vector<float>& out,
                       int32_t numPoints,
                       int floatsPerPoint) {
    if (!data || data->count == 0) {
        out.assign(static_cast<size_t>(numPoints) * floatsPerPoint, 0.0f);
        return;
    }
    out.resize(static_cast<size_t>(numPoints) * floatsPerPoint);
    std::memcpy(out.data(), data->buffer.get(),
                static_cast<size_t>(numPoints) * floatsPerPoint * sizeof(float));
}

// Helper: try to request a property group; returns nullptr if any property is missing.
static std::shared_ptr<tinyply::PlyData> tryRequest(
    tinyply::PlyFile& file,
    const std::string& element,
    const std::vector<std::string>& props)
{
    try {
        return file.request_properties_from_element(element, props);
    } catch (...) {
        return nullptr;
    }
}

GaussianCloud loadGaussiansFromPly(const std::string& filepath) {
    std::ifstream stream(filepath, std::ios::binary);
    if (!stream.is_open()) {
        throw std::runtime_error("Cannot open file: " + filepath);
    }

    tinyply::PlyFile file;
    file.parse_header(stream);

    // --- Mandatory properties ---
    auto posData = file.request_properties_from_element("vertex", {"x", "y", "z"});
    auto nrmData = tryRequest(file, "vertex", {"nx", "ny", "nz"});
    auto dcData  = file.request_properties_from_element("vertex", {"f_dc_0", "f_dc_1", "f_dc_2"});
    auto opData  = file.request_properties_from_element("vertex", {"opacity"});
    auto sclData = file.request_properties_from_element("vertex", {"scale_0", "scale_1", "scale_2"});
    auto rotData = file.request_properties_from_element("vertex", {"rot_0", "rot_1", "rot_2", "rot_3"});

    // SH rest coefficients are stored as f_rest_0 … f_rest_N in the PLY vertex element.
    // The layout follows the original 3DGS training code: coefficients are interleaved
    // by index across all three colour channels — R₀G₀B₀ R₁G₁B₁ … — not channel-first.
    // The SH degree varies per file (degree 0 → 0 props, degree 3 → 45 props), so we
    // count what is actually present before calling request_properties_from_element,
    // which throws if asked for a property that doesn't exist in the header.
    std::vector<std::string> restNames;
    for (int i = 0; i < 45; ++i)
        restNames.push_back("f_rest_" + std::to_string(i));

    int restCount = 0;
    for (const auto& element : file.get_elements()) {
        if (element.name != "vertex") continue;
        for (const auto& prop : element.properties) {
            if (prop.name.rfind("f_rest_", 0) == 0)
                ++restCount;
        }
        break;
    }

    std::shared_ptr<tinyply::PlyData> shData;
    if (restCount > 0) {
        std::vector<std::string> activeRest(restNames.begin(),
                                            restNames.begin() + restCount);
        shData = file.request_properties_from_element("vertex", activeRest);
    }

    // --- Read all data ---
    file.read(stream);

    if (!posData || posData->count == 0) {
        throw std::runtime_error("PLY file has no vertex positions: " + filepath);
    }

    GaussianCloud cloud;
    cloud.numPoints = static_cast<int32_t>(posData->count);
    cloud.shDegree  = degreeForRestCount(restCount);

    copyBuffer(posData, cloud.positions,  cloud.numPoints, 3);
    copyBuffer(nrmData, cloud.normals,    cloud.numPoints, 3);
    copyBuffer(dcData,  cloud.colors,     cloud.numPoints, 3);
    copyBuffer(opData,  cloud.opacities,  cloud.numPoints, 1);
    copyBuffer(sclData, cloud.scales,     cloud.numPoints, 3);
    copyBuffer(rotData, cloud.rotations,  cloud.numPoints, 4);

    if (shData && restCount > 0) {
        cloud.sh.resize(static_cast<size_t>(cloud.numPoints) * restCount);
        std::memcpy(cloud.sh.data(), shData->buffer.get(),
                    static_cast<size_t>(cloud.numPoints) * restCount * sizeof(float));
    }

    return cloud;
}

void saveGaussiansToPly(const GaussianCloud& cloud, const std::string& filepath)
{
    std::ofstream outstream(filepath, std::ios::binary);
    if (!outstream) throw std::runtime_error("Cannot open for writing: " + filepath);

    const int32_t N         = cloud.numPoints;
    const int     restCount = cloud.shCoeffsPerPoint();

    // tinyply's write API takes a non-const uint8_t*. The buffer is only read
    // during write(), never modified, so const_cast is safe here.
    auto buf = [](const std::vector<float>& v) {
        return reinterpret_cast<uint8_t*>(const_cast<float*>(v.data()));
    };

    tinyply::PlyFile ply;

    // Property order matches the original 3DGS training output so that viewers
    // and render scripts that read properties by position rather than name work
    // correctly.
    ply.add_properties_to_element("vertex", {"x", "y", "z"},
        tinyply::Type::FLOAT32, static_cast<size_t>(N), buf(cloud.positions),
        tinyply::Type::INVALID, 0);

    ply.add_properties_to_element("vertex", {"nx", "ny", "nz"},
        tinyply::Type::FLOAT32, static_cast<size_t>(N), buf(cloud.normals),
        tinyply::Type::INVALID, 0);

    ply.add_properties_to_element("vertex", {"f_dc_0", "f_dc_1", "f_dc_2"},
        tinyply::Type::FLOAT32, static_cast<size_t>(N), buf(cloud.colors),
        tinyply::Type::INVALID, 0);

    if (restCount > 0) {
        std::vector<std::string> restNames;
        restNames.reserve(restCount);
        for (int i = 0; i < restCount; ++i)
            restNames.push_back("f_rest_" + std::to_string(i));

        ply.add_properties_to_element("vertex", restNames,
            tinyply::Type::FLOAT32, static_cast<size_t>(N), buf(cloud.sh),
            tinyply::Type::INVALID, 0);
    }

    ply.add_properties_to_element("vertex", {"opacity"},
        tinyply::Type::FLOAT32, static_cast<size_t>(N), buf(cloud.opacities),
        tinyply::Type::INVALID, 0);

    ply.add_properties_to_element("vertex", {"scale_0", "scale_1", "scale_2"},
        tinyply::Type::FLOAT32, static_cast<size_t>(N), buf(cloud.scales),
        tinyply::Type::INVALID, 0);

    ply.add_properties_to_element("vertex", {"rot_0", "rot_1", "rot_2", "rot_3"},
        tinyply::Type::FLOAT32, static_cast<size_t>(N), buf(cloud.rotations),
        tinyply::Type::INVALID, 0);

    ply.write(outstream, true); // binary little-endian
}
