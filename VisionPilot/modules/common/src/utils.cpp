#include <common/utils.hpp>

#include <cstdlib>
#include <stdexcept>
#include <vector>

namespace {

std::string env_or_empty(const char* name)
{
    const char* v = std::getenv(name);
    return (v && *v) ? std::string(v) : std::string();
}

}  // namespace

// Search order (first hit wins; only step 1 when VISIONPILOT_CONFIG_DIR_STRICT is set):
//   1. $VISIONPILOT_CONFIG_DIR/<filename>      (e.g. vehicle-specific config set by a launch file)
//   2. ./config/<filename>                     (running from the build/source tree)
//   3. $VISIONPILOT_SHARE_DIR/config/<filename> (ROS 2 install: share/vision_pilot)
//   4. /usr/share/visionpilot/config/<filename> (Debian package / Docker image)
std::string find_config(const std::string& filename)
{
    std::vector<std::string> candidates;
    const auto config_dir = env_or_empty("VISIONPILOT_CONFIG_DIR");
    if (!config_dir.empty())
        candidates.push_back(config_dir + "/" + filename);

    // VISIONPILOT_CONFIG_DIR_STRICT=1: never fall back to the generic (passenger-car)
    // defaults. Used by vehicle integrations so that a missing vehicle file is an error.
    if (!config_dir.empty() && !env_or_empty("VISIONPILOT_CONFIG_DIR_STRICT").empty())
    {
        if (std::filesystem::exists(candidates.front())) return candidates.front();
        throw std::runtime_error("Config file not found (strict): " + candidates.front());
    }

    candidates.push_back("config/" + filename);
    if (const auto d = env_or_empty("VISIONPILOT_SHARE_DIR"); !d.empty())
        candidates.push_back(d + "/config/" + filename);
    candidates.push_back("/usr/share/visionpilot/config/" + filename);

    for (const auto& c : candidates)
        if (std::filesystem::exists(c)) return c;

    throw std::runtime_error("Config file not found: " + filename);
}

// Search order: ./<relpath>, $VISIONPILOT_SHARE_DIR/<relpath>, /usr/share/visionpilot/<relpath>
std::string find_resource(const std::string& relpath)
{
    std::vector<std::string> candidates{relpath};
    if (const auto d = env_or_empty("VISIONPILOT_SHARE_DIR"); !d.empty())
        candidates.push_back(d + "/" + relpath);
    candidates.push_back("/usr/share/visionpilot/" + relpath);

    for (const auto& c : candidates)
        if (std::filesystem::exists(c)) return c;

    return {};
}

cv::Mat load_matrix(const std::string& filename, const std::string& matrix)
{
    const std::string path = find_config(filename);
    const cv::FileStorage fs(path, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        throw std::runtime_error("Failed to open calibration file: " + path);
    }
    cv::Mat M;
    fs[matrix] >> M;
    if (M.empty())
    {
        throw std::runtime_error("Matrix '" + matrix + "' not found in " + path);
    }

    return M;
}

cv::Mat compute_homography_C_matrix(const cv::Mat& H_in)
{
    // DO NOT MODIFY: VisionPilot model-view homography (1024x512 pixel -> world).
    // Must match V in scripts/find_homography_C_matrix.py.
    const cv::Matx33d V(
        0.00209514907, -0.000941721466, -9.24906396,
        0.00662758637, -0.000352940531, -3.33396502,
        0.000120077371, -0.00411343505, 1.0);
    // Canonical ground points (X fwd, Y left) in metres.
    const cv::Vec3d world[4] = {{15, 5, 1}, {150, 5, 1}, {15, -5, 1}, {150, -5, 1}};

    cv::Mat H;
    H_in.convertTo(H, CV_64F);
    const cv::Matx33d H_inv = cv::Matx33d(H).inv();
    const cv::Matx33d V_inv = V.inv();

    std::vector<cv::Point2f> uv, pq;
    for (const auto& w : world)
    {
        const cv::Vec3d a = H_inv * w;
        const cv::Vec3d b = V_inv * w;
        uv.emplace_back(static_cast<float>(a[0] / a[2]), static_cast<float>(a[1] / a[2]));
        pq.emplace_back(static_cast<float>(b[0] / b[2]), static_cast<float>(b[1] / b[2]));
    }
    cv::Mat C = cv::findHomography(uv, pq, 0);
    if (C.empty())
        throw std::runtime_error("Failed to compute homography C from H (degenerate H.yaml?)");
    cv::Mat C32;
    C.convertTo(C32, CV_32F);
    return C32;
}
