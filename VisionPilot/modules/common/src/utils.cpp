#include <common/utils.hpp>

std::string find_config(const std::string& filename)
{
    const std::string local = "config/" + filename;
    const std::string system = "/usr/share/visionpilot/config/" + filename;

    if (std::filesystem::exists(local)) return local;
    if (std::filesystem::exists(system)) return system;

    throw std::runtime_error("Config file not found: " + filename);
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
