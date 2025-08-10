#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <ikd-Tree/ikd_Tree.h>
#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <boost/filesystem.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <chrono>
#include <ctime>
#include "cluster_manager.hpp"
#include "ndt_3d.h"
#include "matplotlibcpp.h"
#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)
#include <nlohmann/json.hpp>
#include "lapjav.hpp"
#include "linear_sum_assignment.hpp"
#include "ssc.h"
#include "pmc.h"
// #include "ssc.cpp"
#ifdef USE_VGICP_CUDA
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <teaser/ply_io.h>
#include <teaser/registration.h>
namespace plt = matplotlibcpp; 
typedef std::vector<Eigen::Vector3f> Points;
typedef std::vector<int> IntVector;
typedef std::vector<std::vector<uint16_t>> BEVType;
typedef std::vector<std::vector<int>> CLDType;

// Constant definitions
const int BEV_WIDTH = 100;   // Width of BEV image
const int BEV_HEIGHT = 100;  // Height of BEV image
const double RESOLUTION = 0.8;   // Resolution, representing the actual distance per pixel (unit: meters)
const int EDGE_WEIGHT = 10;  // Edge weight
const int dis_thre = 3;      // Distance threshold when reading bin files
    // Define height intervals
float z_min = -5.0f, z_max = 20.0f;
float interval_size = 2.0f;  // 2 meters per interval
int num_intervals = 13;      // 13 intervals
// Hash function for pixel coordinates, used for hash table
struct PixelCoordHash {
    std::size_t operator()(const std::pair<int, int>& key) const {
        return std::hash<int>()(key.first) ^ (std::hash<int>()(key.second) << 1);
    }
};

// Create lookup table function to store the count of 1s in each number from 0-8192(2^13)
std::vector<uint16_t> create_lookup_table() {
    std::vector<uint16_t> lookup_table(1<<num_intervals, 0);
    for (int i = 0; i < 1<<num_intervals; i++) {
        int n = i;
        int count = 0;
        while (n) {
            count += n & 1;  // Check if current bit is 1
            n >>= 1;         // Right shift by one bit
        }
        lookup_table[i] = num_intervals - count;  // Save 13 minus the count of 1s
    }
    return lookup_table;
}

// Global lookup table
std::vector<uint16_t> lookup_table = create_lookup_table();

// Function to read point cloud files in bin format
void readBinFile(const std::string &file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) 
{
    // Open binary file for reading
    std::ifstream input(file_name.c_str(), std::ios::binary);
    if (!input.good()) {
        std::cerr << "Unable to read file: " << file_name << std::endl;
        return;
    }

    // Move to beginning of file
    input.seekg(0, std::ios::beg);

    // Read points from binary file
    while (input.good() && !input.eof()) {
        pcl::PointXYZ point;

        // Read 3 floats for x, y, z coordinates
        input.read((char *)&point.x, 3 * sizeof(float));

        // Skip intensity value (not used)
        if(dis_thre == 3)
            input.ignore(sizeof(float));

        // If read successfully, add point to point cloud
        if (input.good()) {
            cloud->points.push_back(point);
        }
    }

    // Close file after reading is complete
    input.close();

    // Set width and height of point cloud
    cloud->width = cloud->points.size();  // Number of points in point cloud
    cloud->height = 1;                    // Height set to 1 (single row of points)
    cloud->is_dense = true;               // Point cloud contains no NaN points
}

// Function to convert PCL point cloud to BEV (Bird's Eye View)
BEVType point_cloud_to_bev(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
    // Define physical size of BEV image (meters)
    double bev_size_meters = std::max(BEV_WIDTH, BEV_HEIGHT) * RESOLUTION;
    double half_size = bev_size_meters / 2;

    // Filter points outside z-axis range
    std::vector<pcl::PointXYZ> filtered_points;
    for (const auto& point : point_cloud->points) {
        if (point.z >= -5 && point.z < 20) {
            filtered_points.push_back(point);
        }
    }

    // Convert point cloud coordinates to image coordinates
    std::vector<int> x_img;
    std::vector<int> y_img;
    std::vector<pcl::PointXYZ> valid_points;

    for (const auto& point : filtered_points) {
        // Divide physical coordinates by resolution to get pixel coordinates, and translate origin to image center
        int x = static_cast<int>((point.x / RESOLUTION) + (BEV_WIDTH / 2));
        int y = static_cast<int>((point.y / RESOLUTION) + (BEV_HEIGHT / 2));

        // Filter out points outside image range
        if (x >= 0 && x < BEV_WIDTH && y >= 0 && y < BEV_HEIGHT) {
            x_img.push_back(x);
            y_img.push_back(y);
            valid_points.push_back(point);
        }
    }

    // Generate BEV image
    BEVType bev_img(BEV_HEIGHT, std::vector<uint16_t>(BEV_WIDTH, 0));

    // Create height statistics for each pixel position
    std::unordered_map<std::pair<int, int>, std::vector<int>, PixelCoordHash> height_counts;

    for (size_t i = 0; i < valid_points.size(); i++) {
        std::pair<int, int> key(y_img[i], x_img[i]);
        if (height_counts.find(key) == height_counts.end()) {
            height_counts[key] = std::vector<int>(num_intervals, 0);
        }

        // Calculate height interval of the point
        int interval_idx = static_cast<int>((valid_points[i].z - z_min) / interval_size);
        if (interval_idx >= 0 && interval_idx < num_intervals) {
            height_counts[key][interval_idx]++;
        }
    }

    // Generate binary representation and convert to decimal
    for (const auto& entry : height_counts) {
        int y = entry.first.first;
        int x = entry.first.second;
        const auto& counts = entry.second;

        uint16_t binary_value = 0;
        for (int i = 0; i < num_intervals; i++) {
            if (counts[i] >= 1) {  // If number of points in an interval is >= 1
                binary_value |= (1 << i);  // Set 1 at corresponding position
            }
        }
        bev_img[y][x] = binary_value;
        if (lookup_table[binary_value] == 12) {
            bev_img[y][x] = 0;  // Set value according to lookup table
        }
    }

    // Create final BEV image with edge padding
    BEVType final_bev(
        BEV_HEIGHT + EDGE_WEIGHT * 2, 
        std::vector<uint16_t>(BEV_WIDTH + EDGE_WEIGHT * 2, 0)
    );

    // Copy bev_img to center area of final_bev, forming edge padding
    for (int y = 0; y < BEV_HEIGHT; y++) {
        for (int x = 0; x < BEV_WIDTH; x++) {
            final_bev[y + EDGE_WEIGHT][x + EDGE_WEIGHT] = bev_img[y][x];
        }
    }

    return final_bev;
}

/**
 * Binary correlation calculation using OpenMP
 * 
 * @param A Source BEV image
 * @param B Template BEV image
 * @param lookup_table Lookup table for calculating Hamming distance
 * @return Correlation results
 */
std::vector<double> binary_correlation_2d_openmp(
    const BEVType& A,
    const BEVType& B,
    const std::vector<uint16_t>& lookup_table,
    std::vector<std::vector<int>> W) {
    
    // Get image dimensions
    int m_A = A.size();
    int n_A = A[0].size();
    int m_B = B.size();
    int n_B = B[0].size();
    
    // Calculate output dimensions
    int m_out = m_A - m_B + 1;
    int n_out = n_A - n_B + 1;
    // std::cout<<"m_out:"<<m_out<<" n_out:"<<n_out<<std::endl;
    // Check if dimensions are valid
    if (m_out <= 0 || n_out <= 0) {
        return std::vector<double>(2*2, 0.0);
    }
    

    
    // Calculate total number of non-zero elements for normalization
    int W_sum = m_B * n_B;
    if (W_sum == 0) {
        W_sum = m_B * n_B;  // Avoid division by zero
    }
    
    // Create result matrix
    // std::vector<std::vector<double>> result(m_out, std::vector<double>(n_out, 0.0));
    std::vector<double> result(m_out * n_out, 0.0);
    // Parallelize outer loop using OpenMP
    #pragma omp parallel for
    for (int i = 0; i < m_out; ++i) {
        for (int j = 0; j < n_out; ++j) {
            double weighted_sum = 0.0;
            for (int k = 0; k < m_B; ++k) {
                for (int l = 0; l < n_B; ++l) {
                    if (W[k][l] > 0) { // Only calculate non-zero weight positions
                        uint16_t xor_val = A[i+k][j+l] ^ B[k][l];
                        weighted_sum += lookup_table[xor_val];
                    }
                }
            }
            // Use one-dimensional index: i * n_out + j
            result[i * n_out + j] = weighted_sum / W_sum;
        }
    }
    
    return result;
}
// #endif // _OPENMP
std::vector<double> binary_correlation_2d_optimized_120x120(
    const BEVType& A,  // Fixed at 120×120
    const BEVType& B,  // Smaller than A
    const std::vector<uint16_t>& lookup_table,
    const std::vector<std::vector<int>>& W) {
    
    // B dimensions (dynamic but known to be smaller than 120×120)
    const int m_B = B.size();
    const int n_B = B[0].size();
    
    // Calculate output dimensions (A is known to be 120×120)
    const int m_out = 120 - m_B + 1;
    const int n_out = 120 - n_B + 1;
    
    // Pre-calculate effective weights
    int W_sum = 0;
    // Create flattened weight matrix and B matrix for better cache locality
    std::vector<int> flat_W;
    std::vector<uint16_t> flat_B;
    flat_W.reserve(m_B * n_B);
    flat_B.reserve(m_B * n_B);
    
    for (int k = 0; k < m_B; ++k) {
        for (int l = 0; l < n_B; ++l) {
            flat_W.push_back(W[k][l]);
            flat_B.push_back(B[k][l]);
            if (W[k][l] > 0) {
                W_sum += W[k][l];
            }
        }
    }
    
    if (W_sum == 0) {
        W_sum = m_B * n_B;
    }
    
    const double inv_W_sum = 1.0 / W_sum;
    
    // Create result vector
    std::vector<double> result(m_out * n_out, 0.0);
    
    // If B matrix is small (e.g. smaller than 20×20), use fully unrolled method
    if (m_B * n_B <= 400) {
        // Create specific optimization path for small templates
        #pragma omp parallel for collapse(2)
        for (int i = 0; i < m_out; ++i) {
            for (int j = 0; j < n_out; ++j) {
                double weighted_sum = 0.0;
                int flat_idx = 0;
                
                // Fully unrolled loop (compiler will optimize further)
                for (int k = 0; k < m_B; ++k) {
                    for (int l = 0; l < n_B; ++l) {
                        int w = flat_W[flat_idx];
                        if (w > 0) {
                            uint16_t xor_val = A[i+k][j+l] ^ flat_B[flat_idx];
                            weighted_sum += lookup_table[xor_val] * w;
                        }
                        flat_idx++;
                    }
                }
                
                result[i * n_out + j] = weighted_sum * inv_W_sum;
            }
        }
    } else {
        // For larger B matrices, use SIMD-friendly tiling method
        #pragma omp parallel
        {
            // Each thread creates a local accumulator to avoid false sharing
            const int tile_size = 16; // Adjust to fit L1 cache
            
            #pragma omp for
            for (int i_block = 0; i_block < m_out; i_block += tile_size) {
                for (int j_block = 0; j_block < n_out; j_block += tile_size) {
                    // Process this tile block
                    const int i_end = std::min(i_block + tile_size, m_out);
                    const int j_end = std::min(j_block + tile_size, n_out);
                    
                    for (int i = i_block; i < i_end; ++i) {
                        for (int j = j_block; j < j_end; ++j) {
                            double weighted_sum = 0.0;
                            int flat_idx = 0;
                            
                            // Inner loop optimized for 120×120
                            for (int k = 0; k < m_B; ++k) {
                                const int row_idx = i + k;
                                for (int l = 0; l < n_B; ++l) {
                                    int w = flat_W[flat_idx];
                                    if (w > 0) {
                                        uint16_t xor_val = A[row_idx][j+l] ^ flat_B[flat_idx];
                                        weighted_sum += lookup_table[xor_val] * w;
                                    }
                                    flat_idx++;
                                }
                            }
                            
                            result[i * n_out + j] = weighted_sum * inv_W_sum;
                        }
                    }
                }
            }
        }
    }
    
    return result;
}

/**
 * Crop BEV image to minimum size containing all non-zero values
 * 
 * @param bev_image Input BEV image 2D array
 * @return Tuple containing cropped image, starting coordinates, and crop dimensions
 */
BEVType //, std::pair<int, int>, std::pair<int, int>> 
crop_to_minimum_bev(const BEVType& bev_image) {
    // Image dimension check
    if (bev_image.empty() || bev_image[0].empty()) {
        return BEVType();
    }
    
    int height = bev_image.size();
    int width = bev_image[0].size();
    
    // Initialize boundary values
    int min_y = std::numeric_limits<int>::max();
    int max_y = -1;
    int min_x = std::numeric_limits<int>::max();
    int max_x = -1;
    
    // Find boundaries of all non-zero values
    bool has_non_zero = false;
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (bev_image[y][x] > 0) {
                has_non_zero = true;
                min_y = std::min(min_y, y);
                max_y = std::max(max_y, y);
                min_x = std::min(min_x, x);
                max_x = std::max(max_x, x);
            }
        }
    }
    
    // If no non-zero values, return original image and zero dimensions
    if (!has_non_zero) {
        return bev_image;
    }
    
    // Calculate crop dimensions
    int crop_width = max_x - min_x + 1;
    int crop_height = max_y - min_y + 1;
    
    // Create cropped image
    BEVType cropped_image(
        crop_height, 
        std::vector<uint16_t>(crop_width, 0)
    );
    
    // Crop image
    for (int y = 0; y < crop_height; ++y) {
        for (int x = 0; x < crop_width; ++x) {
            cropped_image[y][x] = bev_image[min_y + y][min_x + x];
        }
    }
    return cropped_image;
    // return std::make_tuple(
    //     cropped_image, 
    //     std::make_pair(min_x, min_y), 
    //     std::make_pair(crop_width, crop_height)
    // );
}


/**
 * Load pose data from file
 * 
 * Read pose data from specified file, each line format is (t,x,y,z,qx,qy,qz,qw),
 * where t is timestamp, (x,y,z) is position coordinates, (qx,qy,qz,qw) is quaternion for rotation.
 * 
 * @param filepath Path to input file
 * @return Vector of position and rotation matrix pairs, where position is 3D vector and rotation is 3×3 matrix
 */
std::vector<std::pair<Eigen::Vector3f, Eigen::Matrix3f>>
load_poses_from_hfx(const std::string filepath) {
    double t, x, y, z, qx, qy, qz, qw;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Matrix3f>> pose_vec;
    
    std::ifstream posereader(filepath);

    int count = 0;
    while (posereader >> t >> x >> y >> z >> qx >> qy >> qz >> qw) {
        // Create translation vector directly from x, y, z
        Eigen::Vector3f translation(x, y, z);
        
        // Convert quaternion to rotation matrix
        Eigen::Quaterniond quaternion(qw, qx, qy, qz); // Note: Eigen constructor is (w,x,y,z)
        quaternion.normalize(); // Ensure the quaternion is normalized
        Eigen::Matrix3f rotation = quaternion.toRotationMatrix().cast<float>();
        
        // Apply BASE2OUSTER transformation if needed (keeping this from original code)
        Eigen::Matrix4f temp_pose = Eigen::Matrix4f::Identity();
        temp_pose.block<3, 3>(0, 0) = rotation;
        temp_pose.block<3, 1>(0, 3) = translation;
        // temp_pose = temp_pose * BASE2OUSTER;
        
        // Extract the transformed rotation and translation
        rotation = temp_pose.block<3, 3>(0, 0);
        translation = temp_pose.block<3, 1>(0, 3);
        
        // Add to the pose vector
        pose_vec.push_back(std::make_pair(translation, rotation));
        count++;
    }
    
    return pose_vec;
}

void compute_adj_rpe(Eigen::Matrix4d& gt,
                     Eigen::Matrix4d& lo,
                     double& t_e,
                     double& r_e) {
    Eigen::Matrix4d delta_T = lo.inverse() * gt;

    t_e = delta_T.topRightCorner(3, 1).norm();

    r_e = std::abs(std::acos(
                  fmin(fmax((delta_T.block<3, 3>(0, 0).trace() - 1) / 2, -1.0),
                       1.0))) /
          M_PI * 180;
}

double time_inc(std::chrono::_V2::system_clock::time_point &t_end,
                std::chrono::_V2::system_clock::time_point &t_begin) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(t_end -
                                                                   t_begin)
             .count() *
         1000;
}



///2025-03-07///


/**
 * Find start and end positions of the unique continuous zero region in descriptor
 * 
 * @param descriptor One-dimensional array, LiDAR descriptor
 * @param zero_threshold Float, values less than or equal to this threshold are considered "zero values"
 * @return Pair containing start index and end index of zero region (end index not included)
 *         Returns {-1, -1} if no zero values found
 */
std::pair<int, int> find_single_zero_region(const std::vector<float>& descriptor, float zero_threshold = 0.0f) 
{
    // Return directly if descriptor is empty
    if (descriptor.empty()) {
        return {-1, -1};
    }
    
    // Create index array marking zero value positions
    std::vector<int> zero_indices;
    for (int i = 0; i < descriptor.size(); ++i) {
        if (descriptor[i] <= zero_threshold) {
            zero_indices.push_back(i);
        }
    }
    
    // Return directly if no zero values
    if (zero_indices.empty()) {
        return {-1, -1};
    }
    
    // If only single zero value
    if (zero_indices.size() == 1) {
        return {zero_indices[0], zero_indices[0] + 1};
    }
    
    // Check if it's a continuous region
    if (zero_indices.back() - zero_indices.front() == zero_indices.size() - 1) {
        return {zero_indices.front(), zero_indices.back() + 1};
    }
    
    // If zero values are not continuous, find the longest continuous segment
    std::vector<std::vector<int>> segments;
    std::vector<int> current_segment = {zero_indices[0]};
    
    for (int i = 1; i < zero_indices.size(); ++i) {
        if (zero_indices[i] == zero_indices[i-1] + 1) {
            // Current index is continuous with previous index
            current_segment.push_back(zero_indices[i]);
        } else {
            // Current index is not continuous with previous index, start new segment
            segments.push_back(current_segment);
            current_segment.clear();
            current_segment.push_back(zero_indices[i]);
        }
    }
    
    // Add the last segment
    if (!current_segment.empty()) {
        segments.push_back(current_segment);
    }
    
    // Find the longest segment
    auto longest_segment = std::max_element(
        segments.begin(), segments.end(),
        [](const std::vector<int>& a, const std::vector<int>& b) {
            return a.size() < b.size();
        }
    );
    
    // Return start and end indices of longest segment
    // return {(*longest_segment).front(), (*longest_segment).back() + 1};
    if (((*longest_segment).back() + 1 - (*longest_segment).front()) <= descriptor.size()/3) {
        return {-1, -1};
    } else {
        return {(*longest_segment).front(), (*longest_segment).back() + 1};
    }
}
/**
 * Calculate similarity between two descriptors and handle zero regions
 * 
 * @param a First 2D descriptor
 * @param b Second 2D descriptor
 * @param newtable Lookup table for similarity calculation
 * @return Similarity score vector
 */
std::vector<float> complete_similarity(const CLDType& b, 
                                      const CLDType& a,
                                      const std::vector<int>& newtable,
                                      int& query_size,
                                      bool& flag_rot,
                                      int& offset_sector,
                                      int& zero_len) {
    // Calculate sum of each row
    std::vector<float> row_sums(a.size(), 0.0f);
    for (size_t i = 0; i < a.size(); ++i) {
        row_sums[i] = std::accumulate(a[i].begin(), a[i].end(), 0.0f);
    }
    
    // Find zero region
    auto [zero_start, zero_end] = find_single_zero_region(row_sums);
    // std::cout<<"find_single_zero_region"<<std::endl;
    // Create final_CLD, removing zero region
    CLDType final_CLD;
    // final_CLD.reserve(a.size()); // Pre-allocate memory for efficiency
    
    // Add part from zero_end to end
    if (zero_start!=zero_end)
    {
        for (size_t i = zero_end; i < a.size(); ++i) {
            final_CLD.push_back(a[i]);
        }
        
        // Add part from beginning to zero_start
        zero_len = zero_end - zero_start;
        for (size_t i = 0; i < zero_start; ++i) {
            final_CLD.push_back(a[i]);
        }
        flag_rot = true;
        if (zero_len <= a.size()/2)
            offset_sector = zero_end - 1; // - 2
        else 
            offset_sector = int(final_CLD.size()/2) -1; // - 2
        
    }
    else{
        final_CLD = a;
    }
    // std::cout<<" final_CLD:"<<final_CLD.size()<<std::endl;
    // std::cout<<"find_single_zero_region1"<<std::endl;
    // Create final_map, expanding b
    CLDType final_map;
    // final_map.reserve(b.size() + final_CLD.size() - 1); // Pre-allocate memory
    
    // Add rear part of b
    size_t start_idx = (b.size() > final_CLD.size() - 1) ? 
                      (b.size() - final_CLD.size() + 1) : 0;
    for (size_t i = start_idx; i < b.size(); ++i) {
        final_map.push_back(b[i]);
    }
    
    // Add all parts of b
    for (const auto& row : b) {
        final_map.push_back(row);
    }
    // std::cout<<"find_single_zero_region2"<<std::endl;
    // Initialize result array
    size_t result_size = final_map.size() - final_CLD.size() + 1;
    std::vector<float> result_cosine(result_size, 0.0f);
    query_size = final_CLD.size();
    // std::cout<<"result_size"<<result_size<<" final_map"<<final_map.size()<<" final_CLD"<<final_CLD.size()<<std::endl;
    // Efficient sliding window calculation
    for (size_t i = 0; i < result_size; ++i) {
        float sum = 0.0f;
        
        // Calculate similarity for current window
        for (size_t j = 0; j < final_CLD.size(); ++j) {
            const auto& window_row = final_map[i + j];
            const auto& cld_row = final_CLD[j];
            
            // Ensure both rows have same length
            size_t col_size = std::min(window_row.size(), cld_row.size());
            
            for (size_t k = 0; k < col_size; ++k) {
                // Calculate index
                int index = static_cast<int>(window_row[k] + cld_row[k]);
                sum += newtable[index];
            }
        }
        
        result_cosine[i] = sum;
    }
    
    return result_cosine;
}

/**
 * Build sector feature descriptor
 * 
 * @param point_cloud Input point cloud
 * @param num_sectors Number of sectors divided, each sector has degrees of 360/num_sectors
 * @param max_range Maximum point cloud distance
 * @param min_points_per_sector Minimum number of point clouds
 * @return Sector feature descriptor
 */
CLDType generate_cld_my(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, 
                                                int num_sectors = 24, 
                                                float max_range = 50.0f, 
                                                int min_points_per_sector = 3) {
    // Basic preprocessing and validity check
    if (point_cloud->points.empty()) {
        // Return all-zero descriptor
        return CLDType(num_sectors, std::vector<int>(0, 0));
    }
    
    // Height filtering parameters
    const float z_min = -1.0f, z_max = 12.0f;
    const float interval_size = 1.0f;
    const float interval_size_r = 2.0f;
    
    // Store filtered points
    std::vector<pcl::PointXYZ> valid_points;

    // Filter points based on height and distance
    for (const auto& point : point_cloud->points) {
        float r_xy = std::sqrt(point.x * point.x + point.y * point.y);
        
        if (point.z >= z_min && point.z < z_max && r_xy < max_range) {
            valid_points.push_back(point);
        }
    }
    
    // Check if there are enough points after filtering
    if (valid_points.size() < min_points_per_sector) {
        return CLDType (num_sectors, std::vector<int>(0, 0));
    }
    
    // Calculate sector size
    const float sector_size = 2.0f * M_PI / num_sectors;
    
    // Initialize descriptor
    int rz_dim = int(max_range / interval_size_r) * int((z_max - z_min) / interval_size);
    CLDType sector_vector(num_sectors, std::vector<int>(rz_dim, 0));
    
    // Count points in each sector
    std::vector<int> sector_points_count(num_sectors, 0);
    std::vector<std::vector<std::pair<int, int>>> sector_rz(num_sectors);
    
    // Assign points to sectors
    for (size_t i = 0; i < valid_points.size(); i++) {
        const auto& point = valid_points[i];
        
        // Calculate polar coordinates
        float r = std::sqrt(point.x * point.x + point.y * point.y);
        float theta = std::atan2(point.y, point.x);
        
        // Ensure theta is in [0, 2π) range
        if (theta < 0) theta += 2.0f * M_PI;
        
        // Determine sector index
        int sector_idx = int(std::floor(theta / sector_size)) % num_sectors;
        
        // Store r and z values for this sector
        sector_rz[sector_idx].push_back(std::make_pair(r, point.z));
        sector_points_count[sector_idx]++;
    }
    
    // Process each sector
    for (int sector_idx = 0; sector_idx < num_sectors; sector_idx++) {
        // Skip sectors with insufficient points
        if (sector_points_count[sector_idx] < min_points_per_sector) {
            continue;
        }
        
        // Create 2D grid representation (rz plane)
        CLDType new_rz(
            int((z_max - z_min) / interval_size),
            std::vector<int>(int(max_range / interval_size_r), 0)
        );
        
        // Fill grid with point data
        for (const auto& rz_pair : sector_rz[sector_idx]) {
            float r_value = rz_pair.first;
            float z_value = rz_pair.second;
            
            int interval_idz = int((z_value - z_min) / interval_size);
            int interval_idr = int(r_value / interval_size_r);
            
            // Boundary check
            if (interval_idz >= 0 && interval_idz < new_rz.size() && 
                interval_idr >= 0 && interval_idr < new_rz[0].size()) {
                new_rz[interval_idz][interval_idr] = 1;
            }
        }
        
        // Flatten 2D grid to 1D array
        std::vector<int>& sector_data = sector_vector[sector_idx];
        for (int iz = 0; iz < new_rz.size(); iz++) {
            for (int ir = 0; ir < new_rz[iz].size(); ir++) {
                int flat_idx = iz * new_rz[iz].size() + ir;
                if (flat_idx < sector_data.size()) {
                    sector_data[flat_idx] = new_rz[iz][ir];
                }
            }
        }
    }
    
    return sector_vector;
}

/**
 * Feature descriptor storage structure
 * 1. Contains point cloud from map part (stores corresponding path)
 * 2. Corresponding retrieved rotation change
 * 3. Added score
 * 4. Pose change
 * 
 * 
 */
struct Gene_BPR{
    float rotation;
    Eigen::Matrix4f pose;
    double final_score;
    std::string path;
    Eigen::Matrix4f transformation;
    Eigen::Matrix4f rotation_matrix;
    int list_n;
    Gene_BPR(const float& rotation,
               const Eigen::Matrix4f& pose,
               const double& final_score,
               const std::string& path,
               const Eigen::Matrix4f& transformation,
               const Eigen::Matrix4f& rotation_matrix,
               const int& list_n)
        : rotation(rotation), pose(pose), final_score(final_score),path(path),transformation(transformation),rotation_matrix(rotation_matrix),list_n(list_n){}
    Gene_BPR(){};
};

using Pose = std::pair<Eigen::Vector3f, Eigen::Matrix3f>;
using PoseVector = std::vector<Pose>;
/**
 * Find the pose closest in position to query pose in pose vector
 * @param poses Pose vector
 * @param query_pose Query pose
 * @return Index and distance of nearest pose
 */
float findNearestPosePosition(
    const PoseVector& poses,
    const Pose& query_pose) {
    
    float min_dist = std::numeric_limits<float>::max();
    size_t min_idx = 0;
    
    // Query position
    const Eigen::Vector3f& query_position = query_pose.first;
    
    // Traverse all poses, comparing only positions
    for (size_t i = 0; i < poses.size(); ++i) {
        // Calculate Euclidean distance between positions
        float dist = (poses[i].first - query_position).norm();
        
        if (dist < min_dist) {
            min_dist = dist;
            min_idx = i;
        }
    }
    
    return min_dist;
}

double computeOverlapFast(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& small_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& large_cloud,
    float distance_threshold = 0.05,
    float voxel_size = 0.1) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr small_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr large_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Downsample to improve efficiency (if voxel_size > 0)
    if (voxel_size > 0) {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        
        vg.setInputCloud(small_cloud);
        vg.setLeafSize(voxel_size, voxel_size, voxel_size);
        vg.filter(*small_filtered);
        
        vg.setInputCloud(large_cloud);
        vg.setLeafSize(voxel_size, voxel_size, voxel_size);
        vg.filter(*large_filtered);
    } else {
        small_filtered = small_cloud;
        large_filtered = large_cloud;
    }
    
    // Build KD tree (using large point cloud)
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(large_filtered);
    
    // Pre-calculate squared threshold to avoid repeated calculations
    float squared_threshold = distance_threshold * distance_threshold;
    
    // Statistics variables
    int overlap_count = 0;
    const int small_cloud_size = small_filtered->size();
    
    // Parallel processing for efficiency
    #pragma omp parallel
    {
        // Local counter for each thread
        int local_count = 0;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        
        // Parallel block processing
        #pragma omp for nowait
        for (int i = 0; i < small_cloud_size; ++i) {
            // Use radius search instead of K-nearest neighbor to avoid unnecessary distance sorting
            if (kdtree.radiusSearch(small_filtered->points[i], distance_threshold, 
                                   pointIdxRadiusSearch, pointRadiusSquaredDistance, 1) > 0) {
                local_count++;
            }
        }
        
        // Merge thread local results
        #pragma omp atomic
        overlap_count += local_count;
    }
    
    // Calculate overlap ratio
    return static_cast<double>(overlap_count) / small_cloud_size;
}