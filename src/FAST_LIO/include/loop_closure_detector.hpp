#pragma once
/**
 * @file loop_closure_detector.hpp
 * @brief Loop Closure Detection Module based on CLD + FastGICP Geometric Verification
 *
 * 基于 seu_hfx.hpp 中的 CLD 描述符做初筛，再用 FastGICP + Overlap 做几何验证。
 * 流程与 HL_BPR_v2.cpp 完全一致：
 *   1) CLD 相似度排序 → Top-K 候选
 *   2) 对每个候选：用旋转偏移初始化 → FastGICP 配准 → computeOverlapFast 打分
 *   3) overlap > threshold → 确认回环，提前终止
 *
 * 用法:
 *   LoopClosureDetector detector(config);
 *   detector.addFrame(cloud);
 *   auto result = detector.addAndDetect(cloud);
 *   if (result.detected)   回环已确认，result.transformation 可用 
 */

#include <vector>
#include <deque>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <iostream>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>

#include <fast_gicp/gicp/fast_gicp.hpp>


// ============================================================
//  回环检测结果
// ============================================================
struct LoopClosureResult {
    bool   detected          = false;
    int    query_frame_id    = -1;
    int    match_frame_id    = -1;

    // ---- CLD 阶段 ----
    float  cld_similarity    = 0.0f;   // CLD 相似度得分
    int    rotation_index    = 0;      // 估计旋转偏移（扇区）

    // ---- GICP 几何验证阶段 ----
    double overlap_score     = 0.0;    // Overlap 打分（最终判定依据）
    double gicp_fitness      = 0.0;    // GICP 原始 fitness score
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();  // GICP 变换矩阵
    Eigen::Matrix4f rotation_init  = Eigen::Matrix4f::Identity();  // CLD 旋转初始化矩阵

    // ---- 耗时 ----
    double cld_time_ms       = 0.0;    // CLD 检索耗时
    double gicp_time_ms      = 0.0;    // GICP 验证耗时
    double total_time_ms     = 0.0;    // 总耗时
};

// ============================================================
//  配置参数
// ============================================================
struct LoopClosureConfig {
    // ----- CLD 描述符参数 -----
    int    sector_num       = 24;
    float  max_range        = 50.0f;
    int    min_points       = 3;
    std::vector<int> newtable = {0, -1, 3};

    // ----- CLD 初筛参数 -----
    int    exclude_recent       = 50;    // 排除最近 N 帧
    int    top_k_candidates     = 50;    // CLD 初筛 Top-K（进入 GICP 验证）

    // ----- GICP 几何验证参数 -----
    int    gicp_max_iterations      = 10;
    int    gicp_num_neighbors       = 20;
    int    gicp_num_threads         = 24;

    // ----- 点云预处理参数 -----
    double leaf_size         = 2.0;     // 体素降采样尺寸
    double z_filter_min      = -1.0;    // Z 轴直通滤波下限
    double z_filter_max      = 15.0;    // Z 轴直通滤波上限

    // ----- Overlap 判定参数 -----
    double distance_threshold   = 2.0;  // Overlap 距离阈值
    double overlap_threshold    = 0.6;  // Overlap ≥ 此值 → 确认回环并提前终止
};

// ============================================================
//  帧描述符记录（同时存储预处理后的点云，供 GICP 使用）
// ============================================================
struct FrameDescriptor {
    int      frame_id   = -1;
    CLDType  cld;
    double   timestamp  = 0.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  // 降采样 + 滤波后的点云

    FrameDescriptor() = default;
    FrameDescriptor(int id, const CLDType& desc,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr c,
                    double ts = 0.0)
        : frame_id(id), cld(desc), cloud(c), timestamp(ts) {}
};

// ============================================================
//  回环检测器主类
// ============================================================
class LoopClosureDetector {
public:

    LoopClosureDetector() : config_(LoopClosureConfig()) {}
    explicit LoopClosureDetector(const LoopClosureConfig& cfg) : config_(cfg) {}

    void setConfig(const LoopClosureConfig& cfg) { config_ = cfg; }
    const LoopClosureConfig& getConfig() const { return config_; }

    // ================================================================
    //  点云预处理（与 HL_BPR_v2.cpp 保持一致）
    // ================================================================

    /**
     * @brief 去零点 → 体素降采样 → Z 轴直通滤波
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& raw) const
    {
        // 1) 去除原点附近的无效点
        pcl::PointCloud<pcl::PointXYZ>::Ptr cleaned(
            new pcl::PointCloud<pcl::PointXYZ>(*raw));
        cleaned->erase(
            std::remove_if(cleaned->begin(), cleaned->end(),
                [](const pcl::PointXYZ& pt) {
                    return pt.getVector3fMap().squaredNorm() < 1e-3f;
                }),
            cleaned->end());

        // 2) 体素降采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> vg;
        vg.setLeafSize(config_.leaf_size, config_.leaf_size, config_.leaf_size);
        vg.setInputCloud(cleaned);
        vg.filter(*downsampled);

        // 3) Z 轴直通滤波
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(downsampled);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(config_.z_filter_min, config_.z_filter_max);
        pass.filter(*filtered);

        return filtered;
    }

    // ================================================================
    //  addFrame —— 增加一帧
    // ================================================================

    /**
     * @brief 从原始点云生成 CLD + 预处理点云，加入数据库
     * @return 帧 ID
     */
    int addFrame(const pcl::PointCloud<pcl::PointXYZ>::Ptr& raw_cloud,
                 double timestamp = 0.0)
    {
        CLDType cld = generate_cld_my(raw_cloud,
                                      config_.sector_num,
                                      config_.max_range,
                                      config_.min_points);
        auto filtered = preprocessCloud(raw_cloud);
        return addDescriptor(cld, filtered, timestamp);
    }

    /**
     * @brief 添加已有描述符 + 已预处理点云
     */
    int addDescriptor(const CLDType& cld,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                      double timestamp = 0.0)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        int id = next_frame_id_++;
        database_.emplace_back(id, cld, cloud, timestamp);
        return id;
    }

    // ================================================================
    //  detectLoopClosure —— CLD 初筛 + GICP 几何验证
    // ================================================================

    /**
     * @brief 从原始点云检测回环
     */
    LoopClosureResult detectLoopClosure(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& raw_cloud)
    {
        CLDType query_cld = generate_cld_my(raw_cloud,
                                            config_.sector_num,
                                            config_.max_range,
                                            config_.min_points);
        auto query_filtered = preprocessCloud(raw_cloud);
        return detectLoopClosure(query_cld, query_filtered);
    }

    /**
     * @brief 核心检测流程
     *
     * 第一阶段：CLD 相似度全库检索，取 Top-K
     * 第二阶段：对 Top-K 依次做 旋转初始化 → FastGICP → Overlap 验证
     *          一旦 overlap >= threshold 即提前终止
     */
    LoopClosureResult detectLoopClosure(
        const CLDType& query_cld,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& query_cloud)
    {
        auto t_total_start = std::chrono::high_resolution_clock::now();
        LoopClosureResult result;
        result.query_frame_id = next_frame_id_;

        // ==========================
        //  第一阶段：CLD 初筛
        // ==========================
        auto t_cld_start = std::chrono::high_resolution_clock::now();

        struct CLDCandidate {
            int   db_index;      // 在 database_ 中的下标
            int   frame_id;
            float score;
            int   rot_index;     // 原始最大相似度位置
            bool  flag_rot;
            int   offset_sector;
        };

        std::vector<CLDCandidate> cld_candidates;
        {
            std::lock_guard<std::mutex> lock(mutex_);

            int db_size    = static_cast<int>(database_.size());
            int search_end = db_size - config_.exclude_recent;
            if (search_end <= 0) {
                auto t_now = std::chrono::high_resolution_clock::now();
                result.total_time_ms = ms_elapsed(t_total_start, t_now);
                return result;
            }

            cld_candidates.reserve(search_end);

            #pragma omp parallel
            {
                std::vector<CLDCandidate> local;
                #pragma omp for schedule(dynamic, 8) nowait
                for (int i = 0; i < search_end; ++i) {
                    int  qsize = 0, off = 0, zlen = 0;
                    bool frot = false;

                    std::vector<float> sim = complete_similarity(
                        database_[i].cld, query_cld,
                        config_.newtable, qsize, frot, off, zlen);

                    auto it = std::max_element(sim.begin(), sim.end());
                    float  val = *it;
                    int    idx = static_cast<int>(std::distance(sim.begin(), it));
                    int    rot = frot ? (idx - off) : idx;

                    local.push_back({i, database_[i].frame_id,
                                     val, rot, frot, off});
                }
                #pragma omp critical
                {
                    cld_candidates.insert(cld_candidates.end(),
                                          local.begin(), local.end());
                }
            }
        }

        // 按 CLD 得分降序
        std::sort(cld_candidates.begin(), cld_candidates.end(),
                  [](const CLDCandidate& a, const CLDCandidate& b) {
                      return a.score > b.score;
                  });

        auto t_cld_end = std::chrono::high_resolution_clock::now();
        result.cld_time_ms = ms_elapsed(t_cld_start, t_cld_end);

        // ==========================
        //  第二阶段：GICP 几何验证
        // ==========================
        auto t_gicp_start = std::chrono::high_resolution_clock::now();

        int verify_count = std::min(config_.top_k_candidates,
                                    static_cast<int>(cld_candidates.size()));

        for (int si = 0; si < verify_count; ++si) {
            const auto& cand = cld_candidates[si];
            if (cand.score < 1000 )continue;
            // ---- 1) 取目标点云 ----
            pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                target_cloud = database_[cand.db_index].cloud;
            }
            if (!target_cloud || target_cloud->empty()) continue;

            // ---- 2) CLD 旋转初始化（绕 Z 轴，与 HL_BPR_v2.cpp 一致） ----
            double angle_rad = 0.0;
            if (cand.rot_index != 0) {
                angle_rad = cand.rot_index
                            * (360.0 / static_cast<double>(config_.sector_num))
                            * M_PI / 180.0;
            }
            Eigen::Matrix4f rot_init = Eigen::Matrix4f::Identity();
            float cos_t = static_cast<float>(std::cos(angle_rad));
            float sin_t = static_cast<float>(std::sin(angle_rad));
            rot_init(0, 0) =  cos_t;  rot_init(0, 1) = -sin_t;
            rot_init(1, 0) =  sin_t;  rot_init(1, 1) =  cos_t;

            // 用旋转初值变换 source 点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr src_rotated(
                new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*query_cloud, *src_rotated, rot_init);

            // ---- 3) FastGICP 配准 ----
            fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> reg;
            reg.setMaximumIterations(config_.gicp_max_iterations);
            reg.setCorrespondenceRandomness(config_.gicp_num_neighbors);
            reg.setNumThreads(config_.gicp_num_threads);
            reg.setInputTarget(target_cloud);
            reg.setInputSource(src_rotated);

            pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(
                new pcl::PointCloud<pcl::PointXYZ>);
            reg.align(*aligned);
            Eigen::Matrix4f T_gicp = reg.getFinalTransformation();

            // ---- 4) 配准后变换 → Overlap 打分 ----
            pcl::PointCloud<pcl::PointXYZ>::Ptr src_final(
                new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*src_rotated, *src_final, T_gicp);

            double overlap = computeOverlapFast(
                src_final, target_cloud,
                config_.distance_threshold, config_.leaf_size);

            // ---- 5) 达标 → 确认回环并提前终止 ----
            if (overlap >= config_.overlap_threshold) {
                result.detected       = true;
                result.match_frame_id = cand.frame_id;
                result.cld_similarity = cand.score;
                result.rotation_index = cand.rot_index;
                result.overlap_score  = overlap;
                result.gicp_fitness   = reg.getFitnessScore();
                result.transformation = T_gicp;
                result.rotation_init  = rot_init;
                break;  // 与 HL_BPR_v2.cpp 一致：overlap 达标即终止
            }
        }

        auto t_gicp_end = std::chrono::high_resolution_clock::now();
        result.gicp_time_ms  = ms_elapsed(t_gicp_start, t_gicp_end);
        result.total_time_ms = ms_elapsed(t_total_start, t_gicp_end);

        return result;
    }

    // ================================================================
    //  addAndDetect —— 先检测，再入库（SLAM 主循环典型用法）
    // ================================================================

    LoopClosureResult addAndDetect(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& raw_cloud,
        double timestamp = 0.0)
    {
        CLDType cld = generate_cld_my(raw_cloud,
                                      config_.sector_num,
                                      config_.max_range,
                                      config_.min_points);
        auto filtered = preprocessCloud(raw_cloud);

        // 先检测
        LoopClosureResult result = detectLoopClosure(cld, filtered);

        // 再入库
        int id = addDescriptor(cld, filtered, timestamp);
        result.query_frame_id = id;

        return result;
    }

    // ================================================================
    //  辅助接口
    // ================================================================

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return database_.size();
    }

    const FrameDescriptor* getFrame(int frame_id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& fd : database_) {
            if (fd.frame_id == frame_id) return &fd;
        }
        return nullptr;
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        database_.clear();
        next_frame_id_ = 0;
    }

private:
    static double ms_elapsed(
        const std::chrono::high_resolution_clock::time_point& t0,
        const std::chrono::high_resolution_clock::time_point& t1)
    {
        return std::chrono::duration<double, std::milli>(t1 - t0).count();
    }

    LoopClosureConfig              config_;
    std::vector<FrameDescriptor>   database_;
    int                            next_frame_id_ = 0;
    mutable std::mutex             mutex_;
};