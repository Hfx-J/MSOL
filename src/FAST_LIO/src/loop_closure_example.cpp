/**
 * @file loop_closure_example.cpp
 * @brief 回环检测使用示例（含 GICP 几何验证 + 真值评估 + 详细耗时统计）
 *
 * 用法:
 *   rosrun FAST_LIO loop_closure_example /path/to/bin/files/ /path/to/gt.txt
 *   rosrun FAST_LIO loop_closure_example /path/to/bin/files/ /path/to/gt.txt 10
 *
 * 真值格式（每行）:
 *   timestamp x y z qx qy qz qw
 */

 #include "seu_hfx.hpp"
 #include "loop_closure_detector.hpp"
 
 #include <iostream>
 #include <iomanip>
 #include <fstream>
 #include <string>
 #include <vector>
 #include <algorithm>
 #include <cmath>
 #include <boost/filesystem.hpp>
 
 // ============================================================
 //  真值相关
 // ============================================================
 
 struct GTPose {
     double          timestamp;
     Eigen::Vector3f translation;
     Eigen::Matrix3f rotation;
 };
 
 std::vector<GTPose> loadGroundTruth(const std::string& filepath)
 {
     std::vector<GTPose> poses;
     std::ifstream ifs(filepath);
     if (!ifs.is_open()) {
         std::cerr << "[ERROR] Cannot open GT file: " << filepath << std::endl;
         return poses;
     }
 
     double t, x, y, z, qx, qy, qz, qw;
     while (ifs >> t >> x >> y >> z >> qx >> qy >> qz >> qw) {
         GTPose p;
         p.timestamp   = t;
         p.translation = Eigen::Vector3f(x, y, z);
         Eigen::Quaterniond q(qw, qx, qy, qz);
         q.normalize();
         p.rotation = q.toRotationMatrix().cast<float>();
         poses.push_back(p);
     }
 
     std::cout << "Loaded " << poses.size() << " GT poses" << std::endl;
     return poses;
 }
 
 void computePoseError(const GTPose& a, const GTPose& b,
                       double& trans_err, double& rot_err_deg)
 {
     trans_err = (a.translation - b.translation).norm();
 
     Eigen::Matrix3f R_delta = a.rotation.transpose() * b.rotation;
     double trace = R_delta.trace();
     double cos_angle = std::min(std::max((trace - 1.0) / 2.0, -1.0), 1.0);
     rot_err_deg = std::acos(cos_angle) / M_PI * 180.0;
 }
 
 // ============================================================
 //  简单统计辅助类
 // ============================================================
 struct TimingStat {
     std::vector<double> values;
 
     void add(double v) { values.push_back(v); }
     int    count() const { return values.size(); }
     double sum()   const { double s = 0; for (double v : values) s += v; return s; }
     double mean()  const { return count() > 0 ? sum() / count() : 0.0; }
     double max_val() const {
         double m = 0;
         for (double v : values) m = std::max(m, v);
         return m;
     }
     double min_val() const {
         double m = std::numeric_limits<double>::max();
         for (double v : values) m = std::min(m, v);
         return count() > 0 ? m : 0.0;
     }
     double median() const {
         if (values.empty()) return 0.0;
         std::vector<double> sorted = values;
         std::sort(sorted.begin(), sorted.end());
         int n = sorted.size();
         return (n % 2 == 0) ? (sorted[n/2 - 1] + sorted[n/2]) / 2.0 : sorted[n/2];
     }
 };
 
 // ============================================================
 //  主函数
 // ============================================================
 int main(int argc, char** argv)
 {
     if (argc < 3) {
         std::cerr << "Usage: " << argv[0]
                   << " <bin_dir> <gt_file> [step]" << std::endl;
         return 1;
     }
 
     std::string data_dir = argv[1];
     std::string gt_file  = argv[2];
     int step = (argc > 3) ? std::atoi(argv[3]) : 1;
 
     // ========================================
     //  1. 配置回环检测器
     // ========================================
     LoopClosureConfig config;
 
     config.sector_num       = 24;
     config.max_range        = 50.0f;
     config.min_points       = 3;
     config.newtable         = {0, -1, 3};
 
     config.exclude_recent   = 200;
     config.top_k_candidates = 50;
 
     config.gicp_max_iterations = 10;
     config.gicp_num_neighbors  = 20;
     config.gicp_num_threads    = 24;
 
     config.leaf_size    = 2.0;
     config.z_filter_min = -1.0;
     config.z_filter_max = 15.0;
 
     config.distance_threshold = 2.0;
     config.overlap_threshold  = 0.8;
 
     LoopClosureDetector detector(config);
 
     const double GT_POSITIVE_DIST = 5.0;
     const double GT_CORRECT_DIST  = 10.0;
 
     // ========================================
     //  2. 加载数据
     // ========================================
     std::vector<std::string> cloud_files;
     for (const auto& entry :
          boost::filesystem::recursive_directory_iterator(data_dir)) {
         if (boost::filesystem::is_regular_file(entry))
             cloud_files.push_back(entry.path().string());
     }
     std::sort(cloud_files.begin(), cloud_files.end());
     std::cout << "Total cloud files: " << cloud_files.size() << std::endl;
 
     std::vector<GTPose> gt_poses = loadGroundTruth(gt_file);
     if (gt_poses.empty()) {
         std::cerr << "[ERROR] No GT poses loaded, aborting." << std::endl;
         return 1;
     }
     if (gt_poses.size() < cloud_files.size()) {
         std::cerr << "[WARN] GT poses (" << gt_poses.size()
                   << ") < cloud files (" << cloud_files.size()
                   << "), truncating to " << gt_poses.size() << " frames."
                   << std::endl;
         cloud_files.resize(gt_poses.size());
     }
 
     // ========================================
     //  3. 逐帧处理 + 统计
     // ========================================
     int total_processed  = 0;
     int total_searchable = 0;
     int TP = 0, FP = 0, FN = 0, TN = 0;
 
     std::vector<double> tp_trans_errors;
     std::vector<double> tp_rot_errors;
     std::vector<double> tp_overlaps;
 
     // ---- 耗时收集 ----
     TimingStat time_all_total, time_all_cld, time_all_gicp;   // 全部帧
     TimingStat time_loop_total, time_loop_cld, time_loop_gicp; // 检测到回环的帧
     TimingStat time_noloop_total, time_noloop_cld, time_noloop_gicp; // 无回环的帧
 
     std::vector<int> frame_to_file_idx;
 
     auto wall_start = std::chrono::high_resolution_clock::now();
 
     for (size_t idx = 0; idx < cloud_files.size(); idx += step) {
         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
             new pcl::PointCloud<pcl::PointXYZ>);
         readBinFile(cloud_files[idx], cloud);
 
         LoopClosureResult result = detector.addAndDetect(cloud);
         frame_to_file_idx.push_back(idx);
         total_processed++;
 
         // 收集耗时
         time_all_total.add(result.total_time_ms);
         time_all_cld.add(result.cld_time_ms);
         time_all_gicp.add(result.gicp_time_ms);
 
         if (result.detected) {
             time_loop_total.add(result.total_time_ms);
             time_loop_cld.add(result.cld_time_ms);
             time_loop_gicp.add(result.gicp_time_ms);
         } else {
             time_noloop_total.add(result.total_time_ms);
             time_noloop_cld.add(result.cld_time_ms);
             time_noloop_gicp.add(result.gicp_time_ms);
         }
 
         int cur_frame     = result.query_frame_id;
         int db_search_end = cur_frame - config.exclude_recent;
 
         if (db_search_end <= 0) {
             if (idx % 100 == 0) {
                 std::cout << "Progress: " << idx << "/" << cloud_files.size()
                           << "  DB=" << detector.size()
                           << "  (building...)" << std::endl;
             }
             continue;
         }
         total_searchable++;
 
         // ==== GT 真值判定 ====
         double gt_min_dist    = std::numeric_limits<double>::max();
         int    gt_nearest_frm = -1;
         for (int fi = 0; fi < db_search_end; ++fi) {
             int file_idx = frame_to_file_idx[fi];
             double d = (gt_poses[file_idx].translation -
                         gt_poses[idx].translation).norm();
             if (d < gt_min_dist) {
                 gt_min_dist    = d;
                 gt_nearest_frm = fi;
             }
         }
         bool gt_has_loop = (gt_min_dist < GT_POSITIVE_DIST);
 
         if (result.detected) {
             int match_file_idx = frame_to_file_idx[result.match_frame_id];
             double trans_err, rot_err;
             computePoseError(gt_poses[idx], gt_poses[match_file_idx],
                              trans_err, rot_err);
 
             if (trans_err < GT_CORRECT_DIST) {
                 TP++;
                 tp_trans_errors.push_back(trans_err);
                 tp_rot_errors.push_back(rot_err);
                 tp_overlaps.push_back(result.overlap_score);
 
                 std::cout << "[TP] frame " << cur_frame
                           << " <-> " << result.match_frame_id
                           << "  GT_dist=" << std::fixed << std::setprecision(2)
                           << trans_err << "m"
                           << "  rot=" << rot_err << "deg"
                           << "  overlap=" << result.overlap_score
                           << "  [CLD " << result.cld_time_ms << "ms"
                           << " + GICP " << result.gicp_time_ms << "ms"
                           << " = " << result.total_time_ms << "ms]"
                           << std::endl;
             } else {
                 FP++;
                 std::cout << "[FP] frame " << cur_frame
                           << " <-> " << result.match_frame_id
                           << "  GT_dist=" << std::fixed << std::setprecision(2)
                           << trans_err << "m (too far!)"
                           << "  overlap=" << result.overlap_score
                           << "  [" << result.total_time_ms << "ms]"
                           << std::endl;
             }
         } else {
             if (gt_has_loop) {
                 FN++;
                 if (FN <= 20) {
                     std::cout << "[FN] frame " << cur_frame
                               << "  GT nearest=" << gt_nearest_frm
                               << "  GT_dist=" << std::fixed
                               << std::setprecision(2) << gt_min_dist << "m"
                               << "  [CLD " << result.cld_time_ms << "ms"
                               << " + GICP " << result.gicp_time_ms << "ms"
                               << " = " << result.total_time_ms << "ms]"
                               << std::endl;
                 }
             } else {
                 TN++;
             }
         }
 
         if (idx % 200 == 0) {
             std::cout << "--- Progress: " << idx << "/" << cloud_files.size()
                       << "  DB=" << detector.size()
                       << "  TP=" << TP << " FP=" << FP
                       << " FN=" << FN << " TN=" << TN << std::endl;
         }
     }
 
     auto wall_end = std::chrono::high_resolution_clock::now();
     double wall_sec = std::chrono::duration<double>(wall_end - wall_start).count();
 
     // ========================================
     //  4. 统计报告
     // ========================================
     std::cout << std::fixed << std::setprecision(2);
     std::cout << "\n";
     std::cout << "========================================================" << std::endl;
     std::cout << "             LOOP CLOSURE EVALUATION REPORT              " << std::endl;
     std::cout << "========================================================" << std::endl;
 
     std::cout << "\n[Data]" << std::endl;
     std::cout << "  Total frames processed  : " << total_processed << std::endl;
     std::cout << "  Searchable frames       : " << total_searchable << std::endl;
     std::cout << "  GT positive dist thresh : " << GT_POSITIVE_DIST << " m" << std::endl;
     std::cout << "  GT correct dist thresh  : " << GT_CORRECT_DIST << " m" << std::endl;
 
     std::cout << "\n[Confusion Matrix]" << std::endl;
     std::cout << "  TP (correct loop)       : " << TP << std::endl;
     std::cout << "  FP (wrong loop)         : " << FP << std::endl;
     std::cout << "  FN (missed loop)        : " << FN << std::endl;
     std::cout << "  TN (correct no-loop)    : " << TN << std::endl;
 
     double precision = (TP + FP > 0) ? (double)TP / (TP + FP) : 0.0;
     double recall    = (TP + FN > 0) ? (double)TP / (TP + FN) : 0.0;
     double f1        = (precision + recall > 0)
                        ? 2.0 * precision * recall / (precision + recall) : 0.0;
 
     std::cout << std::setprecision(4);
     std::cout << "\n[Metrics]" << std::endl;
     std::cout << "  Precision               : " << precision
               << "  (" << TP << "/" << (TP + FP) << ")" << std::endl;
     std::cout << "  Recall                  : " << recall
               << "  (" << TP << "/" << (TP + FN) << ")" << std::endl;
     std::cout << "  F1 Score                : " << f1 << std::endl;
 
     // ---- TP 误差统计 ----
     if (!tp_trans_errors.empty()) {
         int n = tp_trans_errors.size();
         double sum_t = 0, sum_r = 0, sum_o = 0;
         double max_t = 0, max_r = 0;
         double rmse_t = 0, rmse_r = 0;
 
         for (int i = 0; i < n; ++i) {
             sum_t  += tp_trans_errors[i];
             sum_r  += tp_rot_errors[i];
             sum_o  += tp_overlaps[i];
             max_t   = std::max(max_t, tp_trans_errors[i]);
             max_r   = std::max(max_r, tp_rot_errors[i]);
             rmse_t += tp_trans_errors[i] * tp_trans_errors[i];
             rmse_r += tp_rot_errors[i]   * tp_rot_errors[i];
         }
         rmse_t = std::sqrt(rmse_t / n);
         rmse_r = std::sqrt(rmse_r / n);
 
         std::cout << std::setprecision(3);
         std::cout << "\n[TP Pose Error]  (" << n << " samples)" << std::endl;
         std::cout << "  Trans  mean=" << sum_t / n
                   << "m  RMSE=" << rmse_t
                   << "m  max=" << max_t << "m" << std::endl;
         std::cout << "  Rot    mean=" << sum_r / n
                   << "deg  RMSE=" << rmse_r
                   << "deg  max=" << max_r << "deg" << std::endl;
         std::cout << "  Overlap mean=" << sum_o / n << std::endl;
     }
 
     // ---- 耗时统计 ----
     std::cout << std::setprecision(2);
     std::cout << "\n[Timing - All Frames]  (" << time_all_total.count() << " frames)" << std::endl;
     std::cout << "              mean      median    max       min" << std::endl;
     std::cout << "  Total   :  "
               << std::setw(8) << time_all_total.mean()  << "  "
               << std::setw(8) << time_all_total.median() << "  "
               << std::setw(8) << time_all_total.max_val() << "  "
               << std::setw(8) << time_all_total.min_val() << "  ms" << std::endl;
     std::cout << "  CLD     :  "
               << std::setw(8) << time_all_cld.mean()  << "  "
               << std::setw(8) << time_all_cld.median() << "  "
               << std::setw(8) << time_all_cld.max_val() << "  "
               << std::setw(8) << time_all_cld.min_val() << "  ms" << std::endl;
     std::cout << "  GICP    :  "
               << std::setw(8) << time_all_gicp.mean()  << "  "
               << std::setw(8) << time_all_gicp.median() << "  "
               << std::setw(8) << time_all_gicp.max_val() << "  "
               << std::setw(8) << time_all_gicp.min_val() << "  ms" << std::endl;
 
     if (time_loop_total.count() > 0) {
         std::cout << "\n[Timing - Loop Detected]  (" << time_loop_total.count() << " frames)" << std::endl;
         std::cout << "              mean      median    max       min" << std::endl;
         std::cout << "  Total   :  "
                   << std::setw(8) << time_loop_total.mean()  << "  "
                   << std::setw(8) << time_loop_total.median() << "  "
                   << std::setw(8) << time_loop_total.max_val() << "  "
                   << std::setw(8) << time_loop_total.min_val() << "  ms" << std::endl;
         std::cout << "  CLD     :  "
                   << std::setw(8) << time_loop_cld.mean()  << "  "
                   << std::setw(8) << time_loop_cld.median() << "  "
                   << std::setw(8) << time_loop_cld.max_val() << "  "
                   << std::setw(8) << time_loop_cld.min_val() << "  ms" << std::endl;
         std::cout << "  GICP    :  "
                   << std::setw(8) << time_loop_gicp.mean()  << "  "
                   << std::setw(8) << time_loop_gicp.median() << "  "
                   << std::setw(8) << time_loop_gicp.max_val() << "  "
                   << std::setw(8) << time_loop_gicp.min_val() << "  ms" << std::endl;
     }
 
     if (time_noloop_total.count() > 0) {
         std::cout << "\n[Timing - No Loop]  (" << time_noloop_total.count() << " frames)" << std::endl;
         std::cout << "              mean      median    max       min" << std::endl;
         std::cout << "  Total   :  "
                   << std::setw(8) << time_noloop_total.mean()  << "  "
                   << std::setw(8) << time_noloop_total.median() << "  "
                   << std::setw(8) << time_noloop_total.max_val() << "  "
                   << std::setw(8) << time_noloop_total.min_val() << "  ms" << std::endl;
         std::cout << "  CLD     :  "
                   << std::setw(8) << time_noloop_cld.mean()  << "  "
                   << std::setw(8) << time_noloop_cld.median() << "  "
                   << std::setw(8) << time_noloop_cld.max_val() << "  "
                   << std::setw(8) << time_noloop_cld.min_val() << "  ms" << std::endl;
         std::cout << "  GICP    :  "
                   << std::setw(8) << time_noloop_gicp.mean()  << "  "
                   << std::setw(8) << time_noloop_gicp.median() << "  "
                   << std::setw(8) << time_noloop_gicp.max_val() << "  "
                   << std::setw(8) << time_noloop_gicp.min_val() << "  ms" << std::endl;
     }
 
     std::cout << "\n[Timing - Summary]" << std::endl;
     std::cout << "  Wall clock              : " << wall_sec << " s" << std::endl;
     std::cout << "  Sum of all frames       : " << time_all_total.sum() / 1000.0 << " s" << std::endl;
     std::cout << "    CLD total             : " << time_all_cld.sum() / 1000.0 << " s  ("
               << std::setprecision(1) << (time_all_cld.sum() / time_all_total.sum() * 100.0)
               << "%)" << std::endl;
     std::cout << "    GICP total            : " << std::setprecision(2) << time_all_gicp.sum() / 1000.0 << " s  ("
               << std::setprecision(1) << (time_all_gicp.sum() / time_all_total.sum() * 100.0)
               << "%)" << std::endl;
 
     std::cout << "\n========================================================" << std::endl;
 
     return 0;
 }