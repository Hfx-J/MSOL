#include "seu_hfx.hpp"
#include "utility.hpp"
#include <faiss/IndexFlat.h>
using idx_t = faiss::idx_t;
// #include "libpmc.h"
#include <random>
int main(int argc, char** argv)
{

    ros::init(argc, argv, "heliplace");
    
    ros::NodeHandle nh;
    //load parameters
    std::string file_path,query_dataset,query_type,map_dataset,map_type;
    nh.param<string>("load_file/file_path",file_path,"");
    nh.param<string>("load_file/query_dataset",query_dataset,"");
    nh.param<string>("load_file/query_type",query_type,"");
    nh.param<string>("load_file/map_dataset",map_dataset,"");
    nh.param<string>("load_file/map_type",map_type,"");

    
    int temp_sector_num;
    double temp_max_range;
    int temp_min_points;
    // std::vector<int> new_table ;
    nh.param<int>("CLD/sector_num", temp_sector_num, 24);
    nh.param<double>("CLD/max_range", temp_max_range, 50.0);
    nh.param<int>("CLD/min_points", temp_min_points, 3);
    // nh.param<std::vector<int>>("CLD/cld_table", new_table);
    
    // Set as const
    const int sector_num = temp_sector_num;
    const double max_range = temp_max_range;
    const int min_points = temp_min_points;
    
    // Read HL_BPR parameters
    int step;
    int num_threads;
    int num_neighbors;
    int max_iterations;
    double leaf_size;
    double z_max, z_min, x_max, x_min, y_max, y_min;
    
    nh.param<int>("HL_BPR/step", step, 10);
    nh.param<int>("HL_BPR/num_threads", num_threads, 24);
    nh.param<int>("HL_BPR/num_neighbors", num_neighbors, 20);
    nh.param<int>("HL_BPR/max_iterations", max_iterations, 10);
    nh.param<double>("HL_BPR/leaf_size", leaf_size, 2.0);
    nh.param<double>("HL_BPR/z_max", z_max, 15.0);
    nh.param<double>("HL_BPR/z_min", z_min, -1.0);
    nh.param<double>("HL_BPR/x_max", x_max, 40.0);
    nh.param<double>("HL_BPR/x_min", x_min, -40.0);
    nh.param<double>("HL_BPR/y_max", y_max, 40.0);
    nh.param<double>("HL_BPR/y_min", y_min, -40.0);
    // Read overlap parameters - these don't need to be const
    double distance_threshold;
    double overlap_threshold;

    nh.param<double>("overlap/distance_threshold", distance_threshold, 2.0);
    nh.param<double>("overlap/overlap_threshold", overlap_threshold, 0.6);
    std::vector<double> map_x, map_y;
    std::vector<double> data_x, data_y;
    std::vector<double> fina_x, fina_y;
    std::vector<double> finas_x, finas_y;
    std::vector<std::vector<double>> pair_x, pair_y;


    //Read the query set.
    Eigen::Matrix4f BASE2OUSTER;
    BASE2OUSTER <<  0.9999346552051229, 0.003477624535771754, -0.010889970036688295, -0.060649229060416594,//MCD
                    0.003587143302461965, -0.9999430279821171, 0.010053516443599904, -0.012837544242408117,
                    -0.010854387257665576, -0.01009192338171122, -0.999890161647627, -0.020492606896077407,
                    0,0,0,1;
    Eigen::Matrix4f BASE2LIVOX;
    BASE2LIVOX << 0.9998581316050806, -0.0002258196228773494, -0.016842377168758943, -0.010514839241742317,   //Mulran
                -0.00017111407692153792, -0.999994705851753, 0.0032494597148797887, -0.008989784841758377,
                -0.016843021794484745, -0.003246116751423288, -0.9998528768488223, 0.03735646863833463,
                0.0, 0.0, 0.0, 1.0;
    std::string query_file = file_path + query_dataset + "/" + query_type +"/";
    std::vector<std::string> query_set;
    cout<<query_file<<endl;
    for (const auto& entry : boost::filesystem::recursive_directory_iterator(query_file)) {
        if (boost::filesystem::is_regular_file(entry)) {
            query_set.push_back(entry.path().string());
        }
    }
    // Sort the filenames of the query set.
    std::sort(query_set.begin(), query_set.end());

    //Read the map set.
    std::string map_file = file_path + map_dataset +"/" + map_type +"/";
    std::vector<std::string> map_set;
    for (const auto& entry : boost::filesystem::recursive_directory_iterator(map_file)) {
        if (boost::filesystem::is_regular_file(entry)) {
            map_set.push_back(entry.path().string());
        }
    }
    // Sort the filenames of the map set.
    std::sort(map_set.begin(), map_set.end());

    
    //generate query cld feature
    std::cout<<"Read the query CLD."<<std::endl;
    auto t_begin_query_cld = std::chrono::high_resolution_clock::now();
    std::vector<CLDType> query_cld(query_set.size());
    #pragma omp parallel for
    for(int i = 0;i < query_set.size() ; i++ )
    {
        std::string this_name = query_set[i];
        pcl::PointCloud<pcl::PointXYZ>::Ptr this_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        readBinFile(this_name,this_cloud);
        CLDType this_cld = generate_cld_my(this_cloud,sector_num,max_range,min_points);
        query_cld[i]=this_cld; 
    }
    
    auto t_end_query_cld = std::chrono::high_resolution_clock::now();
    std::cout<<"query CLD:"<<time_inc(t_end_query_cld, t_begin_query_cld)<<std::endl;

    //generate map cld feature
    std::cout<<"Read the map CLD."<<std::endl;
    auto t_begin_map_cld = std::chrono::high_resolution_clock::now();
    std::vector<CLDType> map_cld(map_set.size());
    std::vector<BEVType> map_bev(map_set.size());
    
    #pragma omp parallel for
    for(int i = 0;i < map_set.size() ; i++ )
    {
        // auto t_begin = std::chrono::high_resolution_clock::now();
        std::string this_name = map_set[i];
        pcl::PointCloud<pcl::PointXYZ>::Ptr this_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        readBinFile(this_name,this_cloud);
        CLDType this_cld = generate_cld_my(this_cloud,sector_num,max_range,min_points); 
        map_cld[i]=this_cld;
        // BEVType bev_img = point_cloud_to_bev(this_cloud);
        // map_bev[i]=bev_img;
    }
    auto t_end_map_cld = std::chrono::high_resolution_clock::now();
    std::cout<<"map CLD:"<<time_inc(t_end_map_cld, t_begin_map_cld)<<std::endl;

    //get query frame pose
    std::string query_pose_path = file_path + query_dataset +"/" + query_type +"_gt.txt";
    std::vector<std::pair<Eigen::Vector3f, Eigen::Matrix3f>> query_pose = load_poses_from_hfx(query_pose_path);

    //get map frame pose
    std::string map_pose_path = file_path + map_dataset +"/" + map_type +"_gt.txt";
    std::vector<std::pair<Eigen::Vector3f, Eigen::Matrix3f>> map_pose = load_poses_from_hfx(map_pose_path);
    for(int i = 0;i < map_set.size() ; i++ )
    {
        map_x.push_back(map_pose[i].first[0]);
        map_y.push_back(map_pose[i].first[1]);
    }

    std::vector<uint16_t> lookup_table = create_lookup_table();
    std::vector<int> new_table = {0,-1,3}; //Aeva
    //ma
    double positive = 0,negative = 0;
    double T_all = 0.0,R_all = 0.0;
    double SR = 0.0, Turth_SR= 0.0;
    double total_num = 0.0;
    double total_time =0.0, total_time_score =0.0, ValidTime = 0.0;

    double fail_rot = 0,fail_trans=0;
    int error_num = 0;
    double trans_min = 0;
    std::vector<double> transErrors;  // Translation errors
    std::vector<double> rotErrors;    // Rotation errors
    std::vector<double> timeCosts;    // Time costs
    std::vector<int> ValidIndex;

    std::string transErrorsPath = file_path + query_dataset +"/" + query_type + "2" + map_type  +"_HLBPR_trans.txt";
    std::string rotErrorsPath = file_path + query_dataset +"/" + query_type + "2" + map_type  +"_HLBPR_rot.txt";
    std::string timeCostsPath = file_path + query_dataset +"/" + query_type + "2" + map_type  +"_HLBPR_time.txt";
    std::string ValidIndexPath = file_path + query_dataset +"/" + query_type + "2" + map_type  +"_HLBPR_idx.txt";

    std::string GEN_time_dir = file_path + query_dataset +"/" + query_type + "2" + map_type  +"_HLBPR_GEN_time.txt";
    std::string Search_time_dir = file_path + query_dataset +"/" + query_type + "2" + map_type  +"_HLBPR_Search_time.txt";
    std::string PE_time_dir = file_path + query_dataset +"/" + query_type + "2" + map_type  +"_HLBPR_PE_time.txt";
    std::vector<double> GEN_time,Search_time,PE_time;
    std::vector<int> answer_list(101,0);
    // int num_threads=24,num_neighbors = 20,max_iterations = 10 ;
    double firness_all = 0;
    // float leaf_size = 2.0;
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setLeafSize(leaf_size,leaf_size, leaf_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());

    int list_all[100] = {0};
    memset(list_all,0,sizeof(list_all));
    for(int idx = 0; idx< query_set.size(); idx+=step)
    {
        double min_dist = findNearestPosePosition(map_pose,query_pose[idx]);
        if(min_dist<=15)
            total_num++;
        else continue;

        auto t_begin_cld = std::chrono::high_resolution_clock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr this_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        readBinFile(query_set[idx],this_cloud);
        CLDType test_cld = generate_cld_my(this_cloud,sector_num,max_range,min_points);
        auto t_end_cld = std::chrono::high_resolution_clock::now();

        data_x.push_back(query_pose[idx].first[0]);
        data_y.push_back(query_pose[idx].first[1]);

        std::cout<<"/****************** "<<idx<<"/"<<query_set.size()<<" ******************/"<<std::endl;
        auto t_begin = std::chrono::high_resolution_clock::now();
        CLDType this_cld = query_cld[idx];
        Eigen::Vector3f translation = query_pose[idx].first;
        Eigen::Matrix3f rotation = query_pose[idx].second;
        std::vector<std::vector<float>> score;

        int query_size = 0;
        bool flag_rot =false;
        auto t_begin_bc2d = std::chrono::high_resolution_clock::now();
        // Declare variables inside the parallel region
        // Pre-allocate score vector to avoid frequent reallocation during multithreading competition
        // std::vector<std::vector<float>> score;
        score.reserve(map_set.size());

        // Create thread-local storage result sets
        std::vector<std::vector<std::vector<float>>> thread_local_scores;
        #pragma omp parallel
        {
            // Create a local result set for each thread
            #pragma omp single
            {
                thread_local_scores.resize(omp_get_num_threads());
                for (auto& thread_scores : thread_local_scores) {
                    thread_scores.reserve(map_set.size() / omp_get_num_threads() + 1);
                }
            }

            // Process each map element in parallel
            #pragma omp for schedule(dynamic, 8) nowait
            for (int jdx = 0; jdx < map_set.size(); jdx++)
            {
                int thread_id = omp_get_thread_num();
                bool flag_rot = false;
                int query_size = 0;
                int offset_sector = 0;
                int zero_len = 0;
                // Calculate similarity
                std::vector<float> result = complete_similarity(map_cld[jdx], this_cld, new_table, query_size, flag_rot ,offset_sector,zero_len);
                
                // Find maximum value and its index
                auto max_it = std::max_element(result.begin(), result.end());
                int max_index = std::distance(result.begin(), max_it);
                float max_value = *max_it;
                // flag_rot = false;
                // Create result vector and store to thread-local set
                std::vector<float> this_score = {
                    max_value,
                    flag_rot ? static_cast<float>(max_index - offset_sector) : static_cast<float>(max_index),
                    static_cast<float>(jdx),
                    offset_sector,
                    zero_len

                };
                
                thread_local_scores[thread_id].push_back(std::move(this_score));
            }
        }

        // Merge results from all threads
        score.clear();
        for (const auto& thread_scores : thread_local_scores) {
            score.insert(score.end(), thread_scores.begin(), thread_scores.end());
        }
        auto t_end_bc2d = std::chrono::high_resolution_clock::now();
        // Sort in descending order
        std::sort(score.begin(), score.end(), 
                [](const std::vector<float>& a, const std::vector<float>& b) {
                    return a[0] > b[0]; 
                });


        //Gene_BPR Vector 

        double score_time = 0.0;

        auto t_start_bev = std::chrono::high_resolution_clock::now();
        std::vector<Gene_BPR> Map_BPR;
        int best_si = 9999;
        double best_error = 99999.0;
        int best_index = 9999;
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        readBinFile(query_set[idx],source_cloud);
        source_cloud->erase(
            std::remove_if(source_cloud->begin(), source_cloud->end(), [=](const pcl::PointXYZ& pt) { return pt.getVector3fMap().squaredNorm() < 1e-3; }),
            source_cloud->end());
        voxelgrid.setInputCloud(source_cloud);
        voxelgrid.filter(*filtered);
        source_cloud = filtered;
        // Assume filtered is input point cloud, source_cloud is output point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>);

        // Assume x_min, x_max, y_min, y_max, z_min, z_max have been read from config file
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(filtered);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(x_min, x_max);  // Use config parameters to limit x-axis
        pass_x.filter(*temp_cloud1);

        // Apply Y-axis filtering
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(temp_cloud1);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(y_min, y_max);  // Use config parameters to limit y-axis
        pass_y.filter(*temp_cloud2);

        // Apply Z-axis filtering
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(filtered);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(z_min, z_max);  // Use config parameters to limit z-axis
        pass_z.filter(*source_cloud);
        // pcl::transformPointCloud(*source_cloud, *source_cloud, BASE2LIVOX);
        double min_dist1 = 9999,min_rot = 9999,min_fit = 0;
        int min_dinex = 0;
        for(int si = 0; si < 50; si++)
        {
            
            int jdx = score[si][2];
            Eigen::Vector3f translation_map = map_pose[jdx].first;
            Eigen::Matrix3f rotation_map = map_pose[jdx].second;
            double dist_e = (translation - translation_map).norm();
            Eigen::Matrix3f R_delta = rotation.inverse() * rotation_map;
            double r_e = std::abs(std::acos(
             fmin(fmax((R_delta.trace() - 1) / 2, -1.0),
                  1.0))) / M_PI * 180;
            if (min_dist1 > dist_e)
            {
                min_dist1 = dist_e;
                min_rot = r_e;
                min_dinex = si;
            }
                
        }
        if(min_dist1 > 30)continue;
        for(int si = 0; si < 50; si++)
        {
            
            int jdx = score[si][2];
            Eigen::Vector3f translation_map = map_pose[jdx].first;
            Eigen::Matrix3f rotation_map = map_pose[jdx].second;
            Eigen::Matrix4f this_pose = Eigen::Matrix4f::Identity();;
            this_pose.block<3, 3>(0, 0) = rotation_map;
            this_pose.block<3, 1>(0, 3) = translation_map;
            // double dist_e = (translation - translation_map).norm();
            // if (min_dist1 > dist_e)
            //     min_dist1 = dist_e;

            //fastgicp
            fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> reg;
            reg.setMaximumIterations(max_iterations);
            reg.setCorrespondenceRandomness(num_neighbors);
            reg.setNumThreads(num_threads);

            pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
            Eigen::Matrix4f transformation;  
            pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            *src_cloud = *source_cloud;
            readBinFile(map_set[jdx],target_cloud);

            target_cloud->erase(
                std::remove_if(target_cloud->begin(), target_cloud->end(), [=](const pcl::PointXYZ& pt) { return pt.getVector3fMap().squaredNorm() < 1e-3; }),
                target_cloud->end());

            // downsampling

            filtered.reset(new pcl::PointCloud<pcl::PointXYZ>());
            voxelgrid.setInputCloud(target_cloud);
            voxelgrid.filter(*filtered);
            target_cloud = filtered;
            // pcl::transformPointCloud(*target_cloud, *target_cloud, BASE2OUSTER);
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(filtered);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(z_min, z_max);
            pass.filter(*target_cloud);
            Eigen::Matrix4f new_svd = Eigen::Matrix4f::Identity();

            // Convert angle to radians
            double angle_radians = 0;
            if (score[si][1] == 0)angle_radians = 0;
            else if (score[si][1] > 0) angle_radians =((score[si][1])*(360.0/float(sector_num)))* M_PI / 180.0;
            else if (score[si][1] < 0) angle_radians =((score[si][1])*(360.0/float(sector_num)))* M_PI / 180.0;
             
            
            // Create 3×3 matrix for rotation around z-axis
            double cos_theta = cos(angle_radians);
            double sin_theta = sin(angle_radians);
            Eigen::Matrix3f rotation_z;
            rotation_z << cos_theta, -sin_theta, 0,
                          sin_theta, cos_theta, 0,
                          0, 0, 1;
            new_svd.block<3, 3>(0, 0) = rotation_z;
            // this_pose.block<3, 3>(0, 0) = rotation_map;
            pcl::transformPointCloud(*src_cloud, *src_cloud, new_svd);
            reg.clearTarget();
            reg.clearSource();
            reg.setInputTarget(target_cloud);
            reg.setInputSource(src_cloud);
            reg.align(*aligned);
            double fitness_score = reg.getFitnessScore();
            transformation = reg.getFinalTransformation();

            pcl::transformPointCloud(*src_cloud, *src_cloud, transformation);
            auto t_start_fitness = std::chrono::high_resolution_clock::now();
            fitness_score = computeOverlapFast(src_cloud,target_cloud,distance_threshold,leaf_size);
            auto t_end_fitness = std::chrono::high_resolution_clock::now();
            score_time += time_inc(t_end_fitness,t_start_fitness);
            // BEVType bev_img = point_cloud_to_bev(src_cloud);
            // BEVType bev_img_new = crop_to_minimum_bev(bev_img);

            // int m_B = bev_img_new.size();
            // int n_B = bev_img_new[0].size();
            // // Calculate weight matrix W
            // std::vector<std::vector<int>> W(m_B, std::vector<int>(n_B, 0));
            // for (int i = 0; i < m_B; ++i) {
            //     for (int j = 0; j < n_B; ++j) {
            //         if (bev_img_new[i][j] != 0) {
            //             W[i][j] = 1;
            //         }
            //     }
            // }
            // std::vector<double> result = binary_correlation_2d_openmp(map_bev[jdx],bev_img_new,lookup_table,W);
            // std::pair<double, int> this_score;
            // auto max_it = std::max_element(result.begin(), result.end());
            // double max_value = *max_it;  
            Gene_BPR this_BPR(score[si][1],this_pose ,fitness_score,map_set[jdx],transformation,new_svd,si);
            Map_BPR.push_back(this_BPR);
            if(min_dinex == si)min_fit=fitness_score;
            if (fitness_score > overlap_threshold)
            break;
        }
        if (min_dist1 <= 8) 
        {
            Turth_SR++;
            list_all[min_dinex+50]++;
            // if (min_rot>=20)fail_rot++;
        }
        // Sort in descending order
        std::sort(Map_BPR.begin(), Map_BPR.end(), 
                [](const Gene_BPR& a, const Gene_BPR& b) {
                    return a.final_score > b.final_score; 
                });
        auto t_end_bev = std::chrono::high_resolution_clock::now();


        auto t_end_gicp = std::chrono::high_resolution_clock::now();

        // Assume Map_BPR[0].transformation is TLO (point cloud registration result)
        Eigen::Matrix4f TLO = Map_BPR[0].transformation;

        // Calculate transformation from vehicle coordinate system A to vehicle coordinate system B

        Eigen::Matrix4f map_pose = (Map_BPR[0].pose ) *  Map_BPR[0].transformation * Map_BPR[0].rotation_matrix ;// * BASE2OUSTER * BASE2LIVOX.inverse() ;// 
        Eigen::Matrix4f query_pose = Eigen::Matrix4f::Identity();
        query_pose.block<3, 3>(0, 0) = rotation;
        query_pose.block<3, 1>(0, 3) = translation;
        double T_error,R_error,T_o,R_o,T_r,R_r;
        query_pose = query_pose ;
        // map_pose = map_pose * BASE2OUSTER;
        compute_adj_rpe(query_pose ,map_pose,T_error,R_error);
        Eigen::Matrix4f M_pose = Map_BPR[0].pose ;
        compute_adj_rpe(query_pose,M_pose,T_o,R_o);
        Eigen::Matrix4f R_pose =Map_BPR[0].pose * Map_BPR[0].rotation_matrix ;
        compute_adj_rpe(query_pose,R_pose,T_r,R_r);
        auto t_end = std::chrono::high_resolution_clock::now();

        Eigen::Vector3f final_t = map_pose.block<3, 1>(0, 3);
        if (T_error < 5 && R_error < 10.0) //
        {
            SR++;
            T_all += T_error;
            R_all += R_error;
            firness_all += Map_BPR[0].final_score;
            list_all[Map_BPR[0].list_n]++;
            transErrors.push_back(T_error);
            rotErrors.push_back(R_error);
            timeCosts.push_back(time_inc(t_end, t_begin));
            ValidIndex.push_back(idx);
            ValidTime+=time_inc(t_end, t_begin);
            finas_x.push_back(final_t(0));
            finas_y.push_back(final_t(1));
            GEN_time.push_back(time_inc(t_end_cld, t_begin_cld));
            Search_time.push_back(time_inc(t_end_bc2d, t_begin_bc2d));
            PE_time.push_back(time_inc(t_end_bev, t_start_bev));
        }
        else if(Map_BPR[0].final_score <= overlap_threshold && min_dist1<=10)
        {
            error_num++;
        }
        if (!(T_error < 5 && R_error < 10.0)){
            fina_x.push_back(final_t(0));
            fina_y.push_back(final_t(1));
        }
        std::vector<double> pair_xx,pair_yy;
        Eigen::Vector3f map_p = Map_BPR[0].pose.block<3, 1>(0, 3);
        Eigen::Vector3f query_p = translation;
        if (min_dist1 < 10 || T_error < 5)
        {
            pair_yy.push_back(query_p(1));
            pair_xx.push_back(query_p(0));
            pair_xx.push_back(final_t(0));
            pair_yy.push_back(final_t(1));
            pair_yy.push_back(map_p(1));
            pair_xx.push_back(map_p(0));
            pair_x.push_back(pair_xx);
            pair_y.push_back(pair_yy);
        }
        if(T_error < 5 && R_error >= 10.0 )
        {
            fail_rot++;
        }else if(T_error >= 5 && R_error < 10.0){
            fail_trans++;
        }

        if((T_r < 5 && R_r < 10.0 && R_o > R_r))
            negative ++;
        else if(T_o < 5 && R_o < 10.0) positive++;

        
        if(min_dist<=20)
        {
            total_time += time_inc(t_end, t_begin);
            total_time_score += score_time;
        }
        if(abs(min_rot - (score[min_dinex][1])*7.5) < 20 && min_dist1<8)
            trans_min++;
        // std::cout<<"query_pose:"<<query_pose<<std::endl;
        // std::cout<<"Map_BPR[0].pose:"<<Map_BPR[0].pose<<std::endl;
        // std::cout<<"Map_BPR[0].transformation:"<<Map_BPR[0].transformation<<std::endl;
        std::cout<<"fitness:"<<Map_BPR[0].final_score<<" "<<firness_all/SR<<" "<<Map_BPR[0].list_n<<std::endl;
        std::cout<<"offset:"<<score[Map_BPR[0].list_n][3]<<" maxindex:"<<score[Map_BPR[0].list_n][1]<<" zero_len:"<<score[Map_BPR[0].list_n][4]<<endl;
        std::cout<<"min_dinex:"<<min_dinex<<" min_rot:"<<min_rot<<" sector_error:"<<min_rot - (score[min_dinex][1])*7.5<<std::endl;
        std::cout<<"             min_dist1:"<<min_dist1<<" min_fit:"<<min_fit<<std::endl;
        std::cout<<"             min_offset:"<<score[min_dinex][3]<<" min_maxindex:"<<score[min_dinex][1]<<std::endl;
        std::cout<<"T_error:"<<T_error<<" R_error:"<<R_error<<std::endl;
        std::cout<<"T_o:"<<T_o<<" R_o:"<<R_o<<" "<<min_dist1<<" "<<trans_min/Turth_SR<<std::endl;
        std::cout<<"T_r:"<<T_r<<" R_r:"<<R_r<<std::endl; 
        std::cout<<"positive:"<<positive<<" negative:"<<negative<<std::endl;
        std::cout<<"T_all:"<<T_all/SR<<" R_all:"<<R_all/SR<<std::endl;
        std::cout<<"BEV cost time:"<<time_inc(t_end_bc2d, t_begin_bc2d)<<std::endl;
        std::cout<<"GICP cost time:"<<time_inc(t_end_bev, t_start_bev)-score_time<<std::endl;
        std::cout<<"overlap cost time:"<<score_time<<std::endl;
        std::cout<<"cost time:"<<time_inc(t_end, t_begin)<<std::endl;
        std::cout<<"average time:"<<total_time/total_num<<" " <<total_time_score/total_num<< std::endl;
        std::cout<<"SR:"<<SR/total_num<<" "<<Turth_SR/total_num<<" "<<error_num/total_num<<std::endl;
        std::cout<<"fail_rot:"<<fail_rot/total_num<<" fail_trans:"<<fail_trans/total_num<<std::endl;
        std::cout<<"Q:"<<query_set[idx]<<std::endl;
        std::cout<<"M:"<<Map_BPR[0].path<<std::endl;
        std::cout<<"L:"<<map_set[score[min_dinex][2]]<<std::endl;
        for (int ii = 0 ; ii < 100;ii++)
        {
            cout<<list_all[ii]<<" ";
            if (ii%10 == 9)cout<<endl;
        }
    }
    std::cout<<"T_all:"<<T_all/SR<<" R_all:"<<R_all/SR<<std::endl;
    std::cout<<"T_RSME:"<<calculateRMSE(transErrors)<<" R_RSME:"<<calculateRMSE(rotErrors)<<std::endl;
    std::cout<<"average time:"<<ValidTime/SR<< std::endl;
    std::cout<<"SR:"<<SR/total_num<<" "<<Turth_SR/total_num<<" "<<error_num/total_num<<std::endl;
    std::cout<<"fail_rot:"<<fail_rot/total_num<<" fail_trans:"<<fail_trans/total_num<<std::endl;





    std::map<std::string, std::string> red_map;
    std::map<std::string, std::string> blue_map;
    std::map<std::string, std::string> yellow_map;
    std::map<std::string, std::string> green_map;

    red_map["color"] = "red";
    blue_map["color"] = "blue";
    yellow_map["color"] = "yellow";
    green_map["color"] = "green";
    plt::figure(2);
    plt::scatter(map_x,map_y, 10.0,red_map);
    plt::scatter(data_x,data_y, 10.0,blue_map);
    plt::scatter(fina_x, fina_y, 20.0,yellow_map);
    plt::scatter(finas_x, finas_y, 20.0,green_map);
    for (int i =0;i < pair_x.size();i++)
    {
        std::vector<double> sub_x = pair_x[i];
        std::vector<double> sub_y = pair_y[i];
        plt::plot(sub_x,sub_y);
    }
    std::cout<<"map_x:"<<map_x.size()<<std::endl;
    std::cout<<"data_x:"<<data_x.size()<<std::endl;
    plt::show();

    return 0;

}