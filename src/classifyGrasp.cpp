#include <armadillo>

std::vector<GraspSet> GraspDetector::classifyGraspCandidates(const CloudCamera& cloud_cam,
  std::vector<GraspSet>& candidates)
{
  // Create a grasp image for each grasp candidate
  std::cout << "Creating grasp images for classifier input ...\n";
  int num_orientations = candidates[0].getHypotheses().size();

  // Create the grasp images.
  std::vector<cv::Mat> image_list = learning_->createImages(cloud_cam, candidates);
  std::cout << " Image creation time: " << omp_get_wtime() - t0 << std::endl;
  std::vector<Grasp> valid_grasps;
  std::vector<cv::Mat> valid_images;
  extractGraspsAndImages(candidates, image_list, valid_grasps, valid_images);
  std::vector<float> scores;
  std::vector<GraspSet> grasp_list;
  for (int i=0;i<num_iters;i++)
  {
    double t0_prediction = omp_get_wtime();
    // Classify the grasp images.
    scores = classifier_->classifyImages(valid_images);
    grasp_list.assign(valid_grasps.begin(), valid_grasps.end());
    std::sort( grasp_list.begin(), grasp_list.end(), isScoreGreater);//sort the grtaps based on their score

    std::vector<Grasp> elite_grasps;
    num_grasps=selected_grasps.size();
    gmm_refit=0.25;
    num_refit =max(ceil(gmm_refit * num_grasps),1); //max #include <algorithm>
    elite_grasps.assign(grasp_list.begin(), grasp_list.begin() + num_refit);
    arma::mat elite_grasps_arr;
    arma::mat elite_grasps_v;
    for (auto& g : elite_grasps)
      elite_grasps_v=feature_vec(g);
      elite_grasps_arr=arma::join_rows(elite_grasps_arr,elite_grasps_v)

    elite_grasps_mean=arma::mean(elite_grasps_arr,dim=1);
    elite_grasps_std=arma::stddev(elite_grasps_arr,dim=1);
    for (auto& g : elite_grasps)
      if (g == 0)
         g = 1.0;
    elite_grasp_arr = (elite_grasp_arr - elite_grasp_mean) / elite_grasp_std;

    arma::gmm_full gmm_model;
    bool status = gmm_model.learn(elite_grasp_arr, 3, maha_dist, random_subset, 10, 5, 1e-10, true);
    num_gmm_samples=50;
    arma::mat grasp_vecs_mat=gmm_model.generate(num_gmm_samples);

    std::vector<Grasp>& gmm_candidates=from_feature_vec(grasp_vecs_mat);

    image_list = learning_->createImages(cloud_cam, gmm_candidates);
    extractGraspsAndImages(candidates, image_list, valid_grasps, valid_images);
  }

  std::cout << "Total classification time: " << omp_get_wtime() - t0 << std::endl;

  if (plot_valid_grasps_)
  {
    Plot plotter;
    plotter.plotFingers(valid_grasps, cloud_cam.getCloudOriginal(), "Valid Grasps");
  }

  return valid_grasps;
}

arma::mat feature_vec(Grasp& elite_grasp)
{
    Eigen::Vector3d v_bottom,v_surface,v_sample;
    Eigen::Matrix3d v_frame;
    v_bottom=elite_grasp[j].getGraspBottom();
    v_surface=elite_grasp[j].getGraspSurface();
    v_frame=elite_grasp[j].getFrame();
    v_sample=elite_grasp[j].getSample();
    arma::mat arma_data;
    Eigen::MatrixXd v;
    v << v_bottom, v_surface, v_frame, v_sample;
    Map<VectorXf> v_vector(v.data(), v.size());
    arma_v = arma::mat(v_vector.data(), v_vector.rows(), v_vector.cols(),false, false);
    return arma_v;
}

std::vector<Grasp> from_feature_vec(arma::mat& v)
{
  std::vector<Grasp> grasps;
  Eigen::MatrixXd eigen_v = Eigen::Map<Eigen::MatrixXd>(v.memptr(),
                                                        v.n_rows,
                                                        v.n_cols);
  for (int i=0; i<v.n_cols,i++)
  {
    Eigen::VectorXd grasp_mat;
    grasp_mat=eigen_v(:,i);
    grasp_mat.resize(6,3)
    Grasp vector_grasp;
    vector_grasp.pose_.bottom_=grasp_mat.block<3,1>(0,0);
    vector_grasp.pose_.surface_=grasp_mat.block<3,1>(0,1);
    vector_grasp.pose_.frame_=grasp_mat.block<3,3>(0,2);
    vector_grasp.sample_=grasp_mat.block<3,1>(0,5);
    Eigen::Vector3d pos_;
    pos_<< hand_depth,0.0,0.0;
    vector_grasp.pose_.top_ = vector_grasp.pose_.bottom_+vector_grasp.pose_.frame_ * pos_;

    Eigen::Vector3d pos_bottom;
    pos_bottom=(vector_grasp.pose_.bottom_-vector_grasp.sample_)/vector_grasp.pose_.frame_;
    vector_grasp.config_1d_.bottom_ = pos_bottom(0);
    vector_grasp.config_1d_.top_ = pos_bottom(0)+hand_depth;
    vector_grasp.config_1d_.center_ = pos_bottom(1);
  }
    grasps.push_back(vector_grasp);
    return grasps;
}
