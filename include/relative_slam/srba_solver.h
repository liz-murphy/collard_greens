#ifndef KARTO_SRBASOLVER_H
#define KARTO_SRBASOLVER_H

#include <srba/srba.h>
#include <srba/srba_types.h>
#include <visualization_msgs/MarkerArray.h>
#include <OpenKarto/SensorData.h>
using namespace srba;
//using namespace mrpt::utils;

struct RBA_OPTIONS : public srba::RBA_OPTIONS_DEFAULT
{
  //typedef ecps::local_areas_fixed_size            edge_creation_policy_t;  //!< One of the most important choices: how to construct the relative coordinates graph problem
  typedef ecps::classic_linear_rba edge_creation_policy_t;  //!< One of the most important choices: how to construct the relative coordinates graph problem
  //  typedef options::sensor_pose_on_robot_none      sensor_pose_on_robot_t;  //!< The sensor pose coincides with the robot pose
  typedef options::observation_noise_constant_matrix<observations::RelativePoses_2D>   obs_noise_matrix_t;      // The sensor noise matrix is the same for all observations and equal to some given matrix
  //  typedef options::solver_LM_schur_dense_cholesky solver_t;                //!< Solver algorithm (Default: Lev-Marq, with Schur, with dense Cholesky)
};

typedef RbaEngine<
  kf2kf_poses::SE2,               // Parameterization  of KF-to-KF poses
  landmarks::RelativePoses2D,     // Parameterization of landmark positions
  observations::RelativePoses_2D, // Type of observations
  RBA_OPTIONS
  >  srba_t;

typedef TRBA_Problem_state<
  kf2kf_poses::SE2,               // Parameterization  of KF-to-KF poses
  landmarks::RelativePoses2D,     // Parameterization of landmark positions
  observations::RelativePoses_2D, // Type of observations
  RBA_OPTIONS
  > problem_state_t;

struct MY_FEAT_VISITOR
{
  bool visit_filter_feat(
    const TLandmarkID lm_ID,
    const topo_dist_t cur_dist)
  {
    // Return true if it's desired to visit this keyframe node
  }

  void visit_feat(
    const TLandmarkID lm_ID,
    const topo_dist_t cur_dist)
  {
    // Process this keyframe node
  }
};

struct MY_KF_VISITOR
{
  bool visit_filter_kf(
    const TKeyFrameID kf_ID,
    const topo_dist_t cur_dist)
  {
    // Return true if it's desired to visit this keyframe node
    return true;
  }

  void visit_kf(
    const TKeyFrameID kf_ID,
    const topo_dist_t cur_dist)
  {
    // Process this keyframe node
  }
};

struct MY_K2K_EDGE_VISITOR
{
  bool visit_filter_k2k(
    const TKeyFrameID current_kf,
    const TKeyFrameID next_kf,
    const srba_t::k2k_edge_t* edge,
    const topo_dist_t cur_dist)
  {
    // Return true if it's desired to visit this keyframe node
  }

  void visit_k2k(
    const TKeyFrameID current_kf,
    const TKeyFrameID next_kf,
    const srba_t::k2k_edge_t* edge,
    const topo_dist_t cur_dist)
  {
    // Process this keyframe node
  }
};

struct MY_K2F_EDGE_VISITOR
{
  bool visit_filter_k2f(
    const TKeyFrameID current_kf,
    const srba_t::k2f_edge_t* edge,
    const topo_dist_t cur_dist)
  {
    // Return true if it's desired to visit this keyframe node
  }

  void visit_k2f(
    const TKeyFrameID current_kf,
    const srba_t::k2f_edge_t* edge,
    const topo_dist_t cur_dist)
  {
    // Process this keyframe node
  }
};


class SRBASolver 
{
public:
  SRBASolver();
  virtual ~SRBASolver();

public:
  virtual void Clear();
  //virtual void Compute();
  //virtual const karto::ScanSolver::IdPoseVector& GetCorrections() const;

  int AddNode();
  void AddConstraint(int sourceId, int targetId, const karto::Pose2 &rDiff, const karto::Matrix3& rCovariance);

  //virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge);
  virtual void getGraph(std::vector<float> &g);
  virtual void getActiveIds(std::vector<int> &ids);

  void publishGraphVisualization(visualization_msgs::MarkerArray &marray);

  void GetNearLinkedObjects(int kf_id);

protected:
//  karto::ScanSolver::IdPoseVector corrections_;
  srba_t rba_;
  srba_t::new_kf_observations_t list_obs_;
  int curr_kf_id_;
  bool first_keyframe_;
  bool first_edge_;
};

#endif // KARTO_SRBA_SOLVER_H

