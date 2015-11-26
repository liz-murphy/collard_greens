#include "ros/ros.h"
#include "ros/console.h"
#include "message_filters/subscriber.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "visualization_msgs/MarkerArray.h"

#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"

//#include "OpenKarto/ScanMatcher.h"
//#include "OpenKarto/ScanManager.h"
#include "OpenKarto/OpenMapper.h"
#include <relative_slam/srba_solver.h>
#include <boost/thread.hpp>

#include <string>
#include <map>
#include <vector>
#include <list>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

#define MAX_VARIANCE            500.0
using namespace karto;
using namespace srba;

// Also belongs back in karto


/*class NearScanVisitor : public Visitor<LocalizedObjectPtr>
{
public:
  NearScanVisitor(LocalizedLaserScan* pScan, kt_double maxDistance, kt_bool useScanBarycenter)
    : m_MaxDistanceSquared(math::Square(maxDistance))
    , m_UseScanBarycenter(useScanBarycenter)
  {
    m_CenterPose = pScan->GetReferencePose(m_UseScanBarycenter);
  }

  virtual kt_bool Visit(Vertex<LocalizedObjectPtr>* pVertex)
  {
    LocalizedObject* pObject = pVertex->GetVertexObject();

    LocalizedLaserScan* pScan = dynamic_cast<LocalizedLaserScan*>(pObject);
    
    // object is not a scan or wasn't scan matched, ignore
    if (pScan == NULL)
    {
      return false;
    }
    
    Pose2 pose = pScan->GetReferencePose(m_UseScanBarycenter);

    kt_double squaredDistance = pose.GetPosition().SquaredDistance(m_CenterPose.GetPosition());
    return (squaredDistance <= m_MaxDistanceSquared - KT_TOLERANCE);
  }

protected:
  Pose2 m_CenterPose;
  kt_double m_MaxDistanceSquared;
  kt_bool m_UseScanBarycenter;

}; // NearScanVisitor

*/


class RelativeSlam
{
  public:
    RelativeSlam();
    ~RelativeSlam();

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                                 nav_msgs::GetMap::Response &res);

  private:
    bool getOdomPose(karto::Pose2& karto_pose, const ros::Time& t);
    karto::LaserRangeFinder* getLaser(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool addScan(karto::LaserRangeFinder* laser,
      const sensor_msgs::LaserScan::ConstPtr& scan,
      karto::Pose2& karto_pose);
    bool updateMap();
    void publishTransform();
    void publishLoop(double transform_publish_period);
    void publishVis(double vis_publish_period);
    void publishGraphVisualization();
    bool hasMovedEnough(karto::LocalizedRangeScan* pScan, karto::LocalizedRangeScan* pLastScan) const;
    bool process(karto::LocalizedRangeScan* pScan);

    // These really should be moved back into karto once the graph stuff has been ripped out
    bool addEdges(karto::LocalizedObject *pObject);
    void LinkObjects(LocalizedObject* pFromObject, LocalizedObject* pToObject, const Pose2& rMean, const Matrix3& rCovariance);
    bool AddEdges(LocalizedLaserScanPtr pScan, const Matrix3& rCovariance);
    void LinkChainToScan(const LocalizedLaserScanList& rChain, LocalizedLaserScanPtr pScan, const Pose2& rMean, const Matrix3& rCovariance);
    void LinkNearChains(LocalizedLaserScanPtr pScan, Pose2List& rMeans, List<Matrix3>& rCovariances);
    Pose2 ComputeWeightedMean(const Pose2List& rMeans, const List<Matrix3>& rCovariances) const;
    LocalizedLaserScanPtr GetClosestScanToPose(const LocalizedLaserScanList& rScans, const Pose2& rPose) const;
    List<LocalizedLaserScanList> FindNearChains(LocalizedLaserScanPtr pScan);
    LocalizedLaserScanList FindNearLinkedScans(LocalizedLaserScanPtr pScan, kt_double maxDistance);   
    kt_bool TryCloseLoop(LocalizedLaserScanPtr pScan, const Identifier& rSensorName);
    std::list<LocalizedLaserScanPtr> FindPossibleLoopClosure(LocalizedLaserScanPtr pScan, const Identifier& rSensorName, kt_int32u& rStartScanIndex);
    void CorrectPoses();

     // ROS handles
    ros::NodeHandle node_;
    tf::TransformListener tf_;
    tf::TransformBroadcaster* tfB_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    ros::Publisher sst_;
    ros::Publisher marker_publisher_;
    ros::Publisher sstm_;
    ros::ServiceServer ss_;

    // The map that will be published / send to service callers
    nav_msgs::GetMap::Response map_;

    // Storage for ROS parameters
    std::string odom_frame_;
    std::string relative_map_frame_;
    std::string base_frame_;
    int throttle_scans_;
    ros::Duration map_update_interval_;
    double resolution_;
    boost::mutex map_mutex_;
    boost::mutex map_to_odom_mutex_;

    boost::mutex scan_manager_mutex_;
    
    // Karto bookkeeping
    karto::MapperSensorManager* scan_manager_;
    karto::ScanMatcher* sequential_scan_matcher_;
    karto::ScanMatcher* loop_scan_matcher_;
    SRBASolver solver_;
    std::map<std::string, karto::LaserRangeFinder*> lasers_;
    std::map<std::string, bool> lasers_inverted_;
    karto::Identifier sensor_name_;

    // Internal state
    bool got_map_;
    boost::thread* transform_thread_;
    boost::thread* vis_thread_;
    tf::Transform relative_map_to_odom_;
    bool inverted_laser_;

    // These need to be params here
    int scan_buffer_size_; 
    double scan_buffer_max_distance_;
    double corr_search_space_dim_;
    double corr_search_space_res_;
    double corr_search_space_smear_dev_;
    double laser_range_threshold_;
    double minimum_travel_heading_;
    double minimum_travel_distance_;
    double link_match_min_response_fine_; 
    bool use_scan_barycenter_;
    double link_scan_max_distance_;
    int laser_count_;
    int loop_match_min_chain_size_;
    double loop_match_max_variance_coarse_;
    double loop_match_min_response_coarse_;
    double loop_match_min_response_fine_;
    double loop_search_space_dim_;
    double loop_search_space_res_;
    double loop_search_space_smear_dev_;
    double loop_search_max_distance_;
    bool is_multithreaded_;
};

RelativeSlam::RelativeSlam() : got_map_(false),
  transform_thread_(NULL),
  scan_buffer_size_(70),
  scan_buffer_max_distance_(20),
  corr_search_space_dim_(0.3),
  corr_search_space_res_(0.01),
  corr_search_space_smear_dev_(0.03),
  laser_range_threshold_(25.0),
  minimum_travel_heading_(0.35),
  minimum_travel_distance_(0.2),
  link_match_min_response_fine_(0.8),
  use_scan_barycenter_(true),
  link_scan_max_distance_(10.0),
  is_multithreaded_(false),
  loop_match_min_chain_size_(10),
  loop_match_max_variance_coarse_(100),
  loop_match_min_response_coarse_(0.05),
  loop_match_min_response_fine_(0.05),
  loop_search_space_dim_(8),
  loop_search_space_res_(0.05),
  loop_search_space_smear_dev_(0.03),
  loop_search_max_distance_(4.0),
  laser_count_(0)
{
  relative_map_to_odom_.setIdentity();
  // Retrieve parameters
  ros::NodeHandle private_nh_("~");
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";
  if(!private_nh_.getParam("relative_map_frame", relative_map_frame_))
    relative_map_frame_ = "relative_map";
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if(!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  double tmp;
  if(!private_nh_.getParam("map_update_interval", tmp))
    tmp = 5.0;
  map_update_interval_.fromSec(tmp);
  if(!private_nh_.getParam("resolution", resolution_))
  {
    // Compatibility with slam_gmapping, which uses "delta" to mean
    // resolution
    if(!private_nh_.getParam("delta", resolution_))
      resolution_ = 0.05;
  }
  double transform_publish_period;
  private_nh_.param("transform_publish_period", transform_publish_period, 0.05);
  double vis_publish_period;
  private_nh_.param("vis_publish_period", vis_publish_period, 5.0);

  // Set up advertisements and subscriptions
  tfB_ = new tf::TransformBroadcaster();
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &RelativeSlam::mapCallback, this);
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&RelativeSlam::laserCallback, this, _1));
  marker_publisher_ = node_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",1);

  // Create a thread to periodically publish the latest map->odom
  // transform; it needs to go out regularly, uninterrupted by potentially
  // long periods of computation in our main loop.
  transform_thread_ = new boost::thread(boost::bind(&RelativeSlam::publishLoop, this, transform_publish_period));
  vis_thread_ = new boost::thread(boost::bind(&RelativeSlam::publishVis, this, vis_publish_period));

  // Initialize Karto structures
  scan_manager_ = new karto::MapperSensorManager(scan_buffer_size_, scan_buffer_max_distance_);
  sequential_scan_matcher_ = karto::ScanMatcher::Create(corr_search_space_dim_, corr_search_space_res_, corr_search_space_smear_dev_, laser_range_threshold_, false);
  loop_scan_matcher_ = karto::ScanMatcher::Create(loop_search_space_dim_, loop_search_space_res_, loop_search_space_smear_dev_, laser_range_threshold_, false);

  // Use SRBA for graph structures and solving
  //SRBASolver* solver_ = new SRBASolver();
}

RelativeSlam::~RelativeSlam()
{
  if(transform_thread_)
  {
    transform_thread_->join();
    delete transform_thread_;
  }
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
  if (scan_manager_)
    delete scan_manager_;
  if (sequential_scan_matcher_)
    delete sequential_scan_matcher_;
  //if (solver_)
   // delete solver_;
}

void RelativeSlam::publishLoop(double transform_publish_period)
{
  if(transform_publish_period == 0)
    return;

   ros::Rate r(1.0 / transform_publish_period);
   while(ros::ok())
   {
    publishTransform();
    r.sleep();
   }
}

void RelativeSlam::publishTransform()
{
  boost::mutex::scoped_lock(map_to_odom_mutex_);
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(0.05);
  tfB_->sendTransform(tf::StampedTransform (relative_map_to_odom_, ros::Time::now(), relative_map_frame_, odom_frame_));
}

void RelativeSlam::publishVis(double vis_publish_period)
{
  if(vis_publish_period == 0)
    return;

   ros::Rate r(1.0 / vis_publish_period);
   while(ros::ok())
   {
    publishGraphVisualization();
    r.sleep();
   }
}

karto::LaserRangeFinder*
RelativeSlam::getLaser(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // Check whether we know about this laser yet
  if(lasers_.find(scan->header.frame_id) == lasers_.end())
  {
    // New laser; need to create a Karto device for it.

    // Get the laser's pose, relative to base.
    tf::Stamped<tf::Pose> ident;
    tf::Stamped<tf::Transform> laser_pose;
    ident.setIdentity();
    ident.frame_id_ = scan->header.frame_id;
    ident.stamp_ = scan->header.stamp;
    try
    {
      tf_.transformPose(base_frame_, ident, laser_pose);
    }
    catch(tf::TransformException e)
    {
      ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
         e.what());
      return NULL;
    }

    double yaw = tf::getYaw(laser_pose.getRotation());

    ROS_INFO("laser %s's pose wrt base: %.3f %.3f %.3f",
       scan->header.frame_id.c_str(),
       laser_pose.getOrigin().x(),
       laser_pose.getOrigin().y(),
       yaw);
    // To account for lasers that are mounted upside-down,
    // we create a point 1m above the laser and transform it into the laser frame
    // if the point's z-value is <=0, it is upside-down

    tf::Vector3 v;
    v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
    tf::Stamped<tf::Vector3> up(v, scan->header.stamp, base_frame_);

    try
    {
      tf_.transformPoint(scan->header.frame_id, up, up);
      ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
    }
    catch (tf::TransformException& e)
    {
      ROS_WARN("Unable to determine orientation of laser: %s", e.what());
      return NULL;
    }

    bool inverse = lasers_inverted_[scan->header.frame_id] = up.z() <= 0;
    if (inverse)
      ROS_INFO("laser is mounted upside-down");


    // Create a laser range finder device and copy in data from the first
    // scan
    sensor_name_.SetName(karto::String(scan->header.frame_id.c_str()));
    karto::LaserRangeFinder* laser = 
      karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, sensor_name_);
    laser->SetOffsetPose(karto::Pose2(laser_pose.getOrigin().x(),
              laser_pose.getOrigin().y(),
              yaw));
    laser->SetMinimumRange(scan->range_min);
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);
    // TODO: expose this, and many other parameters
    //laser_->SetRangeThreshold(12.0);

    // Store this laser device for later
    lasers_[scan->header.frame_id] = laser;

    // Register with the "Mapper"
    scan_manager_->RegisterSensor(sensor_name_);
  }
  return lasers_[scan->header.frame_id];
}

bool
RelativeSlam::getOdomPose(karto::Pose2& karto_pose, const ros::Time& t)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                           tf::Vector3(0,0,0)), t, base_frame_);
  tf::Stamped<tf::Transform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  karto_pose = 
          karto::Pose2(odom_pose.getOrigin().x(),
                       odom_pose.getOrigin().y(),
                       yaw);
  return true;
}

void RelativeSlam::publishGraphVisualization()
{
  visualization_msgs::MarkerArray marray;
  solver_.publishGraphVisualization(marray); 
  marker_publisher_.publish(marray);
}

void RelativeSlam::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  static ros::Time last_map_update(0,0);

  // Check whether we know about this laser yet
  karto::LaserRangeFinder* laser = getLaser(scan);

  if(!laser)
  {
    ROS_WARN("Failed to create laser device for %s; discarding scan",
       scan->header.frame_id.c_str());
    return;
  }

  karto::Pose2 odom_pose;
  if(addScan(laser, scan, odom_pose))
  {
    ROS_INFO("added scan at pose: %.3f %.3f %.3f", 
              odom_pose.GetX(),
              odom_pose.GetY(),
              odom_pose.GetHeading());

    if(!got_map_ || 
       (scan->header.stamp - last_map_update) > map_update_interval_)
    {
      if(updateMap())
      {
        last_map_update = scan->header.stamp;
        got_map_ = true;
        ROS_DEBUG("Updated the map");
      }
    }
  }
}

bool RelativeSlam::hasMovedEnough(karto::LocalizedRangeScan* pScan, karto::LocalizedRangeScan* pLastScan) const
{
    // test if first scan
    if (pLastScan == NULL)
    {
      return true;
    }

    karto::Pose2 lastScannerPose = pLastScan->GetSensorAt(pLastScan->GetOdometricPose());
    karto::Pose2 scannerPose = pScan->GetSensorAt(pScan->GetOdometricPose());

    // test if we have turned enough
    kt_double deltaHeading = karto::math::NormalizeAngle(scannerPose.GetHeading() - lastScannerPose.GetHeading());
    if (fabs(deltaHeading) >= minimum_travel_heading_)
    {
      return true;
    }

    // test if we have moved enough
    kt_double squaredTravelDistance = lastScannerPose.GetPosition().SquaredDistance(scannerPose.GetPosition());
    if (squaredTravelDistance >= karto::math::Square(minimum_travel_distance_) - karto::KT_TOLERANCE)
    {
      return true;
    }

    return false;
}

bool RelativeSlam::process(karto::LocalizedRangeScan* pScan)
{

  karto::LocalizedObject* pLocalizedObject = dynamic_cast<karto::LocalizedObject*>(pScan);
  if (pScan != NULL)
  {
    karto::LaserRangeFinder* pLaserRangeFinder = pScan->GetLaserRangeFinder();
    
    // validate scan
    if (pLaserRangeFinder == NULL)
    {
      return false;
    }

    pLaserRangeFinder->Validate(pScan);

    // ensures sensor has been registered with mapper--does nothing if the sensor has already been registered
    scan_manager_->RegisterSensor(pLocalizedObject->GetSensorIdentifier());

    
    boost::mutex::scoped_lock(scan_manager_mutex_);
    karto::LocalizedRangeScan* pLastScan = dynamic_cast<karto::LocalizedRangeScan *>(scan_manager_->GetLastScan(pLocalizedObject->GetSensorIdentifier()));
    
    // update scans corrected pose based on last correction
    if (pLastScan != NULL)
    {
      karto::Transform lastTransform(pLastScan->GetOdometricPose(), pLastScan->GetCorrectedPose());
      pScan->SetCorrectedPose(lastTransform.TransformPose(pScan->GetOdometricPose()));
      
      // test if scan is outside minimum boundary or if heading is larger then minimum heading
      if (!hasMovedEnough(pScan, pLastScan))
      {
        return false;
      }

      karto::Matrix3 covariance;
      covariance.SetToIdentity();

      // Correct scan
      karto::Pose2 bestPose;
      sequential_scan_matcher_->MatchScan(pScan,
                               scan_manager_->GetRunningScans(pScan->GetSensorIdentifier()),
                                           bestPose,
                                           covariance);
      pScan->SetSensorPose(bestPose);
  
      // Hook called before loop closing 
      //karto::ScanMatched(pScan); 
    }

    // Add scan to buffer and assign id
    int id = solver_.AddNode();
    pScan->SetUniqueId(id);
    scan_manager_->AddLocalizedObject(pScan);
    
    // Add edges
    if(pLastScan != NULL)
    {
      addEdges(pScan); 
    
      scan_manager_->AddRunningScan(pScan);
  
      // TO-DO: Loop closing attempts here
      List<Identifier> sensorNames = scan_manager_->GetSensorNames();
      karto_const_forEach(List<Identifier>, &sensorNames)
      {
        TryCloseLoop(pScan, *iter);
      } 
    }
    else
      scan_manager_->AddRunningScan(pScan);
    
    if(pScan == NULL)
    {
      ROS_ERROR("SCAN IS NULL!!!!!!");
    }
    else
    {
      scan_manager_->SetLastScan(pScan);
    }

    //karto::ScanMatchingEnd(pScan);
    return true;
  }
  return false;
}

bool RelativeSlam::addEdges(karto::LocalizedObject *pObject)
{
  // loose "spring"
  Matrix3 covariance;
  covariance(0, 0) = MAX_VARIANCE;
  covariance(1, 1) = MAX_VARIANCE;
  covariance(2, 2) = MAX_VARIANCE;
    
  karto::LocalizedLaserScanPtr pScan = dynamic_cast<karto::LocalizedLaserScan*>(pObject);
  if (pScan != NULL)
  {      
    AddEdges(pScan, covariance);
  }
  else
  {
    //MapperSensorManager* pSensorManager = m_pOpenMapper->m_pMapperSensorManager;      
    const Identifier& rSensorName = pObject->GetSensorIdentifier();
      
    boost::mutex::scoped_lock(scan_manager_mutex_);
    LocalizedLaserScan* pLastScan = scan_manager_->GetLastScan(rSensorName);
    if (pLastScan != NULL)
    {
      LinkObjects(pLastScan, pObject, pObject->GetCorrectedPose(), covariance);
    }
  }
}

void RelativeSlam::LinkObjects(LocalizedObject* pFromObject, LocalizedObject* pToObject, const Pose2& rMean, const Matrix3& rCovariance)
{
    //kt_bool isNewEdge = true;
    //Edge<LocalizedObjectPtr>* pEdge = AddEdge(pFromObject, pToObject, isNewEdge);
    
    // only attach link information if the edge is new
    //if (isNewEdge == true)
    //{

    // Calculate the difference
    LocalizedLaserScanPtr pScan = dynamic_cast<LocalizedLaserScan*>(pFromObject);
    Pose2 pose1, pose2;
    if (pScan != NULL)
    {
        pose1 = pScan->GetSensorPose();
    }
    else
    {
        pose1 = pScan->GetCorrectedPose();
    }

    // Do the update
    // transform second pose into the coordinate system of the first pose
    Transform transform(pose1, Pose2());
    Pose2 poseDiff = transform.TransformPose(rMean);

    // transform covariance into reference of first pose
    Matrix3 rotationMatrix;
    rotationMatrix.FromAxisAngle(0, 0, 1, -pose1.GetHeading());

    
    Matrix3 covariance = rotationMatrix * rCovariance * rotationMatrix.Transpose();
    ROS_INFO("Adding constraint:  %f, %f, %f", poseDiff.GetX(), poseDiff.GetY(), poseDiff.GetHeading()); 
    solver_.AddConstraint(pFromObject->GetUniqueId(), pToObject->GetUniqueId(), poseDiff, covariance);
}

bool RelativeSlam::AddEdges(LocalizedLaserScanPtr pScan, const Matrix3& rCovariance)
{
    const Identifier& rSensorName = pScan->GetSensorIdentifier();
    
    Pose2List means;
    List<Matrix3> covariances;
    
    boost::mutex::scoped_lock(scan_manager_mutex_);
    LocalizedLaserScanPtr pLastScan = scan_manager_->GetLastScan(rSensorName);
    if (pLastScan == NULL)
    {
      // first scan (link to first scan of other robots)

      boost::mutex::scoped_lock(scan_manager_mutex_);
      assert(scan_manager_->GetScans(rSensorName).Size() == 1);
      
      List<Identifier> sensorNames = scan_manager_->GetSensorNames();
      karto_const_forEach(List<Identifier>, &sensorNames)
      {
        const Identifier& rCandidateSensorName = *iter;
        
        // skip if candidate sensor is the same or other sensor has no scans
        if ((rCandidateSensorName == rSensorName) || (scan_manager_->GetScans(rCandidateSensorName).IsEmpty()))
        {
          continue;
        }
        
        Pose2 bestPose;
        Matrix3 covariance;
        kt_double response = sequential_scan_matcher_->MatchScan(pScan, scan_manager_->GetScans(rCandidateSensorName), bestPose, covariance);
        LinkObjects(scan_manager_->GetScans(rCandidateSensorName)[0], pScan, bestPose, covariance);
        
        // only add to means and covariances if response was high "enough"
        //if (response > m_pOpenMapper->m_pLinkMatchMinimumResponseFine->GetValue())
        if (response > link_match_min_response_fine_)
        {
          means.Add(bestPose);
          covariances.Add(covariance);
        }
      }
    }
    else
    {
      // link to previous scan
      LinkObjects(pLastScan, pScan, pScan->GetSensorPose(), rCovariance);

      // link to running scans
      Pose2 scanPose = pScan->GetSensorPose();
      means.Add(scanPose);
      covariances.Add(rCovariance);
      LinkChainToScan(scan_manager_->GetRunningScans(rSensorName), pScan, scanPose, rCovariance);
    }
    
    // link to other near chains (chains that include new scan are invalid)
    LinkNearChains(pScan, means, covariances);
    
    if (!means.IsEmpty())
    {
      pScan->SetSensorPose(ComputeWeightedMean(means, covariances));
    }
}

void RelativeSlam::LinkChainToScan(const LocalizedLaserScanList& rChain, LocalizedLaserScanPtr pScan,
                                  const Pose2& rMean, const Matrix3& rCovariance)
{
  Pose2 pose = pScan->GetReferencePose(use_scan_barycenter_);

  LocalizedLaserScanPtr pClosestScan = GetClosestScanToPose(rChain, pose);
  assert(pClosestScan != NULL);

  Pose2 closestScanPose = pClosestScan->GetReferencePose(use_scan_barycenter_);

  kt_double squaredDistance = pose.GetPosition().SquaredDistance(closestScanPose.GetPosition());
  if (squaredDistance < math::Square(link_scan_max_distance_) + KT_TOLERANCE)
  {
    ROS_INFO("LinkChainToScan calling LinkObjects on %d to %d", pClosestScan->GetUniqueId(), pScan->GetUniqueId());
    if(pClosestScan->GetUniqueId() < 0)
    {
      ROS_ERROR("Invalid scan id!!!!");
      return;
    }
    LinkObjects(pClosestScan, pScan, rMean, rCovariance);
  }
}

void RelativeSlam::LinkNearChains(LocalizedLaserScanPtr pScan, Pose2List& rMeans, List<Matrix3>& rCovariances)
{
    const List<LocalizedLaserScanList> nearChains = FindNearChains(pScan);

    kt_bool gotTbb = false;
    if (is_multithreaded_)
    {
#ifdef USE_TBB
      gotTbb = true;
      kt_bool* pWasChainLinked = new kt_bool[nearChains.Size()];

      Pose2List means;
      means.Resize(nearChains.Size());

      List<Matrix3> covariances;
      covariances.Resize(nearChains.Size());

      int grainSize = 100;
      Parallel_LinkNearChains myTask(m_pOpenMapper, pScan, &nearChains, pWasChainLinked, &means, &covariances,
        loop_match_min_chain_size_,
        link_match_min_response_fine_);
      tbb::parallel_for(tbb::blocked_range<kt_int32s>(0, static_cast<kt_int32s>(nearChains.Size()), grainSize), myTask);

      for (kt_int32u i = 0; i < nearChains.Size(); i++)
      {
        if (pWasChainLinked[i] == true)
        {
          rMeans.Add(means[i]);
          rCovariances.Add(covariances[i]);
          LinkChainToScan(nearChains[i], pScan, means[i], covariances[i]);
        }
      }

      delete [] pWasChainLinked;
#endif
    }

    if (gotTbb == false)
    {
      karto_const_forEach(List<LocalizedLaserScanList>, &nearChains)
      {
#ifdef KARTO_DEBUG2
        std::cout << "Near chain for " << pScan->GetStateId() << ": [ ";
        karto_const_forEachAs(LocalizedLaserScanList, iter, iter2)
        {
          std::cout << (*iter2)->GetStateId() << " ";
        }
        std::cout << "]: ";
#endif

        if (iter->Size() < loop_match_min_chain_size_)
        {
#ifdef KARTO_DEBUG2
          std::cout << iter->Size() << "(< " << loop_match_min_chain_size_ << ") REJECTED" << std::endl;
#endif
          continue;
        }

        Pose2 mean;
        Matrix3 covariance;
        // match scan against "near" chain
        kt_double response = sequential_scan_matcher_->MatchScan(pScan, *iter, mean, covariance, false);
        if (response > link_match_min_response_fine_ - KT_TOLERANCE)
        {
#ifdef KARTO_DEBUG2
          std::cout << " ACCEPTED" << std::endl;
#endif
          rMeans.Add(mean);
          rCovariances.Add(covariance);
          LinkChainToScan(*iter, pScan, mean, covariance);
        }
        else
        {
#ifdef KARTO_DEBUG2
          std::cout << response << "(< " << link_match_min_response_fine_ << ") REJECTED" << std::endl;
#endif        
        }
      }
    }
  }



bool RelativeSlam::updateMap()
{
  boost::mutex::scoped_lock(map_mutex_);

  boost::mutex::scoped_lock(scan_manager_mutex_);
  karto::OccupancyGrid* occ_grid = 
          karto::OccupancyGrid::CreateFromScans(scan_manager_->GetRunningScans(sensor_name_), resolution_);

  if(!occ_grid)
  {
    ROS_INFO("No occupancy grid");
    return false;
  }

  ROS_INFO("Got occupancy grid");
  
  if(!got_map_) {
    map_.map.info.resolution = resolution_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  } 

  // Translate to ROS format
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset = occ_grid->GetCoordinateConverter()->GetOffset();

  ROS_INFO("Offset is %f, %f", offset.GetX(), offset.GetY());
  if(map_.map.info.width != (unsigned int) width || 
     map_.map.info.height != (unsigned int) height ||
     map_.map.info.origin.position.x != offset.GetX() ||
     map_.map.info.origin.position.y != offset.GetY())
  {
    //map_.map.info.origin.position.x = 0;
    map_.map.info.origin.position.x = offset.GetX();
    map_.map.info.origin.position.y = offset.GetY();
    //map_.map.info.origin.position.y = 0;
    map_.map.info.width = width;
    map_.map.info.height = height;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
  }

  for (kt_int32s y=0; y<height; y++)
  {
    for (kt_int32s x=0; x<width; x++) 
    {
      // Getting the value at position x,y
      kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));

      switch (value)
      {
        case karto::GridStates_Unknown:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
          break;
        case karto::GridStates_Occupied:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
          break;
        case karto::GridStates_Free:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
          break;
        default:
          ROS_WARN("Encountered unknown cell value at %d, %d", x, y);
          break;
      }
    }
  }
  
  // Set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = odom_frame_;

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);

  //delete occ_grid;

  return true;
}

bool RelativeSlam::addScan(karto::LaserRangeFinder* laser,
       const sensor_msgs::LaserScan::ConstPtr& scan, 
                   karto::Pose2& karto_pose)
{
  if(!getOdomPose(karto_pose, scan->header.stamp))
     return false;
  
  // Create a vector of doubles for karto
  std::vector<kt_double> readings;

  if (lasers_inverted_[scan->header.frame_id]) {
    for(std::vector<float>::const_reverse_iterator it = scan->ranges.rbegin();
      it != scan->ranges.rend();
      ++it)
    {
      readings.push_back(*it);
    }
  } else {
    for(std::vector<float>::const_iterator it = scan->ranges.begin();
      it != scan->ranges.end();
      ++it)
    {
      readings.push_back(*it);
    }
  }
  
  // create localized range scan
  karto::LocalizedRangeScan* range_scan = 
    new karto::LocalizedRangeScan(scan->header.frame_id.c_str(), readings);
  range_scan->SetOdometricPose(karto_pose);
  range_scan->SetCorrectedPose(karto_pose);

  // Add the localized range scan to the mapper
  bool processed;
  if((processed = process(range_scan)))
  {
    //std::cout << "Pose: " << range_scan->GetOdometricPose() << " Corrected Pose: " << range_scan->GetCorrectedPose() << std::endl;
    
    karto::Pose2 corrected_pose = range_scan->GetCorrectedPose();

    // Compute the map->odom transform
  /*  tf::Stamped<tf::Pose> odom_to_map;
    try
    {
      tf_.transformPose(odom_frame_,tf::Stamped<tf::Pose> (tf::Transform(tf::createQuaternionFromRPY(0, 0, corrected_pose.GetHeading()),
                                                                    tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0)),
                                                                    scan->header.stamp, base_frame_), odom_to_map);
    }
    catch(tf::TransformException e)
    {
      ROS_ERROR("Transform from base_link to odom failed\n");
      odom_to_map.setIdentity();
    }
*/
    map_to_odom_mutex_.lock();
    //relative_map_to_odom_ = tf::Transform(tf::Quaternion( odom_to_map.getRotation() ),
                                 //tf::Point(      odom_to_map.getOrigin() ) );
    relative_map_to_odom_ = tf::Transform( tf::createQuaternionFromRPY(0,0,corrected_pose.GetHeading()), tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0)).inverse();
    map_to_odom_mutex_.unlock();


    // Add the localized range scan to the dataset (for memory management)
    //dataset_->Add(range_scan);
  }
  //else
   // delete range_scan;

  return processed;
}

bool RelativeSlam::mapCallback(nav_msgs::GetMap::Request  &req,
                       nav_msgs::GetMap::Response &res)
{
  boost::mutex::scoped_lock(map_mutex_);
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res = map_;
    return true;
  }
  else
    return false;
}

Pose2 RelativeSlam::ComputeWeightedMean(const Pose2List& rMeans, const List<Matrix3>& rCovariances) const
{
  assert(rMeans.Size() == rCovariances.Size());
  
  // compute sum of inverses and create inverse list
  List<Matrix3> inverses;
  inverses.EnsureCapacity(rCovariances.Size());
  
  Matrix3 sumOfInverses;
  karto_const_forEach(List<Matrix3>, &rCovariances)
  {
    Matrix3 inverse = iter->Inverse();
    inverses.Add(inverse);
    
    sumOfInverses += inverse;
  }
  Matrix3 inverseOfSumOfInverses = sumOfInverses.Inverse();
  
  // compute weighted mean
  Pose2 accumulatedPose;
  kt_double thetaX = 0.0;
  kt_double thetaY = 0.0;
  
  Pose2List::ConstIterator meansIter = rMeans.GetConstIterator();
  karto_const_forEach(List<Matrix3>, &inverses)
  {
    Pose2 pose = *meansIter;
    kt_double angle = pose.GetHeading();
    thetaX += cos(angle);
    thetaY += sin(angle);
    
    Matrix3 weight = inverseOfSumOfInverses * (*iter);
    accumulatedPose += weight * pose;
    
    meansIter++;
  }
  
  thetaX /= rMeans.Size();
  thetaY /= rMeans.Size();
  accumulatedPose.SetHeading(atan2(thetaY, thetaX));
  
  return accumulatedPose;
}

List<LocalizedLaserScanList> RelativeSlam::FindNearChains(LocalizedLaserScanPtr pScan)
{
  List<LocalizedLaserScanList> nearChains;
  
  Pose2 scanPose = pScan->GetReferencePose(use_scan_barycenter_);
  
  // to keep track of which scans have been added to a chain
  LocalizedLaserScanList processed;
  
  const LocalizedLaserScanList nearLinkedScans = FindNearLinkedScans(pScan, link_scan_max_distance_);
  karto_const_forEach(LocalizedLaserScanList, &nearLinkedScans)
  {
    LocalizedLaserScanPtr pNearScan = *iter;
    
    if (pNearScan == pScan)
    {
      continue;
    }
    
    // scan has already been processed, skip
    if (processed.Contains(pNearScan) == true)
    {
      continue;
    }
    
#ifdef KARTO_DEBUG2
    std::cout << "BUILDING CHAIN: Scan " << pScan->GetStateId() << " is near " << pNearScan->GetStateId() << " (< " << link_scan_max_distance_ << ")" << std::endl;
#endif
    
    processed.Add(pNearScan);
    
    // build up chain
    kt_bool isValidChain = true;
    LocalizedLaserScanList chain;
     
    boost::mutex::scoped_lock(scan_manager_mutex_);
    LocalizedLaserScanList scans = scan_manager_->GetScans(pNearScan->GetSensorIdentifier());
    
    kt_int32s nearScanIndex = scan_manager_->GetScanIndex(pNearScan);
    assert(nearScanIndex >= 0);
    
    // add scans before current scan being processed
    for (kt_int32s candidateScanIndex = nearScanIndex - 1; candidateScanIndex >= 0; candidateScanIndex--)
    {
      LocalizedLaserScanPtr pCandidateScan = scans[candidateScanIndex];
      
      // chain is invalid--contains scan being added
      if (pCandidateScan == pScan)
      {
#ifdef KARTO_DEBUG2
        std::cout << "INVALID CHAIN: Scan " << pScan->GetStateId() << " is not allowed in chain." << std::endl;
#endif
        isValidChain = false;
      }
      
      Pose2 candidatePose = pCandidateScan->GetReferencePose(use_scan_barycenter_);
      kt_double squaredDistance = scanPose.GetPosition().SquaredDistance(candidatePose.GetPosition());
      
      if (squaredDistance < math::Square(link_scan_max_distance_ + KT_TOLERANCE))
      {
        chain.Add(pCandidateScan);
        processed.Add(pCandidateScan);
        
#ifdef KARTO_DEBUG2
        std::cout << "Building chain for " << pScan->GetStateId() << ": [ ";
        karto_const_forEachAs(LocalizedLaserScanList, &chain, iter2)
        {
          std::cout << (*iter2)->GetStateId() << " ";
        }
        std::cout << "]" << std::endl;
#endif          
      }
      else
      {
        break;
      }
    }
    
    chain.Add(pNearScan);
    
    // add scans after current scan being processed
    kt_size_t end = scans.Size();
    for (kt_size_t candidateScanIndex = nearScanIndex + 1; candidateScanIndex < end; candidateScanIndex++)
    {
      LocalizedLaserScanPtr pCandidateScan = scans[candidateScanIndex];
      
      if (pCandidateScan == pScan)
      {
#ifdef KARTO_DEBUG2
        std::cout << "INVALID CHAIN: Scan " << pScan->GetStateId() << " is not allowed in chain." << std::endl;
#endif          
        isValidChain = false;
      }
      
      Pose2 candidatePose = pCandidateScan->GetReferencePose(use_scan_barycenter_);;
      kt_double squaredDistance = scanPose.GetPosition().SquaredDistance(candidatePose.GetPosition());
      
      if (squaredDistance < math::Square(link_scan_max_distance_) + KT_TOLERANCE)
      {
        chain.Add(pCandidateScan);
        processed.Add(pCandidateScan);
        
#ifdef KARTO_DEBUG2
        std::cout << "Building chain for " << pScan->GetStateId() << ": [ ";
        karto_const_forEachAs(LocalizedLaserScanList, &chain, iter2)
        {
          std::cout << (*iter2)->GetStateId() << " ";
        }
        std::cout << "]" << std::endl;
#endif                    
      }
      else
      {
        break;
      }
    }
    
    if (isValidChain)
    {
      // add chain to collection
      nearChains.Add(chain);
    }
  }
  
  return nearChains;
}

LocalizedLaserScanPtr RelativeSlam::GetClosestScanToPose(const LocalizedLaserScanList& rScans, const Pose2& rPose) const
{
  LocalizedLaserScanPtr pClosestScan = NULL;
  kt_double bestSquaredDistance = DBL_MAX;
  
  karto_const_forEach(LocalizedLaserScanList, &rScans)
  {
    Pose2 scanPose = (*iter)->GetReferencePose(use_scan_barycenter_);
    
    kt_double squaredDistance = rPose.GetPosition().SquaredDistance(scanPose.GetPosition());
    if (squaredDistance < bestSquaredDistance)
    {
      bestSquaredDistance = squaredDistance;
      pClosestScan = *iter;
    }
  }
  
  return pClosestScan;
}


LocalizedLaserScanList RelativeSlam::FindNearLinkedScans(LocalizedLaserScanPtr pScan, kt_double maxDistance)
{
  //NearScanVisitor* pVisitor = new NearScanVisitor(pScan, maxDistance, use_scan_barycenter_);
  //LocalizedObjectList nearLinkedObjects = m_pTraversal->Traverse(GetVertex(pScan), pVisitor);
  //LocalizedObjectList nearLinkedObjects = solver_.bfs_visitor(pScan->GetUniqueId(), 100, false, pVisitor, NULL, NULL, NULL);
  //LocalizedObjectList nearLinkedObjects; 
  int max_topo_distance = maxDistance/minimum_travel_distance_;
  std::vector<int> linked_scans_ids = solver_.GetNearLinkedObjects(pScan->GetUniqueId(),max_topo_distance );
  
  LocalizedLaserScanList nearLinkedScans;
  //karto_const_forEach(LocalizedObjectList, &nearLinkedObjects)
  //{
  //  LocalizedObject* pObject = *iter;
  //  LocalizedLaserScan* pScan = dynamic_cast<LocalizedLaserScan*>(pObject);
  //  if (pScan != NULL)
  //  {
  //    nearLinkedScans.Add(pScan);
  //  }
  //}
  //
  for(int i=0; i < linked_scans_ids.size(); i++)
  {
    LocalizedObject* pObject = scan_manager_->GetLocalizedObject(linked_scans_ids[i]);
    LocalizedLaserScanPtr pScan = dynamic_cast<LocalizedLaserScan*>(pObject);
    if (pScan != NULL)
      nearLinkedScans.Add(pScan);
  }

  return nearLinkedScans;
}

kt_bool RelativeSlam::TryCloseLoop(LocalizedLaserScanPtr pScan, const Identifier& rSensorName)
  {
    kt_bool loopClosed = false;
    
    kt_int32u scanIndex = 0;
    
    std::list<LocalizedLaserScanPtr> candidateChainTemp = FindPossibleLoopClosure(pScan, rSensorName, scanIndex);
   

    while (!candidateChainTemp.empty())
    {

      // Nasty, but for now TODO FIX THIS
      LocalizedLaserScanList candidateChain;
      std::cout << "Candidate chain for " << pScan->GetStateId() << ": [ ";
      for( std::list<LocalizedLaserScanPtr>::iterator iter = candidateChainTemp.begin();iter != candidateChainTemp.end(); ++iter)
      {
        std::cout << (*iter)->GetStateId() << " ";
        candidateChain.Add(*iter);
      }
      std::cout << "]" << std::endl;
        
      Pose2 bestPose;
      Matrix3 covariance;
      ROS_INFO("Computing coarse response");
      kt_double coarseResponse = loop_scan_matcher_->MatchScan(pScan, candidateChain, bestPose, covariance, false, false);
      ROS_INFO("Done");
      
      StringBuilder message;
      ROS_INFO_STREAM("COARSE RESPONSE: " << coarseResponse << " (> " << loop_match_min_response_coarse_ << ")");
      ROS_INFO_STREAM("            var: " << covariance(0, 0) << ",  " << covariance(1, 1) << " (< " << loop_match_max_variance_coarse_ << ")");
      
      // This is just messaging 
      //MapperEventArguments eventArguments(message.ToString());
      //m_pOpenMapper->Message.Notify(this, eventArguments);
      
      if ( ( (coarseResponse > loop_match_min_response_coarse_) &&
           (covariance(0, 0) < loop_match_max_variance_coarse_) &&
           (covariance(1, 1) < loop_match_max_variance_coarse_ ))
          ||
          // be more lenient if the variance is really small
          ((coarseResponse > 0.9 * loop_match_min_response_coarse_ ) &&
           (covariance(0, 0) < 0.01 * loop_match_max_variance_coarse_) &&
           (covariance(1, 1) < 0.01 * loop_match_max_variance_coarse_)))
      {
        // save for reversion
        Pose2 oldPose = pScan->GetSensorPose();
        
        pScan->SetSensorPose(bestPose);
        kt_double fineResponse = sequential_scan_matcher_->MatchScan(pScan, candidateChain, bestPose, covariance, false);
        
        //message.Clear();
        ROS_INFO_STREAM("FINE RESPONSE: " << fineResponse << " (>" << loop_match_min_response_fine_ << ")");
        //MapperEventArguments eventArguments(message.ToString());
        //m_pOpenMapper->Message.Notify(this, eventArguments);
        
        if (fineResponse < loop_match_min_response_fine_)
        {
          // failed verification test, revert
          pScan->SetSensorPose(oldPose);
          
          //MapperEventArguments eventArguments("REJECTED!");
          //m_pOpenMapper->Message.Notify(this, eventArguments);
          ROS_INFO_STREAM("Rejected");
        }
        else
        {
          //MapperEventArguments eventArguments1("Closing loop...");
          //m_pOpenMapper->PreLoopClosed.Notify(this, eventArguments1);
          ROS_INFO_STREAM("Closing loop..."); 
          pScan->SetSensorPose(bestPose);
          LinkChainToScan(candidateChain, pScan, bestPose, covariance);
          CorrectPoses();

          //MapperEventArguments eventArguments2("Loop closed!");
          //m_pOpenMapper->PostLoopClosed.Notify(this, eventArguments2);
          
          //m_pOpenMapper->ScansUpdated.Notify(this, karto::EventArguments::Empty());      
          ROS_INFO_STREAM("Loop closed!");
          loopClosed = true;
        }
      }
      
      candidateChainTemp = FindPossibleLoopClosure(pScan, rSensorName, scanIndex);
    }
    return loopClosed;
  }

  std::list<LocalizedLaserScanPtr> RelativeSlam::FindPossibleLoopClosure(LocalizedLaserScanPtr pScan, const Identifier& rSensorName, kt_int32u& rStartScanIndex)
  {
    // This is pretty nasty, Clear() calls destructor on smart pointer objects ...
    //LocalizedLaserScanList chain; // return value
    std::list<LocalizedLaserScanPtr> chain;

    Pose2 pose = pScan->GetReferencePose(use_scan_barycenter_);
    
    // possible loop closure chain should not include close scans that have a
    // path of links to the scan of interest
    const LocalizedLaserScanList nearLinkedScans = FindNearLinkedScans(pScan, loop_search_max_distance_);
   
    boost::mutex::scoped_lock(scan_manager_mutex_);
    LocalizedLaserScanList scans = scan_manager_->GetScans(rSensorName);
    kt_size_t nScans = scans.Size();
    for (; rStartScanIndex < nScans; rStartScanIndex++)
    {
      LocalizedLaserScanPtr pCandidateScan = scans[rStartScanIndex];
      
      Pose2 candidateScanPose = pCandidateScan->GetReferencePose(use_scan_barycenter_);
      
      kt_double squaredDistance = candidateScanPose.GetPosition().SquaredDistance(pose.GetPosition());
      if (squaredDistance < math::Square(loop_search_max_distance_) + KT_TOLERANCE)
      {
        // a linked scan cannot be in the chain
        if (nearLinkedScans.Contains(pCandidateScan) == true)
        {
          chain.clear();
        }
        else
        {
          chain.push_back(pCandidateScan);
        }
      }
      else
      {
        // return chain if it is long "enough"
        if (chain.size() >= loop_match_min_chain_size_) 
        {
          return chain;
        }
        else
        {
          chain.clear();
        }
      }
    }
    ROS_INFO("Possible loop closures: %d", chain.size());
    return chain;
  }
  
  void RelativeSlam::CorrectPoses()
  {
    // optimize scans!
      solver_.Compute();
     
      IdPoseVector vec = solver_.GetCorrections(); 
      for(int i=0; i < vec.size(); i++)
      {
        boost::mutex::scoped_lock(scan_manager_mutex_);
        LocalizedObject* pObject = scan_manager_->GetLocalizedObject(vec[i].first);
        LocalizedLaserScanPtr pScan = dynamic_cast<LocalizedLaserScan*>(pObject);
        
        if (pScan != NULL)
        {
          pScan->SetSensorPose(vec[i].second);
        }
        else
        {
          pObject->SetCorrectedPose(vec[i].second);
        }
      }
      
      solver_.Clear();
  } 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "relative_slam");

  RelativeSlam rs;

  ros::spin();

  return 0;
}
