#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/surface/mls.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <ros/ros.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <fstream>  //save data lligen added

int
main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PCLPointCloud2 cloud,cloud_filtered ;
  pcl::PCLPointCloud2Ptr cloudPtr(&cloud);
  pcl::io::loadPCDFile ("piont_white_0022.pcd", *cloudPtr);
  pcl::PointCloud<pcl::PointXYZ> cloudXYZ;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZPtr(&cloudXYZ);
//  // Perform the actual filtering
//  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//  sor.setInputCloud (cloudPtr);
//  sor.setLeafSize (0.01, 0.01, 0.01);
//  sor.filter (cloud_filtered);
//  // Convert to the templated PointCloud
//  pcl::fromPCLPointCloud2 (cloud_filtered, cloudXYZ);
  pcl::fromPCLPointCloud2 (cloud, cloudXYZ);

  pcl::PCLPointCloud2 cloud2 ;
  pcl::PCLPointCloud2Ptr cloudPtr2(&cloud2);
  pcl::io::loadPCDFile ("piont_white_0031.pcd", *cloudPtr2);
  pcl::PointCloud<pcl::PointXYZ> cloudXYZ2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ2Ptr(&cloudXYZ2);
  pcl::fromPCLPointCloud2 (cloud2, cloudXYZ2);

  // Executing the transformation
  Eigen::Matrix4f transform;
  transform <<0.570647,	0.074308,	0.817820,	0.054933,  //rotate -90 ,"+" anti-wiseclock
              0.091349,	0.983966,	-0.153145,	0.167663,
             -0.816093,	0.162100,	0.554713,	0.057849,
              0.000000,	0.000000,	0.000000,	1.000000;
  /*  METHOD #2: Using a Affine3f
    This method is easier and less error prone
  */
  //  Translation 0.105066 0.341642 0.163335
  //  Rotation 0.0692543 0.511385 0.00178486 0.856555

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
//  transform_2.translation() << 0.105066, 0.341642, 0.163335;
//  transform_2.rotate (Eigen::Quaternionf (0.856555, 0.0692543, 0.511385, 0.00178486 ));
  //transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  std::ifstream fin("/home/f2/catkin_ws_h/devel/lib/rgbdslam_2kinect/test.txt",std::ios::in);
//  char ch[255];
//  fin.getline(ch,255-1,0);
  fin >>transform_2.translation().x()>>transform_2.translation().y() >>transform_2.translation().z();
  float x,y,z,w;
  fin >>x>>y>>z>>w;
  transform_2.rotate (Eigen::Quaternionf(w,x,y,z));
  cout<<transform_2.matrix()<<endl;
  fin.close();


  pcl::PointCloud<pcl::PointXYZ> transformed;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (&transformed);
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (cloudXYZ2, transformed, transform_2);


//  // Create a KD-Tree
//  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//  // Output has the PointNormal type in order to store the normals calculated by MLS
//  pcl::PointCloud<pcl::PointNormal> mls_points;
//  pcl::PointCloud<pcl::PointNormal>::Ptr mlsPtr(&mls_points);
//  // Init object (second point type is for the normals, even if unused)
//  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
//  mls.setComputeNormals (true);
//  mls.setInputCloud (cloudXYZPtr);
//  mls.setPolynomialFit (true);
//  mls.setSearchMethod (tree);
//  mls.setSearchRadius (0.01);
//  // Reconstruct
//  mls.process (mls_points);

//  // estimate normals   !!!!!!not been filtered!!!!
//  pcl::PointCloud<pcl::Normal>::Ptr normalsPtr (new pcl::PointCloud<pcl::Normal>);
//  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
//  ne.setMaxDepthChangeFactor(0.02f);
//  ne.setNormalSmoothingSize(10.0f);
//  ne.setInputCloud(cloudXYZPtr);
//  ne.compute(*normalsPtr);

  //plane1 parameters
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);
//  seg.setInputCloud (cloudXYZPtr);
//  seg.segment (*inliers, coefficients);
//  std::cerr << "plane1 para is :" << coefficients <<std::endl;

  // Create the filtering object
  pcl::PointCloud<pcl::PointXYZ>  cloud_p, cloud_f;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_pPtr(&cloud_p);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  int i = 0, nr_points = (int) cloudXYZ.points.size ();
  // While 30% of the original cloud is still there
  while (cloudXYZ.points.size () > 0.5 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloudXYZPtr);
    seg.segment (*inliers, coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    // Extract the inliers
    extract.setInputCloud (cloudXYZPtr);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (cloud_p);
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (cloud_f);
    cloudXYZ.swap (cloud_f);
    i++;
  }
  std::cerr << "plane1 para is :" << coefficients <<std::endl;

  //plane2 parameters
  pcl::ModelCoefficients coefficients2;
  pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg2;
  // Optional
  seg2.setOptimizeCoefficients (true);
  // Mandatory
  seg2.setModelType (pcl::SACMODEL_PLANE);
  seg2.setMethodType (pcl::SAC_RANSAC);
  seg2.setMaxIterations (1000);
  seg2.setDistanceThreshold (0.01);
//  seg.setInputCloud (cloudXYZ2Ptr);
//  seg.segment (*inliers2, coefficients2);
//  std::cerr << "plane2 para is :" << coefficients2 <<std::endl;


  //2 Create the filtering object
  pcl::PointCloud<pcl::PointXYZ>  cloud_p2, cloud_f2;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_p2Ptr(&cloud_p2);
  pcl::ExtractIndices<pcl::PointXYZ> extract2;
  int j = 0, nr_points2 = (int) transformed.points.size ();
  // While 30% of the original cloud is still there
  while (transformed.points.size () > 0.5 * nr_points2)
  {
    // Segment the largest planar component from the remaining cloud
    seg2.setInputCloud (transformed_cloud);
    seg2.segment (*inliers2, coefficients2);
    if (inliers2->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    // Extract the inliers
    extract2.setInputCloud (transformed_cloud);
    extract2.setIndices (inliers2);
    extract2.setNegative (false);
    extract2.filter (cloud_p2);
    // Create the filtering object
    extract2.setNegative (true);
    extract2.filter (cloud_f2);
    transformed.swap (cloud_f2);
    j++;
  }
  std::cerr << "plane2 para is :" << coefficients2 <<std::endl;

  //angle of plane

  // visualize normals
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.addCoordinateSystem();
  viewer.setBackgroundColor (0.0, 0.0, 0.5);
  // Define R,G,B colors for the point cloud
// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloudXYZ_color_handler (cloudXYZPtr, 0, 255, 255);
// // We add the point cloud to the viewer and pass the color handler
// viewer.addPointCloud (cloudXYZPtr, cloudXYZ_color_handler, "cloud1");
// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloudXYZ2_color_handler (cloudXYZ2Ptr, 255, 0, 255);
// viewer.addPointCloud (cloudXYZ2Ptr, cloudXYZ2_color_handler, "cloud2");
// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_handler (transformed_cloud, 255, 0, 255);
// viewer.addPointCloud (transformed_cloud, transformed_handler, "transformed_cloud2");
 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloudXYZ_color_handler (cloud_pPtr, 0, 255, 255);
 viewer.addPointCloud (cloud_pPtr, cloudXYZ_color_handler, "cloud1");
 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_handler2 (cloud_p2Ptr, 100, 100, 0);
 viewer.addPointCloud (cloud_p2Ptr, transformed_handler2, "transformed_cloud22");

//  viewer.addPointCloudNormals<pcl::PointXYZ,pcl::PointNormal>(cloudXYZPtr, mlsPtr);
// viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloudXYZPtr, normalsPtr);


  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
  return 0;
  // Save output
 // pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);
}
