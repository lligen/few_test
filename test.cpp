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
#include <math.h>
#include <limits.h>

#include <fstream>  //save data lligen added
using namespace std;

#define  PI      3.1415
#define  max(a, b)  (((a) > (b)) ? (a) : (b))

void getAngDist(const pcl::ModelCoefficients::Ptr &n1,const pcl::ModelCoefficients::Ptr &n2,double& ang,double& normDist){
    //Unit vector
    double edge1,edge2,dot_abs,ang2;
    for(int i =0;i<=2;i++){
        dot_abs += n1->values[i] * n2->values[i];
    }
    dot_abs = fabs(dot_abs);
    ang = 180/PI*acos(dot_abs);//(n1_norm*n2_norm)  1*1
    edge1 = 2* pow(max(n1->values[3],n2->values[3]),2) *(1 - cos(ang));
    edge1 = sqrt(edge1);
    ang2 = (180-ang)/2;
    edge2 = pow(fabs(n1->values[3] - n2->values[3]),2) + pow(edge1,2) - 2* fabs(n1->values[3] - n2->values[3]) *edge1;
    edge2 = sqrt(edge2);
    normDist = edge2 ;
}
void computeTransform(const pcl::ModelCoefficients::Ptr &n1,const pcl::ModelCoefficients::Ptr &n2,Eigen::Affine3f& trans){
    double alfa1,alfa2,beta1,beta2;
    double alfa,beta;
    double tran_x,tran_y,tran_z;
    Eigen::Affine3f tr = Eigen::Affine3f::Identity();
    alfa1 = atan(n1->values[1]/n1->values[0]);
    beta1 = atan(n1->values[2]/n1->values[0]);
    alfa2 = atan(n2->values[1]/n2->values[0]);
    beta2 = atan(n2->values[2]/n2->values[0]);
    alfa = alfa1 - alfa2;
    beta = beta1 - beta2;
    tran_x = (n2->values[3]-n1->values[3])/n1->values[3] *n1->values[0];
    tran_y = (n2->values[3]-n1->values[3])/n1->values[3] *n1->values[1];
    tran_z = (n2->values[3]-n1->values[3])/n1->values[3] *n1->values[2];
//    cout<<tran_x<<"  "<<tran_y<<"  "<<tran_z<<"  "<<endl;
    tr.translation() << -tran_x,-tran_y,-tran_z;
    tr.rotate(Eigen::AngleAxisf (alfa, Eigen::Vector3f::UnitZ()));
    tr.rotate(Eigen::AngleAxisf (-beta, Eigen::Vector3f::UnitY()));
    trans = tr;
}
//sub-sample filter
pcl::PointCloud<pcl::PointXYZ> VoxelGridFilter(pcl::PCLPointCloud2Ptr& cloudPtr){
    // Perform the actual filtering
    pcl::PCLPointCloud2 cloud_filtered ;
    pcl::PointCloud<pcl::PointXYZ> cloudXYZ;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.01, 0.01, 0.01);
    sor.filter (cloud_filtered);
    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2 (cloud_filtered, cloudXYZ);
    return cloudXYZ;
}
//IntegralImage estimate normals   !!!!!!not been filtered!!!!
pcl::PointCloud<pcl::Normal>::Ptr estNorm(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudXYZPtr){
    pcl::PointCloud<pcl::Normal>::Ptr normalsPtr (new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloudXYZPtr);
    ne.compute(*normalsPtr);
    return normalsPtr;
}
// Create a KD-Tree norm vector
pcl::PointCloud<pcl::PointNormal> KDTreeNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZPtr){
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::PointCloud<pcl::PointNormal>::Ptr mlsPtr(&mls_points);
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals (true);
    mls.setInputCloud (cloudXYZPtr);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.01);
    // Reconstruct
    mls.process (mls_points);
    return mls_points;
}
void getPlanePara(string& name,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudXYZPtr,pcl::PointIndices::Ptr& inliers,
                  pcl::ModelCoefficients coefficients){
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloudXYZPtr);
    seg.segment (*inliers, coefficients);
//    std::cout <<name<< "'s plane para is :" << coefficients <<std::endl;
}

int main (int argc, char** argv)
{
    Eigen::Matrix4d poses_ralative_to;
    Eigen::Isometry3d transform;
    poses_ralative_to<<  0.470592,   0.124928,   0.873462,   0.215916,       //loop1
                          -0.0457367,   0.992049,  -0.117248,   0.216757,
                           -0.881165,  0.0152267,   0.472565, -0.0442062,
                                   0,          0,          0,          1;
    transform = poses_ralative_to.cast<double>();
    std::cout<<"initial transform is:"<<std::endl<<transform.matrix()<<std::endl;
    transform = transform.inverse();
    std::cout<<"inverse transform is:"<<std::endl<<transform.matrix()<<std::endl;


  // Load input file into a PointCloud<T> with an appropriate type
  //cloud 1
  pcl::PCLPointCloud2 cloud,cloud_filtered ;
  pcl::PCLPointCloud2Ptr cloudPtr(&cloud);
  pcl::io::loadPCDFile ("cloud__1.pcd", *cloudPtr);
  pcl::PointCloud<pcl::PointXYZ> cloudXYZ;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZPtr(&cloudXYZ);
  pcl::fromPCLPointCloud2 (cloud, cloudXYZ);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
  pcl::ModelCoefficients::Ptr coefficientsPtr(new pcl::ModelCoefficients);

  // Create the filtering object
  pcl::PointCloud<pcl::PointXYZ>  cloud_p, cloud_f;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_pPtr(&cloud_p);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);
  int i = 0, nr_points = (int) cloudXYZ.points.size ();
  // While 30% of the original cloud is still there
  while (cloudXYZ.points.size () > 0.5 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloudXYZPtr);
    seg.segment (*inliers, *coefficientsPtr);
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
//    std::cout <<name<< "'s plane para is :" << coefficients <<std::endl;

  //cloud 2
  pcl::PCLPointCloud2 cloud2 ;
  pcl::PCLPointCloud2Ptr cloudPtr2(&cloud2);
  pcl::io::loadPCDFile ("cloud__2.pcd", *cloudPtr2);
  pcl::PointCloud<pcl::PointXYZ> cloudXYZ2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ2Ptr(&cloudXYZ2);
  pcl::fromPCLPointCloud2 (cloud2, cloudXYZ2);

  //get coefficients2 without transformed
  if(1){
      //plane2 parameters
    //  pcl::ModelCoefficients::Ptr coefficients2Ptr(new pcl::ModelCoefficients);
    //  pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices ());
    //  pcl::SACSegmentation<pcl::PointXYZ> seg2;
    //  seg2.setOptimizeCoefficients (true);
    //  seg2.setModelType (pcl::SACMODEL_PLANE);
    //  seg2.setMethodType (pcl::SAC_RANSAC);
    //  seg2.setMaxIterations (1000);
    //  seg2.setDistanceThreshold (0.01);
    //  pcl::PointCloud<pcl::PointXYZ>  cloud_p2, cloud_f2;
    //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p2Ptr(&cloud_p2);
    //  pcl::ExtractIndices<pcl::PointXYZ> extract2;
    //  int nr_points2 = (int) cloudXYZ2.points.size ();;
    //  while (cloudXYZ2.points.size () > 0.5 * nr_points2)
    //  {
    //      // Segment the largest planar component from the remaining cloud
    //      seg2.setInputCloud (cloudXYZ2Ptr);
    //      seg2.segment (*inliers2, *coefficients2Ptr);
    //      if (inliers2->indices.size () == 0)
    //      {
    //        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    //        break;
    //      }
    //      // Extract the inliers
    //      extract2.setInputCloud (cloudXYZ2Ptr);
    //      extract2.setIndices (inliers2);
    //      extract2.setNegative (false);
    //      extract2.filter (cloud_p2);
    //      // Create the filtering object
    //      extract2.setNegative (true);
    //      extract2.filter (cloud_f2);
    //      cloudXYZ2.swap (cloud_f2);
    //   }
  }

  //store the good result
  double theta,theta_m,distance,distance_m;
  theta_m = 90.0;distance_m = 100.0;
  pcl::ModelCoefficients coefficients_best;
  pcl::ModelCoefficients::Ptr coefficients_bestPtr(&coefficients_best);
  pcl::PointCloud<pcl::PointXYZ> cloudXYZ2_best;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ2_bestPtr(&cloudXYZ2_best);

  Eigen::Affine3f first_transform= Eigen::Affine3f::Identity();//
  pcl::PointCloud<pcl::PointXYZ> transformed;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloudPtr (&transformed);
  float tr_x,tr_y,tr_z;float x,y,z,w;int k = 0;

  //added transform initialize
  Eigen::Affine3f transform_result=Eigen::Affine3f::Identity();
  Eigen::Affine3f second_transform=Eigen::Affine3f::Identity();
  pcl::PointCloud<pcl::PointXYZ> transformed_twice;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_aPtr (&transformed_twice);

  std::ifstream ifile("/home/f2/catkin_ws_h/devel/lib/rgbdslam_2kinect/test.txt",std::ios::in);
  ifile >>tr_x>>tr_y>>tr_z;
  ifile >>x>>y>>z>>w;
  pcl::ModelCoefficients::Ptr coefficients2Ptr(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices ());
  pcl::SACSegmentation<pcl::PointXYZ> seg2;
  seg2.setOptimizeCoefficients (true);
  seg2.setModelType (pcl::SACMODEL_PLANE);
  seg2.setMethodType (pcl::SAC_RANSAC);
  seg2.setMaxIterations (1000);
  seg2.setDistanceThreshold (0.01);
  pcl::PointCloud<pcl::PointXYZ>  cloud_p2, cloud_f2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p2Ptr(&cloud_p2);
  pcl::ExtractIndices<pcl::PointXYZ> extract2;
  int nr_points2 = (int) cloudXYZ2.points.size ();
  while (!ifile.eof())
  {
      k++;
      first_transform.setIdentity();
      first_transform.translation()<<tr_x,tr_y,tr_z;
      first_transform.rotate (Eigen::Quaternionf(w,x,y,z));
      pcl::transformPointCloud (cloudXYZ2, transformed, first_transform);
      //get coefficients2 after first transform
      if(1){
          while (transformed.points.size () > 0.5 * nr_points2)
          {
              // Segment the largest planar component from the remaining cloud
              seg2.setInputCloud (transformed_cloudPtr);
              seg2.segment (*inliers2, *coefficients2Ptr);
              if (inliers2->indices.size () == 0)
              {
                std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
              }
              // Extract the inliers
              extract2.setInputCloud (transformed_cloudPtr);
              extract2.setIndices (inliers2);
              extract2.setNegative (false);
              extract2.filter (cloud_p2);
              // Create the filtering object
              extract2.setNegative (true);
              extract2.filter (cloud_f2);
              transformed.swap (cloud_f2);
           }
      }
//     std::cerr << "plane1 para is :" << *coefficientsPtr <<std::endl;
//     std::cerr << "plane2 para is :" << *coefficients2Ptr <<std::endl;

     getAngDist(coefficientsPtr,coefficients2Ptr,theta,distance);
     std::cerr<<"ang is: "<<theta<<"  dis is: "<<distance<<std::endl;
     //NEED TO FIXED
    if(distance <3){
         if(theta < theta_m){
            theta_m = theta;
            distance_m = distance;
            coefficients_best = *coefficients2Ptr;
            cloudXYZ2_best = *cloud_p2Ptr;
         }
     }

    //second transform compute
     second_transform.setIdentity();
     computeTransform(coefficientsPtr,coefficients2Ptr,second_transform);
     pcl::transformPointCloud (cloud_p2, transformed_twice, second_transform);
     transform_result.setIdentity();
     transform_result = first_transform *second_transform;
//     cout<<"first transform "<<first_transform.matrix()<<endl;
//     cout<<"second transform "<<second_transform.matrix()<<endl;
     cerr<<"result transform "<<transform_result.matrix()<<endl;


//     pcl::visualization::PCLVisualizer viewer3("PCL Viewer");
//     viewer3.addCoordinateSystem();
//     viewer3.setBackgroundColor (0.0, 0.0, 0);

//     //view first and second transformed poindcloud
//     if(1){
//         pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloudXYZ_color_handler1 (cloud_pPtr, 0, 0, 255);
//         viewer3.addPointCloud (cloud_pPtr, cloudXYZ_color_handler1, "cloud11");
//         pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_p2_handler1 (cloud_p2Ptr, 0, 255, 0);
//         viewer3.addPointCloud (cloud_p2Ptr, cloud_p2_handler1, "cloud_p2_cloud1");
//         pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_p2_handler (transformed_cloud_aPtr, 255, 0, 0);
//         viewer3.addPointCloud (transformed_cloud_aPtr, cloud_p2_handler, "cloud_p2_cloud");
//     }
//     while (!viewer3.wasStopped ())
//     {
//       viewer3.spinOnce ();
//     }

     ifile >>tr_x>>tr_y>>tr_z;
     ifile >>x>>y>>z>>w;
  }
  if(ifile.eof()){
      cout<<"end of file reached!!"<<endl;
  }
  cout<<k<<endl;
  std::cerr << "best candidate angle is : " << theta_m<<"  dis is: "<<distance_m<<std::endl;
  ifile.close();

    //second best transform compute
    Eigen::Affine3f transform_bs_result=Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_bssecond=Eigen::Affine3f::Identity();
    pcl::PointCloud<pcl::PointXYZ> transformed_bs;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_bsPtr (&transformed_bs);
    computeTransform(coefficientsPtr,coefficients_bestPtr,transform_bssecond);
    pcl::transformPointCloud (cloud_p2, transformed_bs, transform_bssecond);
    transform_bs_result = first_transform *transform_bssecond;
    cout<<"bs_result~~~~~~"<<transform_bs_result.matrix()<<endl;

  //  // visualize normals
  pcl::visualization::PCLVisualizer viewer2("PCL Viewer2");
  viewer2.addCoordinateSystem();
  viewer2.setBackgroundColor (0.0, 0.0, 0);
  //view initial pointcloud
  if(1){
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloudXYZ_color_handler2 (cloudXYZPtr, 0, 0, 255);
//  viewer2.addPointCloud (cloudXYZPtr, cloudXYZ_color_handler2, "cloud12");
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_handler2 (cloudXYZ2Ptr, 0, 255, 0);
//  viewer2.addPointCloud (cloudXYZ2Ptr, transformed_handler2, "transformed_cloud2");
  }

  //view filtered pointcloud--plane
  if(1){
//      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloudXYZ_color_handler2 (cloud_pPtr, 0, 0, 255);
//      viewer2.addPointCloud (cloud_pPtr, cloudXYZ_color_handler2, "cloud12");
//      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_handler2 (cloud_p2Ptr, 0, 255, 0);
//      viewer2.addPointCloud (cloud_p2Ptr, transformed_handler2, "transformed_cloud2");
  }

  //view best first and second pointcloud--plane
  if(1){
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloudXYZ_color_handler2 (cloud_pPtr, 0, 0, 255);
      viewer2.addPointCloud (cloud_pPtr, cloudXYZ_color_handler2, "cloud12");
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_handler2 (cloudXYZ2_bestPtr, 0, 255, 0);
      viewer2.addPointCloud (cloudXYZ2_bestPtr, transformed_handler2, "transformed_cloud2");
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_handler (transformed_cloud_bsPtr, 255, 0, 0);
      viewer2.addPointCloud (transformed_cloud_bsPtr, transformed_handler, "transformed_cloud22");
      ////  viewer.addPointCloudNormals<pcl::PointXYZ,pcl::PointNormal>(cloudXYZPtr, mlsPtr);
      //// viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloudXYZPtr, normalsPtr);
  }
  while (!viewer2.wasStopped ())
  {
    viewer2.spinOnce ();
  }



  return 0;
  // Save output
 // pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);
}
