#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_src;
  pcl::io::loadPCDFile ("cloud__1.pcd", cloud_src);
  pcl::fromPCLPointCloud2 (cloud_src, *cloud);

//  // Fill in the cloud data
//  cloud->width  = 5;
//  cloud->height = 1;
//  cloud->points.resize (cloud->width * cloud->height);

//  for (size_t i = 0; i < cloud->points.size (); ++i)
//  {
//    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
//    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
//    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
//  }

//  std::cerr << "Cloud before filtering: " << std::endl;
//  for (size_t i = 0; i < cloud->points.size (); ++i)
//    std::cerr << "    " << cloud->points[i].x << " "
//                        << cloud->points[i].y << " "
//                        << cloud->points[i].z << std::endl;

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  pcl::visualization::PCLVisualizer viewer2("PCL Viewer2");
  viewer2.addCoordinateSystem();
  viewer2.setBackgroundColor (0.0, 0.0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloudXYZ_color_handler2 (cloud, 0, 0, 255);
  viewer2.addPointCloud (cloud, cloudXYZ_color_handler2, "cloud12");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_handler2 (cloud_filtered, 0, 255, 0);
  viewer2.addPointCloud (cloud_filtered, transformed_handler2, "transformed_cloud2");
  while (!viewer2.wasStopped ())
  {
    viewer2.spinOnce ();
  }


//  std::cerr << "Cloud after filtering: " << std::endl;
//  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
//    std::cerr << "    " << cloud_filtered->points[i].x << " "
//                        << cloud_filtered->points[i].y << " "
//                        << cloud_filtered->points[i].z << std::endl;
//  pcl::io::savePCDFile ("cloud__1_filtered.pcd", *cloud_filtered);



  return (0);
}
