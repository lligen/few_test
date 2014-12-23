#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

int
main ()
{
     // load point cloud
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::io::loadPCDFile ("piont_white_0022.pcd", *cloud);
     // Load input file into a PointCloud<T> with an appropriate type
     pcl::PCLPointCloud2 cloud22;
     pcl::PCLPointCloud2Ptr cloudPtr(&cloud22);
     pcl::io::loadPCDFile ("piont_white_0022.pcd", *cloudPtr);
     pcl::PointCloud<pcl::PointXYZ> cloudXYZ;
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZPtr(&cloudXYZ);
     pcl::fromPCLPointCloud2 (cloud22, cloudXYZ);

     // estimate normals
     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
     pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
     ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
     ne.setMaxDepthChangeFactor(0.02f);
     ne.setNormalSmoothingSize(10.0f);
     ne.setInputCloud(cloudXYZPtr);
     ne.compute(*normals);

//             for (size_t i = 0; i < normals->points.size (); ++i)
//             {
//               std::cout << normals->points[i] << std::endl;
//               EXPECT_NEAR (normals->points[i].normal[1], -0.369596, 1e-4);
//               EXPECT_NEAR (normals->points[i].normal[2], -0.928511, 1e-4);
//               EXPECT_NEAR (normals->points[i].curvature, 0.0693136, 1e-4);
//             }

     // visualize normals
     pcl::visualization::PCLVisualizer viewer("PCL Viewer");
     viewer.addCoordinateSystem();
     viewer.setBackgroundColor (0.0, 0.0, 0.5);
//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 0, 255, 255);
//     viewer.addPointCloud (cloud, cloud_color_handler, "cloud2");
     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud22_color_handler (cloudXYZPtr, 255, 0, 255);
     viewer.addPointCloud (cloudXYZPtr, cloud22_color_handler, "cloud22");
     //viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
     viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloudXYZPtr, normals);
     while (!viewer.wasStopped ())
     {
       viewer.spinOnce ();
     }
     return 0;
}
