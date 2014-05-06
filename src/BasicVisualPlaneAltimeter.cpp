#include <visual_altimeter/BasicVisualPlaneAltimeter.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

//#include <stdio.h>

#include <sensor_msgs/PointCloud.h>
//#include <cmath>

BasicVisualPlaneAltimeter::BasicVisualPlaneAltimeter(): VisualAltimeter(true)
{

}

void BasicVisualPlaneAltimeter::setupResources()
{
    header_.frame_id = "mikrokopter";
    header_.seq = 0;

    plane_points_pub_ = nh_.advertise<sensor_msgs::PointCloud>("altimeter/plane_points", 10);
    plane_points_transformed_pub_ = nh_.advertise<sensor_msgs::PointCloud>("altimeter/plane_points_transformed", 10);

    lot_ = Eigen::ParametrizedLine<double, 3>::Through(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));
}

void BasicVisualPlaneAltimeter::calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
{
    return;
}

void BasicVisualPlaneAltimeter::calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImuConstPtr& imuMsg)
{
    int c_x = depth_image.cols/2;
    int c_y = depth_image.rows/2;

    cv::Point2f c(camInfoMsg->K[2], camInfoMsg->K[5]);
    cv::Point2f f(camInfoMsg->K[0], camInfoMsg->K[4]);

    float depth_value[] = {depth_image.at<float>(80, 80), depth_image.at<float>(80, 560), depth_image.at<float>(400, 320)};
    pcl::PointXYZ points[3] = {	pcl::PointXYZ(((float)80-c.x)*depth_value[0]/f.x, ((float)80-c.y)*depth_value[0]/f.y, depth_value[0]),
                                pcl::PointXYZ(((float)80-c.x)*depth_value[1]/f.x, ((float)560-c.y)*depth_value[1]/f.y, depth_value[1]),
                                pcl::PointXYZ(((float)400-c.x)*depth_value[2]/f.x, ((float)320-c.y)*depth_value[2]/f.y, depth_value[2])   };

    int suitable_points_found = 3;
    if(suitable_points_found == 3) {
        //ROS_INFO("(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)", points[0].x, points[0].y, points[0].z, points[1].x, points[1].y, points[1].z, points[2].x, points[2].y, points[2].z);

        pcl::PointCloud<pcl::PointXYZ> plane_points;
        for(int i = 0; i < 3; i++) plane_points.push_back(points[i]);

        Eigen::Quaternion<double> quat(imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z, imuMsg->orientation.w);
        Eigen::Transform<double, 3, Eigen::Affine> quad_rot_tranform(quat.toRotationMatrix());

        // Use Eigen only (no PCL) solution.
        pcl::PointCloud<pcl::PointXYZ> plane_points_transformed;
        pcl::transformPointCloud(plane_points, plane_points_transformed, quad_rot_tranform);

        sensor_msgs::PointCloud plane_points_msg;
        plane_points_msg.header = header_;
        geometry_msgs::Point32 p32;
        p32.x = points[0].x;
        p32.y = points[0].y;
        p32.z = points[0].z;
        plane_points_msg.points.push_back(p32);
        p32.x = points[1].x;
        p32.y = points[1].y;
        p32.z = points[1].z;
        plane_points_msg.points.push_back(p32);
        p32.x = points[2].x;
        p32.y = points[2].y;
        p32.z = points[2].z;
        plane_points_msg.points.push_back(p32);

        plane_points_pub_.publish(plane_points_msg);

        sensor_msgs::PointCloud plane_points_transformed_msg;
        plane_points_transformed_msg.header = header_;
        p32.x = plane_points_transformed[0].x;
        p32.y = plane_points_transformed[0].y;
        p32.z = plane_points_transformed[0].z;
        plane_points_transformed_msg.points.push_back(p32);
        p32.x = plane_points_transformed[1].x;
        p32.y = plane_points_transformed[1].y;
        p32.z = plane_points_transformed[1].z;
        plane_points_transformed_msg.points.push_back(p32);
        p32.x = plane_points_transformed[2].x;
        p32.y = plane_points_transformed[2].y;
        p32.z = plane_points_transformed[2].z;
        plane_points_transformed_msg.points.push_back(p32);

        plane_points_transformed_pub_.publish(plane_points_transformed_msg);

        header_.seq++;

        Eigen::Hyperplane<double, 3> plane = Eigen::Hyperplane<double, 3>::Through( Eigen::Vector3d(plane_points_transformed[0].x, plane_points_transformed[0].y, plane_points_transformed[0].z),
                                                                                    Eigen::Vector3d(plane_points_transformed[1].x, plane_points_transformed[1].y, plane_points_transformed[1].z),
                                                                                    Eigen::Vector3d(plane_points_transformed[2].x, plane_points_transformed[2].y, plane_points_transformed[2].z));

        Eigen::Hyperplane<double, 3> plane2 = Eigen::Hyperplane<double, 3>::Through( Eigen::Vector3d(points[0].x, points[0].y, points[0].z),
                                                                                    Eigen::Vector3d(points[1].x, points[1].y, points[1].z),
                                                                                    Eigen::Vector3d(points[2].x, points[2].y, points[2].z));
        double height = lot_.intersection(plane);
        double height2 = lot_.intersection(plane2);
        ROS_INFO("Height: %f, Rotated: %f", height2, height);
    }
    //ROS_INFO("DEPTH: %f, %f, %f", p1_depth, p2_depth, p3_depth);
}
