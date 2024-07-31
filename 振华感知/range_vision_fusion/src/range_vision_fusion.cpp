/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * range_vision_fusion_node.cpp
 *
 *  Created on: July, 05th, 2018
 */

#include "range_vision_fusion/range_vision_fusion.h"

cv::Point3f
ROSRangeVisionFusionApp::TransformPoint(const geometry_msgs::Point &in_point, const tf::StampedTransform &in_transform)
{
  tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
  tf::Vector3 tf_point_t = in_transform * tf_point;
  return cv::Point3f(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
}

cv::Point2i
ROSRangeVisionFusionApp::ProjectPoint(const cv::Point3f &in_point)
{
  auto u = int(in_point.x * fx_ / in_point.z + cx_);
  auto v = int(in_point.y * fy_ / in_point.z + cy_);

  return cv::Point2i(u, v);
}

tf::Vector3
ROSRangeVisionFusionApp::ProjectPoint(const cv::Point2i &in_point)
{
  if(plane_coeff_ok_)
  {
    // 精确解, 但是抖动严重, 暂不使用
    // auto u = in_point.x;
    // auto v = in_point.y;
    // float a = plane_coeff[0] * (u - cx_) / fx_;
    // float b = plane_coeff[1] * (v - cy_) / fy_;
    // float c = plane_coeff[2];
    // float d = plane_coeff[3];

    // float z = -d / (a + b + c);
    // float y = b * z / plane_coeff[1];
    // float x = a * z / plane_coeff[0];
    auto u = in_point.x;
    auto v = in_point.y;
    float y = -plane_coeff[3] / plane_coeff[1];    // 距离地面的y作为y值, 粗糙解, 认为地面基本和相机平行
    float z = y * fy_ / (v - cy_);
    float x = (u - cx_) * z / fx_;

    tf::StampedTransform baselink2camera;

    try
    {
      transform_listener_->lookupTransform("base_link", image_frame_id_, ros::Time(0), baselink2camera);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
    }

    tf::Vector3 point(x, y, z);   
    tf::Matrix3x3 rotation_matrix(baselink2camera.getRotation());         
    tf::Vector3 origin(baselink2camera.getOrigin()); //
    point = rotation_matrix * point + origin;  

    return point;
  }
  return tf::Vector3(0, 0, 0);
}

// 没用的
autoware_msgs::DetectedObject
ROSRangeVisionFusionApp::TransformObject(const autoware_msgs::DetectedObject &in_detection,
                                         const tf::StampedTransform &in_transform)
{
  autoware_msgs::DetectedObject t_obj = in_detection;

  tf::Vector3 in_pos(in_detection.pose.position.x,
                     in_detection.pose.position.y,
                     in_detection.pose.position.z);
  tf::Quaternion in_quat(in_detection.pose.orientation.x,
                         in_detection.pose.orientation.y,
                         in_detection.pose.orientation.w,
                         in_detection.pose.orientation.z);

  tf::Vector3 in_pos_t = in_transform * in_pos;
  tf::Quaternion in_quat_t = in_transform * in_quat;

  t_obj.pose.position.x = in_pos_t.x();
  t_obj.pose.position.y = in_pos_t.y();
  t_obj.pose.position.z = in_pos_t.z();

  t_obj.pose.orientation.x = in_quat_t.x();
  t_obj.pose.orientation.y = in_quat_t.y();
  t_obj.pose.orientation.z = in_quat_t.z();
  t_obj.pose.orientation.w = in_quat_t.w();

  return t_obj;
}

// bool
// ROSRangeVisionFusionApp::IsObjectInImage(const autoware_msgs::DetectedObject &in_detection)
// {
//   cv::Point3f image_space_point = TransformPoint(in_detection.pose.position, camera_lidar_tf_);
//   cv::Point2i image_pixel = ProjectPoint(image_space_point);

//   return (image_pixel.x >= 0)
//          && (image_pixel.x < image_size_.width)
//          && (image_pixel.y >= 0)
//          && (image_pixel.y < image_size_.height)
//          && (image_space_point.z > 0);
// }

bool
ROSRangeVisionFusionApp::IsObjectInImage(const autoware_msgs::CloudCluster &in_detection)
{
  cv::Point3f image_space_point = TransformPoint(in_detection.bounding_box.pose.position, camera_lidar_tf_);
  cv::Point2i image_pixel = ProjectPoint(image_space_point);

  return (image_pixel.x >= 0)
         && (image_pixel.x < image_size_.width)
         && (image_pixel.y >= 0)
         && (image_pixel.y < image_size_.height)
         && (image_space_point.z > 0);
}

// cv::Rect ROSRangeVisionFusionApp::ProjectDetectionToRect(const autoware_msgs::DetectedObject &in_detection)
// {
//   cv::Rect projected_box;

//   Eigen::Vector3f pos;
//   pos << in_detection.pose.position.x,
//     in_detection.pose.position.y,
//     in_detection.pose.position.z;

//   Eigen::Quaternionf rot(in_detection.pose.orientation.w,
//                          in_detection.pose.orientation.x,
//                          in_detection.pose.orientation.y,
//                          in_detection.pose.orientation.z);

//   std::vector<double> dims = {
//     in_detection.dimensions.x,
//     in_detection.dimensions.y,
//     in_detection.dimensions.z
//   };

//   jsk_recognition_utils::Cube cube(pos, rot, dims);

//   Eigen::Affine3f range_vision_tf;
//   tf::transformTFToEigen(camera_lidar_tf_, range_vision_tf);
//   jsk_recognition_utils::Vertices vertices = cube.transformVertices(range_vision_tf);

//   std::vector<cv::Point> polygon;
//   for (auto &vertex : vertices)
//   {
//     cv::Point p = ProjectPoint(cv::Point3f(vertex.x(), vertex.y(), vertex.z()));
//     polygon.push_back(p);
//   }

//   projected_box = cv::boundingRect(polygon);

//   return projected_box;
// }

cv::Rect ROSRangeVisionFusionApp::ProjectDetectionToRect(const autoware_msgs::CloudCluster &in_detection)
{
  cv::Rect projected_box;

  Eigen::Vector3f pos;
  pos << in_detection.bounding_box.pose.position.x,
    in_detection.bounding_box.pose.position.y,
    in_detection.bounding_box.pose.position.z;

  Eigen::Quaternionf rot(in_detection.bounding_box.pose.orientation.w,
                         in_detection.bounding_box.pose.orientation.x,
                         in_detection.bounding_box.pose.orientation.y,
                         in_detection.bounding_box.pose.orientation.z);

  std::vector<double> dims = {
    in_detection.dimensions.x,
    in_detection.dimensions.y,
    in_detection.dimensions.z
  };

  jsk_recognition_utils::Cube cube(pos, rot, dims);

  Eigen::Affine3f range_vision_tf;
  tf::transformTFToEigen(camera_lidar_tf_, range_vision_tf);
  jsk_recognition_utils::Vertices vertices = cube.transformVertices(range_vision_tf);

  std::vector<cv::Point> polygon;
  for (auto &vertex : vertices)
  {
    cv::Point p = ProjectPoint(cv::Point3f(vertex.x(), vertex.y(), vertex.z()));
    polygon.push_back(p);
  }

  projected_box = cv::boundingRect(polygon);

  return projected_box;
}

// void
// ROSRangeVisionFusionApp::TransformRangeToVision(const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections,
//                                                 autoware_msgs::DetectedObjectArray &out_in_cv_range_detections,
//                                                 autoware_msgs::DetectedObjectArray &out_out_cv_range_detections)
// {
//   out_in_cv_range_detections.header = in_range_detections->header;
//   out_in_cv_range_detections.header.frame_id = in_range_detections->header.frame_id;
//   out_in_cv_range_detections.objects.clear();
//   out_out_cv_range_detections.header = in_range_detections->header;
//   out_out_cv_range_detections.header.frame_id = in_range_detections->header.frame_id;
//   out_out_cv_range_detections.objects.clear();
//   for (size_t i = 0; i < in_range_detections->objects.size(); i++)
//   {
//     if (IsObjectInImage(in_range_detections->objects[i]))
//     {
//       out_in_cv_range_detections.objects.push_back(in_range_detections->objects[i]);
//     } else
//     {
//       out_out_cv_range_detections.objects.push_back(in_range_detections->objects[i]);
//     }
//   }
// }

void
ROSRangeVisionFusionApp::TransformRangeToVision(const autoware_msgs::CloudClusterArray::ConstPtr &in_range_detections,
                                                autoware_msgs::CloudClusterArray &out_in_cv_range_detections,
                                                autoware_msgs::CloudClusterArray &out_out_cv_range_detections)
{
  out_in_cv_range_detections.header = in_range_detections->header;
  out_in_cv_range_detections.header.frame_id = in_range_detections->header.frame_id;
  out_in_cv_range_detections.clusters.clear();
  out_out_cv_range_detections.header = in_range_detections->header;
  out_out_cv_range_detections.header.frame_id = in_range_detections->header.frame_id;
  out_out_cv_range_detections.clusters.clear();
  for (size_t i = 0; i < in_range_detections->clusters.size(); i++)
  {
    if (IsObjectInImage(in_range_detections->clusters[i]))
    {
      out_in_cv_range_detections.clusters.push_back(in_range_detections->clusters[i]);
    } else
    {
      out_out_cv_range_detections.clusters.push_back(in_range_detections->clusters[i]);
    }
  }
}


// 没用到
void
ROSRangeVisionFusionApp::CalculateObjectFeatures(autoware_msgs::DetectedObject &in_out_object, bool in_estimate_pose)
{

  float min_x = std::numeric_limits<float>::max();
  float max_x = -std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_y = -std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_z = -std::numeric_limits<float>::max();
  float average_x = 0, average_y = 0, average_z = 0, length, width, height;
  pcl::PointXYZ centroid, min_point, max_point, average_point;

  std::vector<cv::Point2f> object_2d_points;

  pcl::PointCloud<pcl::PointXYZ> in_cloud;
  pcl::fromROSMsg(in_out_object.pointcloud, in_cloud);

  for (const auto &point : in_cloud.points)
  {
    average_x += point.x;
    average_y += point.y;
    average_z += point.z;
    centroid.x += point.x;
    centroid.y += point.y;
    centroid.z += point.z;

    if (point.x < min_x)
      min_x = point.x;
    if (point.y < min_y)
      min_y = point.y;
    if (point.z < min_z)
      min_z = point.z;
    if (point.x > max_x)
      max_x = point.x;
    if (point.y > max_y)
      max_y = point.y;
    if (point.z > max_z)
      max_z = point.z;

    cv::Point2f pt;
    pt.x = point.x;
    pt.y = point.y;
    object_2d_points.push_back(pt);
  }
  min_point.x = min_x;
  min_point.y = min_y;
  min_point.z = min_z;
  max_point.x = max_x;
  max_point.y = max_y;
  max_point.z = max_z;

  if (in_cloud.points.size() > 0)
  {
    centroid.x /= in_cloud.points.size();
    centroid.y /= in_cloud.points.size();
    centroid.z /= in_cloud.points.size();

    average_x /= in_cloud.points.size();
    average_y /= in_cloud.points.size();
    average_z /= in_cloud.points.size();
  }

  average_point.x = average_x;
  average_point.y = average_y;
  average_point.z = average_z;

  length = max_point.x - min_point.x;
  width = max_point.y - min_point.y;
  height = max_point.z - min_point.z;

  geometry_msgs::PolygonStamped convex_hull;
  std::vector<cv::Point2f> hull_points;
  if (object_2d_points.size() > 0)
    cv::convexHull(object_2d_points, hull_points);

  convex_hull.header = in_out_object.header;
  for (size_t i = 0; i < hull_points.size() + 1; i++)
  {
    geometry_msgs::Point32 point;
    point.x = hull_points[i % hull_points.size()].x;
    point.y = hull_points[i % hull_points.size()].y;
    point.z = min_point.z;
    convex_hull.polygon.points.push_back(point);
  }

  for (size_t i = 0; i < hull_points.size() + 1; i++)
  {
    geometry_msgs::Point32 point;
    point.x = hull_points[i % hull_points.size()].x;
    point.y = hull_points[i % hull_points.size()].y;
    point.z = max_point.z;
    convex_hull.polygon.points.push_back(point);
  }

  double rz = 0;
  if (in_estimate_pose)
  {
    cv::RotatedRect box = cv::minAreaRect(hull_points);
    rz = box.angle * 3.14 / 180;
    in_out_object.pose.position.x = box.center.x;
    in_out_object.pose.position.y = box.center.y;
    in_out_object.dimensions.x = box.size.width;
    in_out_object.dimensions.y = box.size.height;
  }

  in_out_object.convex_hull = convex_hull;

  in_out_object.pose.position.x = min_point.x + length / 2;
  in_out_object.pose.position.y = min_point.y + width / 2;
  in_out_object.pose.position.z = min_point.z + height / 2;

  in_out_object.dimensions.x = ((length < 0) ? -1 * length : length);
  in_out_object.dimensions.y = ((width < 0) ? -1 * width : width);
  in_out_object.dimensions.z = ((height < 0) ? -1 * height : height);

  tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, rz);
  tf::quaternionTFToMsg(quat, in_out_object.pose.orientation);
}

// 没用到
autoware_msgs::DetectedObject ROSRangeVisionFusionApp::MergeObjects(const autoware_msgs::DetectedObject &in_object_a,
                                                                    const autoware_msgs::DetectedObject &in_object_b)
{
  autoware_msgs::DetectedObject object_merged;
  object_merged = in_object_b;

  pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_merged;

  if (!in_object_a.pointcloud.data.empty())
    pcl::fromROSMsg(in_object_a.pointcloud, cloud_a);
  if (!in_object_b.pointcloud.data.empty())
    pcl::fromROSMsg(in_object_b.pointcloud, cloud_b);

  cloud_merged = cloud_a + cloud_b;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_merged, cloud_msg);
  cloud_msg.header = object_merged.pointcloud.header;

  object_merged.pointcloud = cloud_msg;

  return object_merged;

}

// double ROSRangeVisionFusionApp::GetDistanceToObject(const autoware_msgs::DetectedObject &in_object)
// {
//   return sqrt(in_object.dimensions.x * in_object.dimensions.x +
//               in_object.dimensions.y * in_object.dimensions.y +
//               in_object.dimensions.z * in_object.dimensions.z);
// }

double ROSRangeVisionFusionApp::GetDistanceToObject(const autoware_msgs::CloudCluster &in_object)
{
  return sqrt(in_object.dimensions.x * in_object.dimensions.x +
              in_object.dimensions.y * in_object.dimensions.y +
              in_object.dimensions.z * in_object.dimensions.z);
}

// void ROSRangeVisionFusionApp::CheckMinimumDimensions(autoware_msgs::DetectedObject &in_out_object)
// {
//   if (in_out_object.label == "car")
//   {
//     if (in_out_object.dimensions.x < car_depth_)
//       in_out_object.dimensions.x = car_depth_;
//     if (in_out_object.dimensions.y < car_width_)
//       in_out_object.dimensions.y = car_width_;
//     if (in_out_object.dimensions.z < car_height_)
//       in_out_object.dimensions.z = car_height_;
//   }
//   if (in_out_object.label == "person")
//   {
//     if (in_out_object.dimensions.x < person_depth_)
//       in_out_object.dimensions.x = person_depth_;
//     if (in_out_object.dimensions.y < person_width_)
//       in_out_object.dimensions.y = person_width_;
//     if (in_out_object.dimensions.z < person_height_)
//       in_out_object.dimensions.z = person_height_;
//   }

//   if (in_out_object.label == "truck" || in_out_object.label == "bus")
//   {
//     if (in_out_object.dimensions.x < truck_depth_)
//       in_out_object.dimensions.x = truck_depth_;
//     if (in_out_object.dimensions.y < truck_width_)
//       in_out_object.dimensions.y = truck_width_;
//     if (in_out_object.dimensions.z < truck_height_)
//       in_out_object.dimensions.z = truck_height_;
//   }
// }

void ROSRangeVisionFusionApp::CheckMinimumDimensions(autoware_msgs::CloudCluster &in_out_object)
{
  // ljc add: 由于雷达线数较少, 因此聚类框可能不够大, 导致躲避失败, 在此除了修改dimensions外, 还需要修改convex_hull
  if (in_out_object.label == "car")
  {
    if (in_out_object.dimensions.x < car_depth_)
      in_out_object.dimensions.x = car_depth_;
    if (in_out_object.dimensions.y < car_width_)
      in_out_object.dimensions.y = car_width_;
    if (in_out_object.dimensions.z < car_height_)
      in_out_object.dimensions.z = car_height_;
    
    /*
    // TODO: 根据车辆中心, 和长宽高, 重新为它设置convex_hull, 仅设置4个点应该就行
    geometry_msgs::Polygon new_polygon;
    geometry_msgs::Point32 edge_point;

    edge_point.x = in_out_object.centroid_point.point.x - in_out_object.dimensions.x / 2;
    edge_point.y = in_out_object.centroid_point.point.y - in_out_object.dimensions.y / 2;
    edge_point.z = in_out_object.centroid_point.point.z;
    new_polygon.points.push_back(edge_point);

    edge_point.x = in_out_object.centroid_point.point.x - in_out_object.dimensions.x / 2;
    edge_point.y = in_out_object.centroid_point.point.y + in_out_object.dimensions.y / 2;
    edge_point.z = in_out_object.centroid_point.point.z;
    new_polygon.points.push_back(edge_point);

    edge_point.x = in_out_object.centroid_point.point.x + in_out_object.dimensions.x / 2;
    edge_point.y = in_out_object.centroid_point.point.y + in_out_object.dimensions.y / 2;
    edge_point.z = in_out_object.centroid_point.point.z;
    new_polygon.points.push_back(edge_point);

    edge_point.x = in_out_object.centroid_point.point.x + in_out_object.dimensions.x / 2;
    edge_point.y = in_out_object.centroid_point.point.y - in_out_object.dimensions.y / 2;
    edge_point.z = in_out_object.centroid_point.point.z;
    new_polygon.points.push_back(edge_point);

    in_out_object.convex_hull.polygon = new_polygon;
    */
    
  }
  if (in_out_object.label == "person")
  {
    if (in_out_object.dimensions.x < person_depth_)
      in_out_object.dimensions.x = person_depth_;
    if (in_out_object.dimensions.y < person_width_)
      in_out_object.dimensions.y = person_width_;
    if (in_out_object.dimensions.z < person_height_)
      in_out_object.dimensions.z = person_height_;
  }

  if (in_out_object.label == "truck" || in_out_object.label == "bus")
  {
    if (in_out_object.dimensions.x < truck_depth_)
      in_out_object.dimensions.x = truck_depth_;
    if (in_out_object.dimensions.y < truck_width_)
      in_out_object.dimensions.y = truck_width_;
    if (in_out_object.dimensions.z < truck_height_)
      in_out_object.dimensions.z = truck_height_;
  }
}

/*
autoware_msgs::DetectedObjectArray
ROSRangeVisionFusionApp::FuseRangeVisionDetections(
  const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections,
  const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections)
{

  autoware_msgs::DetectedObjectArray range_in_cv;
  autoware_msgs::DetectedObjectArray range_out_cv;
  TransformRangeToVision(in_range_detections, range_in_cv, range_out_cv);   // 这一步会去对欧式聚类检测到的点云簇做遍历, 查看每个簇是否在图像内, 若在则放入range_in_cv, 反之放入range_out_cv

  autoware_msgs::DetectedObjectArray fused_objects;
  fused_objects.header = in_range_detections->header;
  fused_objects.header.frame_id = in_range_detections->header.frame_id;   // ljc add

  std::vector<std::vector<size_t> > vision_range_assignments(in_vision_detections->objects.size());
  std::vector<bool> used_vision_detections(in_vision_detections->objects.size(), false);
  std::vector<long> vision_range_closest(in_vision_detections->objects.size());

  // o(n^2)的搜索, 对每个图像中的物体进行遍历, 去匹配点云中的物体
  for (size_t i = 0; i < in_vision_detections->objects.size(); i++)
  {
    auto vision_object = in_vision_detections->objects[i];

    cv::Rect vision_rect(vision_object.x, vision_object.y,
                         vision_object.width, vision_object.height);
    int vision_rect_area = vision_rect.area();
    long closest_index = -1;
    double closest_distance = std::numeric_limits<double>::max();

    for (size_t j = 0; j < range_in_cv.objects.size(); j++)
    {
      double current_distance = GetDistanceToObject(range_in_cv.objects[j]);   // 此处获取的是3D物体的尺寸, 而不是距离

      cv::Rect range_rect = ProjectDetectionToRect(range_in_cv.objects[j]);
      int range_rect_area = range_rect.area();

      cv::Rect overlap = range_rect & vision_rect;
      if ((overlap.area() > range_rect_area * overlap_threshold_)
          || (overlap.area() > vision_rect_area * overlap_threshold_)   // 计算3D点云投影框与2D图像框的IOU
        )
      {
        vision_range_assignments[i].push_back(j);   // vector<vector>,  第一维的个数为图像上bbox的个数, 第二维用于放入与这个bbox的IOU大于阈值的点云簇
        range_in_cv.objects[j].score = vision_object.score;
        range_in_cv.objects[j].label = vision_object.label;   // 为点云簇分配物体检测的标签
        // if (range_in_cv.objects[j].label == "bottle")
        // {
        //   range_in_cv.objects[j].label = "bottle";  // 联合检测到的垃圾, 统一标签为trash
        // }
        range_in_cv.objects[j].color = vision_object.color;
        range_in_cv.objects[j].image_frame = vision_object.image_frame;
        range_in_cv.objects[j].x = vision_object.x;
        range_in_cv.objects[j].y = vision_object.y;  
        range_in_cv.objects[j].width = vision_object.width;
        range_in_cv.objects[j].height = vision_object.height;
        range_in_cv.objects[j].angle = vision_object.angle;
        range_in_cv.objects[j].id = vision_object.id;
        CheckMinimumDimensions(range_in_cv.objects[j]);
        if (vision_object.pose.orientation.x > 0
            || vision_object.pose.orientation.y > 0
            || vision_object.pose.orientation.z > 0)
        {
          range_in_cv.objects[i].pose.orientation = vision_object.pose.orientation;
        }
        if (current_distance < closest_distance)
        {
          closest_index = j;
          closest_distance = current_distance;  // 更新距离它最近的物体, 映射在图像的同一处的物体, 由于透视变换, 其应该都在一条射线上, 故取最近的物体
        }
        used_vision_detections[i] = true;
      }//end if overlap
    }//end for range_in_cv
    vision_range_closest[i] = closest_index;    // 为第i个图像上的物体距离最近的点云簇的索引
  }

  std::vector<bool> used_range_detections(range_in_cv.objects.size(), false);
  //only assign the closest
  for (size_t i = 0; i < vision_range_assignments.size(); i++)
  {
    if (!range_in_cv.objects.empty() && vision_range_closest[i] >= 0)
    {
      used_range_detections[i] = true;
      fused_objects.objects.push_back(range_in_cv.objects[vision_range_closest[i]]);   // 如果这个bbox对应的3D点云簇存在, 则放入到fused_objects中, 顺序无关, 已经添加label了
    }
  }
  // 把没有3D点云簇对应的物体也放进来, 下一句把图像外的点云簇也放入, 因此总体objects是不少的, 只是匹配项做了标签处理
  double closest_dist = 1e6;
  geometry_msgs::Vector3 closest_point_3d;
  bool trash_valid = false;
  for (size_t i = 0; i < used_vision_detections.size(); i++)
  {
    if (!used_vision_detections[i])
    {
      // TODO: 对在地面上的物体(垃圾)进行位置估计

      // 目前以person的底框来添加尝试, 在launch文件中可以使用param来决定是否使用单目估计, 只有在单目true时才有效
      if(use_mono_estimate)
      {
        if (in_vision_detections->objects[i].label == "bottle" || in_vision_detections->objects[i].label == "car" || in_vision_detections->objects[i].label == "person")
        {
          cv::Point2i bottem_point_2d(int(in_vision_detections->objects[i].x + in_vision_detections->objects[i].width / 2), int(in_vision_detections->objects[i].y + in_vision_detections->objects[i].height));
          tf::Vector3 bottem_point_3d = ProjectPoint(bottem_point_2d);   // 转到base_link下 

          if (float(bottem_point_3d.x()) > 30 || float(bottem_point_3d.x()) < 0)  // 如果预测出来的深度过于远, 认为不准, 40m开外也不需要太关心, 忽略, 若深度估计为负, 则可能是由于过于接近图像中心导致, 前置相机不可能如此, 忽略
          {
            fused_objects.objects.push_back(in_vision_detections->objects[i]);    
            continue;
          }

          autoware_msgs::DetectedObject detected_object;
          geometry_msgs::Pose pose;
          geometry_msgs::Vector3 dimensions;
          pose.orientation.x = 0;
          pose.orientation.y = 0;
          pose.orientation.z = 0;
          pose.orientation.w = 1;
          pose.position.x = float(bottem_point_3d.x());
          pose.position.y = float(bottem_point_3d.y());
          pose.position.z = float(bottem_point_3d.z());
          

          if (closest_dist > pose.position.x * pose.position.x + pose.position.y * pose.position.y && in_vision_detections->objects[i].label == "bottle")
          {
            closest_dist = pose.position.x * pose.position.x + pose.position.y * pose.position.y;
            closest_point_3d.x = pose.position.x;
            closest_point_3d.y = pose.position.y;
            closest_point_3d.z = pose.position.z;
            trash_valid = true;
          }

          if (in_vision_detections->objects[i].label == "bottle")
          {
            dimensions.x = 1.0;
            dimensions.y = 1.0;
            dimensions.z = 1.0;
          }
          else if (in_vision_detections->objects[i].label == "car")
          {
            dimensions.x = 1.0;
            dimensions.y = 1.0;
            dimensions.z = 1.0;
          }
          else if (in_vision_detections->objects[i].label == "person")
          {
            dimensions.x = 1.0;
            dimensions.y = 1.0;
            dimensions.z = 1.0;
          }

          detected_object.header = in_vision_detections->objects[i].header;
          detected_object.label = in_vision_detections->objects[i].label;
          // detected_object.label = "bottle";    // 目前将检测到的垃圾统一标签为trash
          detected_object.score = in_vision_detections->objects[i].score;

          detected_object.valid = true;
          detected_object.dimensions = dimensions;
          detected_object.pose = pose;
          fused_objects.objects.push_back(detected_object);      // 如果是要追踪的垃圾, 在这边发送是为了可视化, 但是它不应该被当做imm_track中的障碍物, 所以需要修改逻辑
        }
      }
      else
      {
        fused_objects.objects.push_back(in_vision_detections->objects[i]);    
      }
      
    }
  }
  // 遍历完毕, 如果使用mono detect, 则将最近的垃圾位置发布到/inspection_target, 用于巡检
  if (use_mono_estimate && trash_valid)
  {
    publisher_trash_objects_.publish(closest_point_3d);
  }

  //add also objects outside the image
  for (auto &object: range_out_cv.objects)
  {
    fused_objects.objects.push_back(object);
  }
  //enable merged for visualization
  for (auto &object : fused_objects.objects)
  {
    object.valid = true;
  }

  return fused_objects;
}
*/

autoware_msgs::CloudClusterArray
ROSRangeVisionFusionApp::FuseRangeVisionDetections(
  const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections,
  const autoware_msgs::CloudClusterArray::ConstPtr &in_range_detections)
{

  autoware_msgs::CloudClusterArray range_in_cv;
  autoware_msgs::CloudClusterArray range_out_cv;
  TransformRangeToVision(in_range_detections, range_in_cv, range_out_cv);   // 这一步会去对欧式聚类检测到的点云簇做遍历, 查看每个簇是否在图像内, 若在则放入range_in_cv, 反之放入range_out_cv

  autoware_msgs::CloudClusterArray fused_objects;
  fused_objects.header = in_range_detections->header;
  fused_objects.header.frame_id = in_range_detections->header.frame_id;  

  std::vector<std::vector<size_t> > vision_range_assignments(in_vision_detections->objects.size());
  std::vector<bool> used_vision_detections(in_vision_detections->objects.size(), false);
  std::vector<long> vision_range_closest(in_vision_detections->objects.size());

  // o(n^2)的搜索, 对每个图像中的物体进行遍历, 去匹配点云中的物体
  for (size_t i = 0; i < in_vision_detections->objects.size(); i++)
  {
    auto vision_object = in_vision_detections->objects[i];

    cv::Rect vision_rect(vision_object.x, vision_object.y,
                         vision_object.width, vision_object.height);
    int vision_rect_area = vision_rect.area();
    long closest_index = -1;
    double closest_distance = std::numeric_limits<double>::max();

    for (size_t j = 0; j < range_in_cv.clusters.size(); j++)
    {
      double current_distance = GetDistanceToObject(range_in_cv.clusters[j]);   // 此处获取的是3D物体的尺寸, 而不是距离

      cv::Rect range_rect = ProjectDetectionToRect(range_in_cv.clusters[j]);
      int range_rect_area = range_rect.area();

      cv::Rect overlap = range_rect & vision_rect;
      if ((overlap.area() > range_rect_area * overlap_threshold_)
          || (overlap.area() > vision_rect_area * overlap_threshold_)   // 计算3D点云投影框与2D图像框的IOU
        )
      {
        vision_range_assignments[i].push_back(j);   // vector<vector>,  第一维的个数为图像上bbox的个数, 第二维用于放入与这个bbox的IOU大于阈值的点云簇
        range_in_cv.clusters[j].score = vision_object.score;
        range_in_cv.clusters[j].label = vision_object.label;   // 为点云簇分配物体检测的标签
        // if (range_in_cv.objects[j].label == "bottle")
        // {
        //   range_in_cv.objects[j].label = "bottle";  // 联合检测到的垃圾, 统一标签为trash
        // }
        range_in_cv.clusters[j].id = vision_object.id;
        CheckMinimumDimensions(range_in_cv.clusters[j]);
        if (vision_object.pose.orientation.x > 0
            || vision_object.pose.orientation.y > 0
            || vision_object.pose.orientation.z > 0)
        {
          range_in_cv.clusters[i].bounding_box.pose.orientation = vision_object.pose.orientation;
        }
        if (current_distance < closest_distance)
        {
          closest_index = j;
          closest_distance = current_distance;  // 更新距离它最近的物体, 映射在图像的同一处的物体, 由于透视变换, 其应该都在一条射线上, 故取最近的物体
        }
        used_vision_detections[i] = true;
      }//end if overlap
    }//end for range_in_cv
    vision_range_closest[i] = closest_index;    // 为第i个图像上的物体距离最近的点云簇的索引
  }

  std::vector<bool> used_range_detections(range_in_cv.clusters.size(), false);
  //only assign the closest
  for (size_t i = 0; i < vision_range_assignments.size(); i++)
  {
    if (!range_in_cv.clusters.empty() && vision_range_closest[i] >= 0)
    {
      used_range_detections[i] = true;
      fused_objects.clusters.push_back(range_in_cv.clusters[vision_range_closest[i]]);   // 如果这个bbox对应的3D点云簇存在, 则放入到fused_objects中, 顺序无关, 已经添加label了
    }
  }
  // 把没有3D点云簇对应的物体也放进来, 下一句把图像外的点云簇也放入, 因此总体objects是不少的, 只是匹配项做了标签处理
  double closest_dist = 1e6;
  geometry_msgs::Vector3 closest_point_3d;
  bool trash_valid = false;
  for (size_t i = 0; i < used_vision_detections.size(); i++)
  { 
    // 对没有点云和图像匹配上的图像检测出的类别进行计算
    if (!used_vision_detections[i])
    {
      // TODO: 对在地面上的物体(垃圾)进行位置估计

      // 目前以person的底框来添加尝试, 在launch文件中可以使用param来决定是否使用单目估计, 只有在单目true时才有效
      if(use_mono_estimate)
      { 
        // 情况一: 如果是bottle, 通过地平面估计三维坐标, 并将最近的距离存储到closest_point_3d中
        if (in_vision_detections->objects[i].label == "bottle")
        {
          cv::Point2i bottem_point_2d(int(in_vision_detections->objects[i].x + in_vision_detections->objects[i].width / 2), int(in_vision_detections->objects[i].y + in_vision_detections->objects[i].height));
          tf::Vector3 bottem_point_3d = ProjectPoint(bottem_point_2d);   // 转到base_link下 
          float position_x = float(bottem_point_3d.x());
          float position_y = float(bottem_point_3d.y());
          float position_z = float(bottem_point_3d.z());

          if (position_x > 15 || position_x < 0)  // 如果预测出来的深度过于远, 认为不准, 40m开外也不需要太关心, 忽略, 若深度估计为负, 则可能是由于过于接近图像中心导致, 前置相机不可能如此, 忽略
          {
            continue;
          }
          
          if (closest_dist > position_x * position_x + position_y * position_y && in_vision_detections->objects[i].label == "bottle")
          {
            geometry_msgs::Pose pose;
            geometry_msgs::Vector3 dimensions;
            autoware_msgs::CloudCluster detected_cluster;

            pose.position.x = position_x;    // velodyne往前2m, 车前1m
            pose.position.y = position_y;
            pose.position.z = position_z;
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0;
            pose.orientation.w = 1;

            dimensions.x = 1;
            dimensions.y = 1;
            dimensions.z = 1;

            detected_cluster.header = in_vision_detections->objects[i].header;   // 使用他的时间戳信息
            detected_cluster.header.frame_id = "velodyne";    // 修改frame_id到velodyne

            detected_cluster.label = "bottle";
            detected_cluster.bounding_box.pose = pose;
            detected_cluster.dimensions = dimensions;

            fused_objects.clusters.push_back(detected_cluster);    // 把它存入要发布的障碍物中

            closest_dist = position_x * position_x + position_y * position_y;
            closest_point_3d.x = position_x;
            closest_point_3d.y = position_y;
            closest_point_3d.z = position_z;
            trash_valid = true;
          }
        }

        // 情况二, 出现car或者person, 由于目前前面点云稀疏, 雷达聚类可能失败, 通过图像保证安全
        else if (in_vision_detections->objects[i].label == "person")
        { 
          int center_x = int(in_vision_detections->objects[i].x + in_vision_detections->objects[i].width / 2);
          int center_y = int(in_vision_detections->objects[i].y + in_vision_detections->objects[i].height);
          if (center_x > left_ignort_line_ && center_x < right_ignore_line_ && center_y > bottom_emergency_line_)
          {
            geometry_msgs::Pose pose;
            geometry_msgs::Vector3 dimensions;
            autoware_msgs::CloudCluster detected_cluster;

            geometry_msgs::Polygon new_polygon;
            geometry_msgs::Point32 edge_point;

            // 人为的设置一下突发障碍物的位置, 只需要它进规划的安全框就可以
            pose.position.x = 2.0;    // velodyne往前2m, 车前1m
            pose.position.y = 0.0;
            pose.position.z = 0.0;
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0;
            pose.orientation.w = 1;


            dimensions.x = person_depth_;
            dimensions.y = person_width_;
            dimensions.z = person_height_;

            edge_point.x = pose.position.x - dimensions.x / 2;
            edge_point.y = pose.position.y - dimensions.y / 2;
            edge_point.z = pose.position.z;
            new_polygon.points.push_back(edge_point);

            edge_point.x = pose.position.x + dimensions.x / 2;
            edge_point.y = pose.position.y - dimensions.y / 2;
            edge_point.z = pose.position.z;
            new_polygon.points.push_back(edge_point);

            edge_point.x = pose.position.x + dimensions.x / 2;
            edge_point.y = pose.position.y + dimensions.y / 2;
            edge_point.z = pose.position.z;
            new_polygon.points.push_back(edge_point);

            edge_point.x = pose.position.x - dimensions.x / 2;
            edge_point.y = pose.position.y + dimensions.y / 2;
            edge_point.z = pose.position.z;
            new_polygon.points.push_back(edge_point);

            edge_point.x = pose.position.x - dimensions.x / 2;    // 放初始点, 连成回环
            edge_point.y = pose.position.y - dimensions.y / 2;
            edge_point.z = pose.position.z;
            new_polygon.points.push_back(edge_point);

            detected_cluster.header = in_vision_detections->objects[i].header;   // 使用他的时间戳信息
            detected_cluster.header.frame_id = "velodyne";    // 修改frame_id到velodyne

            detected_cluster.label = in_vision_detections->objects[i].label;
            detected_cluster.bounding_box.pose = pose;
            detected_cluster.dimensions = dimensions;

            detected_cluster.convex_hull.header = detected_cluster.header;
            detected_cluster.convex_hull.polygon = new_polygon;

            fused_objects.clusters.push_back(detected_cluster);    // 把它存入要发布的障碍物中
          }

        }
      }
      
    }
  }
  // 遍历完毕, 如果使用mono detect, 则将最近的垃圾位置发布到/inspection_target, 用于巡检, 这个永远发布在base_link坐标系下
  if (use_mono_estimate && trash_valid)
  {
    publisher_trash_objects_.publish(closest_point_3d);
  }

  //add also objects outside the image
  for (auto &object: range_out_cv.clusters)
  {
    fused_objects.clusters.push_back(object);
  }
  //enable merged for visualization
  // for (auto &object : fused_objects.objects)
  // {
  //   object.valid = true;
  // }

  return fused_objects;
}

/*
void
ROSRangeVisionFusionApp::SyncedDetectionsCallback(
  const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections,
  const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections)
{
  autoware_msgs::DetectedObjectArray fusion_objects;
  fusion_objects.objects.clear();

  if (empty_frames_ > 5)
  {
    ROS_INFO("[%s] Empty Detections. Make sure the vision and range detectors are running.", __APP_NAME__);
  }

  // 两个均为空
  if (nullptr == in_vision_detections
      && nullptr == in_range_detections)
  {
    empty_frames_++;
    return;
  }

  // 视觉为空, 点云不为空
  if (nullptr == in_vision_detections
      && nullptr != in_range_detections
      && !in_range_detections->objects.empty())
  {
    publisher_fused_objects_.publish(in_range_detections);
    empty_frames_++;
    return;
  }

  // 点云为空, 视觉不为空
  if (nullptr == in_range_detections
      && nullptr != in_vision_detections
      && !in_vision_detections->objects.empty())
  {
    publisher_fused_objects_.publish(in_vision_detections);
    empty_frames_++;
    return;
  }

  if (!camera_lidar_tf_ok_)
  {
    // camera_lidar_tf_ = FindTransform(image_frame_id_,
    //                                  in_range_detections->header.frame_id);
    camera_lidar_tf_ = FindTransform(image_frame_id_,
                                     "velodyne");   // ljc add: 在欧式聚类中无论output_frame是什么, 融合有用的信息bbox均在velodyne下, 即使经过lidar kf的tracked objects也不变, 所以这里直接查找到velodyne的变换
  }
  if (
    !camera_lidar_tf_ok_ ||
    !camera_info_ok_)
  {
    ROS_INFO("[%s] Missing Camera-LiDAR TF or CameraInfo", __APP_NAME__);
    return;
  }

  fusion_objects = FuseRangeVisionDetections(in_vision_detections, in_range_detections);

  publisher_fused_objects_.publish(fusion_objects);
  empty_frames_ = 0;

  vision_detections_ = nullptr;
  range_detections_ = nullptr;

}
*/

void
ROSRangeVisionFusionApp::SyncedDetectionsCallback(
  const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections,
  const autoware_msgs::CloudClusterArray::ConstPtr &in_range_detections)
{
  autoware_msgs::CloudClusterArray fusion_objects;
  fusion_objects.clusters.clear();

  if (empty_frames_ > 5)
  {
    ROS_INFO("[%s] Empty Detections. Make sure the vision and range detectors are running.", __APP_NAME__);
  }

  // 两个均为空
  if (nullptr == in_vision_detections
      && nullptr == in_range_detections)
  {
    empty_frames_++;
    return;
  }

  // 视觉为空, 点云不为空
  if (nullptr == in_vision_detections
      && nullptr != in_range_detections
      && !in_range_detections->clusters.empty())
  {
    publisher_fused_objects_.publish(in_range_detections);
    empty_frames_++;
    return;
  }

  // 点云为空, 视觉不为空
  if (nullptr == in_range_detections
      && nullptr != in_vision_detections
      && !in_vision_detections->objects.empty())
  {
    empty_frames_++;
    return;
  }

  if (!camera_lidar_tf_ok_)
  {
    // camera_lidar_tf_ = FindTransform(image_frame_id_,
    //                                  in_range_detections->header.frame_id);
    camera_lidar_tf_ = FindTransform(image_frame_id_,
                                     "velodyne");   // ljc add: 在欧式聚类中无论output_frame是什么, 融合有用的信息bbox均在velodyne下, 即使经过lidar kf的tracked objects也不变, 所以这里直接查找到velodyne的变换
  }
  if (
    !camera_lidar_tf_ok_ ||
    !camera_info_ok_)
  {
    ROS_INFO("[%s] Missing Camera-LiDAR TF or CameraInfo", __APP_NAME__);
    return;
  }

  fusion_objects = FuseRangeVisionDetections(in_vision_detections, in_range_detections);

  publisher_fused_objects_.publish(fusion_objects);
  empty_frames_ = 0;

  vision_detections_ = nullptr;
  range_detections_ = nullptr;

}

void
ROSRangeVisionFusionApp::VisionDetectionsCallback(
  const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections)
{
  // 这里传入的in_vision_detections, 由于它是2D检测框, 即便它是DetectedObjectArray, 它具有的属性与3D点云簇不想同, 它只具有x,y, width, height
  if (!processing_ && !in_vision_detections->objects.empty())
  {
    processing_ = true;
    vision_detections_ = in_vision_detections;
    SyncedDetectionsCallback(in_vision_detections, range_detections_);
    processing_ = false;
  }
}

// void
// ROSRangeVisionFusionApp::RangeDetectionsCallback(
//   const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections)
// {
//   if (!processing_ && !in_range_detections->objects.empty())
//   {
//     processing_ = true;
//     range_detections_ = in_range_detections;
//     SyncedDetectionsCallback(vision_detections_, in_range_detections);
//     processing_ = false;
//   }
// }

void
ROSRangeVisionFusionApp::RangeDetectionsCallback(
  const autoware_msgs::CloudClusterArray::ConstPtr &in_range_detections)
{
  if (!processing_ && !in_range_detections->clusters.empty())
  {
    processing_ = true;
    range_detections_ = in_range_detections;
    SyncedDetectionsCallback(vision_detections_, in_range_detections);
    processing_ = false;
  }
}

void ROSRangeVisionFusionApp::ImageCallback(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
  if (!camera_info_ok_)
    return;
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, "bgr8");
  cv::Mat in_image = cv_image->image;

  cv::Mat undistorted_image;
  cv::undistort(in_image, image_, camera_instrinsics_, distortion_coefficients_);
};

void
ROSRangeVisionFusionApp::IntrinsicsCallback(const sensor_msgs::CameraInfo &in_message)
{
  image_size_.height = in_message.height;
  image_size_.width = in_message.width;

  camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
  for (int row = 0; row < 3; row++)
  {
    for (int col = 0; col < 3; col++)
    {
      camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
    }
  }

  distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
  for (int col = 0; col < 5; col++)
  {
    distortion_coefficients_.at<double>(col) = in_message.D[col];
  }

  fx_ = static_cast<float>(in_message.P[0]);
  fy_ = static_cast<float>(in_message.P[5]);
  cx_ = static_cast<float>(in_message.P[2]);
  cy_ = static_cast<float>(in_message.P[6]);

  intrinsics_subscriber_.shutdown();
  camera_info_ok_ = true;
  image_frame_id_ = in_message.header.frame_id;
  ROS_INFO("[%s] CameraIntrinsics obtained.", __APP_NAME__);
}

void ROSRangeVisionFusionApp::PlaneCoeffCallback(const std_msgs::Float32MultiArrayConstPtr &in_message)
{
  // 能接收到发布的平面方程, 只有当相机初始化完成后才进行, todo: 目前由ray_ground_filter传入的float32multiarray没有header, 因此写死source frame为"base_link"
  if (camera_info_ok_)
  {
    tf::StampedTransform baselink2camera;

    try
    {
      transform_listener_->lookupTransform(image_frame_id_, "base_link", ros::Time(0), baselink2camera);
      plane_coeff_ok_ = true;
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
      return;
    }

    if (plane_coeff_ok_)
    {
      // 初始化时为false, 查询成功一次即可(按理说是fix的), 这样能保证在plane_coeff_ok_为true时plane_coeff一定有值
      tf::Vector3 normal(in_message->data[0], in_message->data[1], in_message->data[2]);   // base_link下的法向量
      tf::Vector3 point(0, 0, -in_message->data[3] / in_message->data[2]);   // 平面上一点
      tf::Matrix3x3 rotation_matrix(baselink2camera.getRotation());          // 旋转矩阵
      tf::Vector3 origin(baselink2camera.getOrigin()); //
      normal = rotation_matrix * normal;    // camera下的法向量
      point = rotation_matrix * point + origin;  // 旋转后的点, 法向量是个方向, 不需要加平移
      float intercept = float(-normal.dot(point));   // 截距: a'x+b'y+c'z+d'=0
      std::vector<float> cur_plane_coeff = {float(normal.x()), float(normal.y()), float(normal.z()), intercept};
      plane_coeff = cur_plane_coeff;

      // ROS_INFO_STREAM(plane_coeff[0] << " " << plane_coeff[1] << " " << plane_coeff[2] << " " << plane_coeff[3]);
      }
  }
  
}

tf::StampedTransform
ROSRangeVisionFusionApp::FindTransform(const std::string &in_target_frame, const std::string &in_source_frame)
{
  tf::StampedTransform transform;

  ROS_INFO("%s - > %s", in_source_frame.c_str(), in_target_frame.c_str());
  camera_lidar_tf_ok_ = false;
  try
  {
    transform_listener_->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
    camera_lidar_tf_ok_ = true;
    ROS_INFO("[%s] Camera-Lidar TF obtained", __APP_NAME__);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
  }

  return transform;
}

void
ROSRangeVisionFusionApp::InitializeROSIo(ros::NodeHandle &in_private_handle)
{
  //get params
  std::string camera_info_src, detected_objects_vision, min_car_dimensions, min_person_dimensions, min_truck_dimensions;
  std::string detected_objects_range, fused_topic_str = "/detection/fusion_tools/objects";
  std::string name_space_str = ros::this_node::getNamespace();
  bool sync_topics = false;

  ROS_INFO(
    "[%s] This node requires: Registered TF(Lidar-Camera), CameraInfo, Vision and Range Detections being published.",
    __APP_NAME__);
  in_private_handle.param<std::string>("detected_objects_range", detected_objects_range,
                                       "/detection/lidar_detector/objects");
  ROS_INFO("[%s] detected_objects_range: %s", __APP_NAME__, detected_objects_range.c_str());

  in_private_handle.param<std::string>("detected_objects_vision", detected_objects_vision,
                                       "/detection/image_detector/objects");
  ROS_INFO("[%s] detected_objects_vision: %s", __APP_NAME__, detected_objects_vision.c_str());

  in_private_handle.param<std::string>("camera_info_src", camera_info_src, "/camera_info");
  ROS_INFO("[%s] camera_info_src: %s", __APP_NAME__, camera_info_src.c_str());

  in_private_handle.param<double>("overlap_threshold", overlap_threshold_, 0.6);
  ROS_INFO("[%s] overlap_threshold: %f", __APP_NAME__, overlap_threshold_);

  in_private_handle.param<std::string>("min_car_dimensions", min_car_dimensions, "[3,2,2]");//w,h,d
  ROS_INFO("[%s] min_car_dimensions: %s", __APP_NAME__, min_car_dimensions.c_str());

  in_private_handle.param<std::string>("min_person_dimensions", min_person_dimensions, "[1,2,1]");
  ROS_INFO("[%s] min_person_dimensions: %s", __APP_NAME__, min_person_dimensions.c_str());

  in_private_handle.param<std::string>("min_truck_dimensions", min_truck_dimensions, "[4,2,2]");
  ROS_INFO("[%s] min_truck_dimensions: %s", __APP_NAME__, min_truck_dimensions.c_str());

  in_private_handle.param<bool>("sync_topics", sync_topics, false);
  ROS_INFO("[%s] sync_topics: %d", __APP_NAME__, sync_topics);

  in_private_handle.param<bool>("use_mono_estimate", use_mono_estimate, false);
  ROS_INFO("[%s] use_mono_estimate: %d", __APP_NAME__, use_mono_estimate);

  in_private_handle.param<int>("left_ignore_line", left_ignort_line_, 200);
  ROS_INFO("[%s] left_ignore_line: %d", __APP_NAME__, left_ignort_line_);

  in_private_handle.param<int>("right_ignore_line", right_ignore_line_, 1720);
  ROS_INFO("[%s] right_ignore_line: %d", __APP_NAME__, right_ignore_line_);

  in_private_handle.param<int>("bottom_emergency_line", bottom_emergency_line_, 1050);
  ROS_INFO("[%s] bottom_emergency_line: %d", __APP_NAME__, bottom_emergency_line_);

  YAML::Node car_dimensions = YAML::Load(min_car_dimensions);
  YAML::Node person_dimensions = YAML::Load(min_person_dimensions);
  YAML::Node truck_dimensions = YAML::Load(min_truck_dimensions);

  if (car_dimensions.size() == 3)
  {
    car_width_ = car_dimensions[0].as<double>();
    car_height_ = car_dimensions[1].as<double>();
    car_depth_ = car_dimensions[2].as<double>();
  }
  if (person_dimensions.size() == 3)
  {
    person_width_ = person_dimensions[0].as<double>();
    person_height_ = person_dimensions[1].as<double>();
    person_depth_ = person_dimensions[2].as<double>();
  }
  if (truck_dimensions.size() == 3)
  {
    truck_width_ = truck_dimensions[0].as<double>();
    truck_height_ = truck_dimensions[1].as<double>();
    truck_depth_ = truck_dimensions[2].as<double>();
  }

  if (name_space_str != "/")
  {
    if (name_space_str.substr(0, 2) == "//")
    {
      name_space_str.erase(name_space_str.begin());
    }
    camera_info_src = name_space_str + camera_info_src;
  }

  //generate subscribers and sychronizers
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, camera_info_src.c_str());
  intrinsics_subscriber_ = in_private_handle.subscribe(camera_info_src,
                                                       1,
                                                       &ROSRangeVisionFusionApp::IntrinsicsCallback, this);
  plane_coeff_subscriber_ = node_handle_.subscribe("points_ground_coeff", 1, &ROSRangeVisionFusionApp::PlaneCoeffCallback, this);

  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, detected_objects_vision.c_str());
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, detected_objects_range.c_str());
  if (!sync_topics)
  {
    detections_vision_subscriber_ = in_private_handle.subscribe(detected_objects_vision,
                                                               1,
                                                               &ROSRangeVisionFusionApp::VisionDetectionsCallback,
                                                               this);

    detections_range_subscriber_ = in_private_handle.subscribe(detected_objects_range,
                                                                1,
                                                                &ROSRangeVisionFusionApp::RangeDetectionsCallback,
                                                                this);
  }
  else
  {
    vision_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_,
                                                                                                    detected_objects_vision,
                                                                                                    1);
    // range_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_,
    //                                                                                                detected_objects_range,
    //                                                                                                1);
    range_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::CloudClusterArray>(node_handle_,
                                                                                                 detected_objects_range,
                                                                                                 1);
    detections_synchronizer_ =
      new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10),
                                                     *vision_filter_subscriber_,
                                                     *range_filter_subscriber_);
    detections_synchronizer_->registerCallback(
      boost::bind(&ROSRangeVisionFusionApp::SyncedDetectionsCallback, this, _1, _2));
  }

  // publisher_fused_objects_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>(fused_topic_str, 1);
  publisher_fused_objects_ = node_handle_.advertise<autoware_msgs::CloudClusterArray>(fused_topic_str, 1);
  publisher_trash_objects_ = node_handle_.advertise<geometry_msgs::Vector3>("/inspection_target", 1);

  ROS_INFO("[%s] Publishing fused objects in %s", __APP_NAME__, fused_topic_str.c_str());

}


void
ROSRangeVisionFusionApp::Run()
{
  ros::NodeHandle private_node_handle("~");
  tf::TransformListener transform_listener;

  transform_listener_ = &transform_listener;

  InitializeROSIo(private_node_handle);

  ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);

  ros::spin();

  ROS_INFO("[%s] END", __APP_NAME__);
}

ROSRangeVisionFusionApp::ROSRangeVisionFusionApp()
{
  camera_lidar_tf_ok_ = false;
  camera_info_ok_ = false;
  processing_ = false;
  plane_coeff_ok_ = false;
  image_frame_id_ = "";
  overlap_threshold_ = 0.5;
  empty_frames_ = 0;
}