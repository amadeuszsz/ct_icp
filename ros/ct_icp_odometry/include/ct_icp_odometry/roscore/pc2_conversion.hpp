#ifndef CT_ICP_ODOMETRY__ROSCORE__PC2_CONVERSION_HPP
#define CT_ICP_ODOMETRY__ROSCORE__PC2_CONVERSION_HPP

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <SlamCore/pointcloud.h>

namespace slam
{

// Converts PCL to Slam Data types
slam::PROPERTY_TYPE ROSPointFieldDTypeToSlamDType(int pcl_type);

// Builds a default Item Schema from a PointCloud2 fields layout
// Note:    The layout in memory might contain padding (unused bytes) which are added as padding_<i> char properties
slam::ItemSchema::Builder SchemaBuilderFromCloud2(const sensor_msgs::msg::PointCloud2 & cloud);

// A PointCloud2 Pointer Wrapper
// It allows a BufferWrapper to keep a shared pointer to the PointCloud2, ensuring that no data is deallocated prior
// The destruction of the BufferWrapper
struct PointCloud2PtrWrapper : slam::BufferWrapper::SmartDataPtrWrapper
{

  ~PointCloud2PtrWrapper() override = default;

  explicit PointCloud2PtrWrapper(sensor_msgs::msg::PointCloud2::SharedPtr _ptr)
  : ptr(_ptr) {}

  sensor_msgs::msg::PointCloud2::SharedPtr ptr = nullptr;
};

// Returns a shallow copy of a point cloud
// Note:    Once the data of `cloud` is deleted, the copy will
slam::PointCloudPtr ROSCloud2ToSlamPointCloudShallow(
  const sensor_msgs::msg::PointCloud2 & cloud,
  std::shared_ptr<PointCloud2PtrWrapper> pointer = nullptr);

// Returns a shallow copy of a point cloud
// Note:    It will add a reference to the cloud pointer, in order to keep it alive
slam::PointCloudPtr ROSCloud2ToSlamPointCloudShallow(
  sensor_msgs::msg::PointCloud2::SharedPtr & cloud);

// Returns a shallow copy of a point cloud
// Note:    It will add a reference to the cloud pointer, in order to keep it alive
slam::PointCloudPtr ROSCloud2ToSlamPointCloudShallow(
  const sensor_msgs::msg::PointCloud2::SharedPtr & cloud);


// Returns a deep copy of a point cloud
// Note:    The data layout will be the same as the input cloud
slam::PointCloudPtr ROSCloud2ToSlamPointCloudDeep(const sensor_msgs::msg::PointCloud2 & cloud);

}  // namespace slam

#endif  // CT_ICP_ODOMETRY__ROSCORE__PC2_CONVERSION_HPP
