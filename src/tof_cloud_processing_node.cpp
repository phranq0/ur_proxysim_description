#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::placeholders;

class PointCloudMerger : public rclcpp::Node
{
public:
    PointCloudMerger()
        : Node("point_cloud_merger"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        discoverPointCloudTopics();

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_point_cloud", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&PointCloudMerger::mergeClouds, this));
    }

private:
    void discoverPointCloudTopics()
    {
        // ROS2 network discovery takes some time
        std::this_thread::sleep_for(std::chrono::seconds(2));
        auto topics_infos = this->get_topic_names_and_types();

        int topic_count = 0;

        for (const auto &topic_it : topics_infos)
        {
            std::string topic_name = topic_it.first;
            std::vector<std::string> topic_types = topic_it.second;
            for (const auto &topic_type : topic_types)
            {
                if (topic_type == "sensor_msgs/msg/PointCloud2" && topic_name != "/merged_point_cloud")
                {
                    auto callback = [this, topic_count](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
                    {
                        this->pointCloudCallback(topic_count, msg);
                    };
                    subscribers_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, 10, callback));

                    // init with empty structured msg
                    auto empty_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
                    empty_cloud->height = 1;
                    empty_cloud->width = 0;
                    empty_cloud->is_bigendian = false;
                    empty_cloud->point_step = sizeof(float) * 3;
                    empty_cloud->row_step = empty_cloud->point_step * empty_cloud->width;
                    empty_cloud->is_dense = false;
                    point_clouds_.push_back(empty_cloud);

                    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", topic_name.c_str());
                    ++topic_count;
                }
            }
        }
    }

    void pointCloudCallback(int index, const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // RCLCPP_INFO(get_logger(), "Received point cloud on topic %s", subscribers_[index]->get_topic_name());
        point_clouds_[index] = msg;
    }

    void mergeClouds()
    {
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcl_clouds;
        std_msgs::msg::Header header;
        std::string target_frame = "base_link";

        for (const auto &cloud : point_clouds_)
        {
            // local point cloud backprojection 
            sensor_msgs::msg::PointCloud2 transformed_cloud;
            if (!transformPointCloud(cloud, transformed_cloud, target_frame))
            {
                RCLCPP_WARN(this->get_logger(), "Failed to transform point cloud to %s", target_frame.c_str());
                continue;
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(transformed_cloud, *pcl_cloud);
            pcl_clouds.push_back(pcl_cloud);
            header = transformed_cloud.header;
        }

        std::cout << pcl_clouds.size() << "\n\n";

        if (pcl_clouds.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No valid point clouds to merge");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        for (const auto &cloud : pcl_clouds)
        {
            *merged_cloud += *cloud;
        }

        sensor_msgs::msg::PointCloud2 merged_cloud_msg;
        pcl::toROSMsg(*merged_cloud, merged_cloud_msg);
        merged_cloud_msg.header = header; // set back header
        merged_cloud_msg.header.frame_id = target_frame;

        publisher_->publish(merged_cloud_msg);
        // RCLCPP_INFO(this->get_logger(), "Published merged point cloud");
    }

    bool hasFields(const sensor_msgs::msg::PointCloud2::SharedPtr cloud, const std::vector<std::string> &fields)
    {
        std::set<std::string> cloud_fields;
        for (const auto &field : cloud->fields)
        {
            cloud_fields.insert(field.name);
        }

        for (const auto &field : fields)
        {
            if (cloud_fields.find(field) == cloud_fields.end())
            {
                return false;
            }
        }
        return true;
    }

    bool transformPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &input_cloud,
                            sensor_msgs::msg::PointCloud2 &output_cloud,
                            const std::string &target_frame)
    {
        try
        {
            auto transform = tf_buffer_.lookupTransform(target_frame, input_cloud->header.frame_id, tf2::TimePointZero);
            tf2::doTransform(*input_cloud, output_cloud, transform);
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", input_cloud->header.frame_id.c_str(), target_frame.c_str(), ex.what());
            return false;
        }
    }

    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscribers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> point_clouds_;
    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudMerger>());
    rclcpp::shutdown();
    return 0;
}
