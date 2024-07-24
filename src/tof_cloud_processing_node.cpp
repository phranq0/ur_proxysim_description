#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::placeholders;

class PointCloudMerger : public rclcpp::Node
{
public:
    PointCloudMerger()
        : Node("point_cloud_merger")
    {
        discoverPointCloudTopics();
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_point_cloud", 10);
        //timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PointCloudMerger::mergeClouds, this));
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
                if (topic_type == "sensor_msgs/msg/PointCloud2")
                {
                    auto callback = [this, topic_count](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                        this->pointCloudCallback(topic_count, msg);
                    };
                    subscribers_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, 10, callback));
                    
                    // init with empty structured msg
                    auto empty_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
                    empty_cloud->height = 1;
                    empty_cloud->width = 0;
                    empty_cloud->is_bigendian = false;
                    empty_cloud->point_step = sizeof(float) * 3; // x, y, z
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
        RCLCPP_INFO(get_logger(), "Received point cloud on topic %s", subscribers_[index]->get_topic_name());
        point_clouds_[index] = msg;
    }

    void mergeClouds()
    {
        std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> valid_clouds;
        for (const auto &cloud : point_clouds_)
        {
            if (cloud)
            {
                valid_clouds.push_back(cloud);
            }
        }

        if (valid_clouds.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No valid point clouds to merge");
            return;
        }

        auto merged_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        merged_cloud->header = valid_clouds[0]->header;
        merged_cloud->height = 1;

        size_t total_points = 0;
        for (const auto &cloud : valid_clouds)
        {
            total_points += cloud->width * cloud->height;
        }

        merged_cloud->width = total_points;
        merged_cloud->is_bigendian = false;
        merged_cloud->point_step = valid_clouds[0]->point_step;
        merged_cloud->row_step = merged_cloud->point_step * merged_cloud->width;
        merged_cloud->is_dense = true;
        merged_cloud->data.resize(merged_cloud->row_step * merged_cloud->height);

        sensor_msgs::PointCloud2Iterator<float> out_iter_x(*merged_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_iter_y(*merged_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_iter_z(*merged_cloud, "z");

        for (const auto &cloud : valid_clouds)
        {
            sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");

            while (iter_x != iter_x.end())
            {
                *out_iter_x = *iter_x;
                *out_iter_y = *iter_y;
                *out_iter_z = *iter_z;
                ++out_iter_x;
                ++out_iter_y;
                ++out_iter_z;
                ++iter_x;
                ++iter_y;
                ++iter_z;
            }
        }

        publisher_->publish(*merged_cloud);
        RCLCPP_INFO(this->get_logger(), "Published merged point cloud");
    }

    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscribers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> point_clouds_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudMerger>());
    rclcpp::shutdown();
    return 0;
}
