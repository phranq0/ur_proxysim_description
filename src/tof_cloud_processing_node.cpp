#include <memory>
#include <vector>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "message_filters/subscriber.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::placeholders;

class PointCloudMerger : public rclcpp::Node
{
public:
    PointCloudMerger() : Node("point_cloud_merger")
    {
        discoverPointCloudTopics();

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_point_cloud", 10);
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
            // iterate over all topic types
            for (const auto &topic_type : topic_types)
            {
                if (topic_type == "sensor_msgs/msg/PointCloud2")
                {
                    auto callback = std::bind(&PointCloudMerger::pointCloudCallback, this, topic_count, _1);
                    subscribers_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, 10, callback));
                    RCLCPP_INFO(this->get_logger(), "ToF Network - Topic: %s, type: %s", topic_name.c_str(), topic_type.c_str());
                }
            }
        }

        // initialize msg structure
    }

    // same callback, storing on different indexes
    void pointCloudCallback(int index, const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received point cloud on topic %s", subscribers_[index]->get_topic_name());

        // Store the received point cloud message at the specified index
        point_clouds_[index] = msg;

        // TODO Make reprojection from here, I can take the correct cloud frame directly from the msg header and store
        // the clouds already backprojected, so I can just sum them up and publish them
    }

    // TODO Fix this, no more using message filters, but running over a timer, also include tf listeners for projections
    void mergeCloudCallback(const std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> &msgs)
    {

        // RCLCPP_INFO(this->get_logger(), "Merged Cloud Callback - msgs num: %d", (int)msgs.size());

        auto merged_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        merged_cloud->header = msgs[0]->header;
        merged_cloud->height = 1;

        size_t total_points = 0;
        for (const auto &msg : msgs)
        {
            total_points += msg->width * msg->height;
        }

        merged_cloud->width = total_points;
        merged_cloud->is_bigendian = false;
        merged_cloud->point_step = msgs[0]->point_step;
        merged_cloud->row_step = merged_cloud->point_step * merged_cloud->width;
        merged_cloud->is_dense = true;
        merged_cloud->data.resize(merged_cloud->row_step * merged_cloud->height);

        sensor_msgs::PointCloud2Iterator<float> out_iter_x(*merged_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_iter_y(*merged_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_iter_z(*merged_cloud, "z");

        for (const auto &msg : msgs)
        {
            sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

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
    }

    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscribers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> point_clouds_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudMerger>());
    rclcpp::shutdown();
    return 0;
}
