#include <memory>
#include <vector>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class PointCloudMerger : public rclcpp::Node
{
public:
    PointCloudMerger()
        : Node("point_cloud_merger")
    {
        discoverPointCloudTopics();

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), subscribers_);
        sync_->registerCallback(std::bind(&PointCloudMerger::pointCloudCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8, std::placeholders::_9, std::placeholders::_10));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_point_cloud", 10);
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;

    // autonomous discovery of pointcloud2 topics
    void discoverPointCloudTopics()
    {
        auto topics_and_types = this->get_topic_names_and_types();
        for (const auto &topic_and_type : topics_and_types)
        {
            const std::string &topic_name = topic_and_type.first;
            const std::vector<std::string> &types = topic_and_type.second;

            for (const std::string &type : types)
            {
                if (type == "sensor_msgs/msg/PointCloud2")
                {
                    auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, topic_name);
                    subscribers_.push_back(sub);
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Found %d PointCloud2 topics.");
    }

    void pointCloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg1,
        const sensor_msgs::msg::PointCloud2::SharedPtr msg2,
        const sensor_msgs::msg::PointCloud2::SharedPtr msg3,
        const sensor_msgs::msg::PointCloud2::SharedPtr msg4,
        const sensor_msgs::msg::PointCloud2::SharedPtr msg5,
        const sensor_msgs::msg::PointCloud2::SharedPtr msg6,
        const sensor_msgs::msg::PointCloud2::SharedPtr msg7,
        const sensor_msgs::msg::PointCloud2::SharedPtr msg8,
        const sensor_msgs::msg::PointCloud2::SharedPtr msg9,
        const sensor_msgs::msg::PointCloud2::SharedPtr msg10)
    {
        auto merged_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        merged_cloud->header = msg1->header;
        merged_cloud->height = 1;

        std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> point_clouds = {msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8, msg9, msg10};
        std::vector<sensor_msgs::PointCloud2Iterator<float>> iters;

        for (auto &pc : point_clouds)
        {
            sensor_msgs::PointCloud2Iterator<float> iter_x(*pc, "x");
            iters.push_back(iter_x);
        }

        size_t total_points = 0;
        for (const auto &pc : point_clouds)
        {
            total_points += pc->width * pc->height;
        }

        merged_cloud->width = total_points;
        merged_cloud->is_bigendian = false;
        merged_cloud->point_step = point_clouds[0]->point_step;
        merged_cloud->row_step = merged_cloud->point_step * merged_cloud->width;
        merged_cloud->is_dense = true;
        merged_cloud->data.resize(merged_cloud->row_step * merged_cloud->height);

        sensor_msgs::PointCloud2Iterator<float> out_iter_x(*merged_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_iter_y(*merged_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_iter_z(*merged_cloud, "z");

        for (auto &pc : point_clouds)
        {
            sensor_msgs::PointCloud2Iterator<float> iter_x(*pc, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(*pc, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(*pc, "z");

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

    std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>> subscribers_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudMerger>());
    rclcpp::shutdown();
    return 0;
}
