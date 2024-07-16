#include <memory>
#include <vector>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "message_filters/subscriber.h"
#include "custom_sync_policy.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class PointCloudMerger : public rclcpp::Node
{
public:
    PointCloudMerger() : Node("point_cloud_merger")
    {
        discoverPointCloudTopics();

        if (sync_policy_->size() < 2)
        {
            RCLCPP_ERROR(this->get_logger(), "Need at least two PointCloud2 topics for synchronization.");
            return;
        }

        sync_policy_->registerCallback(std::bind(&PointCloudMerger::pointCloudCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_point_cloud", 10);
    }

private:
    // custom sync policy, defined for pcd2 messages
    using SyncPolicy = point_cloud_merger::CustomSyncPolicy<sensor_msgs::msg::PointCloud2>;

    void discoverPointCloudTopics()
    {

        // ROS2 network discovery takes some time
        sleep(2);
        auto topics_infos = this->get_topic_names_and_types();

        for (const auto &topic_it : topics_infos)
        {
            std::string topic_name = topic_it.first;
            std::vector<std::string> topic_types = topic_it.second;
            // iterate over all topic types
            for (const auto &topic_type : topic_types)
                {
                    if (topic_type == "sensor_msgs/msg/PointCloud2")
                    {
                        auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, topic_name);
                        sync_policy_->addSubscriber(sub);
                        RCLCPP_INFO(this->get_logger(), "ToF Network - Topic: %s, type: %s", topic_name.c_str(), topic_type.c_str());
                    }
                }

            /*
            for (const auto &topic_and_type : topics_and_types)
            {
                const std::string &topic_name = topic_and_type.first;
                const std::vector<std::string> &types = topic_and_type.second;

                for (const std::string &type : types)
                {
                    if (type == "sensor_msgs/msg/PointCloud2")
                    {
                        auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, topic_name);
                        sync_policy_->addSubscriber(sub);
                    }
                    RCLCPP_INFO(this->get_logger(), "# of topics: %d.", (int)topics_and_types.size());
                }
            }
            RCLCPP_INFO(this->get_logger(), "Discovered %d PointCloud2 topics.", (int)sync_policy_->size());
            */
        }

    }

        void pointCloudCallback(const std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> &msgs)
        {
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

        std::shared_ptr<SyncPolicy> sync_policy_ = std::make_shared<SyncPolicy>(10);
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    };

    int main(int argc, char *argv[])
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<PointCloudMerger>());
        rclcpp::shutdown();
        return 0;
    }
