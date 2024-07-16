#ifndef POINT_CLOUD_MERGER__CUSTOM_SYNC_POLICY_HPP_
#define POINT_CLOUD_MERGER__CUSTOM_SYNC_POLICY_HPP_

#include "message_filters/sync_policies/approximate_time.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace point_cloud_merger
{
    // parametric class for different message types
    template <typename M>
    class CustomSyncPolicy
    {
    public:
        CustomSyncPolicy(int queue_size) : queue_size_(queue_size) {}

        // flexible callback registration (allowing for different callable entities)
        template <class F>
        void registerCallback(F callback)
        {
            callback_ = callback;
        }

        void addSubscriber(std::shared_ptr<message_filters::Subscriber<M>> subscriber)
        {
            subscribers_.push_back(subscriber);
        }

        void signal(const std::vector<typename M::SharedPtr> &msgs)
        {
            callback_(msgs);
        }

        size_t size() const
        {
            return subscribers_.size();
        }

        std::vector<std::shared_ptr<message_filters::Subscriber<M>>> getSubscribers() const
        {
            return subscribers_;
        }

    private:
        int queue_size_;
        std::vector<std::shared_ptr<message_filters::Subscriber<M>>> subscribers_;
        std::function<void(const std::vector<typename M::SharedPtr> &)> callback_;
    };

} // namespace point_cloud_merger

#endif // POINT_CLOUD_MERGER__CUSTOM_SYNC_POLICY_HPP_
