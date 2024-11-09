#include <ros/ros.h>
#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <people_msgs/People.h>
#include <people_msgs/Person.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PeopleTracker {
private:
    ros::NodeHandle nh_;
    ros::Subscriber leg_detector_sub_;
    ros::Publisher people_pub_;
    ros::Publisher marker_pub_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    // 参数
    std::string target_frame_;
    double person_radius_;
    bool publish_markers_;
    double reliability_threshold_;
    std::string input_topic_;   // 输入话题参数
    std::string output_topic_;  // 输出话题参数

public:
    PeopleTracker() : tfListener_(tfBuffer_) {
        // 获取参数
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("fixed_frame", target_frame_, "base_footprint");
        private_nh.param<double>("person_radius", person_radius_, 0.3);
        private_nh.param<bool>("publish_markers", publish_markers_, true);
        private_nh.param<double>("reliability_threshold", reliability_threshold_, 0.7);
        private_nh.param<std::string>("input_topic", input_topic_, "/people_tracker_measurements");
        private_nh.param<std::string>("output_topic", output_topic_, "/people");

        // 初始化订阅和发布
        leg_detector_sub_ = nh_.subscribe(input_topic_, 1,
            &PeopleTracker::legDetectorCallback, this);
        people_pub_ = nh_.advertise<people_msgs::People>(output_topic_, 1);

        if (publish_markers_) {
            marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/people_markers", 1);
        }

        ROS_INFO("People tracker initialized with:");
        ROS_INFO("  Target frame: %s", target_frame_.c_str());
        ROS_INFO("  Input topic: %s", input_topic_.c_str());
        ROS_INFO("  Output topic: %s", output_topic_.c_str());
    }

    void legDetectorCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg) {
        people_msgs::People people_msg;
        visualization_msgs::MarkerArray marker_array;

        people_msg.header.stamp = ros::Time::now();
        people_msg.header.frame_id = target_frame_;

        for (const auto& person : msg->people) {
            try {
                // 转换坐标系
                geometry_msgs::PoseStamped pose_in, pose_out;
                pose_in.header = msg->header;
                pose_in.pose.position = person.pos;
                pose_in.pose.orientation.w = 1.0;

                tfBuffer_.transform(pose_in, pose_out, target_frame_);

                // 检查可靠性阈值
                if (person.reliability < reliability_threshold_) {
                    continue;
                }

                // 创建Person消息
                people_msgs::Person person_out;
                person_out.name = person.object_id;
                person_out.reliability = person.reliability;
                person_out.position = pose_out.pose.position;
                person_out.position.z = 0.0;  // 2D导航使用

                // 设置速度（目前为0，如果leg_detector提供速度信息可以更新）
                person_out.velocity.x = 0.0;
                person_out.velocity.y = 0.0;
                person_out.velocity.z = 0.0;

                // 添加标签
                person_out.tagnames.push_back("radius");
                person_out.tags.push_back(std::to_string(person_radius_));
                person_out.tagnames.push_back("type");
                person_out.tags.push_back("person");

                people_msg.people.push_back(person_out);

                // 创建可视化标记
                if (publish_markers_) {
                    visualization_msgs::Marker marker;
                    marker.header = people_msg.header;
                    marker.ns = "people";
                    marker.id = marker_array.markers.size();
                    marker.type = visualization_msgs::Marker::CYLINDER;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position = person_out.position;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = person_radius_ * 2;
                    marker.scale.y = person_radius_ * 2;
                    marker.scale.z = 1.8;  // 人的大致高度
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                    marker.color.a = 0.5;
                    marker.lifetime = ros::Duration(0.2);

                    marker_array.markers.push_back(marker);
                }
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN_THROTTLE(1, "Transform failed: %s", ex.what());
                continue;
            }
        }

        // 发布消息
        if (!people_msg.people.empty()) {
            people_pub_.publish(people_msg);
            if (publish_markers_) {
                marker_pub_.publish(marker_array);
            }
            ROS_DEBUG("Published %zu people", people_msg.people.size());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "people_tracker_node");

    try {
        PeopleTracker tracker;
        ros::spin();
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception in people_tracker_node: %s", e.what());
        return 1;
    }

    return 0;
}
