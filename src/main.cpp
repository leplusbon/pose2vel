#include <string>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include "quaternion.hpp"
#include "norm.hpp"
#include "control_limits.hpp"

namespace jik {
    double poseTime;
    double twistTime;
    geometry_msgs::Pose desPose;
    double desHeading;
    geometry_msgs::Twist desTwist;
}

using namespace jik;

void poseUpdateCb(const geometry_msgs::PoseStamped::ConstPtr& poseMsg) {
    desPose.position.x = poseMsg->pose.position.x;
    desPose.position.y = poseMsg->pose.position.y;
    desPose.position.z = 0.0;
    desPose.orientation.w = poseMsg->pose.orientation.w;
    desPose.orientation.x = 0.0;
    desPose.orientation.y = 0.0;
    desPose.orientation.z = poseMsg->pose.orientation.z;
    desHeading = quaternion::quaternion_to_theta(desPose.orientation);
    poseTime = poseMsg->header.stamp.toSec();

    return;
}

void twistUpdateCb(const geometry_msgs::TwistStamped::ConstPtr& twistMsg) {
    desTwist.linear.x = twistMsg->twist.linear.x;
    desTwist.linear.y = twistMsg->twist.linear.y;
    desTwist.linear.z = 0.0;
    desTwist.angular.x = 0.0;
    desTwist.angular.y = 0.0;
    desTwist.angular.z = twistMsg->twist.angular.z;
    twistTime = twistMsg->header.stamp.toSec();

    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose2vel");
    ros::NodeHandle n("~");
    std::string poseTopicName, twistTopicName, tfRobotName, tfWorldName, pubTopicName;

    if(
        n.getParam("pose_topic_name", poseTopicName) &&
        n.getParam("twist_topic_name", twistTopicName) &&
        n.getParam("robot_frame_name", tfRobotName) &&
        n.getParam("world_frame_name", tfWorldName)
    ) { }
    else {
        ROS_ERROR("Not enough arguments");
        return 1;
    }

    if(n.getParam("pub_topic_name", pubTopicName)) { }
    else {
        pubTopicName = "/cmd_vel";
    }

    double kx, ky, ktheta;
    double ix, iy, itheta;

    n.param("pgain_x", kx, 0.4);
    n.param("pgain_y", ky, 0.6);
    n.param("pgain_theta", ktheta, 0.6);
    n.param("igain_x", ix, 0.0);
    n.param("igain_y", iy, 0.0);
    n.param("igain_theta", itheta, 0.0);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::Vector3 currLoc;
    geometry_msgs::Vector3 dLoc;
    geometry_msgs::Vector3 integralLoc;
    double currHeading = 0.0;
    double dHeading = 0.0, dHeading2 = 0.0;
    double integralDHeading = 0.0;
    double prevTime, currTime;
    geometry_msgs::Twist cmdVel;
    geometry_msgs::TransformStamped worldToRobot;

    ros::Publisher velPub = n.advertise<geometry_msgs::Twist>(pubTopicName, 100);
    ros::Subscriber poseSub = n.subscribe<geometry_msgs::PoseStamped>(poseTopicName, 100, poseUpdateCb);
    ros::Subscriber twistSub = n.subscribe<geometry_msgs::TwistStamped>(twistTopicName, 100, twistUpdateCb);

    ros::Rate rate(100.0);
    currTime = prevTime = ros::Time::now().toSec();
    while(ros::ok()) {
        cmdVel.linear.x = norm(desTwist.linear.x, desTwist.linear.y);
        cmdVel.linear.y = 0.0;
        cmdVel.linear.z = 0.0;
        cmdVel.angular.x = 0.0;
        cmdVel.angular.y = 0.0;
        cmdVel.angular.z = desTwist.angular.z;
        try {
            worldToRobot = tfBuffer.lookupTransform(tfWorldName, tfRobotName, ros::Time(0));
            currLoc.x = worldToRobot.transform.translation.x;
            currLoc.y = worldToRobot.transform.translation.y;
            currLoc.z = 0.0;
            currHeading = quaternion::quaternion_to_theta(worldToRobot.transform.rotation);

            prevTime = currTime;
            currTime = ros::Time::now().toSec();
            const double dt = currTime - prevTime;

            dHeading = currHeading - desHeading;
            while(1) {
                if(dHeading > M_PI) dHeading -= 2.0 * M_PI;
                else if(dHeading < -M_PI) dHeading += 2.0 * M_PI;
                else break;
            }


            const double c = cos(desHeading);
            const double s = sin(desHeading);
            dLoc.x = c * (currLoc.x - desPose.position.x) + s * (currLoc.y - desPose.position.y);
            dLoc.y = -s * (currLoc.x - desPose.position.x) + c * (currLoc.y - desPose.position.y);
            
            // integral, anti-windup
            integralDHeading += dHeading * dt;
            if(integralDHeading > maxIntegralDHeading) {
                integralDHeading = maxIntegralDHeading;
            }
            else if(integralDHeading < minIntegralDHeading) {
                integralDHeading = minIntegralDHeading;
            }
            integralLoc.x += dLoc.x * dt;
            integralLoc.y += dLoc.y * dt;
            if(integralLoc.x > maxIntegralDX) {
                integralLoc.x = maxIntegralDX;
            }
            else if(integralLoc.x < minIntegralDX) {
                integralLoc.x = minIntegralDX;
            }
            if(integralLoc.y > maxIntegralDY) {
                integralLoc.y = maxIntegralDY;
            }
            else if(integralLoc.y < minIntegralDY) {
                integralLoc.y = minIntegralDY;
            }

            cmdVel.linear.x -= (kx * dLoc.x + ix * integralLoc.x);
            cmdVel.angular.z -= (ky * dLoc.y + iy * integralLoc.y);
            cmdVel.angular.z -= (ktheta * dHeading + itheta * integralDHeading);

        }
        catch(tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
        velPub.publish(cmdVel);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
