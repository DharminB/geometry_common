#ifndef GEOMETRY_COMMON_POSE_2D_H
#define GEOMETRY_COMMON_POSE_2D_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

#include <geometry_common/point.h>

namespace geometry_common
{

class Pose2d
{
    public:
        float x, y, theta;

        Pose2d(float x = 0.0f, float y = 0.0f, float theta = 0.0f):
            x(x), y(y), theta(theta) {};

        Pose2d(const Pose2d &pose):
            x(pose.x), y(pose.y), theta(pose.theta) {};

        Pose2d(const geometry_msgs::PoseStamped &pose);

        Pose2d(const geometry_msgs::Pose &pose);

        Pose2d(const std::vector<float>& mat);

        Pose2d(const tf::StampedTransform &stamped_transform);

        virtual ~Pose2d();

        geometry_msgs::PoseStamped getPoseStamped(std::string frame="map") const;

        geometry_msgs::Pose getPose() const;

        std::vector<float> getMat() const;

        visualization_msgs::Marker getMarker(std::string frame = "base_link",
                float red=1.0f, float green=0.0f, float blue=0.0f,
                float size_x=0.3, float size_y=0.05, float size_z=0.05) const;

        inline float getCartDist(const Pose2d& p) const
        {
            return sqrt(getCartDistSquared(p));
        };

        inline float getCartDistSquared(const Pose2d& p) const
        {
            return pow(x - p.x, 2) + pow(y - p.y, 2);
        };

        inline float getCartDist(const Point& p) const
        {
            return sqrt(pow(x - p.x, 2) + pow(y - p.y, 2));
        };

        static float getThetaFromQuaternion(const geometry_msgs::Quaternion& orientation);

        std::string str() const;

        friend Pose2d operator - (const Pose2d& p1, const Pose2d& p2);

        friend Pose2d operator * (const Pose2d& pose, float scalar);

        friend bool operator == (const Pose2d& p1, const Pose2d& p2);

        friend std::ostream& operator << (std::ostream &out, const Pose2d& pose_2d);
};

} // namespace geometry_common

#endif // GEOMETRY_COMMON_POSE_2D_H
