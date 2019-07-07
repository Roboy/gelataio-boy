#ifndef HAND_PLANNER_HPP_
#define HAND_PLANNER_HPP_

#include <vector>
#include <ros/ros.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

class HandController
{
    public:
        /**
         * Constructor.
         * @param planning_group   Name of the planning group of the arm
         */
        HandController(std::string planning_group, int planning_attempts = 1);
        /**
         * Getter for current pose of the arm.
         * @return     Position and orientation of the current position.
         */
        geometry_msgs::PoseStamped getCurrentPose() {return this->m_move_group_ptr->getCurrentPose();}
        /**
         * Getter for planning group.
         * @return     Position and orientation of the current position.
         */
        std::string getPlanningGroupName() {return this->m_planning_group;}
        /**
         * Move to a position, while keeping orientation.
         * @param target_position   Target position
         * @return true if successful
         */
        bool moveToPosition(geometry_msgs::Point target_position);
        /**
         * Move to a position orientation, while keeping position.
         * @param target_orientation   Target orientation
         * @return true if successful
         */
        bool moveToOrientation(geometry_msgs::Quaternion target_orientation);
        /**
         * Move to a pose.
         * @param target_pose   Target pose
         * @return true if successful
         */
        bool moveToPose(geometry_msgs::Pose target_pose);
        /**
         * Move to pose.
         * @param target_pose   Target pose with frame id
         * @return true if successful
         */
        bool moveToPose(geometry_msgs::PoseStamped target_pose);
        /**
         * Move to poses.
         * @param targets   Vector of target poses
         * @return true if successful
         */
        bool moveToPoses(std::vector<geometry_msgs::Pose> &targets);
        /**
         * Move to a known pose.
         * @param pose_name   Name of known robot poses
         * @return true if successful
         */
        bool moveToKnownPose(std::string pose_name);
        void grasp();
        

    private:
        struct PlanningResult 
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            moveit::planning_interface::MoveItErrorCode planning_status;
        };

        PlanningResult plan();

        bool planAndExecute();

        moveit::planning_interface::MoveGroupInterface* m_move_group_ptr;
        
        std::string m_planning_group;
        
        int m_planning_attempts;
};

#endif