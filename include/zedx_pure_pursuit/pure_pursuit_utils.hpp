#pragma once

#include <cmath>
#include <string>
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For transforming ROS messages [use .hpp for humble]
#include "csvLoader.hpp"

// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*
This header file contains the utilities required for pure pursuit
*/

float euclideanDistance(double point1[], double point2[])
{
    /*
    Get the euclidean distance between two points
    */
   double dx = point2[0] - point1[0];
   double dy = point2[1] - point1[1];

   return std::sqrt(dx*dx + dy*dy);
}

void getLookAheadPt(double currLocation[], double nextLocation[], double lookAheadDist, 
                          std::vector<wayPoint> way_points)
{   
    /*
    Use the current location from the odometry and find the closest waypoint ahead
    to track
    */

    double minDistAhead = 0;
    int tagetPointtIdx = 0;

    // loop through the map locations
    for (unsigned long i=0; i < way_points.size(); i++)
    {   
        double mapPoint[2] = {0, 0};
        mapPoint[0] = way_points[i % way_points.size()].x;
        mapPoint[1] = way_points[i % way_points.size()].y;

        double distance = euclideanDistance(currLocation, mapPoint);

        if (0.9*lookAheadDist < distance && distance < 1.1*lookAheadDist)
        {
            if ((minDistAhead == 0 && (mapPoint[0]-currLocation[0]) > 0) || distance < minDistAhead)
            {
                tagetPointtIdx = i % way_points.size();
                minDistAhead = distance;
            }
        }
    }
    std::cout << tagetPointtIdx << std::endl;

    nextLocation[0] = way_points[tagetPointtIdx].x;
    nextLocation[1] = way_points[tagetPointtIdx].y;
}

void getLookAheadPt(
    geometry_msgs::msg::PointStamped curr_pt_world, 
    geometry_msgs::msg::PointStamped &next_pt_world, 
    geometry_msgs::msg::PointStamped &curr_pt_local, 
    geometry_msgs::msg::PointStamped &next_pt_local, 
    double lookahead_dist, std::vector<wayPoint> way_points, 
    geometry_msgs::msg::TransformStamped &t){

    double target_pt_dist = std::numeric_limits<double>::infinity();
    geometry_msgs::msg::PointStamped next_pt_world_temp;
    geometry_msgs::msg::PointStamped next_pt_local_temp;

    // get current point in local frame in the beginning
    try {
    tf2::doTransform(curr_pt_world, curr_pt_local, t);
    } catch (const tf2::TransformException &ex) {
    std::cout << "current point transformation failed" << std::endl;
    }

    // search in way_points for potential target
    for(auto way_point : way_points){
        double curr_pt_dist = std::sqrt(std::pow((way_point.x - curr_pt_world.point.x), 2) + std::pow((way_point.y - curr_pt_world.point.y), 2));
        // pass points inside lookahead distance
        if(curr_pt_dist < lookahead_dist)   continue;
        if(curr_pt_dist < target_pt_dist){
            //transform potential target to local frame
            try {
            next_pt_world_temp.point.x = way_point.x;
            next_pt_world_temp.point.y = way_point.y;
            tf2::doTransform(next_pt_world_temp, next_pt_local_temp, t);
            } catch (const tf2::TransformException &ex) {
            std::cout << "next point transformation failed" << std::endl;
            }
            
            // pass way_point that falls behind
            if(next_pt_local_temp.point.x <= curr_pt_local.point.x)   continue;

            // update selected next point in world frame
            target_pt_dist = curr_pt_dist;
            next_pt_world.point.x = curr_pt_world.point.x + (way_point.x - curr_pt_world.point.x) * lookahead_dist / curr_pt_dist;
            next_pt_world.point.y = curr_pt_world.point.y + (way_point.y - curr_pt_world.point.y) * lookahead_dist / curr_pt_dist;
        }
    }
    // transform final next point to local frame
    try {
    tf2::doTransform(next_pt_world, next_pt_local, t);
    } catch (const tf2::TransformException &ex) {
    std::cout << "next point transformation failed" << std::endl;
    }
    return;
}

