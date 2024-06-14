#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <mutex>
#include <vector>
#include <queue>
#include <algorithm>
#include <limits>
#include <cmath>

namespace frontier_exploration {
    // Represents a frontier
    struct Frontier {
        std::uint32_t size; // number of points
        double minDistance; // minimum distance from robot position
        double cost;
        geometry_msgs::Point initial;
        geometry_msgs::Point centroid;
        geometry_msgs::Point middle;
        std::vector<geometry_msgs::Point> points;
    };

    class FrontierSearch {
        // Represents state of a cell in costmap during frontier search
        enum CellState {
            // Enqueued by outer BFS
            MAP_OPEN,
            // Dequeued by outer BFS
            MAP_CLOSED,
            // Enqueued by inner BFS
            FRONTIER_OPEN,
            // Dequeued by inner BFS
            FRONTIER_CLOSED,
            // Cell hasn't been checked yet
            UNCHECKED
        };

    public:
        FrontierSearch() = default;

        FrontierSearch(costmap_2d::Costmap2D* costmap, double potentialScale, double gainScale, double minFrontierSize);

        std::vector<Frontier> searchFrom(const geometry_msgs::Point& position);

    protected:
        Frontier buildFrontier(size_t initialCell, size_t reference, std::vector<CellState>& cellStates);
        bool isFrontierCell(size_t idx);
        double frontierCost(const Frontier& frontier);

    private:
        costmap_2d::Costmap2D* _costmap;
        uint8_t* _map;
        size_t _sizeX, _sizeY;
        double _potentialScale, _gainScale;
        double _minFrontierSize;
        bool objectDetected;
        ros::Subscriber objectDetectedSub;

        void objectDetectedCallback(const std_msgs::Bool::ConstPtr& msg);
    };
} // namespace frontier_exploration

#endif // FRONTIER_SEARCH_H_



