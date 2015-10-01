#pragma once

#include <unordered_map>
#include <vector>
#include <cmath>

namespace dijkstra
{
    struct point
    {
        point() : x(0), y(0) { };
        point(float x, float y) : x(x), y(y) { };

        float distance(const point& other) const
        {
            auto dx = (other.x - x);
            auto dy = (other.y - y);
            return sqrt(dx * dx + dy * dy);
        }

        const std::vector<point*>& get_connections() const
        {
            return connections;
        }
        std::vector<point*>& get_connections()
        {
            return connections;
        }

        void connect(point* other)
        {
            connections.push_back(other);
            other->connections.push_back(this);
        }

        friend bool operator==(const point& lhs, const point& rhs)
        {
            return lhs.x == rhs.x
                 & lhs.y == rhs.y;
        }

        float x;
        float y;

    private:
        std::vector<point*> connections;
    };

    class path : public std::vector<point*>
    {
    public:
        path() : cost(NAN) { };

        float get_cost() const
        {
            return cost;
        }

    private:
        float cost;
    };
}

namespace std {
    template <> struct hash<std::pair<dijkstra::point, dijkstra::point>>
    {
        size_t operator()(const std::pair<dijkstra::point, dijkstra::point>& var) const
        {
            return hash<float>()(var.first.x)
                 ^ hash<float>()(var.first.y)
                 ^ hash<float>()(var.second.x)
                 ^ hash<float>()(var.second.y)
                 ;
        }
    };
}


namespace dijkstra
{
    class map
    {
    public:
        map(std::vector<point>&& graph) { };

        void precompute(const std::vector<point>& points)
        {
            for (auto& p1 : points)
            {
                for (auto& p2 : points)
                {
                    find(p1, p2);
                }
            }
        }

        path& find(const point& start_approx, const point& goal_approx)
        {
            point start;
            point goal;

            // find the points already in the map which are the closest
            // to the approximate points
            for (auto& kvp : paths)
            {
                if (kvp.first.first.distance(start_approx) < eta)
                    start = kvp.first.first;
                if (kvp.first.second.distance(goal_approx) < eta)
                    goal = kvp.first.second;
            }

            // if it already exists: it's memoized, return it
            auto it = paths.find(std::make_pair(start, goal));
            if (it != paths.end())
                return it->second;

            // otherwise do some exploring
            return explore(start, goal);
        }

    private:
        path& explore(const point& start, const point& goal)
        {
            // do dijkstra shit and fill in paths
        }

        void insert(const point& start, const point& goal, const path& p)
        {
            paths[std::make_pair(start, goal)] = p;
        }

        std::unordered_map<std::pair<point, point>,
                           path> paths;

        static constexpr float eta = 0.1f;
    };
}