#pragma once

#include <unordered_map>
#include <map>
#include <vector>
#include <set>
#include <algorithm>
#include <cmath>

namespace dijkstra
{
    class point;
}

namespace std {
    template <> struct hash<std::pair<dijkstra::point const*, dijkstra::point const*>>
    {
        size_t operator()(const std::pair<dijkstra::point const*,
                                          dijkstra::point const*>& var) const
        {
            return hash<dijkstra::point const*>()(var.first)
                ^ (hash<dijkstra::point const*>()(var.second) << 16);
        }
    };
}

namespace dijkstra
{
    struct point
    {
        point() : x(0), y(0) { };
        point(float x, float y) : x(x), y(y) { };

        float distance(point const* other) const
        {
            auto dx = (other->x - x);
            auto dy = (other->y - y);
            return sqrt(dx * dx + dy * dy);
        }
        float distance(const point& other) const
        {
            return distance(&other);
        }

        const std::vector<point*>& get_links() const
        {
            return links;
        }
        std::vector<point*>& get_links()
        {
            return links;
        }

        void connect(point* other)
        {
            links.push_back(other);
            other->links.push_back(this);
        }

        friend bool operator==(const point& lhs, const point& rhs)
        {
            return lhs.x == rhs.x
                 & lhs.y == rhs.y;
        }

        friend std::ostream& operator<<(std::ostream& os, const point& p)
        {
            os << "(" << p.x << ", " << p.y << ")";
            return os;
        }

        float x;
        float y;

    private:
        std::vector<point*> links;
    };

    class path : public std::vector<point const*>
    {
    public:
        path()
            : cost(NAN) { };

        void compute_length()
        {
            cost = 0;
            for (iterator it = begin(); (it + 1) != end(); ++it)
            {
                cost += (*it)->distance(*(it + 1));
            }
        }

        float length() const
        {
            return cost;
        }

    private:
        float cost;
    };

    class priority_compare
    {
    public:
        bool operator()(const std::pair<point const*, float>& a,
                        const std::pair<point const*, float>& b)
        {
            if (b.second < a.second)
                return true;
            if (a.second < b.second)
                return false;

            if (b.first < a.first)
                return true;
            return false;
        }
    };

    class map
    {
    public:
        map(std::vector<point>&& graph) : graph(graph) { };
        map(const std::vector<point>& graph) : graph(graph) { };

        void connect(unsigned int a, unsigned int b)
        {
            graph[a].connect(&graph[b]);
            graph[b].connect(&graph[a]);
        }

        void precompute(const std::vector<point>& points)
        {
            for (auto& p1 : points)
            {
                auto start = closest(p1);
                std::vector<point const*> goals;
                goals.reserve(points.size());
                std::transform(points.begin(), points.end(),
                               std::back_inserter(goals),
                               [&](const point& x) { return closest(x); });

                explore(start, goals);
            }
        }

        path& find(const point& start_approx, const point& goal_approx)
        {
            auto start = closest(start_approx);
            auto goal = closest(goal_approx);

            // if it already exists: it's memoized, return it
            auto it = paths.find(std::make_pair(start, goal));
            if (it != paths.end())
            {
                std::cout << "free!" << std::endl;
                return it->second;
            }

            std::cout << "exploring" << std::endl;
            // otherwise do some exploring
            explore(start, {goal});

            // entry should now exist
            return paths[std::make_pair(start, goal)];
        }

    private:
        point const* closest(const point& p_approx)
        {
            point const* p = nullptr;
            float min_p = INFINITY;

            for (auto& node : graph)
            {
                float dist = p_approx.distance(node);
                if (dist < min_p)
                {
                    min_p = dist;
                    p = &node;
                }
            }

            return p;
        }

        void explore(point const* start, std::vector<point const*> goals)
        {
            std::set<std::pair<point const*, float>, priority_compare> unvisited;
            std::unordered_map<point const*, float> visited;
            std::unordered_map<point const*, point const*> previous;

            unvisited.emplace(start, 0);
            visited.emplace(start, 0);

            while (!unvisited.empty())
            {
                auto current = unvisited.begin();

                for (auto link : current->first->get_links())
                {
                    //TODO: check if memoized first

                    if (visited.find(link) != visited.end())
                        continue;

                    auto tentative = current->second + current->first->distance(link);
                    auto it = std::find_if(unvisited.begin(), unvisited.end(),
                        [&](std::pair<point const*, float> kvp) {
                            return kvp.first == link;
                        });
                    if (it != unvisited.end())
                    {
                        if (tentative < it->second)
                        {
                            unvisited.emplace(link, tentative);
                            previous[link] = current->first;
                        }
                    }
                    else
                    {
                        unvisited.emplace(link, tentative);
                        previous[link] = current->first;
                    }

                    visited[link] = current->second;

                    auto goals_it = std::find(goals.begin(), goals.end(), link);
                    if (goals_it != goals.end())
                    {
                        goals.erase(goals_it);

                        if (goals.size() == 0)
                        {
                            insert_all(visited, previous);
                            return;
                        }
                    }
                }
                unvisited.erase(current);
            }
        }

        void insert_all(const std::unordered_map<point const*, float>& visited,
                        const std::unordered_map<point const*, point const*>& previous)
        {
            for (auto& node : visited)
            {
                path p;
                p.push_back(node.first);

                auto it = previous.end();
                while ((it = previous.find(p.front())) != previous.end())
                {
                    // TODO: don't recompute full path
                    // maybe incorporate into push_back and add a pop_front
                    p.insert(p.begin(), it->second);
                    p.compute_length();
                    insert(p);

                    path p2 = p;
                    while (p2.size() > 2)
                    {
                        p2.pop_back();
                        p2.compute_length();
                        insert(p2);
                    }
                }
            }
        }

        void insert(const path& p)
        {
            paths[std::make_pair(p.front(), p.back())] = p;
            paths[std::make_pair(p.back(), p.front())] = p;
        }

        std::vector<point> graph;
        std::unordered_map<std::pair<point const*, point const*>,
                           path> paths;
    };
}