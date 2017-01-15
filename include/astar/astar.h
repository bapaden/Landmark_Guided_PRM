
#ifndef astar_h
#define astar_h

//Basics
#include <iostream>
#include <fstream>
#include <boost/random/mersenne_twister.hpp>
#include <algorithm>
//STL stuff
#include <vector>
#include <memory>
//Boost graph stuff
#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/astar_search.hpp>
#include <forward_list>
//Utilities
#include "../kd_tree/kdtree.h"
#include "../utils/utils.h"
#include "boost/date_time/posix_time/posix_time_types.hpp"




namespace ASTAR {
    
    typedef utils::numT numT;
    typedef utils::vctr vctr;
    typedef utils::path path;
    typedef kdtree::vertexPtr vertexPtr;
    typedef kdtree::vertex vertex;
    
    
    
    typedef PRM::ProbabilisticRoadMap::boost_graph boost_graph;
    typedef PRM::ProbabilisticRoadMap::vertex_t vertex_t;
    typedef PRM::ProbabilisticRoadMap::edge_t edge_t;
    typedef boost::graph_traits<boost_graph>::vertex_iterator vertex_i;
    typedef boost::graph_traits<boost_graph>::edge_iterator edge_i;
    //     typedef boost::graph_traits<boost_graph>::vertex_descriptor vertex_t;
    
    
    //*************************** LANDMARK CLASS **********************************
    class Landmark
    {
        
    public:
        vctr distance;
        std::vector<vertex_t> predecessor;
        vertex_t vertex;
        
        
        Landmark(vertex_t &v, u_int64_t num_vertices)
        {
            distance.resize(num_vertices);
            predecessor.resize(num_vertices);
            vertex = v;
        }
        
    }; // Landmark
    typedef std::shared_ptr<Landmark> landmark_ptr;
    
    
    
    //*************************** Landmark HEURISTICS CLASS **********************************
    class LandmarkHeuristics : public boost::astar_heuristic<boost_graph, numT>
    {
    private:
        int num_landmarks;
        u_int64_t num_vertices;
        PRM::ProbabilisticRoadMap *prm;
        landmark_ptr best_landmark;
        vertex_t goal_vertex;
        vertex_t start_vertex;
        
        void selectRandomLandmarks()    // add other methods later
        {
            boost::mt19937 random((int)time(0));
            //boost::mt19937 random;
            for (int i = 0; i<num_landmarks; i++)
            {
                vertex_t rand_vert = boost::random_vertex(prm->graph,random); //prm->coordToVertex({5, static_cast<float>((2+i*0.8))});
                landmark_ptr ptr(new Landmark(rand_vert,num_vertices));
                landmarks.push_back(ptr);
                
            }
            
        }
        
        void calculateDistances()
        {
            for (int i = 0; i<num_landmarks; i++)
            {
                boost::dijkstra_shortest_paths(prm->graph, landmarks[i]->vertex,
                                               predecessor_map(make_iterator_property_map(landmarks[i]->predecessor.begin(), get(boost::vertex_index, prm->graph))).
                                               distance_map(make_iterator_property_map(landmarks[i]->distance.begin(), get(boost::vertex_index, prm->graph))));
                
                //std::cout << "Dijkstra evaluating Landmark " << i+1 << std::endl;
            }
        }
        
        
    public:
        std::vector<landmark_ptr> landmarks; //vector of pointers to landmarks
        
        LandmarkHeuristics(int num_landmarks_, PRM::ProbabilisticRoadMap *G)
        {
            num_landmarks = num_landmarks_;
            num_vertices = boost::num_vertices(G->graph);
            prm = G;
            
            selectRandomLandmarks();
            calculateDistances();           //calculate distances from all landmarks to all nodes
        }
        
        void findBestLandmark(vertex_t start, vertex_t goal)
        {
            int index = 0;
            for(int i = 0; i < num_landmarks; i++)
            {
                if(std::abs(landmarks[i]->distance[goal] - landmarks[i]->distance[start]) > std::abs(landmarks[index]->distance[goal] - landmarks[index]->distance[start]))
                    index = i;
            }
            
            best_landmark = landmarks[index];
            goal_vertex = goal;
        }
        
        void setGoal(vertex_t goal)
        {
            goal_vertex = goal;
        }
        
        //Calulate actual heuristic:
        numT operator()(vertex_t v)
        {
            //TEST
            numT best = 0.0;
            for(int i = 0; i < num_landmarks; i++)
            {
                best = std::max(best,std::abs(landmarks[i]->distance[goal_vertex] - landmarks[i]->distance[v]));
            }
           return best;
            
           //return std::abs(best_landmark->distance[v]-best_landmark->distance[goal_vertex]);
        }
        
        numT returnHeuristic(vertex_t v)
        {
            numT best = 0.0;
            for(int i = 0; i < num_landmarks; i++)
            {
                best = std::max(best,std::abs(landmarks[i]->distance[goal_vertex] - landmarks[i]->distance[v]));
            }
            return best;
            
           // return std::abs(best_landmark->distance[v]-best_landmark->distance[goal_vertex]);
            
        }
        
        void displayData()
        {
            vertex_i vi,vi_end;
            vertex_t v;
            std::cout << std::endl << std::endl;
            for (int i = 0; i<num_landmarks; i++)
            {
                std::cout << "distances and parents of Landmark nr. " << i << ", which refers to vertex " << landmarks[i]->vertex << std::endl;
                for (boost::tie(vi, vi_end) = boost::vertices(prm->graph); vi != vi_end; ++vi)
                {
                    v = *vi;
                    //std::cout << "Color: " << color[v] << std::endl;
                    std::cout << "distance(" << v << ") = " << landmarks[i]->distance[v] << ", ";
                    std::cout << "parent(" << v << ") = " << landmarks[i]->predecessor[v] << std::
                    endl;
                    
                }
                std::cout << std::endl;
                
            }
        }
        
        void landmarkToFile()
        {
            std::ofstream landmark;
            landmark.open("../python/landmark.csv");
            for(int i=0; i<prm->environment->dimension()-1; i++)
            {
                    landmark << prm->graph[best_landmark->vertex]->coord[i] << ",";
            }
            landmark << prm->graph[best_landmark->vertex]->coord[prm->environment->dimension()-1] << std::endl;
            
            for(int i=0;i<landmarks.size();i++)
            {
                for(int d=0; d<prm->environment->dimension()-1; d++)
                {
                    landmark << prm->graph[landmarks[i]->vertex]->coord[d] << ",";
                }
                landmark << prm->graph[landmarks[i]->vertex]->coord[prm->environment->dimension()-1] << std::endl;
            }
           
            landmark.close();
        }
        
        
        ~LandmarkHeuristics() {};
        
        
    }; // Heuristics
    
    //*************************** Euclidian HEURISTICS CLASS **********************************
    class EuclidianHeuristics : public boost::astar_heuristic<boost_graph, numT>
    {
    private:
        
        PRM::ProbabilisticRoadMap *G_ptr;
        vertex_t goal_vertex;
        
    public:
        
        EuclidianHeuristics(PRM::ProbabilisticRoadMap *G_ptr_, vertex_t goal)
        {
            G_ptr = G_ptr_;
            goal_vertex = goal;
            
        }
        
        numT operator()(vertex_t v)
        {
            return utils::norm(utils::vec_diff(G_ptr->graph[goal_vertex]->coord, G_ptr->graph[v]->coord));
        }
        
        ~EuclidianHeuristics() {};
        
        
    }; // EuclidianHeuristics
    
    //*************************** NO HEURISTICS CLASS **********************************
    class NoHeuristics : public boost::astar_heuristic<boost_graph, numT>
    {
        
    public:
        
        NoHeuristics() {}
        
        numT operator()(vertex_t v)
        {
            return  0;
        }
        
        ~NoHeuristics() {};
        
    }; // EuclidianHeuristics
    
    //*************************** VISITOR CLASS **********************************
    class dijkstra_visitor : boost::default_bfs_visitor
    {
    public:
        dijkstra_visitor() {};
        
        void initialize_vertex(const vertex_t &s, const boost_graph &g) const {}
        void discover_vertex(const vertex_t &s, const boost_graph &g) const {}
        void examine_vertex(const vertex_t &s, const boost_graph &g) const {}
        void examine_edge(const edge_t &e, const boost_graph &g) const {}
        void edge_relaxed(const edge_t &e, const boost_graph &g) const {}
        void edge_not_relaxed(const edge_t &e, const boost_graph &g) const {}
        void finish_vertex(const vertex_t &s, const boost_graph &g) const {}
    };
    
    struct found_goal {};
    
    class astar_visitor : public boost::default_astar_visitor
    {
    private:
        vertex_t goal_vertex;
        
    public:
        u_int32_t* vertices_expanded;
        astar_visitor(u_int32_t* vertices_expanded_)
        {
            vertices_expanded = vertices_expanded_;
        }
        
        void set_goal(vertex_t goal_vertex_)
        {
            goal_vertex = goal_vertex_;
        }
        
        void examine_vertex(const vertex_t &u, const boost_graph &g)
        {
            *vertices_expanded = *vertices_expanded + 1;
            if (u == goal_vertex)
                throw found_goal();
        }
        
    };
    
    
    
    
    
    //**************************** ASTAR CLASS ***********************************
    class astarSearcher
    {
    private:
        boost::posix_time::ptime start_time;
        boost::posix_time::ptime stop_time;
        
    public:
        u_int32_t vertices_expanded;
        std::vector<vertex_t> predecessor;
        std::vector<numT> cost;
        std::vector<vertex_t> shortest_path;
        PRM::ProbabilisticRoadMap* prm;
        bool path_found;
        
        template<typename Heuristic>
        astarSearcher(PRM::ProbabilisticRoadMap* G_ptr, Heuristic* heuristic, vertex_t start_vertex, vertex_t goal_vertex)
        {
            vertices_expanded=0;
            astar_visitor vis(&vertices_expanded);
            vis.set_goal(goal_vertex);
            cost.resize(boost::num_vertices(G_ptr->graph));
            predecessor.resize(boost::num_vertices(G_ptr->graph));
            prm = G_ptr;
            
            try
            {
                start_time = boost::posix_time::microsec_clock::local_time();
               // std::cout << std::endl << "Running Astar" << std::endl;
                boost::astar_search(G_ptr->graph, start_vertex, *heuristic,
                                    predecessor_map(make_iterator_property_map(predecessor.begin(), get(boost::vertex_index, G_ptr->graph))).
                                    distance_map(make_iterator_property_map(cost.begin(), get(boost::vertex_index, G_ptr->graph))).
                                    visitor(vis));
                //throw found_goal();
            }
            
            
            catch(found_goal fg)
            {
                path_found = true;
                stop_time = boost::posix_time::microsec_clock::local_time();
                // Walk backwards from the goal through the predecessor chain adding
                // vertices to the solution path.
                for (vertex_t u = goal_vertex; u != start_vertex; u = predecessor[u])
                {
                    shortest_path.push_back(u);
                }
                shortest_path.push_back(start_vertex);
                std::reverse(shortest_path.begin(),shortest_path.end());
                //std::cout << std::endl << "SHORTEST PATH FOUND" << std::endl;
                return;
                
            }
            
            std::cout << std::endl << "NO SOLUTION FOUND" << std::endl;
            path_found = false;
            
        }
        
        void displayData()
        {
            if(!shortest_path.empty()){
                std::cout << std::endl << std::endl;
                std::cout << "Start Vertex: " << shortest_path.front() << std::endl;
                std::cout << "Goal Vertex:  " << shortest_path.back() << std::endl << std::endl;
//                 std::cout << "Shortest path: " << std::endl;
//                 for(auto& p : shortest_path)
//                 {
//                     std::cout << p << " -> ";
//                     
//                 }
//                 std::cout << "\n\n";
            }
            
            boost::posix_time::time_duration duration = stop_time-start_time;
            std::cout << std::endl << "Vertices Expanded: " << vertices_expanded << std::endl << std::endl;
            std::cout << "Query Time: " << duration.ticks()<< std::endl;
            
        }
        
        void pathToFile()
        {
            std::ofstream path;
            path.open("../python/path.csv");
            vertexPtr v;
            if(!shortest_path.empty()){
                for(auto& p : shortest_path)
                {
                    v=prm->graph[p];
                    for(int i=0;i<prm->environment->dimension()-1;i++)
                    {
                        path << v->coord[i] << ",";
                    }
                    path << v->coord[prm->environment->dimension()-1] << std::endl;
                }
            }
            path.close();
            
        }
        
        void pathToFile(const std::string& name)
        {
            std::ofstream out;
            std::string path("../python/");
            out.open(path+name);
            vertexPtr v;
            if(!shortest_path.empty()){
                for(auto& p : shortest_path)
                {
                    v=prm->graph[p];
                    for(int i=0;i<prm->environment->dimension()-1;i++)
                    {
                        out << v->coord[i] << ",";
                    }
                    out << v->coord[prm->environment->dimension()-1] << std::endl;
                }
            }
            out.close();
            
        }
        
        
        void verticesToFile(const std::string& name)
        {
            std::ofstream vertices;
            std::string path("../python/");
            vertices.open(path+name);
            for(int a=0; a<cost.size(); a++)
            {
                if(cost[a] < std::numeric_limits<numT>::max())
                {
                    for(int i=0; i<prm->environment->dimension(); i++)
                    {
                        vertices << prm->graph[a]->coord[i] << ",";
                    }
                    vertices << cost[a] << std::endl;
                }
            }
            vertices.close();
        }
        
        
        template<typename Heuristic>
        void verticesToFile(Heuristic h,const std::string& name)
        {
            std::ofstream vertices;
            std::string path("../python/");
            vertices.open(path+name);
            for(int a=0; a<cost.size(); a++)
            {
                if(cost[a] < std::numeric_limits<numT>::max())
                {
                    for(int i=0; i<prm->environment->dimension(); i++)
                    {
                        vertices << prm->graph[a]->coord[i] << ",";
                    }
                    vertices << cost[a]+h->returnHeuristic(a) << std::endl;
                }
            }
            vertices.close();
        }
        
        
        
        float efficiency()
        {
            return (float)shortest_path.size()/vertices_expanded;
        }
        
        long long runtime()
        {
            boost::posix_time::time_duration msdiff = stop_time - start_time;
            return msdiff.total_microseconds();
        }
    
    }; //astar
   
    
} //namespace





#endif
