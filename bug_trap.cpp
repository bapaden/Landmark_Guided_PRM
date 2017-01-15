#include <iostream>
#include <cstdlib>
#include <vector>
#include "include/PRM/graphPRM.h"
#include "include/utils/utils.h" 
#include "include/astar/astar.h"
#include <boost/graph/random.hpp>

int main(int argc, char **argv) 
{
    std::cout << "Running the bug trap demo..." << std::endl;
    
    //k-dimensional config. space
    int space_dim(2);
    int graph_size(100000);
    int num_landmarks(100);
    //Create a k dimensional environment with default size
    PRM::bugTrap* E = new PRM::bugTrap(50);
    //initialize PRM graph with environment E
    PRM::ProbabilisticRoadMap G(E);
    
    //Create a PRM on the environment
    G.buildGraph(graph_size);  
    //Save the graph to a csv file
    G.write_to_file();
    
    std::cout << "Number of distance evals: " << G.kd_tree->d_count << std::endl;
    
    //Query
    
    PRM::vctr start({5.0,4.0});
    PRM::vctr goal({5.0,1.0});
    std::pair<PRM::ProbabilisticRoadMap::vertex_t,PRM::ProbabilisticRoadMap::vertex_t> query = G.path_query(start,goal);
    
    //Landmark A*
    std::cout << "\nShortest path query with landmark heuristic " << std::endl;
    ASTAR::LandmarkHeuristics LH(num_landmarks,&G);
    LH.setGoal(query.second);
    ASTAR::astarSearcher A1(&G,&LH,query.first,query.second);
    A1.displayData();
    A1.pathToFile("landmark_shortest_path.csv");
    A1.verticesToFile("landmark_vertices_visited.csv");
    
    //Euclidean Distance A*
    std::cout << "\nShortest path query with Euclidean distance heuristic " << std::endl;
    ASTAR::EuclidianHeuristics EH(&G,query.second);
    ASTAR::astarSearcher A2(&G,&EH,query.first,query.second);
    A2.displayData();
    A2.pathToFile("euclidean_shortest_path.csv");
    A2.verticesToFile("euclidean_vertices_visited.csv");    
   
    //Dijkstra 
    std::cout << "\nShortest path query with a uniform cost search" << std::endl;
    ASTAR::NoHeuristics NH;
    ASTAR::astarSearcher A3(&G,&NH,query.first,query.second);
    A3.displayData();
    A3.pathToFile("dijkstra_shortest_path.csv");
    A3.verticesToFile("dijkstra_vertices_visited.csv");    
    
    return 0;
}
