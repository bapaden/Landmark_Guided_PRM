#ifndef GRAPHPRM_H
#define GRAPHPRM_H

//Boost graph stuff
#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
//Utilities
#include "../kd_tree/kdtree.h"
#include "environment.h"

namespace PRM{
    
    typedef kdtree::vertexPtr vertexPtr;
    typedef kdtree::vertex vertex;
   
    class ProbabilisticRoadMap
    {
    public:
        //~~~~~~~~~~~~~~~~~~~~~~~typedefs for Boost stuff~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
        //This lets us use numTs for the edge weight
        typedef typename  boost::property<boost::edge_weight_t, numT> EdgeWeightProperty;
        
        //Typedef a boost undirected graph using our vertex class
        typedef typename boost::adjacency_list<boost::listS, 
        boost::vecS, 
        boost::undirectedS,
        vertexPtr,
        EdgeWeightProperty> boost_graph;
        
        //These are the edge and vertex types
        typedef boost::graph_traits<boost_graph>::vertex_descriptor vertex_t;
        typedef boost::graph_traits<boost_graph>::edge_descriptor edge_t;
        typedef boost::graph_traits<boost_graph>::vertex_iterator vertex_i;//TODO make sure not defined elsewhere
        typedef boost::graph_traits<boost_graph>::edge_iterator edge_i;
        //~~~~~~~~~~~~~~~~~~~~~~~Boost typedefs~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
        
        //Member objects
        boost_graph graph;
        Environment* environment;
        std::map<vertex,vertex_t> inverse_map;
        
        const numT euler = 2.71828182845904523536;
        long unsigned int n;
        kdtree::Kdtree* kd_tree;
        ProbabilisticRoadMap(Environment* _environment)
        {
            environment=_environment;
            kd_tree = new kdtree::Kdtree(environment->dimension());
        }
        //takes coordinates of a query and returns graph indices of nearest points in graph
        std::pair<vertex_t,vertex_t> path_query(const vctr& _start, const vctr& _goal)
        {
            kdtree::query_results<vertexPtr,numT> start = kd_tree->query(_start,1);
            kdtree::query_results<vertexPtr,numT> goal = kd_tree->query(_goal,1);
            
            vertexPtr s_vtx = start.BPQ.queue.top().vtx_ptr;
            vertexPtr g_vtx = goal.BPQ.queue.top().vtx_ptr;
            
            std::pair<vertex_t,vertex_t> out;
            out.first = inverse_map[*s_vtx];
            out.second = inverse_map[*g_vtx];
            
            return out;
        }
        
        void insertPoint(const vctr& point)
        {
            //make sure the point is in the free space
            if(not environment->collisionFreePoint(point)){return;}
                
            //create a vertex for the new point
            vertexPtr newVertex(new vertex(point));

            //find m nearest neighbors to point where m determined by PRM*
            int d = (numT) boost::num_vertices(graph);
            int m = std::max((int)1, (int)(1.0*(1.0+1.0/environment->dimension())*euler*log(d)) );
            kdtree::query_results<vertexPtr,numT> neighbors = kd_tree->query(newVertex->coord,m);
            
            //now insert the point into the kdtree 
            kd_tree->insert(newVertex);

            //create a boost vertex and add to the graph
            vertex_t v=boost::add_vertex(graph);

            //assign the property of v to newVertex
            graph[v]=newVertex;
            inverse_map[*newVertex]=v;//TODO fix this HACK 
            
            assert(point.size()==2);

            //This attaches an edge to each of the nearest neighbors of newVertex
            int count = 0;
            kdtree::query_node<vertexPtr,numT> neighbor;
            while(count<m and (not neighbors.BPQ.queue.empty()))
            {
                count++;
                neighbor = neighbors.BPQ.queue.top();
                neighbors.BPQ.queue.pop();
                //only if the path is collision free
                if(environment->collisionFreeLine(newVertex->coord,neighbor.vtx_ptr->coord))
                {
                    
                    edge_t edge; 
                    bool b;
                    //for now this is the edge weight TODO: more general cost function
                    numT length = utils::norm( utils::vec_diff(newVertex->coord,neighbor.vtx_ptr->coord) );
                    
                    //retrieve the vertices based on the property value ...HACK
                    vertex_t v1 = inverse_map[*newVertex];
                    vertex_t v2 = inverse_map[*(neighbor.vtx_ptr)];
                    boost::tie(edge,b) = boost::add_edge(v1,v2,length,graph);
                }
            }
            
            return;
        }
        
        void buildGraph(const unsigned int numVertices)
        {
            vctr point;
            for(unsigned int i=0;i<numVertices;i++)
            {
                //assign a random value to point
                environment->random->sample(point);
                //insert point into the graph
                insertPoint(point);
            }
                std::cout << "Built PRM* graph with " << boost::num_vertices(graph) << " vertices," << std::endl;
                std::cout << "and                   " << boost::num_edges(graph) << " edges" << std::endl;
            
        }
        
        void write_to_file()
        {
            
            std::ofstream source;
            std::ofstream sink;
            source.open ("../python/source.csv");
            sink.open("../python/sink.csv");
            
            edge_i ei, ei_end;
            
            for (boost::tie(ei, ei_end) = boost::edges(graph); ei != ei_end; ++ei) 
            { 
                edge_t e = *ei;
                vertex_t u = boost::source(e, graph);
                vertex_t v = boost::target(e, graph);

                for(int i=0;i<environment->dimension()-1;i++)
                {
                source << graph[u]->coord[i] << ",";
                sink << graph[v]->coord[i] << ",";
                }
                //don't add a comma to the last one
                source << graph[u]->coord[environment->dimension()-1] << std::endl;
                sink << graph[v]->coord[environment->dimension()-1] << std::endl;
            }
            source.close();
            sink.close();
            
        }
        
        vertex_t coordToVertex(vctr vector)
        {
            kdtree::query_results<vertexPtr,numT> result = kd_tree->query(vector,1);
            return inverse_map[*result.BPQ.queue.top().vtx_ptr];
        }
        
    };//graphclass
    
    
}//namespace

#endif


