#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

//Basics
#include <iostream>
#include <fstream>
#include <time.h>
#include <random>
//STL stuff
#include <vector>
#include <memory>
//Utilities
#include "../utils/utils.h"
#include "geometry.h"



namespace PRM{
    
    typedef utils::numT numT;
    typedef utils::vctr vctr;
    typedef utils::path path;
    
    
    //~~~~~~~~~~~~~~~~~~~Random Sample Class~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
    class RandomPointGenerator
    {
    public:
        int dimension;
        vctr box_size;
        //Generate a random point in _dimension dimensions in a box of _box_size
        RandomPointGenerator(int& _dimension, vctr& _box_size)
        {
            srand(time(NULL));
            dimension=_dimension;
            box_size=_box_size;
        }
        
        void sample(vctr& point)
        {
            point.clear();
            for(int i=0;i<dimension;i++)
            {
                point.push_back(box_size[i]*( (numT)rand() / (RAND_MAX + 1.0)));
            }
        }
        
        vctr sample()
        {
            vctr point(0);
            for(int i=0;i<dimension;i++)
            {
                point.push_back(box_size[i]*( (numT)rand() / (RAND_MAX + 1.0)));
            }
            return point;
        }
        
    };
    //~~~~~~~~~~~~~~~~~~~~~~~Random Sample Class~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
    
    
    //~~~~~~~~~~~~~~~~~~~~~~~Collision Detection~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
    class Environment
    {
        
        int space_dimension;
    public:
        vctr dimensions;
        RandomPointGenerator* random;
        
        Environment(const int& _space_dimension, vctr _dimensions)
        
        {
            space_dimension=_space_dimension;
            dimensions=_dimensions;
            random = new RandomPointGenerator(space_dimension,dimensions);
            
            return;
        }
        
        Environment(const int& _space_dimension)
        
        {
            space_dimension=_space_dimension;
            dimensions.resize(space_dimension);
            for (auto& l:dimensions){l = 10.0;}
            random = new RandomPointGenerator(space_dimension,dimensions);
            
            return;
        }
        
        int dimension()
        {
            return space_dimension;
        }
        
        void resize(const vctr& size)
        {
            space_dimension=size.size();
            dimensions=size;
            random->box_size=size;
            random->dimension=space_dimension;
        }
        
        ~Environment()
        {
            delete random;
        }
        
        //TODO Abstract point collision checking
        virtual bool collisionFreePoint(const vctr& point)=0;
        
        
        virtual bool collisionFreeLine(const vctr& pointA, const vctr& pointB)=0;
        
        
    };
    
    class Open2DEnvironment : public Environment
    {
    public:
        Open2DEnvironment() : Environment(2)
        {
            return;
        }
        
        bool collisionFreePoint(const vctr& point){return true;}
        bool collisionFreeLine(const vctr& pointA, const vctr& pointB){return true;}
        
    };
    
    
    const std::vector<numT> Random2Ddimensions({1.0,1.0});
    class Random2DEnvironment : public Environment
    {
        std::vector<geo::sphere*> obs;
        int res;
        numT density;
    public:
        
        
        //visibility is prob. a random line is coll free, second argument is line discretization
        Random2DEnvironment(numT visibility,int _res) : Environment(2,Random2Ddimensions), obs(0)
        {
            numT radius = 0.05;
            res=_res;
            
            vctr n({0.0,2.0,2.34420459506696, 2.74764759176653, 3.22052405512188, 3.77478364427019, 4.42443258214090, 5.18588759480934, 6.07839076462640, 7.12449578052488, 8.35063787312080, 9.78780183695499, 11.4723050208974, 13.4467150729987, 15.7609256313398, 18.4734171437477, 21.6527346774811, 25.3792200633584, 29.7470421458702, 34.8665764439998, 40.8671943571388, 47.9005323997497, 56.1443240788235, 65.8068912462534, 77.1324084232695, 90.4070731272049, 105.966338125674, 124.203388378312, 145.579076879663, 170.633570483456, 200});
            vctr vis({1,0.888130290725293,0.870394393930517,0.850134146051528,0.827090192953850,0.801011686804170,0.771671151157411,0.738883786750890,0.702531381926033,0.662590410841997,0.619163003075042,0.572508227465161,0.523069617516020,0.471493298175355,0.418629889021761,0.365513219262306,0.313310585914735,0.263243455302663,0.216484164143025,0.174042179223557,0.136660282901294,0.104743258407416,0.0783365380346228,0.0571599346814001,0.0406860760403166,0.0282412210196583,0.0191031821378836,0.0124372969017211,  0.00402272843267364, 5.54950462863234e-05, 3.07933829396233e-09});
      
//             std::reverse(n.begin(),n.end());
//             std::reverse(vis.begin(),vis.end());    
            
            int iter=1;
            if(visibility>=1.0)
            {
                density=0;
            }
            else
            {
                //Linear interpolation from table
                while(vis[iter]>visibility){iter++;}
                std::cout << "vis[iter]: " << vis[iter] << std::endl;
                std::cout << "vis[iter-1]: " << vis[iter-1] << std::endl;
                numT frac = (visibility-vis[iter-1])/(vis[iter]-vis[iter-1]);
                std::cout << "frac " << frac << std::endl;
                //intensity of the poisson point process
                density = (frac*n[iter]+(1.0-frac)*n[iter-1]);
                std::cout << "Density: " << density << " obs/area " << std::endl;
            }
            
            //Sample number of obstacles from poisson distribution with density*area intensity
            unsigned seed1 = std::chrono::system_clock::now().time_since_epoch().count();
            std::mt19937 generator(seed1);
            //Area is 4
            std::poisson_distribution<int> poisson(4.0*density);
            int num_obs = poisson(generator);
            //create a poisson forest around the sample region
            for(int i=0;i<num_obs;i++)
            {
                vctr s = random->sample();
                s[0]=2.0*s[0]-0.5;
                s[1]=2.0*s[1]-0.5;
                obs.push_back( new geo::sphere(s, radius) );
            }
            std::cout << "Number of obs: " << obs.size() << std::endl;
            return;
        }
        
        bool collisionFreePoint(const vctr& point)
        {
            for(int i=0;i<obs.size();i++)
            {
                if(obs.at(i)->in_region(point))
                {
                    return false;
                }
            }
            return true;
        }
        bool collisionFreeLine(const vctr& pointA, const vctr& pointB)
        {
            vctr p(pointA.size());
            numT l;
            for(int i=0;i<res;i++)
            {
                l=(numT)i/(numT)(res-1);
                for(int j=0;j<pointA.size();j++)
                {
                    p[j]=l*pointA[j]+(1.0-l)*pointB[j];
                }
                if( not collisionFreePoint(p) )
                {
                    return false;
                }
            }
            return true;
            
        }
        void obsToFile()
        {
            std::ofstream centers;
            centers.open("../python/obstacles.csv");
            for(int a=0; a<obs.size(); a++)
            {
                
                for(int i=0; i<dimension()-1; i++)
                {
                    centers << obs[a]->center[i] << ",";
                }
                centers << obs[a]->center[dimension()-1] << std::endl;
                
            }
            centers.close();
        }
        
    };
    
    class bugTrap : public Environment
    {
        int res;
        geo::hyperRectangle* r1;
        geo::hyperRectangle* r2;
        geo::hyperRectangle* r3;
        geo::hyperRectangle* r4;
        geo::hyperRectangle* r5;
        geo::hyperRectangle* r6;
        
        
    public:
        bugTrap(int _res) : Environment(2)
        {
            std::vector<numT> dim({10.0,10.0});
            res=_res;
            resize(dim);
            
            //lower obstacle
            vctr c1({5.0,1.5});
            vctr d1({8.0,1.0});
            r1 = new geo::hyperRectangle(c1,d1);
            
            //left obstacle
            vctr c2({1.5,5.0});
            vctr d2({1.0,8.0});
            r2 = new geo::hyperRectangle(c2,d2);
            
            //left obstacle
            vctr c3({8.5,5.0});
            vctr d3({1.0,8.0});
            r3 = new geo::hyperRectangle(c3,d3);
            // top left obstacle
            vctr c4({2.75,8.5});
            vctr d4({3.5,1.0});
            r4 = new geo::hyperRectangle(c4,d4);
            // top right obstacle
            vctr c5({7.25,8.5});
            vctr d5({3.5,1.0});
            r5 = new geo::hyperRectangle(c5,d5);
            // package obstacle
            vctr c6({5.0,6.0});
            vctr d6({4.0,1.0});
            r6 = new geo::hyperRectangle(c6,d6);
            
            return;
        }
        
        bool collisionFreePoint(const vctr& point)
        {
            if(r1->in_region(point) or r2->in_region(point) or r3->in_region(point) or r4->in_region(point) or r5->in_region(point) or r6->in_region(point))
            {
                return false;
            }
            return true;
            
        }
        bool collisionFreeLine(const vctr& pointA, const vctr& pointB)
        {
            vctr p(pointA.size());
            numT l;
            for(int i=0;i<res;i++)
            {
                l=(numT)i/(numT)(res-1);
                for(int j=0;j<pointA.size();j++)
                {
                    p[j]=l*pointA[j]+(1.0-l)*pointB[j];
                }
                if( not collisionFreePoint(p) )
                {
                    return false;
                }
            }
            return true;
            
        }
        
    };
    
    //~~~~~~~~~~~~~~~~~~~~~~~Collision Detection~~~~~~~~~~~~~~~~~~~//
    
}//PRM namespace

#endif