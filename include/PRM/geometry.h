#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <vector>
#include <cmath>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <random>
#include "../utils/utils.h"


namespace geo 
{
    typedef utils::numT numT;
    
    class region
    {
        bool closed;
    public:
        
        region(bool _closed=true)
        {
            closed = _closed;
        }
        virtual bool in_region(const std::vector<numT>& x){return false;}
        bool not_in_region(const std::vector<numT>& x)
        {
            return (not in_region(x)); 
        }
        bool is_open()
        {
            return (not closed);
        }
        bool is_closed()
        {
            return (closed);
        }
    }; 
    
    class hyperRectangle : public region
    {
        std::vector<numT> center;
        std::vector<numT> dimensions;
    public: 
        hyperRectangle()
        {
            return;
        }

        hyperRectangle(std::vector<numT>& _center, 
                       std::vector<numT>& _dimensions, 
                       bool _closed=true) : region(_closed)
                       {
                           assert(_center.size()==_dimensions.size());
                           center=_center;
                           dimensions=_dimensions;
                       }
                       
                       bool in_region(const std::vector<numT>& x)
                       {
                           if(is_closed())
                           {
                               for (int i = 0 ; i<center.size(); i++)
                               {
                                   if(x[i] > center[i]+dimensions[i]/2.0 or x[i] < center[i]-dimensions[i]/2.0)
                                   {
                                       
                                       return false;
                                   }
                               }
                               
                           } 
                           else
                           {
                               for (int i = 0 ; i<center.size(); i++)
                               {
                                   if( (x[i] >= (center[i]+dimensions[i]/2.0) ) or ( x[i] <= (center[i]-dimensions[i]/2.0) ) )
                                   {
                                       
                                       return false;
                                   }
                               }
                           }
                           return true;
                       }
    };
    
    class sphere : public region
    {
    public: 
        numT radiusSqr,rad;
        std::vector<numT> center;
 
        sphere(const std::vector<numT>& _center, 
               const numT _radius, 
               bool _closed=true) : region(_closed)
               {
                   center=_center;
                   rad=_radius;
                   radiusSqr=utils::sqr(_radius);
               }
               
               bool in_region(const std::vector<numT>& x)
               {
                   if(is_closed())
                   {
                       if(utils::normSqr(utils::vec_diff(x,center))<=radiusSqr)
                       {
                           return true;
                       }
                   }
                   
                   
                   else
                   {
                       if(utils::normSqr(utils::vec_diff(x,center))<radiusSqr)
                       {
                           return true;
                       }
                   }
                   return false;
               }
               
               numT radius()
               {
                   return rad;
               }
    };
    
    class halfSpace : public region
    {
        std::vector<numT> a;
        numT n,b;
    public: 
        halfSpace(std::vector<numT>& _a, 
                  numT _b, 
                  bool _closed=true) : region(_closed)
                  {
                      a=_a;
                      b=_b;
                  }
                  
                  bool in_region(const std::vector<numT>& x)
                  {
                      numT y = utils::dot_product(a,x);
                      
                      if(is_closed() and y>=b)
                      {
                          return true;
                      }
                      if(is_open() and y>b) 
                      {
                          return true;
                      }
                      return false;
                      
                  }
    };    
    
    class hyperCylinder : public region
    {
        std::vector<int> indices;
        numT radius, dist;
    public: 
        std::vector<numT> center;
        hyperCylinder(std::vector<int>& _indices, 
                      numT& _radius,
                      std::vector<numT>& _center,
                      bool _closed=true) : region(_closed)
                      {
                          indices=_indices;
                          radius=_radius;
                          center=_center;
                      }
                      
                      bool in_region(const std::vector<numT>& x)
                      {
                          dist=0;
                          if(is_closed())
                          {
                              for(auto& i : indices)
                              {
                                  dist += utils::sqr(x[i]-center[i]);
                              }
                              if(dist>utils::sqr(radius))
                              {
                                  return false;
                              }
                          }
                          
                          else
                          {
                              for(auto& i : indices)
                              {
                                  for(auto& i : indices)
                                  {
                                      dist += utils::sqr(x[i]-center[i]);
                                  }
                                  if(dist>=utils::sqr(radius))
                                  {
                                      return false;
                                  }
                              }
                          }
                          return true;
                      }
    };
}



#endif