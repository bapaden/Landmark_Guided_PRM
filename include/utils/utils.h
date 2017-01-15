#ifndef _UTILS_H_
#define _UTILS_H_

#include <assert.h>
#include <sstream>
#include <math.h> 
#include <cmath>
#include <algorithm>


namespace utils{

typedef float numT;
typedef std::vector<numT> vctr;
typedef std::vector<vctr> path;

vctr operator-(const vctr& x, const vctr& y)
{
    assert(x.size()-y.size());
    vctr z(x.size());
    for(unsigned long i=0;i<x.size();i++)
    {
        z[i]=x[i]-y[i];
    }
    return z;
}

vctr operator+(const vctr& x, const vctr& y)
{
    assert(x.size()-y.size());
    vctr z(x.size());
    for(unsigned long i=0;i<x.size();i++)
    {
        z[i]=x[i]+y[i];
    }
    return z;
}

template <class Ta, class Tb>
    numT dot_product(const Ta& a, const Tb& b){
        numT rsp = 0;
        typename Ta::const_iterator ai = a.begin();
        typename Tb::const_iterator bi = b.begin();
        while (ai != a.end() && bi != b.end())
            rsp += *ai++ * *bi++;
        return rsp;
    }
    
    template <class Ta, class Tb>
    Ta multiply(const Ta& a, const Tb& b){
        assert(a.size()<=b.size() && "a is bigger than b");
        Ta rsp(a);
        for (int i=0;i<a.size();i++)
            rsp[i] = a[i] * b[i];
        return rsp;
    }
    
    template <class T>
    numT norm(const T& v)
    {
        numT l=0;
        for(int i=0; i<v.size();i++)
        {
            l+=v[i]*v[i];
        }
        return sqrt(l);
    }
    
    template <class T>
    numT normSqr(const T& v)
    {
        numT l=0;
        for(int i=0; i<v.size();i++)
        {
            l+=v[i]*v[i];
        }
        return l;
    }
    
    template <class T>
    T sqr(const T& v)
    {
        return v*v;
    }
    
    template <class Ta, class Tb>
    Ta vec_diff(const Ta& a, const Tb& b){
        assert(a.size()<=b.size() && "a is bigger than b");
        Ta rsp(a);
        for (int i=0;i<a.size();i++)
            rsp[i] = a[i] - b[i];
        return rsp;
    }
    
    template <class Ta>
    Ta vec_pow(const Ta& a, const numT e){
        Ta rsp(a);
        for (int i=0;i<a.size();i++)
            rsp[i] = std::pow(a[i], e);
        return rsp;
    }
    
    template <class Ta, class Tb>
    void vec_sum_inplace(Ta& a, const Tb& b){
        assert(a.size()<=b.size() && "a is bigger than b");
        for (int i=0;i<a.size();i++)
            a[i]+=b[i];
    }
    
    template <class Ta>
    void vec_divide_inplace(Ta& a, numT d){
        for (int i=0;i<a.size();i++)
            a[i]/=d;
    }
    
    template <class Ta>
    numT L2(const Ta& a){
        numT rsp = dot_product(a,a);
        return sqrt(rsp);
    }
    
    template <class Ta, class Tb>
    Ta normalizeL2(const Ta& a){
        Ta rsp(a);
        numT norm = L2(a);
        assert(norm>1e-5 && "Zero-norm while normalizing");
        for (int i=0;i<a.size();i++)
            rsp[i] = 1.0*a[i]/norm;
        
        return rsp;
    }
    
    template <class T>
    inline int sign(const T& n){
        if (n>0)
            return 1;
        if (n<0)
            return -1;
        return 0;
    }
    
    numT to_mpi_pi(numT num){
        return atan2(sin(num),cos(num));
    }
    
    template <class T>
    std::string toString(const T& o){
        std::stringstream rsp;
        rsp << "[";
        int i=0;
        for(const auto& c:o)
            rsp << ((i++>0)?", ":"") << c;
        rsp << "]";
        return rsp.str();
    }
}

#endif