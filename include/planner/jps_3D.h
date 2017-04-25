#ifndef __JPS_3D_H_
#define __JPS_3D_H_

#include <boost/heap/d_ary_heap.hpp>      // heap
#include <cmath>
#include <limits>                         // std::numeric_limits
#include <vector>
#include <list>                           // std::list
#include <array>
#include <unordered_map>

//#include <iostream>

namespace nx
{
  template <class T>
  struct compare_astar
  {
    bool operator()(T* a1, T* a2) const
    {
      double f1 = a1->g + a1->h;
      double f2 = a2->g + a2->h;
      if( ( f1 >= f2 - 0.000001) && (f1 <= f2 +0.000001) )
        return a1->g < a2->g; // if equal compare gvals
      return f1 > f2;
    }
  };

  struct Astate; // forward declaration
  using PriorityQueue = boost::heap::d_ary_heap<Astate*, boost::heap::mutable_<true>,
                        boost::heap::arity<2>, boost::heap::compare< compare_astar<Astate> >>;

  struct Astate
  {
    int ID;
    int dx, dy, dz;
    double g = std::numeric_limits<double>::infinity();
    double h;
    bool cl = false;
    int parent = -1;
    PriorityQueue::handle_type heapkey;
    Astate(int ind, int dx, int dy, int dz): ID(ind), dx(dx), dy(dy), dz(dz){}
  };

  struct JPS_NEIB
  {
    // for each (dx,dy,dz) these contain:
    //    n_: neighbors that are always added
    //    f1_: forced neighbors to check
    //    f2_: neighbors to add if forced
    int n_[3][3][3][3][26];
    int f1_[3][3][3][3][12];
    int f2_[3][3][3][3][12];

    // nsz_ contains the number of neighbors for the four different types of moves:
    // no move (norm 0):        26 neighbors always added
    //                          0 forced neighbors to check (never happens)
    //                          0 neighbors to add if forced (never happens)
    // straight (norm 1):       1 neighbor always added
    //                          8 forced neighbors to check
    //                          8 neighbors to add if forced
    // diagonal (norm sqrt(2)): 3 neighbors always added
    //                          4 forced neighbors to check
    //                          12 neighbors to add if forced
    // diagonal (norm sqrt(3)): 7 neighbors always added
    //                          3 forced neighbors to check
    //                          12 neighbors to add if forced
    static constexpr int nsz_[4][3] = {{26,0,0},{1,8,8},{3,4,12},{7,3,12}};
    JPS_NEIB();

  private:
    void Neib(int dx, int dy, int dz, int norm1, int dev, int& tx, int& ty, int& tz);
    void FNeib( int dx, int dy, int dz, int norm1, int dev,
                          int& fx, int& fy, int& fz,
                          int& nx, int& ny, int& nz);
  };

  struct HashMap
  {
    HashMap(size_t sz)
    {
      if(sz < 1000000 ) hashMapV_.resize(sz);
      else useVec_ = false;
    }
    
    Astate*& operator[] (size_t n)
    {
      if(useVec_) return hashMapV_[n];
      else return hashMapU_[n];
    }
  private:
    std::vector<Astate*> hashMapV_;
    std::unordered_map<size_t, Astate* > hashMapU_;
    bool useVec_ = true;
  };
  

  class JPS_3D
  {
    const char *cMap_;
    const char val_free_;
    int xDim_, yDim_, zDim_;
    int xyDim_, cMapLength_;
    PriorityQueue pq_;
    //std::vector<Astate*> hm_;
    HashMap hm_;
    std::vector<bool> seen_;
    JPS_NEIB jn_;

  public:
    JPS_3D(const char* cMap, const char val_free,
           int xDim, int yDim, int zDim, JPS_NEIB jn = JPS_NEIB())
      : cMap_(cMap), val_free_(val_free),
        xDim_(xDim), yDim_(yDim), zDim_(zDim),
        xyDim_(xDim*yDim), cMapLength_(xyDim_*zDim),
        hm_(cMapLength_+1), seen_(cMapLength_+1), jn_(jn)
    {}

    double plan( int xStart, int yStart, int zStart,
                 int xGoal,  int yGoal,  int zGoal,
                 std::list<std::array<int,3>>& xyzPath,
                 double eps = 1 );

    inline bool isFree(int x, int y, int z) const
    {
      //if( cMap_[x + y*xDim_ + z*xyDim_] == val_free_ )
      if( x >= 0 && x < xDim_ && y >= 0 && y < yDim_ &&
          z >= 0 && z < zDim_ && cMap_[x + y*xDim_ + z*xyDim_] == val_free_)
        return true;
      return false;
    }

    inline bool isOutside(int x, int y, int z) const
    {
      return x < 0 || x >= xDim_ ||
             y < 0 || y >= yDim_ ||
             z < 0 || z >= zDim_;
    }

    inline bool isEdge(int x, int y, int z) const
    {
      return x == 0 || x == xDim_-1 ||
        y == 0 || y == yDim_-1;
        //z == 0 || z == zDim_-1;
    }



  private:
    void spin(Astate *currNode_pt);
    void jump(int x, int y, int z, int dx, int dy, int dz);
    inline bool hasForced(int x, int y, int z, int dx, int dy, int dz, int norm1) const
    {
      dx+=1;dy+=1;dz+=1;
      switch(norm1)
      {
        case 1:
          for( int fn = 0; fn < 8; ++fn )
          {
            int nx = x + jn_.f1_[dx][dy][dz][0][fn];
            int ny = y + jn_.f1_[dx][dy][dz][1][fn];
            int nz = z + jn_.f1_[dx][dy][dz][2][fn];
            if( !isFree(nx,ny,nz) )
              return true;
          }
          return false;
        case 2:
          for( int fn = 0; fn < 4; ++fn )
          {
            int nx = x + jn_.f1_[dx][dy][dz][0][fn];
            int ny = y + jn_.f1_[dx][dy][dz][1][fn];
            int nz = z + jn_.f1_[dx][dy][dz][2][fn];
            if( !isFree(nx,ny,nz) )
              return true;
          }
          return false;
        case 3:
          for( int fn = 0; fn < 3; ++fn )
          {
            int nx = x + jn_.f1_[dx][dy][dz][0][fn];
            int ny = y + jn_.f1_[dx][dy][dz][1][fn];
            int nz = z + jn_.f1_[dx][dy][dz][2][fn];
            if( !isFree(nx,ny,nz) )
              return true;
          }
          return false;
        default:
          return false;
      }
      return false;
     }

    // temp global data
    double eps_;
    int xGoal_, yGoal_, zGoal_;
    int xJump_, yJump_, zJump_;
    bool goal_outside_;
  };
}
#endif


