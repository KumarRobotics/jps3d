#include <planner/jps_3D.h>
#include <algorithm>    // std::min
#include <iostream>

double nx::JPS_3D::plan( int xStart, int yStart, int zStart,
                         int xGoal,  int yGoal,  int zGoal,
                         std::list<std::array<int,3>>& xyzPath,
                         double eps )
{
  // Start is not included in the final path
  goal_outside_ = isOutside(xGoal, yGoal, zGoal);
  eps_ = eps;
  xGoal_ = xGoal; yGoal_ = yGoal; zGoal_ = zGoal;
  int indStart = xStart + xDim_*yStart + xyDim_*zStart; // colmajor
  int indGoal  = (goal_outside_) ? cMapLength_ : xGoal  + xDim_*yGoal  + xyDim_*zGoal;

  //if( goal_outside_ ) std::cout << "GOAL IS OUTSIDE" << std::endl;
  //else std::cout << "GOAL IS INSIDE" << std::endl;  
  //std::cout << "xGoal=" << xGoal << " yGoal=" << yGoal << " zGoal=" << zGoal << std::endl;
  //std::cout << "xDim_=" << xDim_ << " yDim_=" << yDim_ << " zDim_=" << zDim_ << std::endl;
  //std::cout << "indGoal=" << indGoal << std::endl;
  
  // Quit if start or goal state is in collision
  if( cMap_[indStart]!=val_free_ ||
      (!goal_outside_ && cMap_[indGoal]!=val_free_ ))
    return std::numeric_limits<double>::infinity();
  hm_[indStart] = new Astate(indStart,0,0,0);
  hm_[indStart]->h = eps*std::sqrt( (xGoal-xStart)*(xGoal-xStart) +
                                    (yGoal-yStart)*(yGoal-yStart) +
                                    (zGoal-zStart)*(zGoal-zStart) );
  hm_[indStart]->g = 0;
  seen_[indStart] = true;

  //int nNode=0;
  Astate *currNode_pt = hm_[indStart];
  currNode_pt->cl = true;
  while(true)
  {
    //nNode++;
    if( currNode_pt->ID == indGoal ) break;
    spin(currNode_pt);

    if( pq_.empty() )
      return std::numeric_limits<double>::infinity();
    currNode_pt = pq_.top(); pq_.pop(); // get element with smallest cost
    currNode_pt->cl = true;
  }
  //std::cout << "nNode=" << nNode << std::endl;
  //std::cout << "goalFound?=" << seen_[indGoal]<< std::endl;
  
  // recover path
  int indCurrNode = indGoal;
  if( seen_[indCurrNode] )
  {
    int xCurr = xGoal; int yCurr = yGoal; int zCurr = zGoal;

    while( indCurrNode != indStart )
    {
      xyzPath.push_front({xCurr,yCurr,zCurr});
      indCurrNode = hm_[indCurrNode]->parent;
      xCurr = (indCurrNode % xyDim_) % xDim_;
      yCurr = (indCurrNode % xyDim_) / xDim_;
      zCurr = indCurrNode / xyDim_;
    }

    double cost = hm_[indGoal]->g;
    // cleanup
    for (int i = 0; i < (int) seen_.size(); ++i)
      if( seen_[i] )
      {
        delete hm_[i];
        seen_[i] = false;
      }
    pq_.clear();
    //std::cout << "xyzPath.size()=" << xyzPath.size() << std::endl;
    //std::cout << "cost=" << cost << std::endl;
    return cost;
  }
  else
    return std::numeric_limits<double>::infinity();
}


void nx::JPS_3D::spin(Astate *currNode_pt)
{
  int xCurr = (currNode_pt->ID % xyDim_) % xDim_;
  int yCurr = (currNode_pt->ID % xyDim_) / xDim_;
  int zCurr = currNode_pt->ID / xyDim_;
  int norm1 = std::abs(currNode_pt->dx) + std::abs(currNode_pt->dy) + std::abs(currNode_pt->dz);

  //std::cout << "xyzCurr=" << xCurr << " " << yCurr << " " << zCurr << std::endl;
  //std::cout << "dxyzCurr=" << currNode_pt->dx << " " << currNode_pt->dy << " " << currNode_pt->dz << std::endl;

  int dxi = currNode_pt->dx+1, dyi = currNode_pt->dy+1, dzi= currNode_pt->dz+1;
  int num_neib = jn_.nsz_[norm1][0];
  int num_fneib = jn_.nsz_[norm1][2];
  int dev0 = (goal_outside_ && isEdge(xCurr, yCurr, zCurr)) ? -1 : 0; // XXX: ADDON for outside goals
  int dxNeighbor, dyNeighbor, dzNeighbor;

  for( int dev = dev0; dev < num_neib+num_fneib; ++dev)
  {
    if( dev < 0 )
    {
      // XXX: ADDON for outside goals
      xJump_ = xGoal_; yJump_ = yGoal_; zJump_ = zGoal_;
      dxNeighbor = 0; dyNeighbor = 0; dzNeighbor = 0;
    }
    else if( dev < num_neib )
    {
      dxNeighbor = jn_.n_[dxi][dyi][dzi][0][dev];
      dyNeighbor = jn_.n_[dxi][dyi][dzi][1][dev];
      dzNeighbor = jn_.n_[dxi][dyi][dzi][2][dev];
      jump(xCurr,yCurr,zCurr,dxNeighbor,dyNeighbor,dzNeighbor);
      if( xJump_ == -1 ) continue;
    }
    else
    {
      // Check if this neighbor is forced
      int nx = xCurr + jn_.f1_[dxi][dyi][dzi][0][dev-num_neib];
      int ny = yCurr + jn_.f1_[dxi][dyi][dzi][1][dev-num_neib];
      int nz = zCurr + jn_.f1_[dxi][dyi][dzi][2][dev-num_neib];
      if(isFree(nx,ny,nz))
        continue;
      dxNeighbor = jn_.f2_[dxi][dyi][dzi][0][dev-num_neib];
      dyNeighbor = jn_.f2_[dxi][dyi][dzi][1][dev-num_neib];
      dzNeighbor = jn_.f2_[dxi][dyi][dzi][2][dev-num_neib];
      jump(xCurr,yCurr,zCurr,dxNeighbor,dyNeighbor,dzNeighbor);
      if( xJump_ == -1 ) continue;      
    }

    //std::cout << "dxyzNeighbor=" << dxNeighbor << " " << dyNeighbor << " " << dzNeighbor << std::endl;

    /*
    std::cout << "FORCED JUMP" << std::endl;
    std::cout << "xyzNeighbor=" << xJump_ << " " << yJump_ << " " << zJump_ << std::endl;
    std::cout << "dxyzNeighbor=" << dxNeighbor << " " << dyNeighbor << " " << dzNeighbor << std::endl;
    std::cout << "END FORCED JUMP" << std::endl;
    */

    int indNeighbor = (dev<0)? cMapLength_ : xJump_ + xDim_*yJump_ + xyDim_*zJump_;
    // initialize if never seen before
    if( !seen_[indNeighbor] )
    {
      seen_[indNeighbor] = true;
      hm_[indNeighbor] = new Astate(indNeighbor,dxNeighbor,dyNeighbor,dzNeighbor);
    }

    // skip closed nodes
    if(hm_[indNeighbor]->cl) continue;

    double costNeighbor = hm_[currNode_pt->ID]->g +
                                std::sqrt((xCurr - xJump_)*(xCurr - xJump_) +
                                			    (yCurr - yJump_)*(yCurr - yJump_) +
                                			    (zCurr - zJump_)*(zCurr - zJump_));

    if (costNeighbor < hm_[indNeighbor]->g)
    {
      //update the heuristic value
      if( !std::isinf(hm_[indNeighbor]->g) )
      {
        // node seen before
        hm_[indNeighbor]->dx = dxNeighbor;
        hm_[indNeighbor]->dy = dyNeighbor;
        hm_[indNeighbor]->dz = dzNeighbor;
        hm_[indNeighbor]->g = costNeighbor;
        pq_.increase(hm_[indNeighbor]->heapkey);
        // increase == decrease with our comparator (!)
      }else
      {
        // ADD
        hm_[indNeighbor]->h = eps_*std::sqrt((xJump_-xGoal_)*(xJump_-xGoal_) +
              			                         (yJump_-yGoal_)*(yJump_-yGoal_) +
              			                         (zJump_-zGoal_)*(zJump_-zGoal_));
    		hm_[indNeighbor]->g = costNeighbor;
    		hm_[indNeighbor]->heapkey = pq_.push(hm_[indNeighbor]);
			}
			hm_[indNeighbor]->parent = currNode_pt->ID;
    }
  }
}


void nx::JPS_3D::jump(int x, int y, int z, int dx, int dy, int dz)
{
  int nx = x+dx, ny = y+dy, nz = z+dz;
  if(!isFree(nx,ny,nz))
  {
    //std::cout << "DEADEND" << std::endl;
    xJump_ = -1; yJump_ = -1; zJump_ = -1;
    return;
  }
  //****************************************//
  // XXX: ADDON for outside goals
  if(goal_outside_ && isEdge(nx, ny, nz))
  {
    xJump_ = nx; yJump_ = ny; zJump_ = nz;
    return;
  }
  //****************************************//  
  if( nx == xGoal_ && ny == yGoal_ && nz == zGoal_ )
  {
    //std::cout << "GOAL" << std::endl;
    xJump_ = nx; yJump_ = ny; zJump_ = nz;
    return;
  }

  int norm1 = std::abs(dx) + std::abs(dy) +std::abs(dz);
  if( hasForced(nx, ny, nz, dx, dy, dz, norm1) )
  {
    //std::cout << "FORCED" << std::endl;
    xJump_ = nx; yJump_ = ny; zJump_ = nz;
    return;
  }

  int num_neib = jn_.nsz_[norm1][0];
  for( int k = 0; k < num_neib-1; ++k )
  {
    jump(nx,ny,nz,jn_.n_[dx+1][dy+1][dz+1][0][k],
                  jn_.n_[dx+1][dy+1][dz+1][1][k],
                  jn_.n_[dx+1][dy+1][dz+1][2][k]);
    if( xJump_ != -1 )
    {
      xJump_ = nx; yJump_ = ny; zJump_ = nz;
      return;
    }
  }
  jump(nx,ny,nz,dx,dy,dz);
}




constexpr int nx::JPS_NEIB::nsz_[4][3];
nx::JPS_NEIB::JPS_NEIB()
{
  for(int dx = -1; dx <= 1; ++dx)
    for(int dy = -1; dy <= 1; ++dy)
      for(int dz = -1; dz <= 1; ++dz)
      {
        int norm1 = std::abs(dx) + std::abs(dy) +std::abs(dz);
        for(int dev = 0; dev < nsz_[norm1][0]; ++dev)
          Neib(dx,dy,dz,norm1,dev,n_[dx+1][dy+1][dz+1][0][dev],
                                  n_[dx+1][dy+1][dz+1][1][dev],
                                  n_[dx+1][dy+1][dz+1][2][dev]);
        for(int dev = 0; dev < nsz_[norm1][2]; ++dev)
        {
          FNeib(dx,dy,dz,norm1,dev,f1_[dx+1][dy+1][dz+1][0][dev],
                                   f1_[dx+1][dy+1][dz+1][1][dev],
                                   f1_[dx+1][dy+1][dz+1][2][dev],
                                   f2_[dx+1][dy+1][dz+1][0][dev],
                                   f2_[dx+1][dy+1][dz+1][1][dev],
                                   f2_[dx+1][dy+1][dz+1][2][dev]);
        }
      }
}
void nx::JPS_NEIB::Neib(int dx, int dy, int dz, int norm1, int dev,
                        int& tx, int& ty, int& tz)
{
  switch(norm1)
  {
    case 0:
      switch(dev)
      {
        case 0: tx=1; ty=0; tz=0; return;
        case 1: tx=-1; ty=0; tz=0; return;
        case 2: tx=0; ty=1; tz=0; return;
        case 3: tx=1; ty=1; tz=0; return;
        case 4: tx=-1; ty=1; tz=0; return;
        case 5: tx=0; ty=-1; tz=0; return;
        case 6: tx=1; ty=-1; tz=0; return;
        case 7: tx=-1; ty=-1; tz=0; return;
        case 8: tx=0; ty=0; tz=1; return;
        case 9: tx=1; ty=0; tz=1; return;
        case 10: tx=-1; ty=0; tz=1; return;
        case 11: tx=0; ty=1; tz=1; return;
        case 12: tx=1; ty=1; tz=1; return;
        case 13: tx=-1; ty=1; tz=1; return;
        case 14: tx=0; ty=-1; tz=1; return;
        case 15: tx=1; ty=-1; tz=1; return;
        case 16: tx=-1; ty=-1; tz=1; return;
        case 17: tx=0; ty=0; tz=-1; return;
        case 18: tx=1; ty=0; tz=-1; return;
        case 19: tx=-1; ty=0; tz=-1; return;
        case 20: tx=0; ty=1; tz=-1; return;
        case 21: tx=1; ty=1; tz=-1; return;
        case 22: tx=-1; ty=1; tz=-1; return;
        case 23: tx=0; ty=-1; tz=-1; return;
        case 24: tx=1; ty=-1; tz=-1; return;
        case 25: tx=-1; ty=-1; tz=-1; return;
      }
    case 1:
      tx = dx; ty = dy; tz = dz; return;
    case 2:
      switch(dev)
      {
        case 0:
          if(dz == 0){
            tx = 0; ty = dy; tz = 0; return;
          }else{
            tx = 0; ty = 0; tz = dz; return;
          }
        case 1:
          if(dx == 0){
            tx = 0; ty = dy; tz = 0; return;
          }else{
            tx = dx; ty = 0; tz = 0; return;
          }
        case 2:
          tx = dx; ty = dy; tz = dz; return;
      }
    case 3:
      switch(dev)
      {
        case 0: tx = dx; ty =  0; tz =  0; return;
        case 1: tx =  0; ty = dy; tz =  0; return;
        case 2: tx =  0; ty =  0; tz = dz; return;
        case 3: tx = dx; ty = dy; tz =  0; return;
        case 4: tx = dx; ty =  0; tz = dz; return;
        case 5: tx =  0; ty = dy; tz = dz; return;
        case 6: tx = dx; ty = dy; tz = dz; return;
      }
  }
}
void nx::JPS_NEIB::FNeib( int dx, int dy, int dz, int norm1, int dev,
                          int& fx, int& fy, int& fz,
                          int& nx, int& ny, int& nz)
{
  switch(norm1)
  {
    case 1:
      switch(dev)
      {
        case 0: fx= 0; fy= 1; fz = 0; break;
        case 1: fx= 0; fy=-1; fz = 0; break;
        case 2: fx= 1; fy= 0; fz = 0; break;
        case 3: fx= 1; fy= 1; fz = 0; break;
        case 4: fx= 1; fy=-1; fz = 0; break;
        case 5: fx=-1; fy= 0; fz = 0; break;
        case 6: fx=-1; fy= 1; fz = 0; break;
        case 7: fx=-1; fy=-1; fz = 0; break;
      }
      nx = fx; ny = fy; nz = dz;
      // switch order if different direction
      if(dx != 0){
        fz = fx; fx = 0;
        nz = fz; nx = dx;
      }if(dy != 0){
        fz = fy; fy = 0;
        nz = fz; ny = dy;
      }
      return;
    case 2:
      if(dx == 0){
        switch(dev)
        {
          case 0:
            fx = 0; fy = 0; fz = -dz;
            nx = 0; ny = dy; nz = -dz;
            return;
          case 1:
            fx = 0; fy = -dy; fz = 0;
            nx = 0; ny = -dy; nz = dz;
            return;
          case 2:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = dy; nz = dz;
            return;
          case 3:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = dy; nz = dz;
            return;
          case 4:
            fx = 1; fy = 0; fz = -dz;
            nx = 1; ny = dy; nz = -dz;
            return;
          case 5:
            fx = 1; fy = -dy; fz = 0;
            nx = 1; ny = -dy; nz = dz;
            return;
          case 6:
            fx = -1; fy = 0; fz = -dz;
            nx = -1; ny = dy; nz = -dz;
            return;
          case 7:
            fx = -1; fy = -dy; fz = 0;
            nx = -1; ny = -dy; nz = dz;
            return;
          // Extras
          case 8:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = dy; nz = 0;
            return;
          case 9:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = 0; nz = dz;
            return;
          case 10:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = dy; nz = 0;
            return;
          case 11:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = 0; nz = dz;
            return;
        }
      }else if(dy == 0){
        switch(dev)
        {
          case 0:
            fx = 0; fy = 0; fz = -dz;
            nx = dx; ny = 0; nz = -dz;
            return;
          case 1:
            fx = -dx; fy = 0; fz = 0;
            nx = -dx; ny = 0; nz = dz;
            return;
          case 2:
            fx = 0; fy = 1; fz = 0;
            nx = dx; ny = 1; nz = dz;
            return;
          case 3:
            fx = 0; fy = -1; fz = 0;
            nx = dx; ny = -1;nz = dz;
            return;
          case 4:
            fx = 0; fy = 1; fz = -dz;
            nx = dx; ny = 1; nz = -dz;
            return;
          case 5:
            fx = -dx; fy = 1; fz = 0;
            nx = -dx; ny = 1; nz = dz;
            return;
          case 6:
            fx = 0; fy = -1; fz = -dz;
            nx = dx; ny = -1; nz = -dz;
            return;
          case 7:
            fx = -dx; fy = -1; fz = 0;
            nx = -dx; ny = -1; nz = dz;
            return;
          // Extras
          case 8:
            fx = 0; fy = 1; fz = 0;
            nx = dx; ny = 1; nz = 0;
            return;
          case 9:
            fx = 0; fy = 1; fz = 0;
            nx = 0; ny = 1; nz = dz;
            return;
          case 10:
            fx = 0; fy = -1; fz = 0;
            nx = dx; ny = -1; nz = 0;
            return;
          case 11:
            fx = 0; fy = -1; fz = 0;
            nx = 0; ny = -1; nz = dz;
            return;
        }
      }else{// dz==0
        switch(dev)
        {
          case 0:
            fx = 0; fy = -dy; fz = 0;
            nx = dx; ny = -dy; nz = 0;
            return;
          case 1:
            fx = -dx; fy = 0; fz = 0;
            nx = -dx; ny = dy; nz = 0;
            return;
          case 2:
            fx =  0; fy = 0; fz = 1;
            nx = dx; ny = dy; nz = 1;
            return;
          case 3:
            fx =  0; fy = 0; fz = -1;
            nx = dx; ny = dy; nz = -1;
            return;
          case 4:
            fx = 0; fy = -dy; fz = 1;
            nx = dx; ny = -dy; nz = 1;
            return;
          case 5:
            fx = -dx; fy = 0; fz = 1;
            nx = -dx; ny = dy; nz = 1;
            return;
          case 6:
            fx = 0; fy = -dy; fz = -1;
            nx = dx; ny = -dy; nz = -1;
            return;
          case 7:
            fx = -dx; fy = 0; fz = -1;
            nx = -dx; ny = dy; nz = -1;
            return;
          // Extras
          case 8:
            fx =  0; fy = 0; fz = 1;
            nx = dx; ny = 0; nz = 1;
            return;
          case 9:
            fx = 0; fy = 0; fz = 1;
            nx = 0; ny = dy; nz = 1;
            return;
          case 10:
            fx =  0; fy = 0; fz = -1;
            nx = dx; ny = 0; nz = -1;
            return;
          case 11:
            fx = 0; fy = 0; fz = -1;
            nx = 0; ny = dy; nz = -1;
            return;
        }
      }
    case 3:
      switch(dev)
      {
        case 0:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = dy; nz = dz;
          return;
        case 1:
          fx = 0; fy = -dy; fz = 0;
          nx = dx; ny = -dy; nz = dz;
          return;
        case 2:
          fx = 0; fy = 0; fz = -dz;
          nx = dx; ny = dy; nz = -dz;
          return;
        // Need to check up to here for forced!
        case 3:
          fx = 0; fy = -dy; fz = -dz;
          nx = dx; ny = -dy; nz = -dz;
          return;
        case 4:
          fx = -dx; fy = 0; fz = -dz;
          nx = -dx; ny = dy; nz = -dz;
          return;
        case 5:
          fx = -dx; fy = -dy; fz = 0;
          nx = -dx; ny = -dy; nz = dz;
          return;
        // Extras
        case 6:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = 0; nz = dz;
          return;
        case 7:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = dy; nz = 0;
          return;
        case 8:
          fx = 0; fy = -dy; fz = 0;
          nx = 0; ny = -dy; nz = dz;
          return;
        case 9:
          fx = 0; fy = -dy; fz = 0;
          nx = dx; ny = -dy; nz = 0;
          return;
        case 10:
          fx = 0; fy = 0; fz = -dz;
          nx = 0; ny = dy; nz = -dz;
          return;
        case 11:
          fx = 0; fy = 0; fz = -dz;
          nx = dx; ny = 0; nz = -dz;
          return;
      }
  }
}






/*
bool nx::JPS_3D::hasForced(int x, int y, int z, int dx, int dy, int dz, int norm1)
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
  }
}
*/

/*
bool nx::JPS_3D::hasForced(int x, int y, int z, int dx, int dy, int dz, int norm1)
{
  // norm1 == 2: (2,8,9), (3,10,11)
  // norm1 == 3: (0,6,7), (1,8,9), (2,10,11)
  // fix indexing
  //if( x == 1 && y == 0 && z == 0 )

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
        {
          nx = x + jn_.f2_[dx][dy][dz][0][fn];
          ny = y + jn_.f2_[dx][dy][dz][1][fn];
          nz = z + jn_.f2_[dx][dy][dz][2][fn];
          if( isFree(nx,ny,nz) )
            return true;
        }
      }
      return false;
    case 2:
      for( int fn = 0; fn < 2; ++fn )
      {
        int nx = x + jn_.f1_[dx][dy][dz][0][fn];
        int ny = y + jn_.f1_[dx][dy][dz][1][fn];
        int nz = z + jn_.f1_[dx][dy][dz][2][fn];
        if( !isFree(nx,ny,nz) )
        {
          nx = x + jn_.f2_[dx][dy][dz][0][fn];
          ny = y + jn_.f2_[dx][dy][dz][1][fn];
          nz = z + jn_.f2_[dx][dy][dz][2][fn];
          if( isFree(nx,ny,nz) )
            return true;
        }
      }
      for( int fn = 2; fn < 4; ++fn )
      {
        int nx = x + jn_.f1_[dx][dy][dz][0][fn];
        int ny = y + jn_.f1_[dx][dy][dz][1][fn];
        int nz = z + jn_.f1_[dx][dy][dz][2][fn];
        if( !isFree(nx,ny,nz) )
        {
          // Check if any of the 3 forced neighbors are free
          nx = x + jn_.f2_[dx][dy][dz][0][fn];
          ny = y + jn_.f2_[dx][dy][dz][1][fn];
          nz = z + jn_.f2_[dx][dy][dz][2][fn];
          if( isFree(nx,ny,nz) )
            return true;
          nx = x + jn_.f2_[dx][dy][dz][0][2*fn+4];
          ny = y + jn_.f2_[dx][dy][dz][1][2*fn+4];
          nz = z + jn_.f2_[dx][dy][dz][2][2*fn+4];
          if( isFree(nx,ny,nz) )
            return true;
          nx = x + jn_.f2_[dx][dy][dz][0][2*fn+5];
          ny = y + jn_.f2_[dx][dy][dz][1][2*fn+5];
          nz = z + jn_.f2_[dx][dy][dz][2][2*fn+5];
          if( isFree(nx,ny,nz) )
            return true;
        }
      }
      return false;
    case 3:
      for( int fn = 0; fn < 3; ++fn )
      {
        int nx = x + jn_.f1_[dx][dy][dz][0][fn];
        int ny = y + jn_.f1_[dx][dy][dz][1][fn];
        int nz = z + jn_.f1_[dx][dy][dz][2][fn];
        if( !isFree(nx,ny,nz) )
        {
          // Check if any of the 3 forced neighbors are free
          nx = x + jn_.f2_[dx][dy][dz][0][fn];
          ny = y + jn_.f2_[dx][dy][dz][1][fn];
          nz = z + jn_.f2_[dx][dy][dz][2][fn];
          if( isFree(nx,ny,nz) )
            return true;
          nx = x + jn_.f2_[dx][dy][dz][0][2*fn+6];
          ny = y + jn_.f2_[dx][dy][dz][1][2*fn+6];
          nz = z + jn_.f2_[dx][dy][dz][2][2*fn+6];
          if( isFree(nx,ny,nz) )
            return true;
          nx = x + jn_.f2_[dx][dy][dz][0][2*fn+7];
          ny = y + jn_.f2_[dx][dy][dz][1][2*fn+7];
          nz = z + jn_.f2_[dx][dy][dz][2][2*fn+7];
          if( isFree(nx,ny,nz) )
            return true;
        }
      }
      return false;
  }
}
*/




/*
bool nx::JPS_3D::hasForced(int x, int y, int z, int dx, int dy, int dz, int norm1)
{
  int nx,ny,nz;
  if( norm1 == 3 )
  {
    for( int fn = 0; fn < 3; ++fn )
    {
      nx = x + jn_.f1_[dx+1][dy+1][dz+1][0][fn];
      ny = y + jn_.f1_[dx+1][dy+1][dz+1][1][fn];
      nz = z + jn_.f1_[dx+1][dy+1][dz+1][2][fn];
      if( nx < 0 || nx >= xDim_ || ny < 0 || ny >= yDim_ ||
          nz < 0 || nz >= zDim_ || !cMap_[nx + ny*xDim_ + nz*xyDim_] )
        return false;
    }
    nx = x + jn_.f2_[dx+1][dy+1][dz+1][0][0];
    ny = y + jn_.f2_[dx+1][dy+1][dz+1][1][0];
    nz = z + jn_.f2_[dx+1][dy+1][dz+1][2][0];
    // Check if free
    if( nx >= 0 && nx < xDim_ && ny >= 0 && ny < yDim_ &&
        nz >= 0 && nz < zDim_ && !cMap_[nx + ny*xDim_ + nz*xyDim_] )
      return true;
    return false;
  }

  int num_fn = jn_.nsz_[norm1][1];
  for( int fn = 0; fn < num_fn; ++fn )
  {
    nx = x + jn_.f1_[dx+1][dy+1][dz+1][0][fn];
    ny = y + jn_.f1_[dx+1][dy+1][dz+1][1][fn];
    nz = z + jn_.f1_[dx+1][dy+1][dz+1][2][fn];
    if( nx >= 0 && nx < xDim_ && ny >= 0 && ny < yDim_ &&
        nz >= 0 && nz < zDim_ && cMap_[nx + ny*xDim_ + nz*xyDim_] )
    {
      nx = x + jn_.f2_[dx+1][dy+1][dz+1][0][fn];
      ny = y + jn_.f2_[dx+1][dy+1][dz+1][1][fn];
      nz = z + jn_.f2_[dx+1][dy+1][dz+1][2][fn];
      // Check if free
      if( nx >= 0 && nx < xDim_ && ny >= 0 && ny < yDim_ &&
          nz >= 0 && nz < zDim_ && !cMap_[nx + ny*xDim_ + nz*xyDim_] )
      {
        int fx = x + jn_.f1_[dx+1][dy+1][dz+1][0][fn];
        int fy = y + jn_.f1_[dx+1][dy+1][dz+1][1][fn];
        int fz = z + jn_.f1_[dx+1][dy+1][dz+1][2][fn];
        //std::cout << "Forced norm=" << norm1 << std::endl;
        //std::cout << "(dx,dy,dz)=" << dx << " " << dy << " " << dz << std::endl;
        //std::cout << "(x,y,z)=" << x << " " << y << " " << z << std::endl;
        //std::cout << "f1=" << fx << " "
        //                   << fy << " "
        //                   << fz << std::endl;
        //std::cout << "f2=" << nx << " "
        //                   << ny << " "
        //                   << nz << std::endl;
        //std::cout << "cmap(x,y,z)=" << cMap_[x + y*xDim_ + z*xyDim_] << std::endl;
        //std::cout << "cmap(fx,fy,fz)=" << cMap_[fx + fy*xDim_ + fz*xyDim_] << std::endl;
        //std::cout << "cmap(nx,ny,nz)=" << cMap_[nx + ny*xDim_ + nz*xyDim_] << std::endl;
        return true;
      }
    }
  }
  return false;
}
*/





/*
void nx::JPS_3D::prune(int dx, int dy, int dz,
           std::vector<std::array<int,3>>& neib,
           std::vector<std::array<int,3>>& fneib1,
           std::vector<std::array<int,3>>& fneib2)
{
  int norm1 = std::abs(dx)+std::abs(dy)+std::abs(dz);
  int tx,ty,tz;
  switch(norm1)
  {
    case 0:
      neib.resize(26);
      neib[0] = {-1,-1,-1};
      neib[1] = {-1,-1,0};
      neib[2] = {-1,-1,1};
      neib[3] = {-1,0,-1};
      neib[4] = {-1,0,0};
      neib[5] = {-1,0,1};
      neib[6] = {-1,1,-1};
      neib[7] = {-1,1,0};
      neib[8] = {-1,1,1};
      neib[9] = {0,-1,-1};
      neib[10] = {0,-1,0};
      neib[11] = {0,-1,1};
      neib[12] = {0,0,-1};
      neib[13] = {0,0,1};
      neib[14] = {0,1,-1};
      neib[15] = {0,1,0};
      neib[16] = {0,1,1};
      neib[17] = {1,-1,-1};
      neib[18] = {1,-1,0};
      neib[19] = {1,-1,1};
      neib[20] = {1,0,-1};
      neib[21] = {1,0,0};
      neib[22] = {1,0,1};
      neib[23] = {1,1,-1};
      neib[24] = {1,1,0};
      neib[25] = {1,1,1};
      break;
    case 1:
      neib.push_back({dx,dy,dz});
      fneib1.resize(8); fneib2.resize(8);

      Neib(dx,dy,dz,0,1,0,tx,ty,tz);   fneib1[0] = {tx,ty,tz};
      Neib(dx,dy,dz,0,-1,0,tx,ty,tz);  fneib1[1] = {tx,ty,tz};
      Neib(dx,dy,dz,0,1,1,tx,ty,tz);   fneib1[2] = {tx,ty,tz};
      Neib(dx,dy,dz,0,0,1,tx,ty,tz);   fneib1[3] = {tx,ty,tz};
      Neib(dx,dy,dz,0,-1,1,tx,ty,tz);  fneib1[4] = {tx,ty,tz};
      Neib(dx,dy,dz,0,1,-1,tx,ty,tz);  fneib1[5] = {tx,ty,tz};
      Neib(dx,dy,dz,0,0,-1,tx,ty,tz);  fneib1[6] = {tx,ty,tz};
      Neib(dx,dy,dz,0,-1,-1,tx,ty,tz); fneib1[7] = {tx,ty,tz};

      Neib(dx,dy,dz,1,1,0,tx,ty,tz);   fneib2[0] = {tx,ty,tz};
      Neib(dx,dy,dz,1,-1,0,tx,ty,tz);  fneib2[1] = {tx,ty,tz};
      Neib(dx,dy,dz,1,1,1,tx,ty,tz);   fneib2[2] = {tx,ty,tz};
      Neib(dx,dy,dz,1,0,1,tx,ty,tz);   fneib2[3] = {tx,ty,tz};
      Neib(dx,dy,dz,1,-1,1,tx,ty,tz);  fneib2[4] = {tx,ty,tz};
      Neib(dx,dy,dz,1,1,-1,tx,ty,tz);  fneib2[5] = {tx,ty,tz};
      Neib(dx,dy,dz,1,0,-1,tx,ty,tz);  fneib2[6] = {tx,ty,tz};
      Neib(dx,dy,dz,1,-1,-1,tx,ty,tz); fneib2[7] = {tx,ty,tz};
      break;
    case 2:
      neib.resize(3); fneib1.resize(4); fneib2.resize(4);
      Neib(dx,dy,dz,1,1,0,tx,ty,tz); neib[0] = {tx,ty,tz};
      Neib(dx,dy,dz,1,-1,0,tx,ty,tz); neib[1] = {tx,ty,tz};
      neib[2] = {dx,dy,dz};

      Neib(dx,dy,dz,-1,1,0,tx,ty,tz); fneib1[0] = {tx,ty,tz};
      Neib(dx,dy,dz,-1,-1,0,tx,ty,tz);fneib1[1] = {tx,ty,tz};
      Neib(dx,dy,dz,0,0,1,tx,ty,tz);  fneib1[2] = {tx,ty,tz};
      Neib(dx,dy,dz,0,0,-1,tx,ty,tz); fneib1[3] = {tx,ty,tz};

      Neib(dx,dy,dz,0,1,0,tx,ty,tz); fneib2[0] = {tx,ty,tz};
      Neib(dx,dy,dz,0,-1,0,tx,ty,tz);fneib2[1] = {tx,ty,tz};
      Neib(dx,dy,dz,1,0,1,tx,ty,tz); fneib2[2] = {tx,ty,tz};
      Neib(dx,dy,dz,1,0,-1,tx,ty,tz);fneib2[3] = {tx,ty,tz};
      break;
    case 3:
      neib.resize(7); fneib1.resize(3); fneib2.resize(1);
      // The order below is important!
      Neib(dx,dy,dz,1,-1,-1,tx,ty,tz);neib[0] = {tx,ty,tz};
      Neib(dx,dy,dz,1,1,-1,tx,ty,tz); neib[1] = {tx,ty,tz};
      Neib(dx,dy,dz,1,0,1,tx,ty,tz);  neib[2] = {tx,ty,tz};
      Neib(dx,dy,dz,1,0,-1,tx,ty,tz); neib[3] = {tx,ty,tz};
      Neib(dx,dy,dz,1,1,0,tx,ty,tz);  neib[4] = {tx,ty,tz};
      Neib(dx,dy,dz,1,-1,0,tx,ty,tz); neib[5] = {tx,ty,tz};
      neib[6] = {dx,dy,dz};

      Neib(dx,dy,dz,-1,0,-1,tx,ty,tz); fneib1[0] = {tx,ty,tz};
      Neib(dx,dy,dz,0,1,-1,tx,ty,tz); fneib1[1] = {tx,ty,tz};
      Neib(dx,dy,dz,0,-1,-1,tx,ty,tz); fneib1[2] = {tx,ty,tz};

      Neib(dx,dy,dz,0,0,-1,tx,ty,tz); fneib2[0] = {tx,ty,tz};
      break;
  }
}
*/


/*
nx::JPS_NEIB::JPS_NEIB()
{
  //std::cout<<"Constructing JPS_NEIB!"<<std::endl;
  int tx,ty,tz;
  for(int dx = -1; dx <= 1; ++dx)
    for(int dy = -1; dy <= 1; ++dy)
      for(int dz = -1; dz <= 1; ++dz)
      {
        int norm1 = std::abs(dx) + std::abs(dy) +std::abs(dz);
        switch(norm1)
        {
          case 0:
          {
            int ptr = 0;
            for(int aa = -1; aa <= 1; ++aa)
              for(int bb = -1; bb <= 1; ++bb)
                for(int cc = -1; cc <= 1; ++cc)
                {
                  if( aa==0 && bb==0 && cc==0 )
                    continue;
                  n_[dx+1][dy+1][dz+1][0][ptr] = aa;
                  n_[dx+1][dy+1][dz+1][1][ptr] = bb;
                  n_[dx+1][dy+1][dz+1][2][ptr] = cc;
                  ++ptr;
                }
            break;
          }
          case 1:
            n_[dx+1][dy+1][dz+1][0][0] = dx;
            n_[dx+1][dy+1][dz+1][1][0] = dy;
            n_[dx+1][dy+1][dz+1][2][0] = dz;

            Neib(dx,dy,dz,0,1,0,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][0] = tx;
            f1_[dx+1][dy+1][dz+1][1][0] = ty;
            f1_[dx+1][dy+1][dz+1][2][0] = tz;
            Neib(dx,dy,dz,0,-1,0,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][1] = tx;
            f1_[dx+1][dy+1][dz+1][1][1] = ty;
            f1_[dx+1][dy+1][dz+1][2][1] = tz;
            Neib(dx,dy,dz,0,1,1,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][2] = tx;
            f1_[dx+1][dy+1][dz+1][1][2] = ty;
            f1_[dx+1][dy+1][dz+1][2][2] = tz;
            Neib(dx,dy,dz,0,0,1,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][3] = tx;
            f1_[dx+1][dy+1][dz+1][1][3] = ty;
            f1_[dx+1][dy+1][dz+1][2][3] = tz;
            Neib(dx,dy,dz,0,-1,1,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][4] = tx;
            f1_[dx+1][dy+1][dz+1][1][4] = ty;
            f1_[dx+1][dy+1][dz+1][2][4] = tz;
            Neib(dx,dy,dz,0,1,-1,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][5] = tx;
            f1_[dx+1][dy+1][dz+1][1][5] = ty;
            f1_[dx+1][dy+1][dz+1][2][5] = tz;
            Neib(dx,dy,dz,0,0,-1,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][6] = tx;
            f1_[dx+1][dy+1][dz+1][1][6] = ty;
            f1_[dx+1][dy+1][dz+1][2][6] = tz;
            Neib(dx,dy,dz,0,-1,-1,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][7] = tx;
            f1_[dx+1][dy+1][dz+1][1][7] = ty;
            f1_[dx+1][dy+1][dz+1][2][7] = tz;

            Neib(dx,dy,dz,1,1,0,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][0] = tx;
            f2_[dx+1][dy+1][dz+1][1][0] = ty;
            f2_[dx+1][dy+1][dz+1][2][0] = tz;
            Neib(dx,dy,dz,1,-1,0,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][1] = tx;
            f2_[dx+1][dy+1][dz+1][1][1] = ty;
            f2_[dx+1][dy+1][dz+1][2][1] = tz;
            Neib(dx,dy,dz,1,1,1,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][2] = tx;
            f2_[dx+1][dy+1][dz+1][1][2] = ty;
            f2_[dx+1][dy+1][dz+1][2][2] = tz;
            Neib(dx,dy,dz,1,0,1,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][3] = tx;
            f2_[dx+1][dy+1][dz+1][1][3] = ty;
            f2_[dx+1][dy+1][dz+1][2][3] = tz;
            Neib(dx,dy,dz,1,-1,1,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][4] = tx;
            f2_[dx+1][dy+1][dz+1][1][4] = ty;
            f2_[dx+1][dy+1][dz+1][2][4] = tz;
            Neib(dx,dy,dz,1,1,-1,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][5] = tx;
            f2_[dx+1][dy+1][dz+1][1][5] = ty;
            f2_[dx+1][dy+1][dz+1][2][5] = tz;
            Neib(dx,dy,dz,1,0,-1,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][6] = tx;
            f2_[dx+1][dy+1][dz+1][1][6] = ty;
            f2_[dx+1][dy+1][dz+1][2][6] = tz;
            Neib(dx,dy,dz,1,-1,-1,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][7] = tx;
            f2_[dx+1][dy+1][dz+1][1][7] = ty;
            f2_[dx+1][dy+1][dz+1][2][7] = tz;
            break;
          case 2:
            Neib(dx,dy,dz,1,1,0,tx,ty,tz);
            n_[dx+1][dy+1][dz+1][0][0] = tx;
            n_[dx+1][dy+1][dz+1][1][0] = ty;
            n_[dx+1][dy+1][dz+1][2][0] = tz;
            Neib(dx,dy,dz,1,-1,0,tx,ty,tz);
            n_[dx+1][dy+1][dz+1][0][1] = tx;
            n_[dx+1][dy+1][dz+1][1][1] = ty;
            n_[dx+1][dy+1][dz+1][2][1] = tz;
            n_[dx+1][dy+1][dz+1][0][2] = dx;
            n_[dx+1][dy+1][dz+1][1][2] = dy;
            n_[dx+1][dy+1][dz+1][2][2] = dz;

            Neib(dx,dy,dz,-1,1,0,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][0] = tx;
            f1_[dx+1][dy+1][dz+1][1][0] = ty;
            f1_[dx+1][dy+1][dz+1][2][0] = tz;
            Neib(dx,dy,dz,-1,-1,0,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][1] = tx;
            f1_[dx+1][dy+1][dz+1][1][1] = ty;
            f1_[dx+1][dy+1][dz+1][2][1] = tz;
            Neib(dx,dy,dz,0,0,1,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][2] = tx;
            f1_[dx+1][dy+1][dz+1][1][2] = ty;
            f1_[dx+1][dy+1][dz+1][2][2] = tz;
            Neib(dx,dy,dz,0,0,-1,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][3] = tx;
            f1_[dx+1][dy+1][dz+1][1][3] = ty;
            f1_[dx+1][dy+1][dz+1][2][3] = tz;
            Neib(dx,dy,dz,-1,1,1,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][4] = tx;
            f1_[dx+1][dy+1][dz+1][1][4] = ty;
            f1_[dx+1][dy+1][dz+1][2][4] = tz;
            Neib(dx,dy,dz,-1,-1,1,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][5] = tx;
            f1_[dx+1][dy+1][dz+1][1][5] = ty;
            f1_[dx+1][dy+1][dz+1][2][5] = tz;
            Neib(dx,dy,dz,-1,1,-1,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][6] = tx;
            f1_[dx+1][dy+1][dz+1][1][6] = ty;
            f1_[dx+1][dy+1][dz+1][2][6] = tz;
            Neib(dx,dy,dz,-1,-1,-1,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][7] = tx;
            f1_[dx+1][dy+1][dz+1][1][7] = ty;
            f1_[dx+1][dy+1][dz+1][2][7] = tz;

            Neib(dx,dy,dz,0,1,0,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][0] = tx;
            f2_[dx+1][dy+1][dz+1][1][0] = ty;
            f2_[dx+1][dy+1][dz+1][2][0] = tz;
            Neib(dx,dy,dz,0,-1,0,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][1] = tx;
            f2_[dx+1][dy+1][dz+1][1][1] = ty;
            f2_[dx+1][dy+1][dz+1][2][1] = tz;
            Neib(dx,dy,dz,1,0,1,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][2] = tx;
            f2_[dx+1][dy+1][dz+1][1][2] = ty;
            f2_[dx+1][dy+1][dz+1][2][2] = tz;
            Neib(dx,dy,dz,1,0,-1,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][3] = tx;
            f2_[dx+1][dy+1][dz+1][1][3] = ty;
            f2_[dx+1][dy+1][dz+1][2][3] = tz;
            Neib(dx,dy,dz,0,1,1,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][4] = tx;
            f2_[dx+1][dy+1][dz+1][1][4] = ty;
            f2_[dx+1][dy+1][dz+1][2][4] = tz;
            Neib(dx,dy,dz,0,-1,1,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][5] = tx;
            f2_[dx+1][dy+1][dz+1][1][5] = ty;
            f2_[dx+1][dy+1][dz+1][2][5] = tz;
            Neib(dx,dy,dz,0,1,-1,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][6] = tx;
            f2_[dx+1][dy+1][dz+1][1][6] = ty;
            f2_[dx+1][dy+1][dz+1][2][6] = tz;
            Neib(dx,dy,dz,0,-1,-1,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][7] = tx;
            f2_[dx+1][dy+1][dz+1][1][7] = ty;
            f2_[dx+1][dy+1][dz+1][2][7] = tz;
            break;
          case 3:
            // The order below is important!
            Neib(dx,dy,dz,1,-1,-1,tx,ty,tz);
            n_[dx+1][dy+1][dz+1][0][0] = tx;
            n_[dx+1][dy+1][dz+1][1][0] = ty;
            n_[dx+1][dy+1][dz+1][2][0] = tz;
            Neib(dx,dy,dz,1,1,-1,tx,ty,tz);
            n_[dx+1][dy+1][dz+1][0][1] = tx;
            n_[dx+1][dy+1][dz+1][1][1] = ty;
            n_[dx+1][dy+1][dz+1][2][1] = tz;
            Neib(dx,dy,dz,1,0,1,tx,ty,tz);
            n_[dx+1][dy+1][dz+1][0][2] = tx;
            n_[dx+1][dy+1][dz+1][1][2] = ty;
            n_[dx+1][dy+1][dz+1][2][2] = tz;
            Neib(dx,dy,dz,1,0,-1,tx,ty,tz);
            n_[dx+1][dy+1][dz+1][0][3] = tx;
            n_[dx+1][dy+1][dz+1][1][3] = ty;
            n_[dx+1][dy+1][dz+1][2][3] = tz;
            Neib(dx,dy,dz,1,1,0,tx,ty,tz);
            n_[dx+1][dy+1][dz+1][0][4] = tx;
            n_[dx+1][dy+1][dz+1][1][4] = ty;
            n_[dx+1][dy+1][dz+1][2][4] = tz;
            Neib(dx,dy,dz,1,-1,0,tx,ty,tz);
            n_[dx+1][dy+1][dz+1][0][5] = tx;
            n_[dx+1][dy+1][dz+1][1][5] = ty;
            n_[dx+1][dy+1][dz+1][2][5] = tz;
            n_[dx+1][dy+1][dz+1][0][6] = dx;
            n_[dx+1][dy+1][dz+1][1][6] = dy;
            n_[dx+1][dy+1][dz+1][2][6] = dz;


            Neib(dx,dy,dz,-1,0,-1,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][0] = tx;
            f1_[dx+1][dy+1][dz+1][1][0] = ty;
            f1_[dx+1][dy+1][dz+1][2][0] = tz;
            Neib(dx,dy,dz,-1,0,-1,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][1] = tx;
            f1_[dx+1][dy+1][dz+1][1][1] = ty;
            f1_[dx+1][dy+1][dz+1][2][1] = tz;
            Neib(dx,dy,dz,-1,0,-1,tx,ty,tz);
            f1_[dx+1][dy+1][dz+1][0][2] = tx;
            f1_[dx+1][dy+1][dz+1][1][2] = ty;
            f1_[dx+1][dy+1][dz+1][2][2] = tz;
            //Neib(dx,dy,dz,0,1,-1,tx,ty,tz);
            //f1_[dx+1][dy+1][dz+1][0][1] = tx;
            //f1_[dx+1][dy+1][dz+1][1][1] = ty;
            //f1_[dx+1][dy+1][dz+1][2][1] = tz;
            //Neib(dx,dy,dz,0,-1,-1,tx,ty,tz);
            //f1_[dx+1][dy+1][dz+1][0][2] = tx;
            //f1_[dx+1][dy+1][dz+1][1][2] = ty;
            //f1_[dx+1][dy+1][dz+1][2][2] = tz;

            Neib(dx,dy,dz,0,0,-1,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][0] = tx;
            f2_[dx+1][dy+1][dz+1][1][0] = ty;
            f2_[dx+1][dy+1][dz+1][2][0] = tz;
            Neib(dx,dy,dz,0,1,-1,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][1] = tx;
            f2_[dx+1][dy+1][dz+1][1][1] = ty;
            f2_[dx+1][dy+1][dz+1][2][1] = tz;
            Neib(dx,dy,dz,0,-1,-1,tx,ty,tz);
            f2_[dx+1][dy+1][dz+1][0][2] = tx;
            f2_[dx+1][dy+1][dz+1][1][2] = ty;
            f2_[dx+1][dy+1][dz+1][2][2] = tz;
            break;
        }
      }
}
*/
/*
void nx::JPS_NEIB::Neib(int dx, int dy, int dz, int nx, int ny, int nz,
                      int& tx, int& ty, int& tz)
{
  if(dx == 0 && dy == 0 && dz == 1){
    tx = -nz; ty = ny; tz = nx;
  }else if(dx == 0 && dy == 0 && dz == -1){
    tx = nz; ty = ny; tz = -nx;
  }else{
    double cos_p, sin_p;
    switch(dz)
    {
      case -1:  cos_p = std::sqrt(2)/2; sin_p = cos_p; break;
      case  0:  cos_p = 1; sin_p = 0; break;
      case  1:  cos_p = std::sqrt(2)/2; sin_p = -cos_p;
    }
    double norm2 = std::sqrt(dx*dx + dy*dy);
    double cos_y = dx/norm2, sin_y = dy/norm2;
    // TODO: This is wrong...
    // scale by std::sqrt(2) to ensure correct truncation
    double aa = std::sqrt(2)*(cos_p*cos_y*nx - sin_y*ny + cos_y*sin_p*nz);
    double bb = std::sqrt(2)*(cos_p*sin_y*nx + cos_y*ny + sin_p*sin_y*nz);
    double cc = std::sqrt(2)*(cos_p*nz-sin_p*nx);

    tx = std::round(std::max(std::min( aa, 1.0 ), -1.0 ));
    ty = std::round(std::max(std::min( bb, 1.0 ), -1.0 ));
    tz = std::round(std::max(std::min( cc, 1.0 ), -1.0 ));
  }
}
*/

/*
void nx::JPS_NEIB::Neib2(int dx, int dy, int dz, int nx, int ny, int nz,
                      double& tx, double& ty, double& tz)
{
  if(dx == 0 && dy == 0 && dz == 1){
    tx = -nz; ty = ny; tz = nx;
  }else if(dx == 0 && dy == 0 && dz == -1){
    tx = nz; ty = ny; tz = -nx;
  }else{
    double cos_p, sin_p;
    switch(dz)
    {
      case -1:  cos_p = std::sqrt(2)/2; sin_p = cos_p; break;
      case  0:  cos_p = 1; sin_p = 0; break;
      case  1:  cos_p = std::sqrt(2)/2; sin_p = -cos_p;
    }
    double norm2 = std::sqrt(dx*dx + dy*dy);
    double cos_y = dx/norm2, sin_y = dy/norm2;
    // TODO: This is wrong...
    // scale by std::sqrt(2) to ensure correct truncation
    tx= (cos_p*cos_y*nx - sin_y*ny + cos_y*sin_p*nz);
    ty = (cos_p*sin_y*nx + cos_y*ny + sin_p*sin_y*nz);
    tz = (cos_p*nz-sin_p*nx);
  }
}
*/
