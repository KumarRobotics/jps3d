/******************************************************************************************
*                                                                                        *
*    Yet Another Graph-Search Based Planning Library (YAGSBPL) *
*    A template-based C++ library for graph search and planning *
*    Version 2.1 *
*    ---------------------------------------------------------- *
*    Copyright (C) 2013  Subhrajit Bhattacharya *
*                                                                                        *
*    This program is free software: you can redistribute it and/or modify *
*    it under the terms of the GNU General Public License as published by *
*    the Free Software Foundation, either version 3 of the License, or *
*    (at your option) any later version. *
*                                                                                        *
*    This program is distributed in the hope that it will be useful, *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the *
*    GNU General Public License for more details <http://www.gnu.org/licenses/>.
**
*                                                                                        *
*                                                                                        *
*    Contact: subhrajit@gmail.com, http://subhrajit.net/ *
*                                                                                        *
*                                                                                        *
******************************************************************************************/
//    For a detailed tutorial and download, visit
//    http://subhrajit.net/index.php?WPage=yagsbpl

#ifndef __YAGSBPL_BASE_2F585H2B321R_H_
#define __YAGSBPL_BASE_2F585H2B321R_H_

#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <limits>
#include <boost/function.hpp>

#define _yagsbpl_abs(x) ((x) > 0 ? (x) : (-x))
/*
#define _yagsbpl_display_version                                               \
  {                                                                            \
    if (!YAGSBPL_vDisplay_done) {                                              \
      printf("\n*** You are using YAGSBPL v 2.1. ***\n");                      \
      YAGSBPL_vDisplay_done = true;                                            \
    }                                                                          \
  }
bool YAGSBPL_vDisplay_done = false;
*/

// Declaration
template <class NodeType, class CostType, class PlannerSpecificVariables>
class SearchGraphNode;

// ---

// This class stores information about edges emanating from or incident to a
// node
template <class NodeType, class CostType, class PlannerSpecificVariables>
class NodeLinks {
public:
  std::vector<SearchGraphNode<NodeType, CostType, PlannerSpecificVariables> *> *
      targets;
  std::vector<CostType> *costs;

  // -------------------
  NodeLinks() {
    targets = NULL;
    costs = NULL;
  }
  bool empty(void) { return (!targets); }
  int size(void) {
    if (targets)
      return targets->size();
    else
      return -1;
  }
  void init(int count = 0) {
    targets = new std::vector<
        SearchGraphNode<NodeType, CostType, PlannerSpecificVariables> *>(count);
    costs = new std::vector<CostType>(count);
  }

  void push_back(
      SearchGraphNode<NodeType, CostType, PlannerSpecificVariables> *node_p,
      CostType cost) {
    targets->push_back(node_p);
    costs->push_back(cost);
  }
  void
  set(int a,
      SearchGraphNode<NodeType, CostType, PlannerSpecificVariables> *node_p,
      CostType cost) {
    targets->operator[](a) = node_p;
    costs->operator[](a) = cost;
  }

  SearchGraphNode<NodeType, CostType, PlannerSpecificVariables> *
  getLinkSearchGraphNode(int a) {
    return (targets->operator[](a));
  }
  CostType getLinkCost(int a) { return (costs->operator[](a)); }

  ~NodeLinks() {
    if (targets)
      delete targets;
    if (costs)
      delete costs;
  }
};

// Extension of NodeType for search problem
template <class NodeType, class CostType, class PlannerSpecificVariables>
class SearchGraphNode {
public:
  NodeType n;

  // ---------------------------------------------------------------
  // TODO for planner: Planner must set each of the following every time a
  // "non-initiated" node is encountered.
  bool initiated; // Used for tracking newly-create nodes by hash table. To be
                  // set to "true" by planner.
  NodeLinks<NodeType, CostType, PlannerSpecificVariables> successors;
  NodeLinks<NodeType, CostType, PlannerSpecificVariables>
      predecessors; // Planner may or may not use this
  SearchGraphNode<NodeType, CostType, PlannerSpecificVariables> *came_from;
  PlannerSpecificVariables
      plannerVars; // Other variables, if required by planner
  CostType f;      // f-value: Used for maintaining the heap

  // ---------------------------------------------------------------
  // These variables are used by heap container
  bool inHeap;
  int heapArrayPos;

  SearchGraphNode() {
    initiated = false;
    came_from = NULL;
    inHeap = false;
  }
};

// ---------------------------------------------------------------------

template <class NodeType, class CostType, class PlannerSpecificVariables>
class HeapContainer {
public:
  typedef SearchGraphNode<NodeType, CostType, PlannerSpecificVariables> *
      heapItem_p;

  int heap_size;
  std::vector<heapItem_p> heapArray;

  HeapContainer(int kc = 20) { heap_size = 0; }

  typedef enum {
    UNKNOWN_BUBDIR,
    UPONLY_BUBDIR,
    DOWNONLY_BUBDIR
  } BubbleDirection;

  void push(heapItem_p np);
  heapItem_p pop(void);
  void update(heapItem_p np, BubbleDirection bubdir = UNKNOWN_BUBDIR);
  void remove(heapItem_p np);
  void clear(void) {
    while (heap_size > 0)
      pop();
  };
  bool empty(void) { return (heap_size == 0); };
  int size(void) { return (heap_size); };
};

// =============================================================================
// Helper classes for 'GenericSearchGraphDescriptor'

template <class NodeType, class CostType>
class SearchGraphDescriptorFunctionContainer {
public:
  bool func_redefined;
  virtual int getHashBin(NodeType &n) = 0;
  virtual bool isAccessible(NodeType &n) = 0;
  virtual void getSuccessors(NodeType &n, std::vector<NodeType> *s,
                             std::vector<CostType> *c) = 0;
  virtual void getPredecessors(NodeType &n, std::vector<NodeType> *s,
                               std::vector<CostType> *c) = 0;
  virtual bool storePath(NodeType &n) = 0;
  virtual bool stopSearch(NodeType &n) = 0;

  virtual CostType getHeuristics(NodeType &n1, NodeType &n2) = 0;
};

template <class NodeType, class CostType, class ContainerClass>
class SearchGraphDescriptorFunctionPointerContainer
    : public SearchGraphDescriptorFunctionContainer<NodeType, CostType> {
public:
  ContainerClass *p;
  int (ContainerClass::*getHashBin_fp)(NodeType &n);
  bool (ContainerClass::*isAccessible_fp)(NodeType &n);
  void (ContainerClass::*getSuccessors_fp)(NodeType &n,
                                           std::vector<NodeType> *s,
                                           std::vector<CostType> *c);
  void (ContainerClass::*getPredecessors_fp)(NodeType &n,
                                             std::vector<NodeType> *s,
                                             std::vector<CostType> *c);
  CostType (ContainerClass::*getHeuristics_fp)(NodeType &n1, NodeType &n2);
  bool (ContainerClass::*storePath_fp)(NodeType &n);
  bool (ContainerClass::*stopSearch_fp)(NodeType &n);
  // ----
  int getHashBin(NodeType &n) {
    if (p && getHashBin_fp)
      return ((p->*getHashBin_fp)(n));
    else
      this->func_redefined = false;
  }
  bool isAccessible(NodeType &n) {
    if (p && isAccessible_fp)
      return ((p->*isAccessible_fp)(n));
    else
      this->func_redefined = false;
  }
  void getSuccessors(NodeType &n, std::vector<NodeType> *s,
                     std::vector<CostType> *c) {
    if (p && getSuccessors_fp)
      return ((p->*getSuccessors_fp)(n, s, c));
    else
      this->func_redefined = false;
  }
  void getPredecessors(NodeType &n, std::vector<NodeType> *s,
                       std::vector<CostType> *c) {
    if (p && getPredecessors_fp)
      return ((p->*getPredecessors_fp)(n, s, c));
    else
      this->func_redefined = false;
  }
  CostType getHeuristics(NodeType &n1, NodeType &n2) {
    if (p && getHeuristics_fp)
      return ((p->*getHeuristics_fp)(n1, n2));
    else
      this->func_redefined = false;
  }
  bool storePath(NodeType &n) {
    if (p && storePath_fp)
      return ((p->*storePath_fp)(n));
    else
      this->func_redefined = false;
  }
  bool stopSearch(NodeType &n) {
    if (p && stopSearch_fp)
      return ((p->*stopSearch_fp)(n));
    else
      this->func_redefined = false;
  }
};

// ----
// TODO for environment: 'GenericSearchGraphDescriptor' is the only class an
// instance of which needs to be created.
//   That needs to be passed into the "init" function of the "GenericPlanner"

template <class NodeType, class CostType> class GenericSearchGraphDescriptor {
public:
  // Primary functions
  // int (*getHashBin_fp)(NodeType &n);
  boost::function<int(NodeType &n)> getHashBin_fp;
  // bool (*isAccessible_fp)(NodeType &n); // optional
  boost::function<bool(NodeType &n)> isAccessible_fp; // optional
  boost::function<void(NodeType &n, std::vector<NodeType> *s,
                       std::vector<CostType> *c)>
      getSuccessors_fp; // s: successors, c: tranition costs
  boost::function<void(NodeType &n, std::vector<NodeType> *s,
                       std::vector<CostType> *c)>
      getPredecessors_fp; // May not be used by all planners
  boost::function<CostType(NodeType &n1, NodeType &n2)>
      getHeuristics_fp;                            // optional
  boost::function<bool(NodeType &n)> storePath_fp; // optional
  boost::function<bool(NodeType &n)>
      stopSearch_fp; // optional if "HeuristicsTargetNode" is given.

  // If pointers to primary functions cannot be provided, we can alternatively
  // use the virtual functions in the FunctionContainer
  SearchGraphDescriptorFunctionContainer<NodeType, CostType> *func_container;

  // Primary variables
  int hashTableSize; // Number of hash bins. "getHashBin" must return a value
                     // between 0 and hashTableSize
  // An initial set of "start" nodes to be put in heap. At least one of the
  // following two need to be set.
  std::vector<NodeType> SeedNodes;
  NodeType SeedNode;
  // "Goal" used for computing Heuristics. Not required if "getHeuristics" is
  // not provided.
  NodeType TargetNode; // If "stopSearch" is not provided, this is used to
                       // determine when to stop.

  // Other variables and functions - constructor chooses a default
  int hashBinSizeIncreaseStep; // optional

  // ---------------------------------------------------------------
  // Constructor and other functions
  GenericSearchGraphDescriptor();
  void init(void); // TODO: Planner is supposed to call this at initialization.
  // Derived functions - planner should use these, and NOT the former functions
  int _getHashBin(NodeType &n);
  bool _isAccessible(NodeType &n);
  void _getSuccessors(NodeType &n, std::vector<NodeType> *s,
                      std::vector<CostType> *c);
  void _getPredecessors(NodeType &n, std::vector<NodeType> *s,
                        std::vector<CostType> *c);
  CostType _getHeuristics(NodeType &n1, NodeType &n2);
  CostType _getHeuristicsToTarget(NodeType &n);
  bool _storePath(NodeType &n);
  bool _stopSearch(NodeType &n);
};

// =============================================================================

template <class NodeType, class CostType, class PlannerSpecificVariables>
class HashTableContainer {
public:
  GenericSearchGraphDescriptor<NodeType, CostType> *friendGraphDescriptor_p;
  int hashTableSize;
  std::vector<SearchGraphNode<NodeType, CostType, PlannerSpecificVariables> *> *
      HashTable;
  void init_HastTable(int hash_table_size);
  SearchGraphNode<NodeType, CostType, PlannerSpecificVariables> *
  getNodeInHash(NodeType n); // Returns pointer to already-existing
  ~HashTableContainer() {
    if (HashTable) {
      for (int a = 0; a < hashTableSize; a++)
        for (unsigned int b = 0; b < HashTable[a].size(); b++)
          delete HashTable[a][b];
      delete[] HashTable;
    }
  };
};

// =============================================================================
// TODO for planner: A planner needs to maintain an instance of this.
//   	Inheritance is also possible, but not recommended because of templates.

template <class NodeType, class CostType, class PlannerSpecificVariables>
class GenericPlanner {
public:
  GenericSearchGraphDescriptor<NodeType, CostType> *GraphDescriptor;
  HashTableContainer<NodeType, CostType, PlannerSpecificVariables> *hash;
  HeapContainer<NodeType, CostType, PlannerSpecificVariables> *heap;

  void init(GenericSearchGraphDescriptor<NodeType, CostType> theEnv,
            int heapKeyNum = 20); // A planner must call this explicitly.
  ~GenericPlanner() {
    delete GraphDescriptor;
    delete hash;
    delete heap;
  }
};

// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
// Since we use templates, the definitions need to be included in the header
// file as well.

#include "yagsbpl_base.cpp"

#endif
