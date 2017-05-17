/******************************************************************************************
*                                                                                        *
*    Yet Another Graph-Search Based Planning Library (YAGSBPL)                           *
*    A template-based C++ library for graph search and planning                          *
*    Version 2.0                                                                         *
*    ----------------------------------------------------------                          *
*    Copyright (C) 2011  Subhrajit Bhattacharya                                          *
*                                                                                        *
*    This program is free software: you can redistribute it and/or modify                *
*    it under the terms of the GNU General Public License as published by                *
*    the Free Software Foundation, either version 3 of the License, or                   *
*    (at your option) any later version.                                                 *
*                                                                                        *
*    This program is distributed in the hope that it will be useful,                     *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of                      *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                       *
*    GNU General Public License for more details <http://www.gnu.org/licenses/>.         *
*                                                                                        *
*                                                                                        *
*    Contact: subhrajit@gmail.com, http://subhrajit.net/                                 *
*                                                                                        *
*                                                                                        *
******************************************************************************************/
//    For a detailed tutorial and download, visit 
//    http://subhrajit.net/index.php?WPage=yagsbpl


#ifndef __A_STAR_2F585H2B321R_H_
#define __A_STAR_2F585H2B321R_H_


#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include "yagsbpl_base.h"

#define _YAGSBPL_A_STAR__VIEW_PROGRESS 1
#define _YAGSBPL_A_STAR__PRINT_VERBOSE 0 // requires 'void print(std::string pre)' as a member function of the node
#define _YAGSBPL_A_STAR__HANDLE_EVENTS 1

template <class CostType>
class A_star_variables
{
public:
	CostType g;
	bool expanded; // Whether in closed list or not
	bool accessible; // Since the environment is assumed to to change, each node has fixed accessibility
	int seedLineage; // stores which seed the node came from
	
	A_star_variables() { expanded=false; seedLineage=-1; }
};

template <class NodeType, class CostType>
class A_star_planner
{
public:
	// typedef's for convenience:
	typedef  A_star_variables<CostType>  PlannerSpecificVariables;
	typedef  SearchGraphNode< NodeType, CostType, PlannerSpecificVariables >*  GraphNode_p;
	
	// Instance of generac planner
	GenericPlanner< NodeType, CostType, PlannerSpecificVariables > GenericPlannerInstance;
	// Re-mapping of generic planner variables for ease of use (coding convenience)
	GenericSearchGraphDescriptor<NodeType,CostType>* GraphDescriptor;
	HashTableContainer<NodeType,CostType,PlannerSpecificVariables>* hash;
	HeapContainer<NodeType,CostType,PlannerSpecificVariables>* heap;
	
	// Member variables
	double subopEps;
	int heapKeyCount;
	int ProgressShowInterval;
	std::vector< GraphNode_p > bookmarkGraphNodes;
	
	// Optional event handlers - Pointers to function that get called when an event take place
	#if _YAGSBPL_A_STAR__HANDLE_EVENTS
		// Node 'n' is expanded. This can also be handeled by 'stopSearch'.
		void (*event_NodeExpanded_g)(NodeType n, CostType gVal, CostType fVal, int seedLineage);
		void (NodeType::*event_NodeExpanded_nm)(CostType gVal, CostType fVal, int seedLineage);
		// Successor 'nn' is in open list and is just initiated or is updated
		void (*event_SuccUpdated_g)(NodeType n, NodeType nn, CostType edgeCost, CostType gVal, CostType fVal, int seedLineage);
		void (NodeType::*event_SuccUpdated_nm)(NodeType nn, CostType edgeCost, CostType gVal, CostType fVal, int seedLineage);
	#endif
	
	// Helper functions for specialized applications
	// Computes the f-value:
	CostType (*heapFun_fp)(NodeType& n, CostType g, CostType h, int s);
	CostType _heapFun(NodeType& n, CostType g, CostType h, int s);
	
	// Initializer and planner
	A_star_planner()
		{ hash = NULL;
		  subopEps = 1.0; heapKeyCount = 20; ProgressShowInterval = 10000; 
		  event_NodeExpanded_g=NULL; event_NodeExpanded_nm=NULL; event_SuccUpdated_g=NULL; event_SuccUpdated_nm=NULL; 
		  heapFun_fp = NULL; }
	void setParams( double eps=1.0 , int heapKeyCt=20 , int progressDispInterval=10000 ) // call to this is optional.
		{ subopEps = eps; heapKeyCount = heapKeyCt; ProgressShowInterval = progressDispInterval; }
	void init( GenericSearchGraphDescriptor<NodeType,CostType>* theEnv_p=NULL , bool createHashAndHeap=true );
	void init( GenericSearchGraphDescriptor<NodeType,CostType> theEnv ) { init( &theEnv); } // for version compatability
	void plan(void);
	
	// Clear the last plan, but not the hash table. Must be called after at least one initialization.
	//   Use this if you intend to re-plan with different start/goal, 
	//   and/or if isAccessible is changed such that the new planning is being done in a sub-graph of the previous graph,
	//   Won't give correct result if cost function has changed or the new graph has new edges attached to nodes explored in old graph.
	void clearLastPlanAndInit( GenericSearchGraphDescriptor<NodeType,CostType>* theEnv_p=NULL );
	
	// Planner output access: ( to be called after plan(), and before destruction of planner )
	std::vector< NodeType > getGoalNodes(void);
	std::vector< GraphNode_p > getGoalGraphNodePointers(void);
	std::vector< std::vector< NodeType > > getPlannedPaths(void);
	std::vector< CostType > getPlannedPathCosts(void);
	A_star_variables<CostType> getNodeInfo(NodeType n);
	
	// Other variables for querying progress of planning process from other functions
	#if  _YAGSBPL_A_STAR__VIEW_PROGRESS
		clock_t  startclock;
		time_t startsecond;
		int expandcount;
	#endif
};

// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
// Since we use templates, the definitions need to be included in the header file as well.

#include "A_star.cpp"

#endif

