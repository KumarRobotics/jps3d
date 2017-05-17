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

#define f_val(ptr) ((ptr)->f)

template <class NodeType, class CostType, class PlannerSpecificVariables>
void HeapContainer<NodeType, CostType, PlannerSpecificVariables>::update(
    heapItem_p np, BubbleDirection bubdir) {
  // Does both bubble up and down
  if (!(np->inHeap))
    return;
  int currentPos = np->heapArrayPos;

  // Bubble up
  if (bubdir != DOWNONLY_BUBDIR) {
    int parentPos = (currentPos - 1) / 2; // position of parent
    if (f_val(heapArray[parentPos]) > f_val(np)) {
      heapArray[currentPos] = heapArray[parentPos];
      heapArray[parentPos] = np;
      heapArray[currentPos]->heapArrayPos = currentPos;
      heapArray[parentPos]->heapArrayPos = parentPos;
      update(heapArray[parentPos], UPONLY_BUBDIR); // recursive call. If heap,
                                                   // downward bubbling won't be
                                                   // needed.
      return; // no need to check children, since chilren will have higher
              // values
    }
    if (bubdir == UPONLY_BUBDIR)
      return;
  }

  // Bubble down
  int leftChildPos = 2 * currentPos + 1;
  int rightChildPos = 2 * currentPos + 2;
  int exchangeChildPos;
  if (leftChildPos < heapArray.size() && rightChildPos < heapArray.size())
    exchangeChildPos =
        (f_val(heapArray[leftChildPos]) > f_val(heapArray[rightChildPos]))
            ? rightChildPos
            : leftChildPos;
  else if (leftChildPos < heapArray.size())
    exchangeChildPos = leftChildPos;
  else if (rightChildPos < heapArray.size())
    exchangeChildPos = rightChildPos;
  else
    return;

  if (f_val(np) > f_val(heapArray[exchangeChildPos])) {
    heapArray[currentPos] = heapArray[exchangeChildPos];
    heapArray[exchangeChildPos] = np;
    heapArray[currentPos]->heapArrayPos = currentPos;
    heapArray[exchangeChildPos]->heapArrayPos = exchangeChildPos;
    update(heapArray[exchangeChildPos], DOWNONLY_BUBDIR); // recursive call. If
                                                          // heap, upward
                                                          // bubbling won't be
                                                          // needed.
    return;
  }
}

// SB_BINHEAP -------------------------------------------------------

template <class NodeType, class CostType, class PlannerSpecificVariables>
void HeapContainer<NodeType, CostType, PlannerSpecificVariables>::push(
    SearchGraphNode<NodeType, CostType, PlannerSpecificVariables> *np) {
  // Assumes np is not in heap. No check is done for that.
  // Add as leaf
  heapArray.push_back(np);
  heap_size++; // SB_BINHEAP2
  np->inHeap = true;
  np->heapArrayPos = heapArray.size() - 1;
  // Bubble up
  update(np, UPONLY_BUBDIR);
}

template <class NodeType, class CostType, class PlannerSpecificVariables>
SearchGraphNode<NodeType, CostType, PlannerSpecificVariables> *
HeapContainer<NodeType, CostType, PlannerSpecificVariables>::pop(void) {
  heapItem_p ret = heapArray[0];
  ret->inHeap = false;
  ret->heapArrayPos = -1;
  // Take last leaf and place it at root
  heapArray[0] = heapArray[heapArray.size() - 1];
  heapArray[0]->heapArrayPos = 0;
  heapArray.pop_back();
  heap_size--;
  // Bubble down
  update(heapArray[0], DOWNONLY_BUBDIR);

  return (ret);
}

template <class NodeType, class CostType, class PlannerSpecificVariables>
void HeapContainer<NodeType, CostType, PlannerSpecificVariables>::remove(
    SearchGraphNode<NodeType, CostType, PlannerSpecificVariables> *np) {
  CostType tmp_h = np->h;
  np->h = -std::numeric_limits<CostType>::infinity();
  update(np);
  pop();
  np->h = tmp_h;
}

// =================================================================================

template <class NodeType, class CostType>
GenericSearchGraphDescriptor<NodeType,
                             CostType>::GenericSearchGraphDescriptor() {
  hashBinSizeIncreaseStep = 128;
  // Initiating pointers to all functions as NULL - makes easy to check if a
  // function was defined
  getHashBin_fp = NULL;
  isAccessible_fp = NULL;
  getSuccessors_fp = NULL;
  getPredecessors_fp = NULL;
  getHeuristics_fp = NULL;
  storePath_fp = NULL;
  stopSearch_fp = NULL;
  func_container = NULL;
}

template <class NodeType, class CostType>
void GenericSearchGraphDescriptor<NodeType, CostType>::init(void) {
  if (SeedNodes.size() == 0) {
    SeedNodes.push_back(SeedNode);
  }
  // Other initializations - to be included in future versions
}

// ---------------------------------

template <class NodeType, class CostType>
int GenericSearchGraphDescriptor<NodeType, CostType>::_getHashBin(NodeType &n) {
  if (getHashBin_fp)
    return (getHashBin_fp(n));

  if (func_container) {
    func_container->func_redefined = true;
    int ret = func_container->getHashBin(n);
    if (func_container->func_redefined)
      return ret;
  }

  return 0;
}

template <class NodeType, class CostType>
bool GenericSearchGraphDescriptor<NodeType, CostType>::_isAccessible(
    NodeType &n) {
  if (isAccessible_fp)
    return (isAccessible_fp(n));

  if (func_container) {
    func_container->func_redefined = true;
    bool ret = func_container->isAccessible(n);
    if (func_container->func_redefined)
      return ret;
  }

  return true;
}

template <class NodeType, class CostType>
void GenericSearchGraphDescriptor<NodeType, CostType>::_getSuccessors(
    NodeType &n, std::vector<NodeType> *s, std::vector<CostType> *c) {
  if (getSuccessors_fp)
    getSuccessors_fp(n, s, c);
  else if (func_container)
    func_container->getSuccessors(n, s, c);
}

template <class NodeType, class CostType>
void GenericSearchGraphDescriptor<NodeType, CostType>::_getPredecessors(
    NodeType &n, std::vector<NodeType> *s, std::vector<CostType> *c) {
  if (getPredecessors_fp)
    getPredecessors_fp(n, s, c);
  else if (func_container)
    func_container->getPredecessors(n, s, c);
}

template <class NodeType, class CostType>
CostType
GenericSearchGraphDescriptor<NodeType, CostType>::_getHeuristics(NodeType &n1,
                                                                 NodeType &n2) {
  if (getHeuristics_fp)
    return (getHeuristics_fp(n1, n2));

  if (func_container) {
    func_container->func_redefined = true;
    CostType ret = func_container->getHeuristics(n1, n2);
    if (func_container->func_redefined)
      return ret;
  }

  return ((CostType)0);
}

template <class NodeType, class CostType>
CostType
GenericSearchGraphDescriptor<NodeType, CostType>::_getHeuristicsToTarget(
    NodeType &n) {
  return (_getHeuristics(n, TargetNode));
}

template <class NodeType, class CostType>
bool GenericSearchGraphDescriptor<NodeType, CostType>::_storePath(NodeType &n) {
  if (storePath_fp)
    return (storePath_fp(n));

  if (func_container) {
    func_container->func_redefined = true;
    bool ret = func_container->storePath(n);
    if (func_container->func_redefined)
      return ret;
  }

  return false;
}

template <class NodeType, class CostType>
bool GenericSearchGraphDescriptor<NodeType, CostType>::_stopSearch(
    NodeType &n) {
  if (stopSearch_fp)
    return (stopSearch_fp(n));

  if (func_container) {
    func_container->func_redefined = true;
    bool ret = func_container->stopSearch(n);
    if (func_container->func_redefined)
      return ret;
  }

  if (n == TargetNode)
    return true;

  return false;
}

// =================================================================================

template <class NodeType, class CostType, class PlannerSpecificVariables>
void HashTableContainer<NodeType, CostType, PlannerSpecificVariables>::
    init_HastTable(int hash_table_size) {
  hashTableSize = hash_table_size;
  HashTable = new std::vector<SearchGraphNode<
      NodeType, CostType, PlannerSpecificVariables> *>[hashTableSize];
  for (int a = 0; a < hashTableSize; a++)
    if (HashTable[a].capacity() >= HashTable[a].size() - 1)
      HashTable[a].reserve(HashTable[a].capacity() +
                           friendGraphDescriptor_p->hashBinSizeIncreaseStep);
}

template <class NodeType, class CostType, class PlannerSpecificVariables>
SearchGraphNode<NodeType, CostType, PlannerSpecificVariables> *
HashTableContainer<NodeType, CostType, PlannerSpecificVariables>::getNodeInHash(
    NodeType n) {
  // Search in bin
  int hashBin = friendGraphDescriptor_p->_getHashBin(n);
  // A SIGSEGV signal generated from here most likely 'getHashBin' returned a
  // bin index larger than (hashTableSize-1).
  for (unsigned int a = 0; a < HashTable[hashBin].size(); a++)
    if (HashTable[hashBin][a]->n == n)
      return (HashTable[hashBin][a]);

  // If new node, create it!
  SearchGraphNode<NodeType, CostType, PlannerSpecificVariables> *
      newSearchGraphNode =
          new SearchGraphNode<NodeType, CostType, PlannerSpecificVariables>;
  newSearchGraphNode->n = n; // WARNING: Nothing else is set yet!
  if (HashTable[hashBin].capacity() <= HashTable[hashBin].size() + 1)
    HashTable[hashBin].reserve(
        HashTable[hashBin].capacity() +
        friendGraphDescriptor_p->hashBinSizeIncreaseStep);
  HashTable[hashBin].push_back(newSearchGraphNode);
  return (newSearchGraphNode);
}

// =================================================================================

template <class NodeType, class CostType, class PlannerSpecificVariables>
void GenericPlanner<NodeType, CostType, PlannerSpecificVariables>::init(
    GenericSearchGraphDescriptor<NodeType, CostType> theEnv, int heapKeyNum) {
  //_yagsbpl_display_version;

  GraphDescriptor = new GenericSearchGraphDescriptor<NodeType, CostType>;
  *GraphDescriptor = theEnv;

  hash = new HashTableContainer<NodeType, CostType, PlannerSpecificVariables>;
  hash->friendGraphDescriptor_p = GraphDescriptor;
  hash->init_HastTable(GraphDescriptor->hashTableSize);

  heap = new HeapContainer<NodeType, CostType, PlannerSpecificVariables>(
      heapKeyNum);
}
