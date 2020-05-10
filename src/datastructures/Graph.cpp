/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/datastructures/Graph.hpp"

namespace gls {

// ============================================================================
void GLS::VertexProperties::setState(StatePtr state) { mState = state; }

// ============================================================================
GLS::StatePtr GLS::VertexProperties::getState() { return mState; }

// ============================================================================
void GLS::VertexProperties::setCostToCome(double cost) { mCostToCome = cost; }

// ============================================================================
double GLS::VertexProperties::getCostToCome() { return mCostToCome; }

// ============================================================================
void GLS::VertexProperties::setHeuristic(double heuristic) {
  mHeuristic = heuristic;
}

// ============================================================================
double GLS::VertexProperties::getHeuristic() { return mHeuristic; }

// ============================================================================
double GLS::VertexProperties::getEstimatedTotalCost() {
  return mCostToCome + mHeuristic;
}

// ============================================================================
void GLS::VertexProperties::setParent(Vertex parent) { mParent = parent; }

// ============================================================================
Vertex GLS::VertexProperties::getParent() { return mParent; }

// ============================================================================
std::set<Vertex>& GLS::VertexProperties::getChildren() { return mChildren; }

// ============================================================================
void GLS::VertexProperties::setChildren(std::set<Vertex> children) {
  mChildren = children;
}

// ============================================================================
void GLS::VertexProperties::addChild(Vertex child) { mChildren.emplace(child); }

// ============================================================================
void GLS::VertexProperties::addChildren(std::set<Vertex> children) {
  for (auto iterS = children.begin(); iterS != children.end(); ++iterS) {
    mChildren.emplace(*iterS);
  }
}

// ============================================================================
void GLS::VertexProperties::removeChild(Vertex child) {
  auto iterS = mChildren.find(child);
  if (iterS != mChildren.end()) mChildren.erase(iterS);
}

// ============================================================================
void GLS::VertexProperties::removeChildren(std::set<Vertex> children) {
  for (auto iterS = children.begin(); iterS != children.end(); ++iterS) {
    auto iterRemove = mChildren.find(*iterS);
    if (iterRemove != mChildren.end()) mChildren.erase(iterRemove);
  }
}

// ============================================================================
void GLS::VertexProperties::removeAllChildren() { mChildren.clear(); }

// ============================================================================
bool GLS::VertexProperties::hasChild(Vertex child) {
  auto iterS = mChildren.find(child);
  if (iterS != mChildren.end()) return true;
  return false;
}

// ============================================================================
void GLS::VertexProperties::setVisitStatus(VisitStatus status) {
  mVisitStatus = status;
}

// ============================================================================
VisitStatus GLS::VertexProperties::getVisitStatus() { return mVisitStatus; }

// ============================================================================
void GLS::VertexProperties::setCollisionStatus(CollisionStatus status) {
  mCollisionStatus = status;
}

// ============================================================================
CollisionStatus GLS::VertexProperties::getCollisionStatus() {
  return mCollisionStatus;
}

// ============================================================================
void GLS::VertexProperties::setSearchIterator(
    const SearchQueue::SearchQueueIterator iterator) {
  mSearchIterator = iterator;
  mInSearchQueue = true;
}

// ============================================================================
GLS::SearchQueue::SearchQueueIterator GLS::VertexProperties::getSearchIterator()
    const {
  return mSearchIterator;
}

// ============================================================================
void GLS::VertexProperties::clearSearchIterator() {
  mSearchIterator = SearchQueue::SearchQueueIterator();
  mInSearchQueue = false;
}

// ============================================================================
bool GLS::VertexProperties::inSearchQueue() const { return mInSearchQueue; }

// ============================================================================
void GLS::EdgeProperties::setLength(double length) { mLength = length; }

// ============================================================================
double GLS::EdgeProperties::getLength() { return mLength; }

// ============================================================================
void GLS::EdgeProperties::setEvaluationStatus(
    EvaluationStatus evaluationStatus) {
  mEvaluationStatus = evaluationStatus;
}

// ============================================================================
EvaluationStatus GLS::EdgeProperties::getEvaluationStatus() {
  return mEvaluationStatus;
}

// ============================================================================
void GLS::EdgeProperties::setCollisionStatus(CollisionStatus status) {
  mCollisionStatus = status;
}

// ============================================================================
CollisionStatus GLS::EdgeProperties::getCollisionStatus() {
  return mCollisionStatus;
}

}  // namespace gls
