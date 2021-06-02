/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/datastructures/Properties.hpp"

namespace gls {
namespace datastructures {

// ============================================================================
bool Vertex::isImplicit() const{
    return mImplicit;
}

// ============================================================================
std::ostream& operator<<(std::ostream &os, const Vertex& vi){
    if (vi.isImplicit()){
        os<<vi.mImplicitVertex;
    }
    else{
        os << vi.mExplicitVertex;
    }
    return os;
}

// ============================================================================
bool operator==(Vertex v1, Vertex v2){
    if (v1.isImplicit()){ // note if there is a explicit implicit mismatch there will be probs 
        return v1.mImplicitVertex == v2.mImplicitVertex;
    }
    else{
        return v1.mExplicitVertex == v2.mExplicitVertex;
    }
}

// ============================================================================
bool operator!=(Vertex v1, Vertex v2){
    if (v1.isImplicit()){ // note if there is a explicit implicit mismatch there will be probs 
        return v1.mImplicitVertex != v2.mImplicitVertex;
    }
    else{
        return v1.mExplicitVertex != v2.mExplicitVertex;
    }
}

// ============================================================================
bool operator<(Vertex v1, Vertex v2){
    if (v1.isImplicit()){ // note if there is a explicit implicit mismatch there will be probs 
        return v1.mImplicitVertex < v2.mImplicitVertex;
    }
    else{
        return v1.mExplicitVertex < v2.mExplicitVertex;
    }
}

// ============================================================================
bool Edge::isImplicit() const{
    return mImplicit;
}

// ============================================================================
bool operator==(Edge e1, Edge e2){
    if (e1.isImplicit()){ // note if there is a explicit implicit mismatch there will be probs 
        return e1.mImplicitEdge.first == e2.mImplicitEdge.first && e1.mImplicitEdge.second == e2.mImplicitEdge.second; // note this does not allow multiple edges between vertices
    }
    else{
        return e1.mExplicitEdge == e2.mExplicitEdge;
    }
}

// Properties
// ============================================================================
void VertexProperties::setState(StatePtr state) {
  mState = state;
}

// ============================================================================
StatePtr VertexProperties::getState() {
  return mState;
}

// ============================================================================
void VertexProperties::setCostToCome(double cost) {
  mCostToCome = cost;
}

// ============================================================================
double VertexProperties::getCostToCome() {
  return mCostToCome;
}

// ============================================================================
void VertexProperties::setHeuristic(double heuristic) {
  mHeuristic = heuristic;
}

// ============================================================================
double VertexProperties::getHeuristic() {
  return mHeuristic;
}

// ============================================================================
double VertexProperties::getEstimatedTotalCost() {
  return mCostToCome + mHeuristic;
}

// ============================================================================
void VertexProperties::setParent(Vertex parent) {
  mParent = parent;
}

// ============================================================================
Vertex VertexProperties::getParent() {
  return mParent;
}

// ============================================================================
std::set<Vertex>& VertexProperties::getChildren() {
  return mChildren;
}

// ============================================================================
void VertexProperties::setChildren(std::set<Vertex> children) {
  mChildren = children;
}

// ============================================================================
void VertexProperties::addChild(Vertex child) {
  mChildren.emplace(child);
}

// ============================================================================
void VertexProperties::addChildren(std::set<Vertex> children) {
  for (auto iterS = children.begin(); iterS != children.end(); ++iterS) {
    mChildren.emplace(*iterS);
  }
}

// ============================================================================
void VertexProperties::removeChild(Vertex child) {
  auto iterS = mChildren.find(child);
  if (iterS != mChildren.end())
    mChildren.erase(iterS);
}

// ============================================================================
void VertexProperties::removeChildren(std::set<Vertex> children) {
  for (auto iterS = children.begin(); iterS != children.end(); ++iterS) {
    auto iterRemove = mChildren.find(*iterS);
    if (iterRemove != mChildren.end())
      mChildren.erase(iterRemove);
  }
}

// ============================================================================
void VertexProperties::removeAllChildren() {
  mChildren.clear();
}

// ============================================================================
bool VertexProperties::hasChild(Vertex child) {
  auto iterS = mChildren.find(child);
  if (iterS != mChildren.end())
    return true;
  return false;
}

// ============================================================================
void VertexProperties::setVisitStatus(VisitStatus status) {
  mVisitStatus = status;
}

// ============================================================================
VisitStatus VertexProperties::getVisitStatus() {
  return mVisitStatus;
}

// ============================================================================
void VertexProperties::setCollisionStatus(CollisionStatus status) {
  mCollisionStatus = status;
}

// ============================================================================
CollisionStatus VertexProperties::getCollisionStatus() {
  return mCollisionStatus;
}

// ============================================================================
void EdgeProperties::setLength(double length) {
  mLength = length;
}

// ============================================================================
double EdgeProperties::getLength() {
  return mLength;
}

// ============================================================================
void EdgeProperties::setEvaluationStatus(EvaluationStatus evaluationStatus) {
  mEvaluationStatus = evaluationStatus;
}

// ============================================================================
EvaluationStatus EdgeProperties::getEvaluationStatus() {
  return mEvaluationStatus;
}

// ============================================================================
void EdgeProperties::setCollisionStatus(CollisionStatus status) {
  mCollisionStatus = status;
}

// ============================================================================
CollisionStatus EdgeProperties::getCollisionStatus() {
  return mCollisionStatus;
}

} // namespace datastructures
} // namespace gls
