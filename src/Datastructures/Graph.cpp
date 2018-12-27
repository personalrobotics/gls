/* Authors: Aditya Vamsikrishna Mandalika */

#include "GLS/Datastructures/Graph.hpp"

namespace gls {
namespace datastructures {

// ============================================================================
void VertexProperties::setCostToCome(double cost)
{
  mCostToCome = cost;
}

// ============================================================================
double VertexProperties::getCostToCome()
{
  return mCostToCome;
}

// ============================================================================
void VertexProperties::setHeuristic(double heuristic)
{
  mHeuristic = heuristic;
}

// ============================================================================
double VertexProperties::getHeuristic()
{
  return mHeuristic;
}

// ============================================================================
double VertexProperties::getEstimatedTotalCost()
{
  return mCostToCome + mHeuristic;
}

// ============================================================================
void VertexProperties::setParent(Vertex parent)
{
  mParent = parent;
}

// ============================================================================
Vertex VertexProperties::getParent()
{
  return mParent;
}

// ============================================================================
std::set<Vertex>& VertexProperties::getChildren()
{
  return mChildren;
}

// ============================================================================
void VertexProperties::setChildren(std::set<Vertex> children)
{
  mChildren = children;
}

// ============================================================================
void VertexProperties::addChild(Vertex child)
{
  mChildren.emplace(child);
}

// ============================================================================
void VertexProperties::addChildren(std::set<Vertex> children)
{
  for (auto iterS = children.begin(); iterS != children.end(); ++iterS)
  {
    mChildren.emplace(*iterS);
  }
}

// ============================================================================
void VertexProperties::removeChild(Vertex child)
{
  auto iterS = mChildren.find(child);
  if (iterS != mChildren.end())
    mChildren.erase(iterS);
}

// ============================================================================
void VertexProperties::removeChildren(std::set<Vertex> children)
{
  for (auto iterS = children.begin(); iterS != children.end(); ++iterS)
  {
    auto iterRemove = mChildren.find(*iterS);
    if (iterRemove != mChildren.end())
      mChildren.erase(iterRemove);
  }
}

// ============================================================================
void VertexProperties::removeAllChildren()
{
  mChildren.clear();
}

// ============================================================================
bool VertexProperties::hasChild(Vertex child)
{
  auto iterS = mChildren.find(child);
  if (iterS != mChildren.end())
    return true;
  return false;
}

// ============================================================================
void VertexProperties::setVisitStatus(VisitStatus status)
{
  mVisitStatus = status;
}

// ============================================================================
VisitStatus VertexProperties::getVisitStatus()
{
  return mVisitStatus;
}

// ============================================================================
void VertexProperties::setCollisionStatus(CollisionStatus status)
{
  mCollisionStatus = status;
}

// ============================================================================
CollisionStatus VertexProperties::getCollisionStatus()
{
  return mCollisionStatus;
}

// ============================================================================
void EdgeProperties::setLength(double length)
{
  mLength = length;
}

// ============================================================================
double EdgeProperties::getLength()
{
  return mLength;
}

// ============================================================================
void EdgeProperties::setEvaluationStatus(EvaluationStatus evaluationStatus)
{
  mEvaluationStatus = evaluationStatus;
}

// ============================================================================
EvaluationStatus EdgeProperties::getEvaluationStatus()
{
  return mEvaluationStatus;
}

// ============================================================================
void EdgeProperties::setCollisionStatus(CollisionStatus status)
{
  mCollisionStatus = status;
}

// ============================================================================
CollisionStatus EdgeProperties::getCollisionStatus()
{
  return mCollisionStatus;
}

} // datastructures
} // gls
