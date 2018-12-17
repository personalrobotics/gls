/* Authors: Aditya Vamsikrishna Mandalika */

#include "GLS/Datastructures/Graph.hpp"

namespace gls {
namespace datastructures {

void VertexProperties::setCostToCome(double cost)
{
  mCostToCome = cost;
}

double VertexProperties::getCostToCome() const
{
  return mCostToCome;
}

void VertexProperties::setHeuristic(double heuristic)
{
  mHeuristic = heuristic;
}

double VertexProperties::getHeuristic() const
{
  return mHeuristic;
}

void VertexProperties::setParent(Vertex parent)
{
  mParent = parent;
}

Vertex VertexProperties::getParent() const
{
  return mParent;
}

void VertexProperties::setChildren(std::set<Vertex> children)
{
  mChildren = children;
}

void VertexProperties::addChild(Vertex child)
{
  mChildren.emplace(child);
}

void VertexProperties::addChildren(std::set<Vertex> children)
{
  for (auto iterS = children.begin(); iterS != children.end(); ++iterS)
  {
    mChildren.emplace(*iterS);
  }
}

void VertexProperties::removeChild(Vertex child)
{
  auto iterS = mChildren.find(child);
  if (iterS != mChildren.end())
    mChildren.erase(iterS);
}

void VertexProperties::removeChildren(std::set<Vertex> children)
{
  for (auto iterS = children.begin(); iterS != children.end(); ++iterS)
  {
    auto iterRemove = mChildren.find(*iterS);
    if (iterRemove != mChildren.end())
      mChildren.erase(iterRemove);
  }
}

void VertexProperties::removeAllChildren()
{
  mChildren.clear();
}

bool VertexProperties::hasChild(Vertex child) const
{
  auto iterS = mChildren.find(child);
  if (iterS != mChildren.end())
    return true;
  return false;
}

void EdgeProperties::setLength(double length)
{
  mLength = length;
}

double EdgeProperties::getLength() const
{
  return mLength;
}

void EdgeProperties::setEvaluationStatus(EvaluationStatus evaluationStatus)
{
  mEvaluated = evaluationStatus;
}

EvaluationStatus EdgeProperties::getEvaluationStatus()
{
  return mEvaluationStatus;
}

} // datastructures
} // gls
