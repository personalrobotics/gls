/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_DATASTRUCTURES_PROPERTIES_HPP_
#define GLS_DATASTRUCTURES_PROPERTIES_HPP_

// STL headers
#include <set>
#include <vector>
#include <string>

// GLS headers
#include "gls/datastructures/State.hpp"
#include "gls/datastructures/Types.hpp"

// TODO (avk): state and length are made public to accomodate the
// roadmapmanager which seems stupid. Change it if possible.

namespace gls {
namespace datastructures {

enum CollisionStatus { Collision, Free };

enum VisitStatus { NotVisited, Visited };

enum EvaluationStatus { NotEvaluated, Evaluated };

// Vertex wrapper class
class Vertex {
    public:
        Vertex(){};
        Vertex(std::string name): mImplicitVertex(name), mImplicit(true){};
        Vertex(EVertex evert): mExplicitVertex(evert){};

        EVertex mExplicitVertex;
        std::string mImplicitVertex;
        friend std::ostream& operator<<(std::ostream &os, const Vertex& vi);
        bool isImplicit() const;
    private:
        bool mImplicit = false;
};

// For unordered map
struct VertexHash{
    std::size_t operator()(const Vertex& k) const{
        if (k.isImplicit()){
            return std::stoi(k.mImplicitVertex);//TODO this could be wrong for -1 in name
        }
        else{
            return k.mExplicitVertex;
        }
    }
};

std::ostream& operator<<(std::ostream &os, const Vertex& vi);
bool operator==(Vertex v1, Vertex v2);
bool operator!=(Vertex v1, Vertex v2);
bool operator<(Vertex v1, Vertex v2);

// Edge wrapper class
class Edge {
    public:
        Edge(){};
        Edge(Vertex v1, Vertex v2, bool implicit):first(v1),second(v2), mImplicit(implicit) {if(implicit){mImplicitEdge = {v1.mImplicitVertex, v2.mImplicitVertex};}};
        Edge(EEdge ei):mExplicitEdge(ei), mImplicit(false) {};
        Edge(std::pair<std::string, std::string> ei):first(ei.first),second(ei.second),mImplicitEdge(ei), mImplicit(true) {};
        Vertex first; // TODO make private
        Vertex second;

        EEdge mExplicitEdge;
        std::pair<std::string, std::string> mImplicitEdge;

        bool isImplicit() const;
    private:
        bool mImplicit = false;
};

// For map
struct EdgeHash{
    std::string operator()(const std::pair<Vertex, Vertex>& k) const{
        if (k.first.isImplicit()){
            return k.first.mImplicitVertex + k.second.mImplicitVertex;
        }
        else{
            return std::to_string(k.first.mExplicitVertex) + std::to_string(k.second.mExplicitVertex);
        }
    }
};

// For fail fast selector
struct EdgePriorHash{
    double operator()(const std::pair<Vertex, Vertex>& k) const{
        return 0.5; // TODO (avk)
    }
};

bool operator==(Edge e1, Edge e2);

class VertexProperties {
public:
  // Set state wrapper around underlying OMPL state.
  void setState(StatePtr state);

  // Get state wrapper around underlying OMPL state.
  StatePtr getState();

  // Set cost-to-come.
  void setCostToCome(double cost);

  // Get cost-to-come.
  double getCostToCome();

  // Set heuristic.
  void setHeuristic(double heuristic);

  // Get heuristic.
  double getHeuristic();

  // Get estimated total cost.
  double getEstimatedTotalCost();

  // Set the vertex parent.
  void setParent(Vertex parent);

  // Get the vertex parent.
  Vertex getParent();

  // Get the set of all children.
  // TODO (avk): Remove this with getDescendents.
  // Since this is only used in a loop.
  std::set<Vertex>& getChildren();

  // Set the children of the vertex in the search tree.
  void setChildren(std::set<Vertex> children);

  // Add a single child to the vertex.
  void addChild(Vertex child);

  // Add multiple children to the vertex.
  void addChildren(std::set<Vertex> children);

  // Remove a single child.
  void removeChild(Vertex child);

  // Remove multiple children.
  void removeChildren(std::set<Vertex> children);

  // Clears the children.
  void removeAllChildren();

  // Checks if vertex has given child.
  bool hasChild(Vertex child);

  // Sets the visit status of the vertex.
  void setVisitStatus(VisitStatus status);

  // Get the visit status of the vertex.
  VisitStatus getVisitStatus();

  // Sets the collision status of the vertex.
  void setCollisionStatus(CollisionStatus status);

  // Get the collision status of the vertex.
  CollisionStatus getCollisionStatus();

  /// Underlying state.
  /// TODO (avk): why is this public?
  StatePtr mState;

private:
  /// Cost-to-Come.
  double mCostToCome{std::numeric_limits<double>::infinity()};

  /// Heuristic value.
  double mHeuristic{std::numeric_limits<double>::infinity()};

  /// Parent.
  Vertex mParent;

  /// Children.
  std::set<Vertex> mChildren;

  /// Visitation status.
  VisitStatus mVisitStatus{VisitStatus::NotVisited};

  /// Collision status.
  CollisionStatus mCollisionStatus{CollisionStatus::Free};
};

class EdgeProperties {
public:
  // Sets the length of the edge.
  void setLength(double length);

  // Get the length of the edge.
  double getLength();

  // Sets the evaluation status.
  void setEvaluationStatus(EvaluationStatus evaluationStatus);

  // Get the evaluation status.
  EvaluationStatus getEvaluationStatus();

  // Sets the collision status.
  void setCollisionStatus(CollisionStatus status);

  // Get the collision status.
  CollisionStatus getCollisionStatus();

  /// The length of the edge using the space distance metric.
  /// TODO (avk): Why is this public?
  double mLength;

private:
  /// Evaluation status.
  EvaluationStatus mEvaluationStatus{EvaluationStatus::NotEvaluated};

  /// Collision status..
  CollisionStatus mCollisionStatus{CollisionStatus::Free};
};

/// Shared pointer to a Vertex.
typedef std::shared_ptr<Vertex> VertexPtr;

/// Shared pointer to a const Vertex.
typedef std::shared_ptr<const Vertex> ConstVertexPtr;

/// Shared pointer to an Edge.
typedef std::shared_ptr<Edge> EdgePtr;

/// Shared pointer to a const Edge.
typedef std::shared_ptr<const Edge> ConstEdgePtr;

/// Path represented by a series of vertices.
typedef std::vector<Vertex> Path;

/// Shared pointer to a Path.
typedef std::shared_ptr<Path> PathPtr;

/// Shared pointer to a const Path.
typedef std::shared_ptr<const Path> ConstPathPtr;

} // namespace datastructures
} // namespace gls

#endif // GLS_DATASTRUCTURES_PROPERTIES_HPP_
