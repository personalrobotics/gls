// Standard C++ libraries
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include <typeinfo>

// OMPL base libraries
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// Custom header files
#include "gls/GLS.hpp"

using namespace gls::datastructures;

std::ostream& operator<<(std::ostream& output, const gls::datastructures::IEdge& e) {
      return output << "(" << e.first << ", " << e.second << ")";
}

/// Dummy collision checker that return true always.
/// This is bound to the stateValidityChecker of the ompl StateSpace.
/// \param[in] state The ompl state to check for validity.
bool isPointValid(const ompl::base::State* state) {
  double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  if (values[0] >= 0.15 && values[0] <= 0.85)
    if (values[1] <= 0.85)
      return false;
  return true;
}

// Discretize state to graph
// This must be user-defined because it varies per state space
StatePtr fit_state2lattice(StatePtr state, gls::io::MotionPrimitiveReader *mReader,std::shared_ptr<ompl::base::RealVectorStateSpace> space){
    std::vector<double> vals;
    vals.reserve(4);
    space->copyToReals(vals, state->getOMPLState());
    vals[0] = (double)CONTXY2DISC(vals[0], mReader->resolution);
    vals[1] = (double)CONTXY2DISC(vals[1], mReader->resolution);
    vals[2] = (double) mReader->ContTheta2DiscNew(vals[2]);

    space->copyFromReals(state->getOMPLState(), vals);
    return state;
}

// Transition function from one state to another
// Each new vertex must have a unique vertex_descriptor for lookup
// The same state should ALWAYS have the same descriptor
// AKA use the state to create the descriptor
std::vector<std::pair<IVertex, VertexProperties>> transition_function(IVertex vi, VertexProperties vp, std::shared_ptr<ompl::base::RealVectorStateSpace> space, gls::io::MotionPrimitiveReader *mReader){
    std::vector<std::pair<IVertex, VertexProperties>> neighbors; 

    // Get State
    double* values = vp.getState()->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    int x = (int)values[0]; 
    int y = (int)values[1]; 
    int theta = (int)values[2];
    int level = (int)values[3]; // TODO use me!

    // OLD
    // Readout state from vertex_descriptor 
    //int x_idx = vi.find("|");
    //int y_idx = vi.find("|", x_idx+1);
    //int x = std::stoi(vi.substr(0, x_idx));
    //int y = std::stoi(vi.substr(x_idx+1, y_idx-x_idx-1));
    //int theta = std::stoi(vi.substr(y_idx+1, vi.size()-y_idx-1));

    // Neighbors
    int nx, ny, ntheta;
    IVertex neighbor;
    VertexProperties neighborProperties;
    std::vector<gls::io::MotionPrimitive> mprims = mReader->mprimV;
    for (gls::io::MotionPrimitive mprim : mprims){

        // prims for current theta
        if(mprim.starttheta_c == theta){
            // Apply mprim
            nx = mprim.endcell.x + x;
            ny = mprim.endcell.y + y;
            ntheta = mprim.endcell.theta;
            neighbor = std::to_string(nx) + 
                "|" + std::to_string(ny) + 
                "|" + std::to_string(ntheta);

            // Set state
            // TODO this seems inefficient to make a new state
            neighborProperties = VertexProperties();
            StatePtr newState(new State(space));
            space->copyFromReals(newState->getOMPLState(), std::vector<double>{nx, ny, ntheta, -1});
            neighborProperties.setState(newState);

            neighbors.push_back(std::pair<IVertex, VertexProperties>(neighbor, neighborProperties));
        }
    }

    return neighbors;
}

int main (int argc, char const *argv[]) {

  // Load Motion Primitives
  gls::io::MotionPrimitiveReader* mReader = new gls::io::MotionPrimitiveReader();
  mReader->ReadMotionPrimitives("/home/schmittle/Research/boxes/pysbpl/pysbpl/mprim/mushr.mprim");

  // Define the state space: R^4
  auto space = std::make_shared<ompl::base::RealVectorStateSpace>(4);
  auto bounds = ompl::base::RealVectorBounds(4); 
  bounds.setLow(-100.0); // TODO set real bounds
  bounds.setHigh(100.0);
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
  space->setLongestValidSegmentFraction(0.1 / space->getMaximumExtent());
  space->setup();

  // Space Information
  std::function<bool(const ompl::base::State*)> isStateValid
      = std::bind(isPointValid, std::placeholders::_1);
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
  si->setStateValidityChecker(isStateValid);
  si->setup();

  // Create implicit graph
  // Bind so we can pass mReader
  ImplicitGraph g(std::bind(&fit_state2lattice,  
                std::placeholders::_1, 
                mReader,
                space), 
              std::bind(&transition_function, 
                    std::placeholders::_1, 
                    std::placeholders::_2, 
                    space, 
                    mReader));
  IVertex sourceVertex = "source";
  IVertex targetVertex = "target";

  StatePtr sourceState(new State(space));
  space->copyFromReals(sourceState->getOMPLState(), std::vector<double>{1, 20, 1, -1});

  StatePtr targetState(new State(space));
  space->copyFromReals(targetState->getOMPLState(), std::vector<double>{3, 3, 3, -1});

  g.addVertex(sourceVertex, sourceState);
  g.addVertex(targetVertex, targetState);

  // TODO setup GLS and create flip
  std::cout << "Vertices, outgoing edges, and adjacent vertices" << std::endl;
  std::vector<IVertex> verts = vertices(g);
  IVertexIter vi_end = verts.end();
  for (IVertexIter vi = verts.begin(); vi != vi_end; vi++) {
    IVertex u = *vi;
    std::cout << "Vertex " << u << ": "<<std::endl;

    std::cout << " Adjacent vertices "<<std::endl;
    std::vector<IVertex> ajs = adjacent_vertices(u, g);
    IVertexIter ai_end = ajs.end();
    IVertexIter ai = ajs.begin();
    for (IVertexIter ai = ajs.begin(); ai != ai_end; ai++) {
      IVertex ni = *ai;
      double* vals = g[ni].getState()->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;
      std::cout <<  vals[0] << " "<< vals[1]<< " "<< vals[2] << std::endl;
    }
    
    std::cout << std::endl;
  }
  std::cout << num_vertices(g) << " vertices" << std::endl << std::endl;

  std::cout << "Edges and weights" << std::endl;
  std::vector<IEdge> edgs = edges(g);
  IEdgeIter ei_end = edgs.end();
  for (IEdgeIter ei = edgs.begin(); ei != ei_end; ei++) {
    IEdge e = *ei;
    std::cout << e << std::endl;
  }
  std::cout << num_edges(g) << " edges"  << std::endl;

  return 0;
}
