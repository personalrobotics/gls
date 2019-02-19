/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_INPUT_ROADMAPMANAGER_HPP_
#define GLS_INPUT_ROADMAPMANAGER_HPP_

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <stdlib.h>

#include <boost/function.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/shared_ptr.hpp>

#include <ompl/base/ScopedState.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace gls {
namespace io {

/* RoadmapFromFilePutStateMap */
/// The map used to decode the .graphml file and populate the vertex states.
/// \tparam PropMap The type of property map for vertex states.
/// \tparam StateWrapper The wrapper for the ompl state.
template <class PropMap, class StateWrapper>
class RoadmapFromFilePutStateMap
{
public:
  typedef boost::writable_property_map_tag category;
  typedef typename boost::property_traits<PropMap>::key_type key_type;
  typedef std::string value_type;
  typedef std::string reference;

  RoadmapFromFilePutStateMap(
      PropMap propMap, ompl::base::StateSpacePtr space, size_t dim)
    : mPropMap{propMap}, mSpace{space}, mDim{dim}
  {
    // Do nothing.
  }

  const PropMap mPropMap;
  ompl::base::StateSpacePtr mSpace;
  const size_t mDim;
};

// Do not allow calling get on this property map
template <class PropMap, class StateWrapper>
inline std::string get(
    const RoadmapFromFilePutStateMap<PropMap, StateWrapper>&,
    const typename RoadmapFromFilePutStateMap<PropMap, StateWrapper>::key_type&)
{
  abort();
}

/// Convert string representation of vector state space to ompl state
template <class PropMap, class StateWrapper>
inline void put(
    const RoadmapFromFilePutStateMap<PropMap, StateWrapper>& map,
    const typename RoadmapFromFilePutStateMap<PropMap, StateWrapper>::key_type&
        k,
    const std::string representation)
{
  get(map.mPropMap, k).reset(new StateWrapper(map.mSpace));
  ompl::base::State* ver_state{get(map.mPropMap, k)->getOMPLState()};
  double* values{
      ver_state->as<ompl::base::RealVectorStateSpace::StateType>()->values};
  std::stringstream ss(representation);
  for (size_t ui = 0; ui < map.mDim; ui++)
  {
    ss >> values[ui];
  }
}

/* RoadmapFromFile */
template <class Graph, class VStateMap, class StateWrapper, class ELength>
class RoadmapFromFile
{
  typedef boost::graph_traits<Graph> GraphTypes;
  typedef typename GraphTypes::vertex_descriptor Vertex;
  typedef typename GraphTypes::vertex_iterator VertexIter;
  typedef typename GraphTypes::edge_descriptor Edge;
  typedef typename GraphTypes::edge_iterator EdgeIter;

public:
  const std::string mFilename;

  RoadmapFromFile(const ompl::base::StateSpacePtr space, std::string filename)
    : mSpace(space), mFilename(filename)
  {
    mDim = mSpace->getDimension();
  }

  ~RoadmapFromFile()
  {
  }

  void generate(Graph& g, VStateMap stateMap, ELength lengthMap)
  {
    boost::dynamic_properties props;
    props.property(
        "state",
        RoadmapFromFilePutStateMap<VStateMap, StateWrapper>(
            stateMap, mSpace, mDim));

    std::ifstream fp;
    fp.open(mFilename.c_str());
    boost::read_graphml(fp, g, props);
    fp.close();

    EdgeIter ei, ei_end;
    for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
    {
      ompl::base::State* state1 = get(stateMap, source(*ei, g))->getOMPLState();
      ompl::base::State* state2 = get(stateMap, target(*ei, g))->getOMPLState();
      put(lengthMap, *ei, mSpace->distance(state1, state2));
    }
  }

private:
  size_t mDim;
  const ompl::base::StateSpacePtr mSpace;
};

} // namespace io
} // namespace gls

#endif // GLS_INPUT_ROADMAPMANAGER_HPP_
