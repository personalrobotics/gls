/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_INPUT_ROADMAPMANAGER_HPP_
#define GLS_INPUT_ROADMAPMANAGER_HPP_

#include <ompl/base/ScopedState.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <stdlib.h>

#include <boost/function.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "gls/GLS.hpp"

namespace gls {

/* RoadmapFromFilePutStateMap */
/// The map used to decode the .graphml file and populate the vertex states.
/// \tparam PropMap The type of property map for vertex states.
/// \tparam StateWrapper The wrapper for the ompl state.
template <class PropMap, class StateWrapper>
class RoadmapFromFilePutStateMap {
 public:
  typedef boost::writable_property_map_tag category;
  typedef typename boost::property_traits<PropMap>::key_type key_type;
  typedef std::string value_type;
  typedef std::string reference;

  RoadmapFromFilePutStateMap(PropMap propMap, ompl::base::StateSpacePtr space,
                             size_t dim)
      : mPropMap{propMap}, mSpace{space}, mDim{dim} {
    // Do nothing.
  }

  const PropMap mPropMap;
  ompl::base::StateSpacePtr mSpace;
  const size_t mDim;
};

// Do not allow calling get on this property map
template <class PropMap, class StateWrapper>
inline std::string get(const RoadmapFromFilePutStateMap<PropMap, StateWrapper>&,
                       const typename RoadmapFromFilePutStateMap<
                           PropMap, StateWrapper>::key_type&) {
  abort();
}

/// Convert string representation of vector state space to ompl state
template <class PropMap, class StateWrapper>
inline void put(
    const RoadmapFromFilePutStateMap<PropMap, StateWrapper>& map,
    const typename RoadmapFromFilePutStateMap<PropMap, StateWrapper>::key_type&
        k,
    const std::string representation) {
  get(map.mPropMap, k).reset(new StateWrapper(map.mSpace));
  ompl::base::State* ver_state{get(map.mPropMap, k)->getOMPLState()};

  std::vector<double> values;
  values.resize(map.mDim);

  std::stringstream ss(representation);
  for (size_t ui = 0; ui < map.mDim; ui++) {
    ss >> values[ui];
  }
  map.mSpace->copyFromReals(ver_state, values);
}

/* RoadmapFromFilePutEdgeLengthMap */
/// The map used to decode the .graphml file and populate the edge length
/// \tparam PropMap The type of property map for vertex states
template <class PropMap>
class RoadmapFromFilePutEdgeLengthMap {
 public:
  typedef boost::writable_property_map_tag category;
  typedef typename boost::property_traits<PropMap>::key_type key_type;
  typedef std::string value_type;
  typedef std::string reference;
  const PropMap mPropMap;

  RoadmapFromFilePutEdgeLengthMap(PropMap propMap) : mPropMap(propMap) {}
};

/// Do not allow calling get on this property map
template <class PropMap>
inline std::string get(
    const RoadmapFromFilePutEdgeLengthMap<PropMap>&,
    const typename RoadmapFromFilePutEdgeLengthMap<PropMap>::key_type&) {
  abort();
}

template <class PropMap>
inline void put(
    const RoadmapFromFilePutEdgeLengthMap<PropMap>& map,
    const typename RoadmapFromFilePutEdgeLengthMap<PropMap>::key_type& k,
    const std::string representation) {
  put(map.mPropMap, k, stod(representation));
}

/* RoadmapFromFile */
template <class Graph, class VStateMap, class StateWrapper, class ELength>
class GLS::RoadmapFromFile {
  typedef boost::graph_traits<Graph> GraphTypes;
  typedef typename GraphTypes::vertex_descriptor Vertex;
  typedef typename GraphTypes::vertex_iterator VertexIter;
  typedef typename GraphTypes::edge_descriptor Edge;
  typedef typename GraphTypes::edge_iterator EdgeIter;

 public:
  const std::string mFilename;

  RoadmapFromFile(const ompl::base::StateSpacePtr space, std::string filename)
      : mFilename(filename), mSpace(space) {
    mDim = mSpace->getDimension();
  }

  ~RoadmapFromFile() {}

  void generate(Graph& g, VStateMap stateMap, ELength lengthMap) {
    boost::dynamic_properties props;
    props.property("state", RoadmapFromFilePutStateMap<VStateMap, StateWrapper>(
                                stateMap, mSpace, mDim));
    props.property("length",
                   RoadmapFromFilePutEdgeLengthMap<ELength>(lengthMap));

    std::ifstream fp;
    fp.open(mFilename.c_str());
    boost::read_graphml(fp, g, props);
    fp.close();
  }

 private:
  size_t mDim;
  const ompl::base::StateSpacePtr mSpace;
};

}  // namespace gls

#endif  // GLS_INPUT_ROADMAPMANAGER_HPP_
