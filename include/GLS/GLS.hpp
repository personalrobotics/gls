/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_GLS_HPP_
#define GLS_GLS_HPP_

// STL headers
#include <vector>
#include <string> 
#include <unordered_set>
#include <queue>
#include <exception>

// Boost headers
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

// OMPL headers
#include <ompl/base/Planner.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

#include "GLS/Event.hpp"
#include "GLS/Selector.hpp"

namespace GLS {

/// The OMPL Planner class that implements the algorithm
class GLS: public ompl::base::Planner
{
public:
  /// Constructor
  /// \param[in] si The OMPL space information manager
  explicit GLS(const ompl::base::SpaceInformationPtr &si);

  /// \param[in] si The OMPL space information manager
  /// \param[in] roadmapFileName The path to the .graphml file that encodes the roadmap.
  GLS(const ompl::base::SpaceInformationPtr &si,
      const std::string& roadmapFileName);

  /// Destructor
  ~GLS(void);

  // OMPL Methods.

  // Setters and const Getters.
  void setEvent(gls::event::Event event);
  // gls::event::ConstEventPtr getEvent() const;

  void setSelector(gls::selector::Selector selector);
  // gls::selector::ConstSelectorPtr getSelector() const;

private:
  // Event
  gls::event::Event mEvent;

  // Selector
  gls::selector::Selector mSelector;

  // Search Methods

}

#endif // GLS_GLS_HPP_