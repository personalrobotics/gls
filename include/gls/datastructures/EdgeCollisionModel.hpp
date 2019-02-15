/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_DATASTRUCTURES_EDGECOLLISIONMODEL_HPP_
#define GLS_DATASTRUCTURES_EDGECOLLISIONMODEL_HPP_

// STL headers
#include <vector>
#include <map>

namespace gls {
namespace datastructures {

/// Generates a Van Der Corput sequence ordering for states to check along an edge.
/// TODO (avk): Move this to another folder and generalize it.
class EdgeCollisionModel
{
public:
  /// Constructor
  EdgeCollisionModel();

  /// Destructor
  ~EdgeCollisionModel(void);

  /// Returns a map from integer index to fractional position along edge.
  /// For an edge that has n states, we generate a sequence of n fractional
  /// positions along the edge to check for collision, based on Van Der Corput Sequences
  /// We start at 1/2, then 1/4, and 3/4 and so on upto n states.
  /// \param[in] n The number of states along the edge
  const std::vector<std::pair<int,int>>& get(int n);

private:
  // Map from integer index to fractional position along edge.
  std::map<int, const std::vector< std::pair<int,int>>> mCache;
};

} // namespace datastructures
} // namespace gls

#endif // GLS_DATASTRUCTURES_EDGECOLLISIONMODEL_HPP_