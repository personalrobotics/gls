/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_SELECTOR_FAILFASTSELECTOR_HPP_
#define GLS_SELECTOR_FAILFASTSELECTOR_HPP_

#include <unordered_map>

#include "gls/selector/Selector.hpp"

namespace gls {
namespace selector {

/// Unordered map defined as: <source name> <target name> <prior>
typedef std::unordered_map<std::pair<std::string, std::string>,
                           double,
                           boost::hash<std::pair<std::string, std::string>>>
    edgeToPriorMap;

/// Selector that evaluates the edge most likely to be in collision.
class FailFastSelector : public Selector
{
public:
  /// Constructor.
  FailFastSelector(const edgeToPriorMap& priorMap);

  /// Documentation inherited.
  gls::datastructures::Path selectEdgesToEvaluate(
      gls::datastructures::Path path) override;

protected:
  /// Map that stores the priors.
  const edgeToPriorMap mPriorMap;
};

} // namespace selector
} // namespace gls

#endif // GLS_SELECTOR_FAILFASTSELECTOR_HPP_
