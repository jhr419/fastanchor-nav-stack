#include "fast_anchor_localization/fast_anchor_localization.hpp"

namespace fast_anchor_localization
{

const char * algorithmMigrationNote()
{
  return "The original ICP algorithm remains in fast_anchor_localization_node.cpp "
         "to avoid changing behavior during the package refactor.";
}

}  // namespace fast_anchor_localization
