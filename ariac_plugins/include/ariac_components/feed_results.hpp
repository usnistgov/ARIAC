#pragma once

#include <string>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>

#include <ariac_interfaces/msg/cell_types.hpp>

namespace ariac_components
{
  struct FeedResults{
    std::map<int, int> cell_counts = {
      {ariac_interfaces::msg::CellTypes::LI_ION, 0},
      {ariac_interfaces::msg::CellTypes::NIMH, 0}
    };
    int num_defective = 0;

    static bool equal(const ariac_components::FeedResults &a, const ariac_components::FeedResults &b){
      return (a.cell_counts == b.cell_counts &&
              a.num_defective == b.num_defective
            );
    }

    friend std::ostream& operator<<(std::ostream& os, const FeedResults& fr){
      os << "Number of defective: " << fr.num_defective
         << ", LI_ION: " << fr.cell_counts.at(static_cast<int>(ariac_interfaces::msg::CellTypes::LI_ION))
         << ", NIMH: " << fr.cell_counts.at(static_cast<int>(ariac_interfaces::msg::CellTypes::NIMH));
      return os;
    }
  };
}

namespace gz::sim::components
{
  using FeedResults = Component<ariac_components::FeedResults, class FeedResultsTag>;

  GZ_SIM_REGISTER_COMPONENT("FeedResults", FeedResults)
}