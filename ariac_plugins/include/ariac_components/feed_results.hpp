#ifndef ARIAC_COMPONENTS_FEED_RESULTS_HH_
#define ARIAC_COMPONENTS_FEED_RESULTS_HH_

#include <map>
#include <string>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

#include <ariac_interfaces/msg/cell_types.hpp>

namespace ariac_components
{
  struct FeedResults
  {
    std::map<int, int> cell_counts = {
      {ariac_interfaces::msg::CellTypes::LI_ION, 0},
      {ariac_interfaces::msg::CellTypes::NIMH, 0}
    };
    int num_defective = 0;

    bool operator==(const FeedResults &_other) const
    {
      return (this->cell_counts == _other.cell_counts &&
              this->num_defective == _other.num_defective);
    }

    static bool equal(const ariac_components::FeedResults &a, const ariac_components::FeedResults &b)
    {
      return a == b;
    }
  };

  namespace serializers
  {
    class FeedResultsSerializer
    {
      public: static std::ostream &Serialize(std::ostream &_out,
                                             const FeedResults &_feedResults)
      {
        _out << _feedResults.cell_counts.size() << " ";
        for (const auto &[key, value] : _feedResults.cell_counts)
        {
          _out << key << " " << value << " ";
        }
        _out << _feedResults.num_defective;
        return _out;
      }

      public: static std::istream &Deserialize(std::istream &_in,
                                               FeedResults &_feedResults)
      {
        size_t map_size;
        _in >> map_size;
        _feedResults.cell_counts.clear();
        for (size_t i = 0; i < map_size; ++i)
        {
          int key, value;
          _in >> key >> value;
          _feedResults.cell_counts[key] = value;
        }
        _in >> _feedResults.num_defective;
        return _in;
      }
    };
  }
}

namespace gz::sim::components
{
  using FeedResults = Component<ariac_components::FeedResults,
                                class FeedResultsTag>;
  GZ_SIM_REGISTER_COMPONENT("ariac_components.FeedResults", FeedResults);
}

#endif
