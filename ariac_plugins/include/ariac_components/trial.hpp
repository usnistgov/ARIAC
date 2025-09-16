#pragma once

#include <string>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>

namespace ariac_components
{
  struct ConveyorMalfunction
  {
    int start_time;
    int duration;
    
    bool operator==(const ConveyorMalfunction& b) const {
      return (start_time == b.start_time && duration == b.duration);
    }
  };

  struct VacuumToolMalfunction
  {
    int tool;
    int grasp_occurrence;
    
    bool operator==(const VacuumToolMalfunction& b) const {
      return (grasp_occurrence == b.grasp_occurrence && tool == b.tool);
    }
  };

  struct VoltageTesterMalfunction
  {
    int tester;
    int start_time;
    int duration;
    
    bool operator==(const VoltageTesterMalfunction& b) const {
      return (tester == b.tester && start_time == b.start_time && duration == b.duration);
    }
  };

  struct Trial
  {
    std::string id;
    int seed;
    double defect_rate;
    int time_limit;
    int num_kits;
    int num_modules;

    std::vector<int> possible_defects;

    std::vector<ConveyorMalfunction> conveyor_malfunctions;
    std::vector<VacuumToolMalfunction> vacuum_tool_malfunctions;
    std::vector<VoltageTesterMalfunction> voltage_tester_malfunctions;

    static bool equal(const ariac_components::Trial &a, const ariac_components::Trial &b)
    {
      return (a.id == b.id &&
              a.seed == b.seed &&
              a.defect_rate == b.defect_rate &&
              a.time_limit == b.time_limit &&
              a.num_kits == b.num_kits &&
              a.num_modules == b.num_modules &&
              a.possible_defects == b.possible_defects &&
              a.conveyor_malfunctions == b.conveyor_malfunctions &&
              a.vacuum_tool_malfunctions == b.vacuum_tool_malfunctions &&
              a.voltage_tester_malfunctions == b.voltage_tester_malfunctions);
    }

    friend std::ostream& operator<<(std::ostream& os, const Trial& t){
      os << "ID: " << t.id
         << ", Seed: " << t.seed
         << ", Defect rate: " << t.defect_rate
         << ", Time limit: " << t.time_limit
         << ", Number of kits: " << t.num_kits
         << ", Number of modules: " << t.num_modules;
      return os;
    }
  };
}

namespace gz::sim::components
{
  struct TrialTag;
  using Trial = Component<ariac_components::Trial, TrialTag>;

  GZ_SIM_REGISTER_COMPONENT("Trial", Trial)
}