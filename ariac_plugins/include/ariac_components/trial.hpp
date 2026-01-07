#ifndef ARIAC_COMPONENTS_TRIAL_HH_
#define ARIAC_COMPONENTS_TRIAL_HH_

#include <string>
#include <vector>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace ariac_components
{
  struct ConveyorMalfunction
  {
    int start_time;
    int duration;

    bool operator==(const ConveyorMalfunction &_other) const
    {
      return (this->start_time == _other.start_time &&
              this->duration == _other.duration);
    }
  };

  struct VacuumToolMalfunction
  {
    int tool;
    int grasp_occurrence;

    bool operator==(const VacuumToolMalfunction &_other) const
    {
      return (this->grasp_occurrence == _other.grasp_occurrence &&
              this->tool == _other.tool);
    }
  };

  struct VoltageTesterMalfunction
  {
    int tester;
    int start_time;
    int duration;

    bool operator==(const VoltageTesterMalfunction &_other) const
    {
      return (this->tester == _other.tester &&
              this->start_time == _other.start_time &&
              this->duration == _other.duration);
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

    bool operator==(const Trial &_other) const
    {
      return (this->id == _other.id &&
              this->seed == _other.seed &&
              this->defect_rate == _other.defect_rate &&
              this->time_limit == _other.time_limit &&
              this->num_kits == _other.num_kits &&
              this->num_modules == _other.num_modules &&
              this->possible_defects == _other.possible_defects &&
              this->conveyor_malfunctions == _other.conveyor_malfunctions &&
              this->vacuum_tool_malfunctions == _other.vacuum_tool_malfunctions &&
              this->voltage_tester_malfunctions == _other.voltage_tester_malfunctions);
    }

    static bool equal(const ariac_components::Trial &a, const ariac_components::Trial &b)
    {
      return a == b;
    }
  };

  namespace serializers
  {
    class TrialSerializer
    {
      public: static std::ostream &Serialize(std::ostream &_out,
                                             const Trial &_trial)
      {
        _out << _trial.id << " "
             << _trial.seed << " "
             << _trial.defect_rate << " "
             << _trial.time_limit << " "
             << _trial.num_kits << " "
             << _trial.num_modules << " ";

        // Serialize possible_defects vector
        _out << _trial.possible_defects.size() << " ";
        for (const auto &defect : _trial.possible_defects)
        {
          _out << defect << " ";
        }

        // Serialize conveyor_malfunctions vector
        _out << _trial.conveyor_malfunctions.size() << " ";
        for (const auto &malfunction : _trial.conveyor_malfunctions)
        {
          _out << malfunction.start_time << " "
               << malfunction.duration << " ";
        }

        // Serialize vacuum_tool_malfunctions vector
        _out << _trial.vacuum_tool_malfunctions.size() << " ";
        for (const auto &malfunction : _trial.vacuum_tool_malfunctions)
        {
          _out << malfunction.tool << " "
               << malfunction.grasp_occurrence << " ";
        }

        // Serialize voltage_tester_malfunctions vector
        _out << _trial.voltage_tester_malfunctions.size() << " ";
        for (const auto &malfunction : _trial.voltage_tester_malfunctions)
        {
          _out << malfunction.tester << " "
               << malfunction.start_time << " "
               << malfunction.duration << " ";
        }

        return _out;
      }

      public: static std::istream &Deserialize(std::istream &_in,
                                               Trial &_trial)
      {
        _in >> _trial.id
            >> _trial.seed
            >> _trial.defect_rate
            >> _trial.time_limit
            >> _trial.num_kits
            >> _trial.num_modules;

        // Deserialize possible_defects vector
        size_t defects_size;
        _in >> defects_size;
        _trial.possible_defects.clear();
        _trial.possible_defects.reserve(defects_size);
        for (size_t i = 0; i < defects_size; ++i)
        {
          int defect;
          _in >> defect;
          _trial.possible_defects.push_back(defect);
        }

        // Deserialize conveyor_malfunctions vector
        size_t conveyor_size;
        _in >> conveyor_size;
        _trial.conveyor_malfunctions.clear();
        _trial.conveyor_malfunctions.reserve(conveyor_size);
        for (size_t i = 0; i < conveyor_size; ++i)
        {
          ConveyorMalfunction malfunction;
          _in >> malfunction.start_time
              >> malfunction.duration;
          _trial.conveyor_malfunctions.push_back(malfunction);
        }

        // Deserialize vacuum_tool_malfunctions vector
        size_t vacuum_size;
        _in >> vacuum_size;
        _trial.vacuum_tool_malfunctions.clear();
        _trial.vacuum_tool_malfunctions.reserve(vacuum_size);
        for (size_t i = 0; i < vacuum_size; ++i)
        {
          VacuumToolMalfunction malfunction;
          _in >> malfunction.tool
              >> malfunction.grasp_occurrence;
          _trial.vacuum_tool_malfunctions.push_back(malfunction);
        }

        // Deserialize voltage_tester_malfunctions vector
        size_t voltage_size;
        _in >> voltage_size;
        _trial.voltage_tester_malfunctions.clear();
        _trial.voltage_tester_malfunctions.reserve(voltage_size);
        for (size_t i = 0; i < voltage_size; ++i)
        {
          VoltageTesterMalfunction malfunction;
          _in >> malfunction.tester
              >> malfunction.start_time
              >> malfunction.duration;
          _trial.voltage_tester_malfunctions.push_back(malfunction);
        }

        return _in;
      }
    };
  }
}

namespace gz::sim::components
{
  using Trial = Component<ariac_components::Trial,
                          class TrialTag,
                          ariac_components::serializers::TrialSerializer>;
  GZ_SIM_REGISTER_COMPONENT("ariac_components.Trial", Trial)
}

#endif
