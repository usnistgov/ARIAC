#ifndef ARIAC_COMPONENTS_INSPECTION_RESULTS_HH_
#define ARIAC_COMPONENTS_INSPECTION_RESULTS_HH_

#include <string>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace ariac_components
{
  struct InspectionResults
  {
    double avg_report_time = 0.0;
    int num_reports_submitted = 0;
    int num_correct_reports = 0;
    int num_correct_report_classifications = 0;

    bool operator==(const InspectionResults &_other) const
    {
      return (this->avg_report_time == _other.avg_report_time &&
              this->num_reports_submitted == _other.num_reports_submitted &&
              this->num_correct_reports == _other.num_correct_reports &&
              this->num_correct_report_classifications == _other.num_correct_report_classifications);
    }

    static bool equal(const ariac_components::InspectionResults &a, const ariac_components::InspectionResults &b)
    {
      return a == b;
    }
  };

  namespace serializers
  {
    class InspectionResultsSerializer
    {
      public: static std::ostream &Serialize(std::ostream &_out,
                                             const InspectionResults &_inspectionResults)
      {
        _out << _inspectionResults.avg_report_time << " "
             << _inspectionResults.num_reports_submitted << " "
             << _inspectionResults.num_correct_reports << " "
             << _inspectionResults.num_correct_report_classifications;
        return _out;
      }

      public: static std::istream &Deserialize(std::istream &_in,
                                               InspectionResults &_inspectionResults)
      {
        _in >> _inspectionResults.avg_report_time
            >> _inspectionResults.num_reports_submitted
            >> _inspectionResults.num_correct_reports
            >> _inspectionResults.num_correct_report_classifications;
        return _in;
      }
    };
  }
}

namespace gz::sim::components
{
  using InspectionResults = Component<ariac_components::InspectionResults,
                                      class InspectionResultsTag,
                                      ariac_components::serializers::InspectionResultsSerializer>;
  GZ_SIM_REGISTER_COMPONENT("ariac_components.InspectionResults", InspectionResults)
}

#endif
