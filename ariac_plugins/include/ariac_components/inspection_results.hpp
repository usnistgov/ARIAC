#pragma once

#include <string>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>

namespace ariac_components
{
  struct InspectionResults{
    double avg_report_time = 0.0;
    int num_reports_submitted = 0;
    int num_correct_reports = 0;
    int num_correct_report_classifications = 0;


    static bool equal(const ariac_components::InspectionResults &a, const ariac_components::InspectionResults &b){
      return (a.avg_report_time == b.avg_report_time &&
              a.num_reports_submitted == b.num_reports_submitted &&
              a.num_correct_reports == b.num_correct_reports &&
              a.num_correct_report_classifications == b.num_correct_report_classifications);
    }

    friend std::ostream& operator<<(std::ostream& os, const InspectionResults& ir){
      os << "Average report time: " << ir.avg_report_time
         << ", Number of reports submitted: " << ir.num_reports_submitted
         << ", Number of correct reports: " << ir.num_correct_reports
         << ", Number of correct report classifications: " << ir.num_correct_report_classifications;
      return os;
    }
  };
}

namespace gz::sim::components
{
  struct InspectionResultsTag;
  using InspectionResults = Component<ariac_components::InspectionResults, InspectionResultsTag>;

  GZ_SIM_REGISTER_COMPONENT("InspectionResults", InspectionResults)
}