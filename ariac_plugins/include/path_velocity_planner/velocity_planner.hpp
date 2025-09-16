#ifndef PATH_VELOCITY_PLANNER__VELOCITY_PLANNER_HPP_
#define PATH_VELOCITY_PLANNER__VELOCITY_PLANNER_HPP_

#include <vector>
#include <optional>
#include <cmath>

namespace path_velocity_planner {
  enum Direction {
    FORWARD,
    BACKWARD
  };

  struct Point {
    double x, y;
  };
  
  struct ArcSegment {
    Point center;
    double radius;
    double arc_length;
    double angle;
  };
  
  struct PathVelocity {
    double linear;
    double angular;
  };
  
  struct PathSegment {
    double start_time;
    double end_time;
    double distance;
    double accel_duration;
    double cruise_duration;
    double decel_duration;
    double rotation;
    double velocity;
  };
  
  class VelocityPlanner {
  public:
    VelocityPlanner(double max_velocity, double max_acceleration);
  
    void set_waypoints(const std::vector<Point>& waypoints, double heading, Direction dir);
    PathVelocity get_velocity_at_time(double time);
    bool is_finished(double time);
  
  private:
    void compute_profile();
    bool is_linear(const Point& a, const Point& b, double heading) const;
    std::optional<ArcSegment> compute_arc_between_points(const Point& p1, const Point& p2, double heading) const;
    ArcSegment compute_arc(const Point& center, const Point& P1, const Point& P2, double tangent_angle) const;
    double cross_2d(const Point& A, const Point& B, const Point& C) const;
    double dot(const Point& a, const Point& b) const;
    double cross(const Point& a, const Point& b) const;
    double signed_angle(const Point& a, const Point& b) const;
  
    std::vector<Point> waypoints_;
    std::vector<PathSegment> segments_;
    Direction direction_;
    double v_max_;
    double acc_;
    double initial_heading_;
    double final_heading_;
  };
  
} // namespace path_velocity_planner
  
#endif // PATH_VELOCITY_PLANNER__VELOCITY_PLANNER_HPP_