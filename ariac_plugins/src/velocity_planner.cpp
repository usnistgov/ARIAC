#include "path_velocity_planner/velocity_planner.hpp"

namespace path_velocity_planner {

VelocityPlanner::VelocityPlanner(double max_velocity, double max_acceleration)
  : v_max_(max_velocity), acc_(max_acceleration) {}

void VelocityPlanner::set_waypoints(const std::vector<Point>& waypoints, double heading, Direction dir) {
  waypoints_ = waypoints;
  direction_ = dir;

  if (direction_ == Direction::BACKWARD) {
    heading += M_PI; 
    if (heading > 2 * M_PI) {
      heading -= 2 * M_PI;
    }
  }
  
  initial_heading_ = heading;
  
  compute_profile();
}

PathVelocity VelocityPlanner::get_velocity_at_time(double time) {
  for (const auto& segment : segments_) {
    if (time >= segment.start_time && time <= segment.end_time) {
      double t = time - segment.start_time;
      double v = 0.0;
      if (t < segment.accel_duration) {
        v = acc_ * t;
      } else if (t < segment.accel_duration + segment.cruise_duration) {
        v = segment.velocity;
      } else {
        double t_decel = t - segment.accel_duration - segment.cruise_duration;
        v = segment.velocity - acc_ * t_decel;
      }

      if (direction_ == Direction::BACKWARD) {
        v *= -1;
      }

      double angular = (segment.rotation / (segment.end_time - segment.start_time));
      return {v, angular};
    }
  }
  return {0.0, 0.0};
}

bool VelocityPlanner::is_finished(double time) {
  return !segments_.empty() && time > segments_.back().end_time;
}

void VelocityPlanner::compute_profile() {
  segments_.clear();
  double current_time = 0.0;
  final_heading_ = initial_heading_;

  for (size_t i = 0; i < waypoints_.size() - 1; ++i) {
    const auto& p1 = waypoints_[i];
    const auto& p2 = waypoints_[i + 1];
    
    double distance = 0;
    double rotation = 0;

    if (is_linear(p1, p2, final_heading_)) {
      distance = std::hypot(p2.x - p1.x, p2.y - p1.y);
    } else {
      ArcSegment arc = compute_arc_between_points(p1, p2, final_heading_).value();
      distance = arc.arc_length;
      rotation = arc.angle;
    }
    
    double accel_time, cruise_time, decel_time, vel;

    if (i == 0){  // acceleration
      // check if trapazoidal profile does not hold
      if (v_max_*v_max_ / (2 * acc_) >= distance) {
        vel = sqrt(2 * distance * acc_);
        cruise_time = 0.0;
      } else {
        vel = v_max_;
        cruise_time = distance / vel - 0.5 * vel / acc_;
      }

      accel_time = vel / acc_;
      decel_time = 0.0;

    } else if (i == waypoints_.size() - 2) { // deceleration
      // check if trapazoidal profile does not hold
      if (v_max_*v_max_ / (2 * acc_) >= distance) {
        vel = sqrt(2 * distance * acc_);
        cruise_time = 0.0;
      } else {
        vel = v_max_;
        cruise_time = distance / vel - 0.5 * vel / acc_;
      }

      decel_time = vel / acc_;
      accel_time = 0.0;
      
    } else { // only cruise
      accel_time = 0.0;
      cruise_time = distance / vel;
      decel_time = 0.0;
    }
  
    double segment_duration = accel_time + cruise_time + decel_time;

    PathSegment segment;
    segment.start_time = current_time;
    segment.end_time = current_time + segment_duration;
    segment.distance = distance;
    segment.accel_duration = accel_time;
    segment.cruise_duration = cruise_time;
    segment.decel_duration = decel_time;
    segment.rotation = rotation;
    segment.velocity = vel;

    segments_.push_back(segment);
    current_time += segment_duration;
    final_heading_ += rotation;
  }
}

bool VelocityPlanner::is_linear(const Point& a, const Point& b, double heading) const {
  Point dir = {std::cos(heading), std::sin(heading)};
  Point delta = {b.x - a.x, b.y - a.y};
  double cross_product = cross(dir, delta);
  return std::abs(cross_product) < 0.05;
}

std::optional<ArcSegment> VelocityPlanner::compute_arc_between_points(const Point& p1, const Point& p2, double heading) const {
  Point dir = {std::cos(heading), std::sin(heading)};
  double side = cross_2d(p1, {p1.x + dir.x, p1.y + dir.y}, p2);
  bool is_left = side > 0;

  double normal_angle = heading + (is_left ? M_PI_2 : -M_PI_2);

  Point normal = {std::cos(normal_angle), std::sin(normal_angle)};

  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  double proj = dx * normal.x + dy * normal.y;

  if (std::abs(proj) < 1e-6) return std::nullopt;

  double d2 = dx * dx + dy * dy;
  double radius = d2 / (2 * proj);

  Point center = {p1.x + radius * normal.x, p1.y + radius * normal.y};

  return compute_arc(center, p1, p2, heading);
}

ArcSegment VelocityPlanner::compute_arc(const Point& center, const Point& p1, const Point& p2, double tangent_angle) const {
  Point v1 = {p1.x - center.x, p1.y - center.y};
  Point v2 = {p2.x - center.x, p2.y - center.y};
  double angle = signed_angle(v1, v2);

  Point heading = {std::cos(tangent_angle), std::sin(tangent_angle)};
  double entry_angle = signed_angle(v1, heading);
  if (angle * entry_angle < 0) {
    angle += (angle > 0) ? -2 * M_PI : 2 * M_PI;
  }

  double arc_length = std::abs(std::hypot(v1.x, v1.y) * angle);
  return {center, std::hypot(v1.x, v1.y), arc_length, angle};
}

double VelocityPlanner::cross_2d(const Point& A, const Point& B, const Point& C) const {
  return (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
}

double VelocityPlanner::dot(const Point& a, const Point& b) const {
  return a.x * b.x + a.y * b.y;
}

double VelocityPlanner::cross(const Point& a, const Point& b) const {
  return a.x * b.y - a.y * b.x;
}

double VelocityPlanner::signed_angle(const Point& a, const Point& b) const {
  return std::atan2(cross(a, b), dot(a, b));
}

} // namespace path_velocity_planner