// Copyright (c) 2024
// Licensed under the Apache License, Version 2.0

#ifndef FIELD_NAV__FIELD_LINE_HPP_
#define FIELD_NAV__FIELD_LINE_HPP_

#include <string>
#include "geographic_msgs/msg/geo_point.hpp"

namespace field_nav
{

struct FieldLine {
  geographic_msgs::msg::GeoPoint start;
  geographic_msgs::msg::GeoPoint end;
  std::string turn;           // "left", "right", or "end"
  double lateral_offset_m{0.0};  // per-swath lateral offset (m, positive=left of heading)
};

struct HeadlandBoundaries {
  geographic_msgs::msg::GeoPoint sw;  // SW end headland boundary point
  geographic_msgs::msg::GeoPoint ne;  // NE end headland boundary point
  bool valid{false};
};

}  // namespace field_nav

#endif  // FIELD_NAV__FIELD_LINE_HPP_
