/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/

#include "modules/map/hdmap/hdmap_common.h"

#include <algorithm>
#include <limits>

#include "cyber/common/log.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/math_utils.h"
#include "modules/map/hdmap/hdmap_impl.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace hdmap {
namespace {
using apollo::common::PointENU;
using apollo::common::math::Vec2d;

// Minimum error in lane segmentation.
// const double kSegmentationEpsilon = 0.2;

// Minimum distance to remove duplicated points.
constexpr double kDuplicatedPointsEpsilon = 1e-7;

// Margin for comparation
constexpr double kEpsilon = 0.1;

//删除重复的点
void RemoveDuplicates(std::vector<Vec2d> *points) {
  RETURN_IF_NULL(points);

  int count = 0;
  const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  for (const auto &point : *points) {
    if (count == 0 || point.DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = point;
    }
  }
  points->resize(count);
}

//从Curve中取点
void PointsFromCurve(const Curve &input_curve, std::vector<Vec2d> *points) {
  RETURN_IF_NULL(points);
  points->clear();

  for (const auto &curve : input_curve.segment()) {
    if (curve.has_line_segment()) {
      for (const auto &point : curve.line_segment().point()) {
        points->emplace_back(point.x(), point.y());
      }
    } else {
      AERROR << "Can not handle curve type.";
    }
  }
  RemoveDuplicates(points);
}

//将polygon转换为Polygon2d
apollo::common::math::Polygon2d ConvertToPolygon2d(const Polygon &polygon) {
  std::vector<Vec2d> points;
  points.reserve(polygon.point_size());
  for (const auto &point : polygon.point()) {
    points.emplace_back(point.x(), point.y());
  }
  RemoveDuplicates(&points);
  while (points.size() >= 2 && points[0].DistanceTo(points.back()) <=
                                   apollo::common::math::kMathEpsilon) {
    points.pop_back();//弹出尾部元素
  }
  return apollo::common::math::Polygon2d(points);
}

//获取curve中的segments
void SegmentsFromCurve(
    const Curve &curve,
    std::vector<apollo::common::math::LineSegment2d> *segments) {
  RETURN_IF_NULL(segments);

  std::vector<Vec2d> points;
  PointsFromCurve(curve, &points);
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    segments->emplace_back(points[i], points[i + 1]);
  }
}
//将Vec2d转换成PointENU
PointENU PointFromVec2d(const Vec2d &point) {
  PointENU pt;
  pt.set_x(point.x());
  pt.set_y(point.y());
  return pt;
}

}  // namespace
//LaneInfo的实现
LaneInfo::LaneInfo(const Lane &lane) : lane_(lane) { Init(); }

//初始化
//将lane的points取出来
//并将LaneInfo的segments_、accumulated_s_、unit_directions_、heading_赋值
//segments_：apollo::common::math::LineSegment2d，需要起始点和终止点
//accumulated_s_：对应segments_的累加s
//unit_directions_：对应segments_的方向
//headings_：unit_directions_对应的角度
//overlap_ids_：覆盖物id
//sampled_left_width_、sampled_right_width_、sampled_left_road_width_、sampled_right_road_width_:左右车道边界距离，左右道路边界距离
//lane_.type道路类型
//创建KD树
void LaneInfo::Init() {
  PointsFromCurve(lane_.central_curve(), &points_);
  CHECK_GE(points_.size(), 2U);
  segments_.clear();
  accumulated_s_.clear();
  unit_directions_.clear();
  headings_.clear();

  double s = 0;
  //
  for (size_t i = 0; i + 1 < points_.size(); ++i) {
    segments_.emplace_back(points_[i], points_[i + 1]);
    accumulated_s_.push_back(s);
    unit_directions_.push_back(segments_.back().unit_direction());
    s += segments_.back().length();
  }

  accumulated_s_.push_back(s);
  total_length_ = s;
  ACHECK(!unit_directions_.empty());
  unit_directions_.push_back(unit_directions_.back());
  for (const auto &direction : unit_directions_) {
    headings_.push_back(direction.Angle());
  }
  for (const auto &overlap_id : lane_.overlap_id()) {
    overlap_ids_.emplace_back(overlap_id.id());
  }
  ACHECK(!segments_.empty());

  sampled_left_width_.clear();
  sampled_right_width_.clear();
  for (const auto &sample : lane_.left_sample()) {
    sampled_left_width_.emplace_back(sample.s(), sample.width());
  }
  for (const auto &sample : lane_.right_sample()) {
    sampled_right_width_.emplace_back(sample.s(), sample.width());
  }

  if (lane_.has_type()) {
    if (lane_.type() == Lane::CITY_DRIVING) {
      for (const auto &p : sampled_left_width_) {
        if (p.second < FLAGS_half_vehicle_width) {
          AERROR
              << "lane[id = " << lane_.id().DebugString()
              << "]. sampled_left_width_[" << p.second
              << "] is too small. It should be larger than half vehicle width["
              << FLAGS_half_vehicle_width << "].";
        }
      }
      for (const auto &p : sampled_right_width_) {
        if (p.second < FLAGS_half_vehicle_width) {
          AERROR
              << "lane[id = " << lane_.id().DebugString()
              << "]. sampled_right_width_[" << p.second
              << "] is too small. It should be larger than half vehicle width["
              << FLAGS_half_vehicle_width << "].";
        }
      }
    } else if (lane_.type() == Lane::NONE) {
      AERROR << "lane_[id = " << lane_.id().DebugString() << "] type is NONE.";
    }
  } else {
    AERROR << "lane_[id = " << lane_.id().DebugString() << "] has NO type.";
  }

  sampled_left_road_width_.clear();
  sampled_right_road_width_.clear();
  for (const auto &sample : lane_.left_road_sample()) {
    sampled_left_road_width_.emplace_back(sample.s(), sample.width());
  }
  for (const auto &sample : lane_.right_road_sample()) {
    sampled_right_road_width_.emplace_back(sample.s(), sample.width());
  }

  CreateKDTree();
}
//通过s获取左右宽度
void LaneInfo::GetWidth(const double s, double *left_width,
                        double *right_width) const {
  if (left_width != nullptr) {
    *left_width = GetWidthFromSample(sampled_left_width_, s);
  }
  if (right_width != nullptr) {
    *right_width = GetWidthFromSample(sampled_right_width_, s);
  }
}

//获取s航向角
double LaneInfo::Heading(const double s) const {
  const double kEpsilon = 0.001;
  //s小于lane中最小的s大于最大的s则返回0
  if (s + kEpsilon < accumulated_s_.front()) {
    AERROR << "s:" << s << " should be >= " << accumulated_s_.front();
    return 0.0;
  }
  if (s - kEpsilon > accumulated_s_.back()) {
    AERROR << "s:" << s << " should be <= " << accumulated_s_.back();
    return 0.0;
  }
  //std::lower_bound() 是返回大于等于 value 值的位置，
  //而 std::upper_bound() 是返回第一个大于 value 值的位置
  auto iter = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  //返回两个迭代器之间的距离，也可以理解为计算两个元素 first 和 last 之间的元素数。
  //获取索引
  int index = static_cast<int>(std::distance(accumulated_s_.begin(), iter));
  //如果索引内容与s接近就用该值
  if (index == 0 || *iter - s <= common::math::kMathEpsilon) {
    return headings_[index];
  }
  //如果不接近就用线性插值
  return common::math::slerp(headings_[index - 1], accumulated_s_[index - 1],
                             headings_[index], accumulated_s_[index], s);
}

//通过s获得曲率
double LaneInfo::Curvature(const double s) const {
  if (points_.size() < 2U) {
    AERROR << "Not enough points to compute curvature.";
    return 0.0;
  }
  const double kEpsilon = 0.001;
  //s过大或过小，返回0
  if (s + kEpsilon < accumulated_s_.front()) {
    AERROR << "s:" << s << " should be >= " << accumulated_s_.front();
    return 0.0;
  }
  if (s > accumulated_s_.back() + kEpsilon) {
    AERROR << "s:" << s << " should be <= " << accumulated_s_.back();
    return 0.0;
  }
  //获得lane中最接近的s,accumulated_s_通过线段的长度累加而来
  auto iter = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  if (iter == accumulated_s_.end()) {
    ADEBUG << "Reach the end of lane.";
    return 0.0;
  }
  int index = static_cast<int>(std::distance(accumulated_s_.begin(), iter));
  if (index == 0) {
    ADEBUG << "Reach the beginning of lane";
    return 0.0;
  }
  //航向角变化/s变化量（近似为弧度）=曲率
  return (headings_[index] - headings_[index - 1]) /
         (accumulated_s_[index] - accumulated_s_[index - 1] + kEpsilon);
}

//通过s获取总宽度
double LaneInfo::GetWidth(const double s) const {
  double left_width = 0.0;
  double right_width = 0.0;
  GetWidth(s, &left_width, &right_width);
  return left_width + right_width;
}
//获取有效宽度，以左右宽度的左右最小值*2
double LaneInfo::GetEffectiveWidth(const double s) const {
  double left_width = 0.0;
  double right_width = 0.0;
  GetWidth(s, &left_width, &right_width);
  return 2 * std::min(left_width, right_width);
}
//获取road的左右宽度
void LaneInfo::GetRoadWidth(const double s, double *left_width,
                            double *right_width) const {
  if (left_width != nullptr) {
    *left_width = GetWidthFromSample(sampled_left_road_width_, s);
  }
  if (right_width != nullptr) {
    *right_width = GetWidthFromSample(sampled_right_road_width_, s);
  }
}
//获取road的总宽度
double LaneInfo::GetRoadWidth(const double s) const {
  double left_width = 0.0;
  double right_width = 0.0;
  GetRoadWidth(s, &left_width, &right_width);
  return left_width + right_width;
}

//从储存的samples插值求s的宽度
double LaneInfo::GetWidthFromSample(
    const std::vector<LaneInfo::SampledWidth> &samples, const double s) const {
  if (samples.empty()) {
    return 0.0;
  }
  if (s <= samples[0].first) {
    return samples[0].second;
  }
  if (s >= samples.back().first) {
    return samples.back().second;
  }
  int low = 0;
  int high = static_cast<int>(samples.size());
  //二分法查找最近的s
  while (low + 1 < high) {
    const int mid = (low + high) / 2;
    if (samples[mid].first <= s) {
      low = mid;
    } else {
      high = mid;
    }
  }
  const LaneInfo::SampledWidth &sample1 = samples[low];
  const LaneInfo::SampledWidth &sample2 = samples[high];
  //插值
  const double ratio = (sample2.first - s) / (sample2.first - sample1.first);
  return sample1.second * ratio + sample2.second * (1.0 - ratio);
}

//判断点是否在lane上
bool LaneInfo::IsOnLane(const Vec2d &point) const {
  double accumulate_s = 0.0;
  double lateral = 0.0;
  //获取点在lane上的sl坐标
  if (!GetProjection(point, &accumulate_s, &lateral)) {
    return false;
  }
  //以s轴判断，如果s过大或过小，则表示不在lane上
  if (accumulate_s > (total_length() + kEpsilon) ||
      (accumulate_s + kEpsilon) < 0.0) {
    return false;
  }
  //以l轴判断，如果在宽度之外，则在lane外
  double left_width = 0.0;
  double right_width = 0.0;
  GetWidth(accumulate_s, &left_width, &right_width);
  if (lateral < left_width && lateral > -right_width) {
    return true;
  }
  return false;
}

//判断box是否在lane上，如果有一个角点不在lane上则就判断其不在lane上
bool LaneInfo::IsOnLane(const apollo::common::math::Box2d &box) const {
  std::vector<Vec2d> corners;
  //获得角点
  box.GetAllCorners(&corners);
  for (const auto &corner : corners) {
    if (!IsOnLane(corner)) {
      return false;
    }
  }
  return true;
}

//用s获得一个平滑点
PointENU LaneInfo::GetSmoothPoint(double s) const {
  PointENU point;
  RETURN_VAL_IF(points_.size() < 2, point);
  //如果s<0则将第一个点给出
  if (s <= 0.0) {
    return PointFromVec2d(points_[0]);
  }
  //若s>total_length则将最后一个点给出
  if (s >= total_length()) {
    return PointFromVec2d(points_.back());
  }
  //查找对应的储存的离散s
  const auto low_itr =
      std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  RETURN_VAL_IF(low_itr == accumulated_s_.end(), point);
  size_t index = low_itr - accumulated_s_.begin();
  //获得差值
  double delta_s = *low_itr - s;
  if (delta_s < apollo::common::math::kMathEpsilon) {
    return PointFromVec2d(points_[index]);
  }
  //获得s的平滑点，用前一个点的单位向量来平滑
  auto smooth_point = points_[index] - unit_directions_[index - 1] * delta_s;

  return PointFromVec2d(smooth_point);
}

//获得点到lan最近的距离
double LaneInfo::DistanceTo(const Vec2d &point) const {
  //通过KD树获得最近的线段
  const auto segment_box = lane_segment_kdtree_->GetNearestObject(point);
  RETURN_VAL_IF_NULL(segment_box, 0.0);
  //得到point到线段的距离
  return segment_box->DistanceTo(point);
}

//
double LaneInfo::DistanceTo(const Vec2d &point, Vec2d *map_point,
                            double *s_offset, int *s_offset_index) const {
  RETURN_VAL_IF_NULL(map_point, 0.0);
  RETURN_VAL_IF_NULL(s_offset, 0.0);
  RETURN_VAL_IF_NULL(s_offset_index, 0.0);
  //通过point获得lane最近的线段，并获得lane上最近的点
  const auto segment_box = lane_segment_kdtree_->GetNearestObject(point);
  RETURN_VAL_IF_NULL(segment_box, 0.0);
  int index = segment_box->id();
  //获取距离和最近的点
  double distance = segments_[index].DistanceTo(point, map_point);
  //索引
  *s_offset_index = index;
  //从lane开始的s偏移距离
  *s_offset =
      accumulated_s_[index] + segments_[index].start().DistanceTo(*map_point);
  return distance;
}

//获得最近的点，和最近的距离
PointENU LaneInfo::GetNearestPoint(const Vec2d &point, double *distance) const {
  PointENU empty_point;
  RETURN_VAL_IF_NULL(distance, empty_point);

  const auto segment_box = lane_segment_kdtree_->GetNearestObject(point);
  RETURN_VAL_IF_NULL(segment_box, empty_point);
  int index = segment_box->id();
  Vec2d nearest_point;
  *distance = segments_[index].DistanceTo(point, &nearest_point);

  return PointFromVec2d(nearest_point);
}

//获得点在sl坐标系的坐标
bool LaneInfo::GetProjection(const Vec2d &point, double *accumulate_s,
                             double *lateral) const {
  RETURN_VAL_IF_NULL(accumulate_s, false);
  RETURN_VAL_IF_NULL(lateral, false);

  if (segments_.empty()) {
    return false;
  }
  double min_dist = std::numeric_limits<double>::infinity();
  int seg_num = static_cast<int>(segments_.size());
  int min_index = 0;
  //计算线段与点的最近距离的平方，获得最近的距离和索引
  for (int i = 0; i < seg_num; ++i) {
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < min_dist) {
      min_index = i;
      min_dist = distance;
    }
  }
  //开根号
  min_dist = std::sqrt(min_dist);
  const auto &nearest_seg = segments_[min_index];
  //线段向量与点和线段起始点组成的向量叉乘|a||b|sin，当a单位向量时，计算的就是b到a的垂直距离
  const auto prod = nearest_seg.ProductOntoUnit(point);
  //线段向量与点和线段起始点组成的向量点乘|a||b|cos，当a单位向量时，计算的就是b投影到a的长度
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  //如果索引为0或者索引为end,判断点的投影点是否在线段外，线段外的投影点用叉乘
  //proj<0，cos<0，说明点的投影点在线段外，需要用叉乘计算投影距离
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
  } else if (min_index == seg_num - 1) {
    //accumulated_s_是最后一段的其实s，将proj如果为正，则有可能超出endseg，所以进行使用叉乘
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * min_dist;

  }
  return true;
}

//后期处理？
void LaneInfo::PostProcess(const HDMapImpl &map_instance) {
  UpdateOverlaps(map_instance);
}
//更新覆盖物
void LaneInfo::UpdateOverlaps(const HDMapImpl &map_instance) {
  //遍历lane的覆盖物
  for (const auto &overlap_id : overlap_ids_) {
    //通过MakeMapId将string类型转换成Id,再通过hdmap_impl查找overlap_ptr
    //overlap_ptr是OverlapInfo类型
    const auto &overlap_ptr =
        map_instance.GetOverlapById(MakeMapId(overlap_id));
    if (overlap_ptr == nullptr) {
      continue;
    }
    //找到了就存入overlaps_
    overlaps_.emplace_back(overlap_ptr);
    //获取单个对象
    for (const auto &object : overlap_ptr->overlap().object()) {
      const auto &object_id = object.id().id();
      if (object_id == lane_.id().id()) {
        continue;
      }
      const auto &object_map_id = MakeMapId(object_id);
      //用所有覆盖物的接口查找该id
      if (map_instance.GetLaneById(object_map_id) != nullptr) {
        cross_lanes_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetSignalById(object_map_id) != nullptr) {
        signals_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetYieldSignById(object_map_id) != nullptr) {
        yield_signs_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetStopSignById(object_map_id) != nullptr) {
        stop_signs_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetCrosswalkById(object_map_id) != nullptr) {
        crosswalks_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetJunctionById(object_map_id) != nullptr) {
        junctions_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetClearAreaById(object_map_id) != nullptr) {
        clear_areas_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetSpeedBumpById(object_map_id) != nullptr) {
        speed_bumps_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetParkingSpaceById(object_map_id) != nullptr) {
        parking_spaces_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetPNCJunctionById(object_map_id) != nullptr) {
        pnc_junctions_.emplace_back(overlap_ptr);
      }
    }
  }
}

//创建lane的kd树
void LaneInfo::CreateKDTree() {
  //给AABoxKDTreeParams设定值
  apollo::common::math::AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 16;
  //std::vector<LaneSegmentBox> segment_box_list_
  //using LaneSegmentBox =ObjectWithAABox<LaneInfo, apollo::common::math::LineSegment2d>;
  //std::vector<apollo::common::math::LineSegment2d> segments_;两个点的向量
  segment_box_list_.clear();
  for (size_t id = 0; id < segments_.size(); ++id) {
    const auto &segment = segments_[id];
    //AABox2b通过两点计算2d box
    segment_box_list_.emplace_back(
        apollo::common::math::AABox2d(segment.start(), segment.end()), this,
        &segment, id);//将box和laneInfo和segment和ID组成ObjectWithAABox
  }
  //using LaneSegmentKDTree = apollo::common::math::AABoxKDTree2d<LaneSegmentBox>;
  //开始递归建立KD树
  //用当前lane中线段组成的box来组建的
  lane_segment_kdtree_.reset(new LaneSegmentKDTree(segment_box_list_, params));
}

//用Junction初始化
JunctionInfo::JunctionInfo(const Junction &junction) : junction_(junction) {
  Init();
}
//初始化
void JunctionInfo::Init() {
  //将proto的polygon转化成Polygon2d
  polygon_ = ConvertToPolygon2d(junction_.polygon());
  CHECK_GT(polygon_.num_points(), 2);
  //将所有的junction覆盖物id储存起来
  for (const auto &overlap_id : junction_.overlap_id()) {
    overlap_ids_.emplace_back(overlap_id);
  }
}

void JunctionInfo::PostProcess(const HDMapImpl &map_instance) {
  UpdateOverlaps(map_instance);
}
//更新覆盖物
void JunctionInfo::UpdateOverlaps(const HDMapImpl &map_instance) {
  for (const auto &overlap_id : overlap_ids_) {
    const auto &overlap_ptr = map_instance.GetOverlapById(overlap_id);
    if (overlap_ptr == nullptr) {
      continue;
    }

    for (const auto &object : overlap_ptr->overlap().object()) {
      const auto &object_id = object.id().id();
      if (object_id == id().id()) {
        continue;
      }
      //路口只添加停止信号的id
      if (object.has_stop_sign_overlap_info()) {
        overlap_stop_sign_ids_.push_back(object.id());
      }
    }
  }
}
//信号info
SignalInfo::SignalInfo(const Signal &signal) : signal_(signal) { Init(); }
//初始化停止线
void SignalInfo::Init() {
  //将curve转换为segments
  for (const auto &stop_line : signal_.stop_line()) {
    SegmentsFromCurve(stop_line, &segments_);
  }
  ACHECK(!segments_.empty());
  std::vector<Vec2d> points;
  //将segment转换为Vec2d，没有存下来使用？？
  for (const auto &segment : segments_) {
    points.emplace_back(segment.start());
    points.emplace_back(segment.end());
  }
  CHECK_GT(points.size(), 0U);
}
//人行横道
CrosswalkInfo::CrosswalkInfo(const Crosswalk &crosswalk)
    : crosswalk_(crosswalk) {
  Init();
}
//初始化，将proto polygon转换成Polygon2d
void CrosswalkInfo::Init() {
  polygon_ = ConvertToPolygon2d(crosswalk_.polygon());
  CHECK_GT(polygon_.num_points(), 2);
}
//停止信号
StopSignInfo::StopSignInfo(const StopSign &stop_sign) : stop_sign_(stop_sign) {
  init();
}
//停止信号初始化,得到segments_和overlap_id
void StopSignInfo::init() {
  for (const auto &stop_line : stop_sign_.stop_line()) {
    SegmentsFromCurve(stop_line, &segments_);
  }
  ACHECK(!segments_.empty());

  for (const auto &overlap_id : stop_sign_.overlap_id()) {
    overlap_ids_.emplace_back(overlap_id);
  }
}

void StopSignInfo::PostProcess(const HDMapImpl &map_instance) {
  UpdateOverlaps(map_instance);
}
//更新覆盖物
void StopSignInfo::UpdateOverlaps(const HDMapImpl &map_instance) {
  for (const auto &overlap_id : overlap_ids_) {
    const auto &overlap_ptr = map_instance.GetOverlapById(overlap_id);
    if (overlap_ptr == nullptr) {
      continue;
    }

    for (const auto &object : overlap_ptr->overlap().object()) {
      const auto &object_id = object.id().id();
      if (object_id == id().id()) {
        continue;
      }
      //添加junction和lane
      if (object.has_junction_overlap_info()) {
        overlap_junction_ids_.push_back(object.id());
      } else if (object.has_lane_overlap_info()) {
        overlap_lane_ids_.push_back(object.id());
      }
    }
  }
  if (overlap_junction_ids_.empty()) {
    AWARN << "stop sign " << id().id() << "has no overlap with any junction.";
  }
}
//让行线，初始化
YieldSignInfo::YieldSignInfo(const YieldSign &yield_sign)
    : yield_sign_(yield_sign) {
  Init();
}

//初始化让行线，获得segments_
void YieldSignInfo::Init() {
  for (const auto &stop_line : yield_sign_.stop_line()) {
    SegmentsFromCurve(stop_line, &segments_);
  }
  // segments_from_curve(yield_sign_.stop_line(), &segments_);
  ACHECK(!segments_.empty());
}
//禁停区
ClearAreaInfo::ClearAreaInfo(const ClearArea &clear_area)
    : clear_area_(clear_area) {
  Init();
}

void ClearAreaInfo::Init() {
  polygon_ = ConvertToPolygon2d(clear_area_.polygon());
  CHECK_GT(polygon_.num_points(), 2);
}
//减速带
SpeedBumpInfo::SpeedBumpInfo(const SpeedBump &speed_bump)
    : speed_bump_(speed_bump) {
  Init();
}

void SpeedBumpInfo::Init() {
  for (const auto &stop_line : speed_bump_.position()) {
    SegmentsFromCurve(stop_line, &segments_);
  }
  ACHECK(!segments_.empty());
}
//覆盖物
OverlapInfo::OverlapInfo(const Overlap &overlap) : overlap_(overlap) {}
//通过id获取覆盖物
const ObjectOverlapInfo *OverlapInfo::GetObjectOverlapInfo(const Id &id) const {
  for (const auto &object : overlap_.object()) {
    if (object.id().id() == id.id()) {
      return &object;
    }
  }
  return nullptr;
}
//道路信息，将section和road_boundaries储存起来
RoadInfo::RoadInfo(const Road &road) : road_(road) {
  for (const auto &section : road_.section()) {
    sections_.push_back(section);
    road_boundaries_.push_back(section.boundary());
  }
}

//获得边界线
const std::vector<RoadBoundary> &RoadInfo::GetBoundaries() const {
  return road_boundaries_;
}
//构造停车位
ParkingSpaceInfo::ParkingSpaceInfo(const ParkingSpace &parking_space)
    : parking_space_(parking_space) {
  Init();
}
//将proto polygon转换为Polygon2d
void ParkingSpaceInfo::Init() {
  polygon_ = ConvertToPolygon2d(parking_space_.polygon());
  CHECK_GT(polygon_.num_points(), 2);
}
//构造PNC路口
PNCJunctionInfo::PNCJunctionInfo(const PNCJunction &pnc_junction)
    : junction_(pnc_junction) {
  Init();
}
//初始化PNC路口
void PNCJunctionInfo::Init() {
  polygon_ = ConvertToPolygon2d(junction_.polygon());
  CHECK_GT(polygon_.num_points(), 2);

  for (const auto &overlap_id : junction_.overlap_id()) {
    overlap_ids_.emplace_back(overlap_id);
  }
}
//路侧单元信息
RSUInfo::RSUInfo(const RSU &rsu) : _rsu(rsu) {}

}  // namespace hdmap
}  // namespace apollo
