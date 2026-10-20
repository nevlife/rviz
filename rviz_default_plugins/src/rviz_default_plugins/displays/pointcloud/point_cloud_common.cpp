/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"

#include <algorithm>
#include <cstring>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <utility>

#include <OgreSceneNode.h>
#include <OgreWireBoundingBox.h>

#include "rclcpp/clock.hpp"

#include "rviz_default_plugins/displays/pointcloud/point_cloud_to_point_cloud2.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp"
#include "rviz_default_plugins/displays/pointcloud/transformers/xyz_pc_transformer.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/editable_enum_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/uniform_string_stream.hpp"
#include "rviz_common/validate_floats.hpp"

namespace rviz_default_plugins
{

CloudInfo::CloudInfo()
: manager_(nullptr),
  scene_node_(nullptr),
  position_(Ogre::Vector3::ZERO)
{}

CloudInfo::~CloudInfo()
{
  clear();
}

void CloudInfo::clear()
{
  if (scene_node_) {
    manager_->destroySceneNode(scene_node_);
    scene_node_ = nullptr;
  }
}

void CloudInfo::setSelectable(
  bool selectable, float selection_box_size, rviz_common::DisplayContext * context)
{
  if (selectable) {
    selection_handler_ = rviz_common::interaction::createSelectionHandler
      <PointCloudSelectionHandler>(selection_box_size, this, context);
    cloud_->setPickColor(
      rviz_common::interaction::SelectionManager::handleToColor(
        selection_handler_->getHandle()));
  } else {
    selection_handler_.reset();
    cloud_->setPickColor(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 0.0f));
  }
}

Ogre::Vector3 CloudInfo::pointPosition(uint64_t index) const
{
  Ogre::Vector3 position;
  const uint8_t * point = message_->data.data() + index * message_->point_step;
  std::memcpy(
    &position.x, point + message_->fields[findChannelIndex(message_, "x")].offset, sizeof(float));
  std::memcpy(
    &position.y, point + message_->fields[findChannelIndex(message_, "y")].offset, sizeof(float));
  std::memcpy(
    &position.z, point + message_->fields[findChannelIndex(message_, "z")].offset, sizeof(float));
  return position;
}

const std::string PointCloudCommon::message_status_name_ = "Message";  // NOLINT allow std::string

PointCloudCommon::PointCloudCommon(rviz_common::Display * display)
: auto_size_(false),
  new_xyz_transformer_(false),
  new_color_transformer_(false),
  needs_retransform_(false),
  transformer_factory_(std::make_unique<PointCloudTransformerFactory>()),
  display_(display)
{
  selectable_property_ = new rviz_common::properties::BoolProperty(
    "Selectable", true,
    "Whether or not the points in this point cloud are selectable.",
    display_, SLOT(updateSelectable()), this);

  style_property_ = new rviz_common::properties::EnumProperty(
    "Style", "Flat Squares",
    "Rendering mode to use, in order of computational complexity.",
    display_, SLOT(updateStyle()), this);
  style_property_->addOption("Points", rviz_rendering::PointCloud::RM_POINTS);
  style_property_->addOption("Squares", rviz_rendering::PointCloud::RM_SQUARES);
  style_property_->addOption("Flat Squares", rviz_rendering::PointCloud::RM_FLAT_SQUARES);
  style_property_->addOption("Spheres", rviz_rendering::PointCloud::RM_SPHERES);
  style_property_->addOption("Boxes", rviz_rendering::PointCloud::RM_BOXES);
  style_property_->addOption("Tiles", rviz_rendering::PointCloud::RM_TILES);
  style_property_->addOption(
    "Depth Fade Points", rviz_rendering::PointCloud::RM_DEPTH_FADE_POINTS);

  point_world_size_property_ = new rviz_common::properties::FloatProperty(
    "Size (m)", 0.01f,
    "Point size in meters.",
    display_, SLOT(updateBillboardSize()), this);
  point_world_size_property_->setMin(0.0001f);

  point_pixel_size_property_ = new rviz_common::properties::FloatProperty(
    "Size (Pixels)", 3,
    "Point size in pixels.",
    display_, SLOT(updateBillboardSize()), this);
  point_pixel_size_property_->setMin(1);

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0f,
    "Amount of transparency to apply to the points.  Note that this is experimental "
    "and does not always look correct.",
    display_, SLOT(updateAlpha()), this);
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  decay_time_property_ = new rviz_common::properties::FloatProperty(
    "Decay Time", 0,
    "Duration, in seconds, to keep the incoming points.  0 means only show the latest points.",
    display_, SLOT(queueRender()));
  decay_time_property_->setMin(0);

  size_field_property_ = new rviz_common::properties::EditableEnumProperty(
    "Size Field", "",
    "Field that scales the size of each point; empty gives all points the same size. "
    "Applies when the cloud is drawn on the GPU (Points, Squares, Flat Squares, Spheres, "
    "Depth Fade Points).",
    display_, SLOT(updateSizeField()), this);
  size_min_scale_property_ = new rviz_common::properties::FloatProperty(
    "Min Scale", 0.1f, "Size factor of the points at Min Value.",
    size_field_property_, SLOT(causeRetransform()), this);
  size_min_scale_property_->setMin(0);
  size_max_scale_property_ = new rviz_common::properties::FloatProperty(
    "Max Scale", 1.0f, "Size factor of the points at Max Value.",
    size_field_property_, SLOT(causeRetransform()), this);
  size_max_scale_property_->setMin(0);
  size_auto_bounds_property_ = new rviz_common::properties::BoolProperty(
    "Autocompute Value Bounds", true,
    "Whether Min Value and Max Value follow the field range of each cloud.",
    size_field_property_, SLOT(updateSizeField()), this);
  size_min_value_property_ = new rviz_common::properties::FloatProperty(
    "Min Value", 0.0f, "Field value drawn at Min Scale.",
    size_field_property_, SLOT(causeRetransform()), this);
  size_max_value_property_ = new rviz_common::properties::FloatProperty(
    "Max Value", 1.0f, "Field value drawn at Max Scale.",
    size_field_property_, SLOT(causeRetransform()), this);
  updateSizeField();

  xyz_transformer_property_ = new rviz_common::properties::EnumProperty(
    "Position Transformer", "",
    "Set the transformer to use to set the position of the points.",
    display_, SLOT(updateXyzTransformer()), this);
  connect(
    xyz_transformer_property_, SIGNAL(
      requestOptions(
        rviz_common::properties::EnumProperty*)),
    this, SLOT(setXyzTransformerOptions(rviz_common::properties::EnumProperty*)));

  color_transformer_property_ = new rviz_common::properties::EnumProperty(
    "Color Transformer", "",
    "Set the transformer to use to set the color of the points.",
    display_, SLOT(updateColorTransformer()), this);
  connect(
    color_transformer_property_,
    SIGNAL(requestOptions(rviz_common::properties::EnumProperty*)),
    this, SLOT(setColorTransformerOptions(rviz_common::properties::EnumProperty*)));
}

void PointCloudCommon::initialize(
  rviz_common::DisplayContext * context,
  Ogre::SceneNode * scene_node)
{
  loadTransformers();

  context_ = context;
  scene_node_ = scene_node;
  clock_ = context->getClock();

  updateStyle();
  updateBillboardSize();
  updateAlpha();
  updateSelectable();
}

void PointCloudCommon::loadTransformers()
{
  auto plugins = transformer_factory_->getDeclaredPlugins();
  for (auto const & plugin : plugins) {
    auto plugin_name_std = plugin.name.toStdString();
    if (transformers_.count(plugin_name_std) > 0) {
      RVIZ_COMMON_LOG_ERROR_STREAM("Transformer type " << plugin_name_std << " is already loaded.");
      continue;
    }

    PointCloudTransformerPtr trans(transformer_factory_->make(plugin.id));
    loadTransformer(trans, plugin_name_std, plugin.id.toStdString());
  }
}

void PointCloudCommon::loadTransformer(
  PointCloudTransformerPtr trans,
  std::string name,
  const std::string & lookup_name)
{
  trans->init();
  connect(trans.get(), SIGNAL(needRetransform()), this, SLOT(causeRetransform()));

  TransformerInfo info;
  info.transformer = trans;
  info.readable_name = name;
  info.lookup_name = lookup_name;

  info.transformer->createProperties(
    display_, PointCloudTransformer::Support_XYZ, info.xyz_props);
  setPropertiesHidden(info.xyz_props, true);

  info.transformer->createProperties(
    display_, PointCloudTransformer::Support_Color, info.color_props);
  setPropertiesHidden(info.color_props, true);

  transformers_[name] = info;
}

void PointCloudCommon::setAutoSize(bool auto_size)
{
  auto_size_ = auto_size;
  for (auto const & cloud_info : cloud_infos_) {
    cloud_info->cloud_->setAutoSize(auto_size);
  }
}

void PointCloudCommon::updateAlpha()
{
  for (auto const & cloud_info : cloud_infos_) {
    bool per_point_alpha = findChannelIndex(cloud_info->message_, "rgba") != -1;
    cloud_info->cloud_->setAlpha(alpha_property_->getFloat(), per_point_alpha);
  }
}

void PointCloudCommon::updateSizeField()
{
  const bool enabled = !size_field_property_->getStdString().empty();
  const bool manual = enabled && !size_auto_bounds_property_->getBool();
  size_min_scale_property_->setHidden(!enabled);
  size_max_scale_property_->setHidden(!enabled);
  size_auto_bounds_property_->setHidden(!enabled);
  size_min_value_property_->setHidden(!manual);
  size_max_value_property_->setHidden(!manual);
  causeRetransform();
}

void PointCloudCommon::updateSelectable()
{
  bool selectable = selectable_property_->getBool();

  for (auto const & cloud_info : cloud_infos_) {
    cloud_info->setSelectable(selectable, getSelectionBoxSize(), context_);
  }
}

void PointCloudCommon::updateStyle()
{
  auto mode = static_cast<rviz_rendering::PointCloud::RenderMode>(style_property_->getOptionInt());
  if (mode == rviz_rendering::PointCloud::RM_POINTS) {
    point_world_size_property_->hide();
    point_pixel_size_property_->show();
  } else {
    point_world_size_property_->show();
    point_pixel_size_property_->hide();
  }
  const bool gpu = rviz_rendering::PointCloud::supportsGpuPoints(mode);
  for (auto const & cloud_info : cloud_infos_) {
    cloud_info->cloud_->setRenderMode(mode);
    if (cloud_info->gpu_ != gpu) {
      causeRetransform();
    }
  }
  updateBillboardSize();
}

void PointCloudCommon::updateBillboardSize()
{
  auto mode = static_cast<rviz_rendering::PointCloud::RenderMode>(style_property_->getOptionInt());
  float size;
  if (mode == rviz_rendering::PointCloud::RM_POINTS) {
    size = point_pixel_size_property_->getFloat();
  } else {
    size = point_world_size_property_->getFloat();
  }
  for (auto & cloud_info : cloud_infos_) {
    cloud_info->cloud_->setDimensions(size, size, size);
    if (cloud_info->selection_handler_) {
      cloud_info->selection_handler_->setBoxSize(getSelectionBoxSize());
    }
  }
  context_->queueRender();
}

void PointCloudCommon::reset()
{
  std::unique_lock<std::mutex> lock(new_clouds_mutex_);
  cloud_infos_.clear();
  new_cloud_infos_.clear();
  spare_cloud_.reset();
}

void PointCloudCommon::causeRetransform()
{
  needs_retransform_ = true;
}

void PointCloudCommon::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;

  float point_decay_time = decay_time_property_->getFloat();
  rclcpp::Time now = clock_->now();

  if (needs_retransform_) {
    retransform();
    needs_retransform_ = false;
  }

  collectObsoleteCloudInfos(point_decay_time, now);
  removeObsoleteCloudInfos();

  insertNewClouds(point_decay_time, now);

  updateTransformerProperties();
  updateStatus();
}

void PointCloudCommon::insertNewClouds(float point_decay_time, const rclcpp::Time & now)
{
  auto mode = static_cast<rviz_rendering::PointCloud::RenderMode>(style_property_->getOptionInt());

  std::unique_lock<std::mutex> lock(new_clouds_mutex_);
  if (!new_cloud_infos_.empty()) {
    float size = getSizeForRenderMode(mode);

    auto it = new_cloud_infos_.begin();
    auto end = new_cloud_infos_.end();
    for (; it != end; ++it) {
      CloudInfoPtr cloud_info = *it;

      // ignore point clouds that are too old, but keep at least one
      auto next = it; next++;
      if (next != end && cloudInfoIsDecayed(cloud_info, point_decay_time, now)) {
        continue;
      }

      bool per_point_alpha = findChannelIndex(cloud_info->message_, "rgba") != -1;

      if (spare_cloud_) {
        cloud_info->cloud_ = std::move(spare_cloud_);
        cloud_info->cloud_->clearAndRemoveAllPoints();
      } else {
        cloud_info->cloud_.reset(new rviz_rendering::PointCloud());
      }
      cloud_info->cloud_->setRenderMode(mode);
      drawCloud(cloud_info);
      cloud_info->cloud_->setAlpha(alpha_property_->getFloat(), per_point_alpha);
      cloud_info->cloud_->setDimensions(size, size, size);
      cloud_info->cloud_->setAutoSize(auto_size_);

      cloud_info->manager_ = context_->getSceneManager();

      cloud_info->scene_node_ = scene_node_->createChildSceneNode(
        cloud_info->position_,
        cloud_info->orientation_);

      cloud_info->scene_node_->attachObject(cloud_info->cloud_.get());

      cloud_info->setSelectable(selectable_property_->getBool(), getSelectionBoxSize(), context_);

      cloud_infos_.push_back(*it);
    }

    new_cloud_infos_.clear();
  }
}

float PointCloudCommon::getSizeForRenderMode(const rviz_rendering::PointCloud::RenderMode & mode)
{
  float size;
  if (mode == rviz_rendering::PointCloud::RM_POINTS) {
    size = point_pixel_size_property_->getFloat();
  } else {
    size = point_world_size_property_->getFloat();
  }
  return size;
}

void PointCloudCommon::updateTransformerProperties()
{
  std::unique_lock<std::mutex> lock(new_clouds_mutex_);

  if (new_xyz_transformer_ || new_color_transformer_) {
    for (auto transformer : transformers_) {
      const std::string & name = transformer.first;
      TransformerInfo & info = transformer.second;

      setPropertiesHidden(info.xyz_props, name != xyz_transformer_property_->getStdString());
      setPropertiesHidden(
        info.color_props, name != color_transformer_property_->getStdString());

      if (name == xyz_transformer_property_->getStdString() ||
        name == color_transformer_property_->getStdString())
      {
        info.transformer->hideUnusedProperties();
      }
    }
  }

  new_xyz_transformer_ = false;
  new_color_transformer_ = false;
}

void PointCloudCommon::collectObsoleteCloudInfos(float point_decay_time, const rclcpp::Time & now)
{
  std::unique_lock<std::mutex> lock(new_clouds_mutex_);

  if (point_decay_time > 0.0 || !new_cloud_infos_.empty()) {
    while (!cloud_infos_.empty() &&
      cloudInfoIsDecayed(cloud_infos_.front(), point_decay_time, now))
    {
      cloud_infos_.front()->clear();
      obsolete_cloud_infos_.push_back(cloud_infos_.front());
      cloud_infos_.pop_front();
      context_->queueRender();
    }
  }
}

void PointCloudCommon::removeObsoleteCloudInfos()
{
  auto it = obsolete_cloud_infos_.begin();
  auto end = obsolete_cloud_infos_.end();
  while (it != end) {
    if (!(*it)->selection_handler_.get() || !(*it)->selection_handler_->hasSelections()) {
      if (!spare_cloud_ && it->use_count() == 1 && (*it)->cloud_.use_count() == 1) {
        spare_cloud_ = std::move((*it)->cloud_);
      }
      it = obsolete_cloud_infos_.erase(it);
    }
    if (it != end) {
      ++it;
    }
  }
}

bool PointCloudCommon::cloudInfoIsDecayed(
  CloudInfoPtr cloud_info, float point_decay_time, const rclcpp::Time & now)
{
  return (now.nanoseconds() - cloud_info->receive_time_.nanoseconds()) / 1E9 >=
         point_decay_time;
}

void PointCloudCommon::setPropertiesHidden(
  const QList<rviz_common::properties::Property *> & props,
  bool hide)
{
  for (auto prop : props) {
    prop->setHidden(hide);
  }
}

void PointCloudCommon::updateTransformers(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  std::string xyz_name = xyz_transformer_property_->getStdString();
  std::string color_name = color_transformer_property_->getStdString();

  xyz_transformer_property_->clearOptions();
  color_transformer_property_->clearOptions();

  // Get the channels that we could potentially render
  typedef std::set<std::pair<uint8_t, std::string>> S_string;
  S_string valid_xyz, valid_color;
  bool cur_xyz_valid = false;
  bool cur_color_valid = false;
  bool has_rgb_transformer = false;
  for (auto transformer : transformers_) {
    const std::string & name = transformer.first;
    const PointCloudTransformerPtr & trans = transformer.second.transformer;
    uint32_t mask = trans->supports(cloud);
    if (mask & PointCloudTransformer::Support_XYZ) {
      valid_xyz.insert(std::make_pair(trans->score(cloud), name));
      if (name == xyz_name) {
        cur_xyz_valid = true;
      }
      xyz_transformer_property_->addOptionStd(name);
    }

    if (mask & PointCloudTransformer::Support_Color) {
      valid_color.insert(std::make_pair(trans->score(cloud), name));
      if (name == color_name) {
        cur_color_valid = true;
      }
      if (name == "RGB8") {
        has_rgb_transformer = true;
      }
      color_transformer_property_->addOptionStd(name);
    }
  }

  if (!cur_xyz_valid) {
    if (!valid_xyz.empty()) {
      xyz_transformer_property_->setStringStd(valid_xyz.rbegin()->second);
    }
  }

  if (!cur_color_valid) {
    if (!valid_color.empty()) {
      if (has_rgb_transformer) {
        color_transformer_property_->setStringStd("RGB8");
      } else {
        color_transformer_property_->setStringStd(valid_color.rbegin()->second);
      }
    }
  }
}

void PointCloudCommon::updateStatus()
{
  std::stringstream ss;
  uint64_t total_point_count = 0;
  for (const auto & cloud_info : cloud_infos_) {
    total_point_count +=
      static_cast<uint64_t>(cloud_info->message_->width) * cloud_info->message_->height;
  }
  ss << "Showing [" << total_point_count << "] points from [" << cloud_infos_.size() <<
    "] messages";
  display_->setStatusStd(rviz_common::properties::StatusProperty::Ok, "Points", ss.str());
}

void PointCloudCommon::processMessage(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  CloudInfoPtr info(new CloudInfo);
  info->message_ = cloud;
  info->receive_time_ = clock_->now();

  if (transformCloud(info, true)) {
    rclcpp::Time time_stamp(cloud->header.stamp, RCL_ROS_TIME);

    std::unique_lock<std::mutex> lock(new_clouds_mutex_);
    new_cloud_infos_.push_back(info);
    display_->emitTimeSignal(time_stamp);
  }
}

void PointCloudCommon::updateXyzTransformer()
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  if (transformers_.count(xyz_transformer_property_->getStdString()) == 0) {
    return;
  }
  new_xyz_transformer_ = true;
  causeRetransform();
}

void PointCloudCommon::updateColorTransformer()
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  if (transformers_.count(color_transformer_property_->getStdString()) == 0) {
    return;
  }
  new_color_transformer_ = true;
  causeRetransform();
}

PointCloudTransformerPtr PointCloudCommon::getXYZTransformer(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  auto it = transformers_.find(xyz_transformer_property_->getStdString());
  if (it != transformers_.end()) {
    const PointCloudTransformerPtr & trans = it->second.transformer;
    if (trans->supports(cloud) & PointCloudTransformer::Support_XYZ) {
      return trans;
    }
  }

  return PointCloudTransformerPtr();
}

PointCloudTransformerPtr PointCloudCommon::getColorTransformer(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  auto it = transformers_.find(color_transformer_property_->getStdString());
  if (it != transformers_.end()) {
    const PointCloudTransformerPtr & trans = it->second.transformer;
    if (trans->supports(cloud) & PointCloudTransformer::Support_Color) {
      return trans;
    }
  }

  return PointCloudTransformerPtr();
}


void PointCloudCommon::retransform()
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);

  for (auto const & cloud_info : cloud_infos_) {
    transformCloud(cloud_info, false);
    drawCloud(cloud_info);
  }
}

void PointCloudCommon::drawCloud(const CloudInfoPtr & cloud_info)
{
  if (!cloud_info->gpu_) {
    cloud_info->cloud_->clear();
    cloud_info->cloud_->addPoints(std::move(cloud_info->transformed_points_));
    return;
  }
  if (cloud_info->cloud_->usesGpuPoints()) {
    cloud_info->cloud_->setGpuColor(cloud_info->gpu_color_);
    cloud_info->cloud_->setGpuSize(cloud_info->gpu_size_);
    return;
  }
  const auto & message = cloud_info->message_;
  cloud_info->cloud_->setGpuPoints(
    std::shared_ptr<const uint8_t>(message, message->data.data()),
    message->width * message->height, cloud_info->gpu_layout_, cloud_info->gpu_color_,
    cloud_info->gpu_size_);
}

rviz_rendering::PointCloud::GpuSize PointCloudCommon::gpuSize(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud, bool gpu)
{
  V_string fields;
  for (const auto & field : cloud->fields) {
    if (!field.name.empty()) {
      fields.push_back(field.name);
    }
  }
  std::sort(fields.begin(), fields.end());
  if (fields != size_fields_) {
    size_field_property_->clearOptions();
    size_field_property_->addOptionStd("");
    for (const auto & field : fields) {
      size_field_property_->addOptionStd(field);
    }
    size_fields_ = fields;
  }

  using GpuColor = rviz_rendering::PointCloud::GpuColor;
  rviz_rendering::PointCloud::GpuSize size;
  const std::string name = size_field_property_->getStdString();
  if (name.empty()) {
    display_->deleteStatusStd("Size Field");
    return size;
  }
  const int32_t index = findChannelIndex(cloud, name);
  if (index == -1) {
    display_->setStatusStd(
      rviz_common::properties::StatusProperty::Warn, "Size Field", "No field '" + name + "'");
    return size;
  }
  switch (cloud->fields[index].datatype) {
    case sensor_msgs::msg::PointField::FLOAT32:
      size.field_type = GpuColor::FLOAT32;
      break;
    case sensor_msgs::msg::PointField::INT8:
    case sensor_msgs::msg::PointField::UINT8:
      size.field_type = GpuColor::UINT8;
      break;
    case sensor_msgs::msg::PointField::INT16:
    case sensor_msgs::msg::PointField::UINT16:
      size.field_type = GpuColor::UINT16;
      break;
    case sensor_msgs::msg::PointField::INT32:
    case sensor_msgs::msg::PointField::UINT32:
      size.field_type = GpuColor::UINT32;
      break;
    default:
      display_->setStatusStd(
        rviz_common::properties::StatusProperty::Warn, "Size Field",
        "Field '" + name + "' is float64, which the GPU renderer does not read");
      return size;
  }
  if (!gpu) {
    display_->setStatusStd(
      rviz_common::properties::StatusProperty::Warn, "Size Field",
      "Not applied: this cloud is not drawn on the GPU");
    return size;
  }
  display_->deleteStatusStd("Size Field");

  float min_value;
  float max_value;
  if (size_auto_bounds_property_->getBool()) {
    fieldRange(*cloud, index, !cloud->is_dense, min_value, max_value);
    size_min_value_property_->setFloat(min_value);
    size_max_value_property_->setFloat(max_value);
  } else {
    min_value = size_min_value_property_->getFloat();
    max_value = size_max_value_property_->getFloat();
  }
  size.enabled = true;
  size.field_offset = cloud->fields[index].offset;
  size.min_value = min_value;
  size.value_range = max_value - min_value;
  size.min_scale = size_min_scale_property_->getFloat();
  size.max_scale = size_max_scale_property_->getFloat();
  return size;
}

bool PointCloudCommon::gpuPathFor(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const PointCloudTransformerPtr & xyz_trans, const PointCloudTransformerPtr & color_trans,
  rviz_rendering::PointCloud::GpuLayout & layout)
{
  auto mode = static_cast<rviz_rendering::PointCloud::RenderMode>(style_property_->getOptionInt());
  if (!dynamic_cast<XYZPCTransformer *>(xyz_trans.get()) ||
    !dynamic_cast<GpuColorTransformer *>(color_trans.get()) ||
    !rviz_rendering::PointCloud::supportsGpuPoints(mode))
  {
    return false;
  }
  uint32_t offsets[3];
  const char * names[3] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    const int32_t index = findChannelIndex(cloud, names[i]);
    if (index == -1 || cloud->fields[index].datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      cloud->fields[index].offset + sizeof(float) > cloud->point_step)
    {
      return false;
    }
    offsets[i] = cloud->fields[index].offset;
  }
  layout.point_step = cloud->point_step;
  layout.x_offset = offsets[0];
  layout.y_offset = offsets[1];
  layout.z_offset = offsets[2];
  layout.dense = cloud->is_dense;
  return true;
}

bool PointCloudCommon::drawsOnGpu(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  rviz_rendering::PointCloud::GpuLayout layout;
  return gpuPathFor(cloud, getXYZTransformer(cloud), getColorTransformer(cloud), layout);
}

bool PointCloudCommon::transformCloud(const CloudInfoPtr & cloud_info, bool update_transformers)
{
  if (!cloud_info->scene_node_) {
    if (!context_->getFrameManager()->getTransform(
        cloud_info->message_->header,
        cloud_info->position_,
        cloud_info->orientation_))
    {
      display_->setMissingTransformToFixedFrame(cloud_info->message_->header.frame_id);
      return false;
    }
    display_->setTransformOk();
  }
  // Remove outdated error message
  display_->deleteStatusStd(message_status_name_);

  V_PointCloudPoint & cloud_points = cloud_info->transformed_points_;
  cloud_points.clear();

  if (!transformPoints(cloud_info, cloud_points, update_transformers)) {
    return false;
  }

  if (!cloud_info->gpu_) {
    setProblematicPointsToInfinity(cloud_points);
  }
  return true;
}

bool PointCloudCommon::transformPoints(
  const CloudInfoPtr & cloud_info, V_PointCloudPoint & cloud_points, bool update_transformers)
{
  Ogre::Matrix4 transform;
  transform.makeTransform(cloud_info->position_, Ogre::Vector3(1, 1, 1), cloud_info->orientation_);

  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  if (update_transformers) {
    updateTransformers(cloud_info->message_);
  }
  PointCloudTransformerPtr xyz_trans = getXYZTransformer(cloud_info->message_);
  PointCloudTransformerPtr color_trans = getColorTransformer(cloud_info->message_);

  if (cloud_info->message_->data.size() !=
    cloud_info->message_->width * cloud_info->message_->height * cloud_info->message_->point_step)
  {
    std::string status = "PointCloud contained not enough or too much data";
    display_->setStatusStd(
      rviz_common::properties::StatusProperty::Error, message_status_name_, status);
    return false;
  }

  if (!xyz_trans) {
    std::string status = "No position transformer available for cloud";
    display_->setStatusStd(
      rviz_common::properties::StatusProperty::Error, message_status_name_, status);
    return false;
  }

  if (!color_trans) {
    std::string status = "No color transformer available for cloud";
    display_->setStatusStd(
      rviz_common::properties::StatusProperty::Error, message_status_name_, status);
    return false;
  }

  cloud_info->gpu_ =
    gpuPathFor(cloud_info->message_, xyz_trans, color_trans, cloud_info->gpu_layout_) &&
    dynamic_cast<GpuColorTransformer *>(color_trans.get())->gpuColor(
    cloud_info->message_, transform, cloud_info->gpu_color_);
  cloud_info->gpu_size_ = gpuSize(cloud_info->message_, cloud_info->gpu_);
  if (cloud_info->gpu_) {
    return true;
  }

  size_t size = cloud_info->message_->width * cloud_info->message_->height;
  rviz_rendering::PointCloud::Point default_pt = {Ogre::Vector3::ZERO, Ogre::ColourValue(1, 1, 1)};
  cloud_points.resize(size, default_pt);

  xyz_trans->transform(
    cloud_info->message_, PointCloudTransformer::Support_XYZ, transform, cloud_points);
  color_trans->transform(
    cloud_info->message_, PointCloudTransformer::Support_Color, transform, cloud_points);
  return true;
}

void PointCloudCommon::setProblematicPointsToInfinity(V_PointCloudPoint & cloud_points)
{
  for (auto & cloud_point : cloud_points) {
    if (!rviz_common::validateFloats(cloud_point.position)) {
      cloud_point.position.x = 999999.0f;
      cloud_point.position.y = 999999.0f;
      cloud_point.position.z = 999999.0f;
    }
  }
}

void PointCloudCommon::addMessage(const sensor_msgs::msg::PointCloud::ConstSharedPtr cloud)
{
  addMessage(convertPointCloudToPointCloud2(cloud));
}

void PointCloudCommon::addMessage(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  processMessage(cloud);
}

void PointCloudCommon::setXyzTransformerOptions(rviz_common::properties::EnumProperty * prop)
{
  fillTransformerOptions(prop, PointCloudTransformer::Support_XYZ);
}

void PointCloudCommon::setColorTransformerOptions(rviz_common::properties::EnumProperty * prop)
{
  fillTransformerOptions(prop, PointCloudTransformer::Support_Color);
}

void PointCloudCommon::fillTransformerOptions(
  rviz_common::properties::EnumProperty * prop,
  uint32_t mask)
{
  prop->clearOptions();

  if (cloud_infos_.empty()) {
    return;
  }

  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg = cloud_infos_.front()->message_;

  for (auto transformer : transformers_) {
    const PointCloudTransformerPtr & trans = transformer.second.transformer;
    if ((trans->supports(msg) & mask) == mask) {
      prop->addOption(QString::fromStdString(transformer.first));
    }
  }
}

void PointCloudCommon::onDisable()
{
  for (auto cloud_info : cloud_infos_) {
    cloud_info->selection_handler_.reset();
  }
  for (auto obsolete_cloud_info : obsolete_cloud_infos_) {
    obsolete_cloud_info->selection_handler_.reset();
  }
}

float PointCloudCommon::getSelectionBoxSize()
{
  return style_property_->getOptionInt() != rviz_rendering::PointCloud::RM_POINTS ?
         point_world_size_property_->getFloat() : 0.004f;
}

}  // namespace rviz_default_plugins
