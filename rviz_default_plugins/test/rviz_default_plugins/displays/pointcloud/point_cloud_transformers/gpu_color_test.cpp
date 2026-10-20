/*
 * Copyright (c) 2026, nevlife.
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


#include <gmock/gmock.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

#include <OgreMatrix4.h>
#include <OgreQuaternion.h>
#include <QList>  // NOLINT: cpplint is unable to handle the include order here

#include "../../../pointcloud_messages.hpp"

#include "rviz_common/properties/property.hpp"
#include "rviz_default_plugins/displays/pointcloud/transformers/axis_color_pc_transformer.hpp"
#include "rviz_default_plugins/displays/pointcloud/transformers/flat_color_pc_transformer.hpp"
#include "rviz_default_plugins/displays/pointcloud/transformers/intensity_pc_transformer.hpp"
#include "rviz_default_plugins/displays/pointcloud/transformers/rgb8_pc_transformer.hpp"

using namespace ::testing;  // NOLINT
using namespace rviz_default_plugins;  // NOLINT

namespace
{

// The shader path must give every point the color the CPU transformer gives it.
void expectSameColors(
  PointCloudTransformer & transformer, const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const Ogre::Matrix4 & transform = Ogre::Matrix4::IDENTITY)
{
  V_PointCloudPoint points(cloud->width * cloud->height);
  transformer.transform(cloud, PointCloudTransformer::Support_Color, transform, points);

  rviz_rendering::PointCloud::GpuColor color;
  ASSERT_TRUE(dynamic_cast<GpuColorTransformer &>(transformer).gpuColor(cloud, transform, color));
  for (size_t i = 0; i < points.size(); ++i) {
    const uint8_t * record = cloud->data.data() + i * cloud->point_step;
    Ogre::Vector3 position;
    std::memcpy(&position, record, sizeof(position));
    const Ogre::ColourValue gpu = color.colorOf(record, position);
    EXPECT_THAT(gpu.r, FloatNear(points[i].color.r, 1e-6f)) << "point " << i;
    EXPECT_THAT(gpu.g, FloatNear(points[i].color.g, 1e-6f)) << "point " << i;
    EXPECT_THAT(gpu.b, FloatNear(points[i].color.b, 1e-6f)) << "point " << i;
    EXPECT_THAT(gpu.a, FloatNear(points[i].color.a, 1e-6f)) << "point " << i;
  }
}

std::vector<PointWithIntensity> ramp()
{
  std::vector<PointWithIntensity> points;
  for (int i = 0; i < 64; ++i) {
    points.emplace_back(0.3f * i - 4.0f, std::sin(0.2f * i), 0.1f * i, 2.5f * i - 17.0f);
  }
  return points;
}

sensor_msgs::msg::PointCloud2::SharedPtr cloudWithIntensityType(uint8_t datatype, size_t size)
{
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(
    *createPointCloud2WithIntensity(ramp()));
  cloud->fields[3].datatype = datatype;
  for (uint32_t i = 0; i < cloud->width; ++i) {
    const uint32_t value = (i * 2654435761u) >> (32 - 8 * size);
    std::memcpy(&cloud->data[i * cloud->point_step + 12], &value, size);
  }
  return cloud;
}

}  // namespace

TEST(GpuColor, intensity_rainbow_matches_the_cpu_colors) {
  IntensityPCTransformer transformer;
  QList<rviz_common::properties::Property *> props;
  transformer.createProperties(nullptr, PointCloudTransformer::Support_Color, props);

  expectSameColors(transformer, createPointCloud2WithIntensity(ramp()));

  ASSERT_THAT(props[2]->getNameStd(), StrEq("Invert Rainbow"));
  props[2]->setValue(true);
  expectSameColors(transformer, createPointCloud2WithIntensity(ramp()));
}

TEST(GpuColor, intensity_min_max_colors_and_manual_bounds_match_the_cpu_colors) {
  IntensityPCTransformer transformer;
  QList<rviz_common::properties::Property *> props;
  transformer.createProperties(nullptr, PointCloudTransformer::Support_Color, props);
  props[1]->setValue(false);
  props[3]->setValue(QColor(10, 20, 200));
  props[4]->setValue(QColor(250, 128, 0));
  expectSameColors(transformer, createPointCloud2WithIntensity(ramp()));

  props[5]->setValue(false);
  props[6]->setValue(-5.0f);
  props[7]->setValue(40.0f);
  expectSameColors(transformer, createPointCloud2WithIntensity(ramp()));
}

TEST(GpuColor, intensity_integer_fields_match_the_cpu_colors) {
  IntensityPCTransformer transformer;
  QList<rviz_common::properties::Property *> props;
  transformer.createProperties(nullptr, PointCloudTransformer::Support_Color, props);

  expectSameColors(transformer, cloudWithIntensityType(sensor_msgs::msg::PointField::UINT8, 1));
  expectSameColors(transformer, cloudWithIntensityType(sensor_msgs::msg::PointField::INT16, 2));
  expectSameColors(transformer, cloudWithIntensityType(sensor_msgs::msg::PointField::UINT32, 4));
}

TEST(GpuColor, intensity_float64_and_missing_channels_stay_on_the_cpu) {
  IntensityPCTransformer transformer;
  QList<rviz_common::properties::Property *> props;
  transformer.createProperties(nullptr, PointCloudTransformer::Support_Color, props);
  rviz_rendering::PointCloud::GpuColor color;

  auto cloud = cloudWithIntensityType(sensor_msgs::msg::PointField::FLOAT64, 4);
  EXPECT_FALSE(transformer.gpuColor(cloud, Ogre::Matrix4::IDENTITY, color));

  props[0]->setValue("reflectivity");
  EXPECT_FALSE(
    transformer.gpuColor(
      createPointCloud2WithIntensity(ramp()), Ogre::Matrix4::IDENTITY, color));
}

TEST(GpuColor, intensity_bounds_skip_points_the_shader_does_not_draw) {
  IntensityPCTransformer transformer;
  QList<rviz_common::properties::Property *> props;
  transformer.createProperties(nullptr, PointCloudTransformer::Support_Color, props);
  auto points = ramp();
  points[0].x = std::nanf("");
  points[63].z = INFINITY;
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(
    *createPointCloud2WithIntensity(points));
  cloud->is_dense = false;
  rviz_rendering::PointCloud::GpuColor color;

  ASSERT_TRUE(transformer.gpuColor(cloud, Ogre::Matrix4::IDENTITY, color));

  EXPECT_THAT(color.min_value, FloatEq(points[1].intensity));
  EXPECT_THAT(color.value_range, FloatEq(points[62].intensity - points[1].intensity));
}

TEST(GpuColor, rgb8_matches_the_cpu_colors) {
  RGB8PCTransformer transformer;
  std::vector<ColoredPoint> points;
  for (int i = 0; i < 32; ++i) {
    points.emplace_back(i, 0, 0, (i * 37) % 256 / 255.0f, (i * 11) % 256 / 255.0f, i / 255.0f);
  }
  expectSameColors(transformer, create8BitColoredPointCloud2(points));

  auto rgba = std::make_shared<sensor_msgs::msg::PointCloud2>(
    *create8BitColoredPointCloud2(points));
  rgba->fields[3].name = "rgba";
  for (uint32_t i = 0; i < rgba->width; ++i) {
    rgba->data[i * rgba->point_step + rgba->fields[3].offset + 3] = static_cast<uint8_t>(7 * i);
  }
  expectSameColors(transformer, rgba);
}

TEST(GpuColor, flat_color_matches_the_cpu_colors) {
  FlatColorPCTransformer transformer;
  QList<rviz_common::properties::Property *> props;
  transformer.createProperties(nullptr, PointCloudTransformer::Support_Color, props);
  props[0]->setValue(QColor(200, 100, 50));

  expectSameColors(transformer, createPointCloud2WithIntensity(ramp()));
}

TEST(GpuColor, axis_color_matches_the_cpu_colors_in_the_fixed_and_local_frame) {
  AxisColorPCTransformer transformer;
  QList<rviz_common::properties::Property *> props;
  transformer.createProperties(nullptr, PointCloudTransformer::Support_Color, props);
  Ogre::Matrix4 transform;
  transform.makeTransform(
    Ogre::Vector3(1.5f, -0.5f, 0.75f), Ogre::Vector3(1, 1, 1),
    Ogre::Quaternion(Ogre::Radian(0.4f), Ogre::Vector3(0.3f, 0.2f, 0.9f).normalisedCopy()));

  expectSameColors(transformer, createPointCloud2WithIntensity(ramp()), transform);

  props[0]->setValue("X");
  props[2]->setValue(false);
  expectSameColors(transformer, createPointCloud2WithIntensity(ramp()), transform);
}
