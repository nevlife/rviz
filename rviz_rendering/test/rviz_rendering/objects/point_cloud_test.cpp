/*
 * Copyright (c) 2017, Bosch Software Innovations GmbH.
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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>
#include <regex>
#include <utility>
#include <vector>

#include <QApplication>  // NOLINT
#include <QMainWindow>  // NOLINT
#include <QTimer>  // NOLINT
#include <QWidget>  // NOLINT

#include "rviz_rendering/objects/point_cloud.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_rendering/custom_parameter_indices.hpp"
#include "../ogre_testing_environment.hpp"

#include "../matcher.hpp"

using namespace ::testing;  // NOLINT

class PointCloudTestFixture : public ::testing::Test
{
protected:
  void SetUp()
  {
    testing_environment_ = std::make_shared<rviz_rendering::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();
  }

  std::shared_ptr<rviz_rendering::OgreTestingEnvironment> testing_environment_;
};

static Ogre::ColourValue colorValue = Ogre::ColourValue(0.5f, 0.5f, 0.5f, 1.0f);

static std::vector<rviz_rendering::PointCloud::Point> squareCenteredAtZero
{
  {Ogre::Vector3(1, 1, 0), colorValue},
  {Ogre::Vector3(1, -1, 0), colorValue},
  {Ogre::Vector3(-1, 1, 0), colorValue},
  {Ogre::Vector3(-1, -1, 0), colorValue}
};

static std::vector<rviz_rendering::PointCloud::Point> singlePointArray
{
  {Ogre::Vector3(1, 2, 3), colorValue}
};

TEST_F(PointCloudTestFixture, addPoints_for_the_first_time_adds_renderable) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();

  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  auto renderables = point_cloud->getRenderables();
  ASSERT_THAT(renderables, SizeIs(1u));
}

TEST_F(PointCloudTestFixture, addPoints_many_points_gets_a_good_bounding_box_for_points) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();

  point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  ASSERT_THAT(
    point_cloud->getBoundingBox(),
    AllOf(
      HasMinimum(Ogre::Vector3(-1, -1, 0)),
      HasMaximum(Ogre::Vector3(1, 1, 0))
  ));
}

TEST_F(PointCloudTestFixture, clear_resets_bounding_box_bounding_radius_and_clears_points) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();

  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  point_cloud->clear();

  auto renderables = point_cloud->getRenderables();
  ASSERT_THAT(renderables, IsEmpty());
  ASSERT_THAT(point_cloud->getBoundingRadius(), FloatEq(0.0f));
}

TEST_F(
  PointCloudTestFixture,
  getBoundingRadius_returns_length_to_point_farthest_away_from_origin) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();

  point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  ASSERT_THAT(point_cloud->getBoundingRadius(), FloatEq(Ogre::Math::Sqrt(2)));
}

TEST_F(
  PointCloudTestFixture,
  for_one_point_getBoundingBox_returns_bounding_box_containing_only_one_point) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();

  std::vector<rviz_rendering::PointCloud::Point> points{{Ogre::Vector3(1, -1, 0), colorValue}};
  point_cloud->addPoints(points.begin(), points.end());

  ASSERT_THAT(
    point_cloud->getBoundingBox(),
    AllOf(
      HasMinimum(points[0].position),
      HasMaximum(points[0].position)
  ));
}

TEST_F(PointCloudTestFixture, getBoundingBox_adding_points_correctly_expands_bounding_box) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();

  point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());
  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  ASSERT_THAT(
    point_cloud->getBoundingBox(),
    AllOf(
      HasMinimum(squareCenteredAtZero[3].position),
      HasMaximum(singlePointArray[0].position)
  ));
}

TEST_F(PointCloudTestFixture, popPoints_removes_renderable_if_empty) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  point_cloud->popPoints(4);

  auto renderables = point_cloud->getRenderables();
  ASSERT_THAT(renderables, IsEmpty());
}

TEST_F(PointCloudTestFixture, popPoints_correctly_limits_bounding_box_on_removing_points) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  point_cloud->popPoints(2);
  ASSERT_THAT(
    point_cloud->getBoundingBox(),
    AllOf(
      HasMinimum(squareCenteredAtZero[3].position),
      HasMaximum(squareCenteredAtZero[2].position)
  ));
}

TEST_F(PointCloudTestFixture, popPoints_removes_bounding_box_when_completely_empty) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  point_cloud->popPoints(4);

  ASSERT_TRUE(point_cloud->getBoundingBox().isNull());
}

TEST_F(PointCloudTestFixture, setHighlightColor_sets_correct_CustomParameter) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  point_cloud->setHighlightColor(0.6f, 0.6f, 0.6f);

  auto renderables = point_cloud->getRenderables();
  for (auto const & renderable : renderables) {
    ASSERT_THAT(
      renderable->getCustomParameter(RVIZ_RENDERING_HIGHLIGHT_PARAMETER),
      Vector4Eq(Ogre::Vector4(0.6f, 0.6f, 0.6f, 0)));
  }
}

TEST_F(PointCloudTestFixture, setDimensions_changes_dimensions_of_points_and_newly_added_points) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  point_cloud->setDimensions(0.1f, 0.2f, 0.3f);

  auto renderables = point_cloud->getRenderables();
  for (auto const & renderable : renderables) {
    ASSERT_THAT(
      renderable->getCustomParameter(RVIZ_RENDERING_SIZE_PARAMETER),
      Vector4Eq(Ogre::Vector4(0.1f, 0.2f, 0.3f, 0)));
  }

  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  renderables = point_cloud->getRenderables();
  for (auto const & renderable : renderables) {
    ASSERT_THAT(
      renderable->getCustomParameter(RVIZ_RENDERING_SIZE_PARAMETER),
      Vector4Eq(Ogre::Vector4(0.1f, 0.2f, 0.3f, 0)));
  }
}

TEST_F(PointCloudTestFixture, setRenderMode_changes_material) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_POINTS);

  auto renderables = point_cloud->getRenderables();
  for (auto const & renderable : renderables) {
    std::regex regex("^PointCloudMaterial\\d+Point$");
    ASSERT_TRUE(std::regex_match(renderable->getMaterial()->getName(), regex));
  }

  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_TILES);

  renderables = point_cloud->getRenderables();
  for (auto const & renderable : renderables) {
    std::regex regex("^PointCloudMaterial\\d+Tiles$");
    ASSERT_TRUE(std::regex_match(renderable->getMaterial()->getName(), regex));
  }
}

TEST_F(
  PointCloudTestFixture,
  setRenderMode_regenerates_renderables_with_different_size_when_geometry_support_changes) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_POINTS);

  auto renderables = point_cloud->getRenderables();
  for (auto const & renderable : renderables) {
    size_t number_of_vertices_per_point = point_cloud->getVerticesPerPoint();
    ASSERT_THAT(renderable->getBuffer()->getNumVertices(), Eq(number_of_vertices_per_point));
  }

  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_BOXES);

  renderables = point_cloud->getRenderables();
  for (auto const & renderable : renderables) {
    size_t number_of_vertices_per_box = point_cloud->getVerticesPerPoint();
    ASSERT_THAT(renderable->getBuffer()->getNumVertices(), Eq(number_of_vertices_per_box));
  }
}

TEST_F(PointCloudTestFixture, addPoints_adds_new_renderable_whenever_it_is_called) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_POINTS);

  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());
  point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  auto renderables = point_cloud->getRenderables();
  ASSERT_THAT(renderables, SizeIs(2));
}


TEST_F(PointCloudTestFixture, addPoints_adds_vertices_with_correct_geometry_when_called) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_FLAT_SQUARES);
  size_t number_of_vertices_per_flat_square = point_cloud->getVerticesPerPoint();

  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  auto renderables = point_cloud->getRenderables();
  for (auto const & renderable : renderables) {
    ASSERT_THAT(renderable->getBuffer()->getNumVertices(), Eq(number_of_vertices_per_flat_square));
  }

  point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());

  renderables = point_cloud->getRenderables();
  size_t number_of_vertices_in_all_renderables = 0;
  for (auto const & renderable : renderables) {
    number_of_vertices_in_all_renderables += renderable->getBuffer()->getNumVertices();
  }
  ASSERT_THAT(number_of_vertices_in_all_renderables, Eq(number_of_vertices_per_flat_square * 5));
}

TEST_F(
  PointCloudTestFixture,
  adding_and_removing_points_many_times_does_not_lead_to_superfluous_renderables) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_BOXES);

  for (int i = 0; i < 1000; i++) {
    point_cloud->addPoints(squareCenteredAtZero.begin(), squareCenteredAtZero.end());
  }
  for (int i = 0; i < 99; i++) {
    point_cloud->popPoints(40);
  }

  ASSERT_THAT(point_cloud->getRenderables(), SizeIs(10));
}

namespace
{

struct GpuRecord
{
  float x, y, z;
  float intensity;
  uint32_t rgb;
};

std::shared_ptr<const uint8_t> gpuRecords(const std::vector<GpuRecord> & records)
{
  auto bytes = std::make_shared<std::vector<uint8_t>>(records.size() * sizeof(GpuRecord));
  std::memcpy(bytes->data(), records.data(), bytes->size());
  return std::shared_ptr<const uint8_t>(bytes, bytes->data());
}

rviz_rendering::PointCloud::GpuLayout gpuLayout()
{
  rviz_rendering::PointCloud::GpuLayout layout;
  layout.point_step = sizeof(GpuRecord);
  layout.x_offset = offsetof(GpuRecord, x);
  layout.y_offset = offsetof(GpuRecord, y);
  layout.z_offset = offsetof(GpuRecord, z);
  return layout;
}

const std::vector<GpuRecord> gpu_square {
  {1, 1, 0, 0.0f, 0x00ff0000},
  {1, -1, 0, 1.0f, 0x0000ff00},
  {-1, 1, 0, 2.0f, 0x000000ff},
  {-1, -1, 0, 4.0f, 0x80402010}
};

}  // namespace

class GpuPointCloudTestFixture : public PointCloudTestFixture
{
protected:
  void SetUp() override
  {
    PointCloudTestFixture::SetUp();
    if (!rviz_rendering::PointCloud::supportsGpuPoints(
        rviz_rendering::PointCloud::RM_FLAT_SQUARES))
    {
      GTEST_SKIP() << "render system cannot draw raw point buffers";
    }
  }
};

TEST_F(GpuPointCloudTestFixture, setGpuPoints_draws_one_instanced_renderable) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_FLAT_SQUARES);

  point_cloud->setGpuPoints(
    gpuRecords(gpu_square), 4, gpuLayout(), rviz_rendering::PointCloud::GpuColor());

  ASSERT_TRUE(point_cloud->usesGpuPoints());
  auto renderables = point_cloud->getRenderables();
  ASSERT_THAT(renderables, SizeIs(1u));
  auto op = renderables[0]->getRenderOperation();
  EXPECT_THAT(op->numberOfInstances, Eq(4u));
  EXPECT_THAT(op->vertexData->vertexCount, Eq(6u));
  EXPECT_THAT(op->operationType, Eq(Ogre::RenderOperation::OT_TRIANGLE_LIST));
  EXPECT_TRUE(op->vertexData->vertexBufferBinding->getBuffer(1)->isInstanceData());
}

TEST_F(GpuPointCloudTestFixture, points_mode_draws_one_vertex_per_point_without_instancing) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_POINTS);

  point_cloud->setGpuPoints(
    gpuRecords(gpu_square), 4, gpuLayout(), rviz_rendering::PointCloud::GpuColor());

  auto op = point_cloud->getRenderables()[0]->getRenderOperation();
  EXPECT_THAT(op->vertexData->vertexCount, Eq(4u));
  EXPECT_THAT(op->numberOfInstances, Eq(1u));
  EXPECT_THAT(op->operationType, Eq(Ogre::RenderOperation::OT_POINT_LIST));
  EXPECT_FALSE(op->vertexData->vertexBufferBinding->getBuffer(0)->isInstanceData());
}

TEST_F(GpuPointCloudTestFixture, bounding_box_skips_points_with_non_finite_coordinates) {
  auto records = gpu_square;
  records.push_back({std::nanf(""), 100, 100, 0, 0});
  records.push_back({100, std::numeric_limits<float>::infinity(), 100, 0, 0});
  auto layout = gpuLayout();
  layout.dense = false;
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();

  point_cloud->setGpuPoints(gpuRecords(records), 6, layout, {});

  EXPECT_THAT(
    point_cloud->getBoundingBox(),
    AllOf(HasMinimum(Ogre::Vector3(-1, -1, 0)), HasMaximum(Ogre::Vector3(1, 1, 0))));
}

TEST_F(GpuPointCloudTestFixture, getPoints_decodes_positions_and_shader_colors) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  rviz_rendering::PointCloud::GpuColor color;
  color.source = rviz_rendering::PointCloud::GpuColor::FIELD;
  color.field_offset = offsetof(GpuRecord, intensity);
  color.min_value = 0.0f;
  color.value_range = 4.0f;
  color.rainbow = false;
  color.min_color = Ogre::ColourValue(0, 0, 0);
  color.max_color = Ogre::ColourValue(1, 0.5f, 0);

  point_cloud->setGpuPoints(gpuRecords(gpu_square), 4, gpuLayout(), color);

  auto points = point_cloud->getPoints();
  ASSERT_THAT(points, SizeIs(4u));
  EXPECT_THAT(points[1].position, Vector3Eq(Ogre::Vector3(1, -1, 0)));
  EXPECT_THAT(points[2].color, Eq(Ogre::ColourValue(0.5f, 0.25f, 0.0f)));
  EXPECT_THAT(points[3].color, Eq(Ogre::ColourValue(1.0f, 0.5f, 0.0f)));

  color.source = rviz_rendering::PointCloud::GpuColor::RGBA;
  color.field_offset = offsetof(GpuRecord, rgb);
  point_cloud->setGpuColor(color);
  points = point_cloud->getPoints();
  EXPECT_THAT(points[0].color, Eq(Ogre::ColourValue(1, 0, 0, 0)));
  EXPECT_THAT(
    points[3].color,
    Eq(Ogre::ColourValue(0x40 / 255.0f, 0x20 / 255.0f, 0x10 / 255.0f, 0x80 / 255.0f)));
}

TEST_F(GpuPointCloudTestFixture, setGpuColor_sets_the_shader_parameters) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->setGpuPoints(gpuRecords(gpu_square), 4, gpuLayout(), {});
  rviz_rendering::PointCloud::GpuColor color;
  color.source = rviz_rendering::PointCloud::GpuColor::AXIS;
  color.axis = Ogre::Vector4(0, 0, 1, 2);
  color.min_value = -3.0f;
  color.value_range = 6.0f;
  color.invert = true;

  point_cloud->setGpuColor(color);

  auto renderable = point_cloud->getRenderables()[0];
  EXPECT_THAT(
    renderable->getCustomParameter(RVIZ_RENDERING_COLOR_MODE_PARAMETER),
    Vector4Eq(Ogre::Vector4(rviz_rendering::PointCloud::GpuColor::AXIS, 0, 1, 1)));
  EXPECT_THAT(
    renderable->getCustomParameter(RVIZ_RENDERING_COLOR_RANGE_PARAMETER),
    Vector4Eq(Ogre::Vector4(-3, 6, 0, 0)));
  EXPECT_THAT(
    renderable->getCustomParameter(RVIZ_RENDERING_COLOR_AXIS_PARAMETER),
    Vector4Eq(Ogre::Vector4(0, 0, 1, 2)));
}

TEST_F(GpuPointCloudTestFixture, flat_color_is_truncated_to_bytes_like_the_cpu_path) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  rviz_rendering::PointCloud::GpuColor color;
  color.min_color = Ogre::ColourValue(0.5f, 0.999f, 0.0f);

  point_cloud->setGpuPoints(gpuRecords(gpu_square), 4, gpuLayout(), color);

  EXPECT_THAT(
    point_cloud->getRenderables()[0]->getCustomParameter(RVIZ_RENDERING_COLOR_A_PARAMETER),
    Vector4Eq(Ogre::Vector4(127 / 255.0f, 254 / 255.0f, 0, 1)));
}

TEST_F(GpuPointCloudTestFixture, render_mode_without_gpu_support_drops_the_gpu_points) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->setGpuPoints(gpuRecords(gpu_square), 4, gpuLayout(), {});

  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_BOXES);

  EXPECT_FALSE(point_cloud->usesGpuPoints());
  EXPECT_THAT(point_cloud->getRenderables(), IsEmpty());
}

TEST_F(GpuPointCloudTestFixture, changing_render_mode_keeps_the_uploaded_points) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_FLAT_SQUARES);
  point_cloud->setGpuPoints(gpuRecords(gpu_square), 4, gpuLayout(), {});

  point_cloud->setRenderMode(rviz_rendering::PointCloud::RM_SPHERES);

  ASSERT_THAT(point_cloud->getRenderables(), SizeIs(1u));
  auto op = point_cloud->getRenderables()[0]->getRenderOperation();
  EXPECT_THAT(op->vertexData->vertexCount, Eq(3u));
  EXPECT_THAT(op->numberOfInstances, Eq(4u));
}

TEST_F(GpuPointCloudTestFixture, addPoints_after_gpu_points_goes_back_to_cpu_buffers) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  point_cloud->setGpuPoints(gpuRecords(gpu_square), 4, gpuLayout(), {});

  point_cloud->addPoints(singlePointArray.begin(), singlePointArray.end());

  EXPECT_FALSE(point_cloud->usesGpuPoints());
  ASSERT_THAT(point_cloud->getPoints(), SizeIs(1u));
  EXPECT_THAT(point_cloud->getPoints()[0].position, Vector3Eq(Ogre::Vector3(1, 2, 3)));
}

TEST_F(PointCloudTestFixture, addPoints_by_value_takes_the_points) {
  auto point_cloud = std::make_shared<rviz_rendering::PointCloud>();
  std::vector<rviz_rendering::PointCloud::Point> points = squareCenteredAtZero;

  point_cloud->addPoints(std::move(points));
  point_cloud->addPoints(std::vector<rviz_rendering::PointCloud::Point>(singlePointArray));

  ASSERT_THAT(point_cloud->getPoints(), SizeIs(5u));
  EXPECT_THAT(point_cloud->getPoints()[4].position, Vector3Eq(Ogre::Vector3(1, 2, 3)));
  EXPECT_THAT(
    point_cloud->getBoundingBox(),
    AllOf(HasMinimum(Ogre::Vector3(-1, -1, 0)), HasMaximum(Ogre::Vector3(1, 2, 3))));
}
