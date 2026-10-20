/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "rviz_rendering/objects/point_cloud.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector.h>
#include <OgreQuaternion.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreBillboardSet.h>
#include <OgreBillboard.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreCamera.h>
#include <OgreViewport.h>
#include <OgreRenderSystem.h>
#include <OgreRenderSystemCapabilities.h>

#include "rviz_rendering/custom_parameter_indices.hpp"
#include "rviz_rendering/logging.hpp"
#include "rviz_rendering/material_manager.hpp"

// TODO(greimela): Add again after clearing up the module dependencies
// #include "rviz_rendering/selection/forwards.hpp"

#define VERTEX_BUFFER_CAPACITY (36 * 1024 * 10)

namespace rviz_rendering
{

static float g_point_vertices[3] =
{
  0.0f, 0.0f, 0.0f
};

static float g_billboard_vertices[6 * 3] =
{
  -0.5f, 0.5f, 0.0f,
  -0.5f, -0.5f, 0.0f,
  0.5f, 0.5f, 0.0f,
  0.5f, 0.5f, 0.0f,
  -0.5f, -0.5f, 0.0f,
  0.5f, -0.5f, 0.0f,
};

static float g_billboard_sphere_vertices[3 * 3] =
{
  0.0f, 1.0f, 0.0f,
  -0.866025404f, -0.5f, 0.0f,
  0.866025404f, -0.5f, 0.0f,
};

static float g_box_vertices[6 * 6 * 3] =
{
  // front
  -0.5f, 0.5f, -0.5f,
  -0.5f, -0.5f, -0.5f,
  0.5f, 0.5f, -0.5f,
  0.5f, 0.5f, -0.5f,
  -0.5f, -0.5f, -0.5f,
  0.5f, -0.5f, -0.5f,

  // back
  -0.5f, 0.5f, 0.5f,
  0.5f, 0.5f, 0.5f,
  -0.5f, -0.5f, 0.5f,
  0.5f, 0.5f, 0.5f,
  0.5f, -0.5f, 0.5f,
  -0.5f, -0.5f, 0.5f,

  // right
  0.5f, 0.5f, 0.5f,
  0.5f, 0.5f, -0.5f,
  0.5f, -0.5f, 0.5f,
  0.5f, 0.5f, -0.5f,
  0.5f, -0.5f, -0.5f,
  0.5f, -0.5f, 0.5f,

  // left
  -0.5f, 0.5f, 0.5f,
  -0.5f, -0.5f, 0.5f,
  -0.5f, 0.5f, -0.5f,
  -0.5f, 0.5f, -0.5f,
  -0.5f, -0.5f, 0.5f,
  -0.5f, -0.5f, -0.5f,

  // top
  -0.5f, 0.5f, -0.5f,
  0.5f, 0.5f, -0.5f,
  -0.5f, 0.5f, 0.5f,
  0.5f, 0.5f, -0.5f,
  0.5f, 0.5f, 0.5f,
  -0.5f, 0.5f, 0.5f,

  // bottom
  -0.5f, -0.5f, -0.5f,
  -0.5f, -0.5f, 0.5f,
  0.5f, -0.5f, -0.5f,
  0.5f, -0.5f, -0.5f,
  -0.5f, -0.5f, 0.5f,
  0.5f, -0.5f, 0.5f,
};

Ogre::String PointCloud::sm_Type = "PointCloud";

uint32_t PointCloud::getVerticesPerPoint()
{
  if (current_mode_supports_geometry_shader_) {
    return 1;
  }
  switch (render_mode_) {
    case RM_POINTS:
      return 1;
    case RM_SQUARES:
      return 6;
    case RM_FLAT_SQUARES:
      return 6;
    case RM_SPHERES:
      return 3;
    case RM_TILES:
      return 6;
    case RM_BOXES:
      return 36;
    case RM_DEPTH_FADE_POINTS:
      return 1;
    default:
      throw std::runtime_error("unexpected render_mode_");
  }
}

float * PointCloud::getVertices()
{
  if (current_mode_supports_geometry_shader_) {
    return g_point_vertices;
  }
  switch (render_mode_) {
    case RM_POINTS:
      return g_point_vertices;
    case RM_SQUARES:
      return g_billboard_vertices;
    case RM_FLAT_SQUARES:
      return g_billboard_vertices;
    case RM_SPHERES:
      return g_billboard_sphere_vertices;
    case RM_TILES:
      return g_billboard_vertices;
    case RM_BOXES:
      return g_box_vertices;
    case RM_DEPTH_FADE_POINTS:
      return g_point_vertices;
    default:
      throw std::runtime_error("unexpected render_mode_");
  }
}

static const char * gpuMaterialName(PointCloud::RenderMode mode)
{
  switch (mode) {
    case PointCloud::RM_POINTS:
      return "rviz/PointCloudGpuPoint";
    case PointCloud::RM_SQUARES:
      return "rviz/PointCloudGpuSquare";
    case PointCloud::RM_FLAT_SQUARES:
      return "rviz/PointCloudGpuFlatSquare";
    case PointCloud::RM_SPHERES:
      return "rviz/PointCloudGpuSphere";
    case PointCloud::RM_DEPTH_FADE_POINTS:
      return "rviz/PointCloudGpuDepthFadePoint";
    default:
      return nullptr;
  }
}

Ogre::MaterialPtr PointCloud::getGpuMaterial(RenderMode mode)
{
  switch (mode) {
    case RM_POINTS:
      return gpu_point_material_;
    case RM_SQUARES:
      return gpu_square_material_;
    case RM_FLAT_SQUARES:
      return gpu_flat_square_material_;
    case RM_SPHERES:
      return gpu_sphere_material_;
    case RM_DEPTH_FADE_POINTS:
      return gpu_depth_fade_point_material_;
    default:
      return Ogre::MaterialPtr();
  }
}

Ogre::MaterialPtr PointCloud::getMaterialForRenderMode(RenderMode mode)
{
  switch (mode) {
    case RM_POINTS:
      return Ogre::MaterialPtr(point_material_);
    case RM_SQUARES:
      return Ogre::MaterialPtr(square_material_);
    case RM_FLAT_SQUARES:
      return Ogre::MaterialPtr(flat_square_material_);
    case RM_SPHERES:
      return Ogre::MaterialPtr(sphere_material_);
    case RM_TILES:
      return Ogre::MaterialPtr(tile_material_);
    case RM_BOXES:
      return Ogre::MaterialPtr(box_material_);
    case RM_DEPTH_FADE_POINTS:
      return Ogre::MaterialPtr(depth_fade_point_material_);
    default:
      throw std::runtime_error("unexpected render_mode_");
  }
}

PointCloud::PointCloud()
: point_count_(0),
  common_direction_(Ogre::Vector3::NEGATIVE_UNIT_Z),
  common_up_vector_(Ogre::Vector3::UNIT_Y),
  color_by_index_(false),
  current_mode_supports_geometry_shader_(false)
{
  std::stringstream ss;
  static int count = 0;
  ss << "PointCloudMaterial" << count++;
  point_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudPoint");
  square_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudSquare");
  flat_square_material_ = Ogre::MaterialManager::getSingleton().getByName(
    "rviz/PointCloudFlatSquare");
  sphere_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudSphere");
  tile_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudTile");
  box_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudBox");
  depth_fade_point_material_ = Ogre::MaterialManager::getSingleton().getByName(
    "rviz/PointCloudDepthFadePoint");

  point_material_ = Ogre::MaterialPtr(point_material_)->clone(ss.str() + "Point");
  square_material_ = Ogre::MaterialPtr(square_material_)->clone(ss.str() + "Square");
  flat_square_material_ = Ogre::MaterialPtr(flat_square_material_)->clone(ss.str() + "FlatSquare");
  sphere_material_ = Ogre::MaterialPtr(sphere_material_)->clone(ss.str() + "Sphere");
  tile_material_ = Ogre::MaterialPtr(tile_material_)->clone(ss.str() + "Tiles");
  box_material_ = Ogre::MaterialPtr(box_material_)->clone(ss.str() + "Box");
  depth_fade_point_material_ =
    Ogre::MaterialPtr(depth_fade_point_material_)->clone(ss.str() + "DepthFadePoint");

  point_material_->load();
  square_material_->load();
  flat_square_material_->load();
  sphere_material_->load();
  tile_material_->load();
  box_material_->load();
  depth_fade_point_material_->load();

  setAlpha(1.0f);
  setRenderMode(RM_SPHERES);
  setDimensions(0.01f, 0.01f, 0.01f);

  clear();
}

static void removeMaterial(Ogre::MaterialPtr & material)
{
  Ogre::ResourcePtr resource(material);
  Ogre::MaterialManager::getSingleton().remove(resource);
}

PointCloud::~PointCloud()
{
  clear();

  point_material_->unload();
  square_material_->unload();
  flat_square_material_->unload();
  sphere_material_->unload();
  tile_material_->unload();
  box_material_->unload();
  depth_fade_point_material_->unload();

  removeMaterial(point_material_);
  removeMaterial(square_material_);
  removeMaterial(flat_square_material_);
  removeMaterial(sphere_material_);
  removeMaterial(tile_material_);
  removeMaterial(box_material_);
  removeMaterial(depth_fade_point_material_);

  for (auto material : {&gpu_point_material_, &gpu_square_material_, &gpu_flat_square_material_,
      &gpu_sphere_material_, &gpu_depth_fade_point_material_})
  {
    if (*material) {
      (*material)->unload();
      removeMaterial(*material);
    }
  }
}

const Ogre::AxisAlignedBox & PointCloud::getBoundingBox() const
{
  return bounding_box_;
}

float PointCloud::getBoundingRadius() const
{
  return bounding_box_.isNull() ?
         0.0f :
         Ogre::Math::Sqrt(
    std::max(
      bounding_box_.getMaximum().squaredLength(),
      bounding_box_.getMinimum().squaredLength()));
}

void PointCloud::getWorldTransforms(Ogre::Matrix4 * xform) const
{
  *xform = _getParentNodeFullTransform();
}

void PointCloud::clear()
{
  point_count_ = 0;
  bounding_box_.setNull();
  gpu_data_.reset();

  if (getParentSceneNode()) {
    for (auto const & renderable : renderables_) {
      getParentSceneNode()->detachObject(renderable.get());
    }
    getParentSceneNode()->needUpdate();
  }

  for (auto const & renderable : renderables_) {
    if (renderable != gpu_renderable_) {
      spare_renderables_.push_back(renderable);
    }
  }
  renderables_.clear();
}

void PointCloud::clearAndRemoveAllPoints()
{
  clear();
  points_.clear();
}

void PointCloud::regenerateAll()
{
  if (gpu_data_) {
    rebuildGpuRenderable();
    return;
  }
  if (point_count_ == 0) {
    return;
  }

  V_Point points;
  points.swap(points_);

  clear();

  addPoints(points.begin(), points.end());
}

void PointCloud::setColorByIndex(bool set)
{
  color_by_index_ = set;
  // The Pick1 technique of the GPU materials always writes index colors.
  if (!gpu_data_) {
    regenerateAll();
  }
}

void PointCloud::setHighlightColor(float r, float g, float b)
{
  Ogre::Vector4 highlight(r, g, b, 0.0f);

  for (auto & renderable : renderables_) {
    renderable->setCustomParameter(RVIZ_RENDERING_HIGHLIGHT_PARAMETER, highlight);
  }
}

void PointCloud::setRenderMode(RenderMode mode)
{
  render_mode_ = mode;

  current_material_ = getMaterialForRenderMode(mode);
  current_material_->load();

  if (changingGeometrySupportIsNecessary(current_material_)) {
    if (!gpu_data_) {
      renderables_.clear();
    }
    spare_renderables_.clear();
  }

  if (!gpu_data_) {
    for (auto & renderable : renderables_) {
      renderable->setMaterial(current_material_);
    }
  }

  regenerateAll();
}

bool PointCloud::changingGeometrySupportIsNecessary(Ogre::MaterialPtr const material)
{
  bool geom_support_changed = false;
  Ogre::Technique * best = material->getBestTechnique();
  if (best) {
    if (material->getBestTechnique()->getName() == "gp") {
      if (!current_mode_supports_geometry_shader_) {
        geom_support_changed = true;
      }

      current_mode_supports_geometry_shader_ = true;
    } else {
      if (current_mode_supports_geometry_shader_) {
        geom_support_changed = true;
      }

      current_mode_supports_geometry_shader_ = false;
    }
  } else {
    geom_support_changed = true;
    current_mode_supports_geometry_shader_ = false;

    RVIZ_RENDERING_LOG_ERROR_STREAM(
      "No techniques available for material [" << material->getName().c_str() << "]");
  }
  return geom_support_changed;
}

void PointCloud::setDimensions(float width, float height, float depth)
{
  point_extensions_ = Ogre::Vector4(width, height, depth, 0.0f);

  for (auto & renderable : renderables_) {
    renderable->setCustomParameter(RVIZ_RENDERING_SIZE_PARAMETER, point_extensions_);
  }
}

void PointCloud::setAutoSize(bool auto_size)
{
  for (auto & renderable : renderables_) {
    renderable->setCustomParameter(RVIZ_RENDERING_AUTO_SIZE_PARAMETER, Ogre::Vector4(auto_size));
  }
}

void PointCloud::setCommonDirection(const Ogre::Vector3 & vec)
{
  common_direction_ = vec;

  for (auto & renderable : renderables_) {
    renderable->setCustomParameter(RVIZ_RENDERING_NORMAL_PARAMETER, Ogre::Vector4(vec));
  }
}

void PointCloud::setCommonUpVector(const Ogre::Vector3 & vec)
{
  common_up_vector_ = vec;
  for (auto & renderable : renderables_) {
    renderable->setCustomParameter(RVIZ_RENDERING_UP_PARAMETER, Ogre::Vector4(vec));
  }
}

void setAlphaBlending(const Ogre::MaterialPtr & mat)
{
  if (mat->getBestTechnique()) {
    mat->getBestTechnique()->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    mat->getBestTechnique()->setDepthWriteEnabled(false);
  }
}

void setReplace(const Ogre::MaterialPtr & mat)
{
  if (mat->getBestTechnique()) {
    mat->getBestTechnique()->setSceneBlending(Ogre::SBT_REPLACE);
    mat->getBestTechnique()->setDepthWriteEnabled(true);
  }
}

void PointCloud::setAlpha(float alpha, bool per_point_alpha)
{
  alpha_ = alpha;
  per_point_alpha_ = per_point_alpha;

  const bool blend = alpha < rviz_rendering::unit_alpha_threshold || per_point_alpha;
  for (auto material : {&point_material_, &square_material_, &flat_square_material_,
      &sphere_material_, &tile_material_, &box_material_, &depth_fade_point_material_,
      &gpu_point_material_, &gpu_square_material_, &gpu_flat_square_material_,
      &gpu_sphere_material_, &gpu_depth_fade_point_material_})
  {
    if (!*material) {
      continue;
    }
    if (blend) {
      setAlphaBlending(*material);
    } else {
      setReplace(*material);
    }
  }

  Ogre::Vector4 alpha4(alpha_, alpha_, alpha_, alpha_);
  for (auto & renderable : renderables_) {
    renderable->setCustomParameter(RVIZ_RENDERING_ALPHA_PARAMETER, alpha4);
  }
}

void PointCloud::setColor(const Ogre::ColourValue & color)
{
  for (auto & point : points_) {
    point.setColor(color.r, color.g, color.b, color.a);
  }
  regenerateAll();
}

void PointCloud::addPoints(
  std::vector<Point>::iterator start_iterator,
  std::vector<Point>::iterator stop_iterator)
{
  if (stop_iterator - start_iterator <= 0) {
    return;
  }
  if (gpu_data_) {
    clear();
  }
  const auto first = static_cast<uint32_t>(points_.size());
  points_.insert(points_.cend(), start_iterator, stop_iterator);
  fillHardwareBuffers(first);
}

void PointCloud::addPoints(std::vector<Point> && points)
{
  if (points.empty()) {
    return;
  }
  if (gpu_data_) {
    clear();
  }
  const auto first = static_cast<uint32_t>(points_.size());
  if (points_.empty()) {
    points_ = std::move(points);
  } else {
    points_.insert(points_.cend(), points.begin(), points.end());
  }
  fillHardwareBuffers(first);
}

void PointCloud::fillHardwareBuffers(uint32_t first)
{
  const auto start_iterator = points_.begin() + first;
  const auto stop_iterator = points_.end();
  auto num_points = static_cast<uint32_t>(std::distance(start_iterator, stop_iterator));

  RenderableInternals internals = createNewRenderable(num_points);

  const uint32_t vertices_per_point = getVerticesPerPoint();
  const float * vertices = getVertices();
  const bool write_vertex_offsets = !current_mode_supports_geometry_shader_;

  for (auto current_point = start_iterator; current_point < stop_iterator; ++current_point) {
    if (internals.bufferIsFull()) {
      assert(internals.noBufferOverflowOccurred());

      finishRenderable(
        internals,
        static_cast<uint32_t>(internals.rend->getBuffer()->getNumVertices()));

      internals = createNewRenderable(static_cast<uint32_t>(stop_iterator - current_point));
    }
    internals.aabb.merge(current_point->position);
    addPointToHardwareBuffer(
      internals, current_point,
      static_cast<uint32_t>(current_point - start_iterator),
      vertices_per_point, vertices, write_vertex_offsets);
  }

  finishRenderable(internals, internals.current_vertex_count);
  spare_renderables_.clear();

  point_count_ += num_points;

  if (getParentSceneNode()) {
    getParentSceneNode()->needUpdate();
  }
}

PointCloud::RenderableInternals
PointCloud::createNewRenderable(uint32_t number_of_points_to_be_added)
{
  RenderableInternals internals;
  internals.buffer_size = std::min<uint32_t>(
    VERTEX_BUFFER_CAPACITY,
    number_of_points_to_be_added * getVerticesPerPoint());

  internals.rend = reuseSpareRenderable(internals.buffer_size, getRenderOperationType());
  const bool reused = internals.rend != nullptr;
  if (!reused) {
    internals.rend = createRenderable(internals.buffer_size, getRenderOperationType());
  }

  internals.float_buffer = reinterpret_cast<float *>(internals.rend->getBuffer()
    ->lock(reused ? Ogre::HardwareBuffer::HBL_DISCARD : Ogre::HardwareBuffer::HBL_NO_OVERWRITE));

  internals.aabb.setNull();
  return internals;
}

Ogre::RenderOperation::OperationType PointCloud::getRenderOperationType() const
{
  Ogre::RenderOperation::OperationType op_type;
  if (current_mode_supports_geometry_shader_) {
    op_type = Ogre::RenderOperation::OT_POINT_LIST;
  } else {
    if (render_mode_ == RM_POINTS || render_mode_ == RM_DEPTH_FADE_POINTS) {
      op_type = Ogre::RenderOperation::OT_POINT_LIST;
    } else {
      op_type = Ogre::RenderOperation::OT_TRIANGLE_LIST;
    }
  }
  return op_type;
}

void
PointCloud::finishRenderable(
  const PointCloud::RenderableInternals & internals,
  uint32_t vertex_count_of_renderable)
{
  Ogre::RenderOperation * op = internals.rend->getRenderOperation();
  op->vertexData->vertexCount = vertex_count_of_renderable - op->vertexData->vertexStart;
  internals.rend->setBoundingBox(internals.aabb);
  bounding_box_.merge(internals.aabb);
  assert(
    op->vertexData->vertexCount + op->vertexData->vertexStart <=
    internals.rend->getBuffer()->getNumVertices());

  internals.rend->getBuffer()->unlock();
}

uint32_t PointCloud::getColorForPoint(
  uint32_t current_point,
  std::vector<PointCloud::Point>::iterator point) const
{
  uint32_t color;

  if (color_by_index_) {
    // convert to ColourValue, so we can then convert to the rendersystem-specific color type
    color = (current_point + point_count_ + 1);
    Ogre::ColourValue c;
    c.a = 1.0f;
    c.r = ((color >> 16) & 0xff) / 255.0f;
    c.g = ((color >> 8) & 0xff) / 255.0f;
    c.b = (color & 0xff) / 255.0f;
    color = c.getAsBYTE();
  } else {
    color = point->color.getAsBYTE();
  }
  return color;
}

void
PointCloud::addPointToHardwareBuffer(
  PointCloud::RenderableInternals & internals,
  std::vector<PointCloud::Point>::iterator point, uint32_t current_point,
  uint32_t vertices_per_point, const float * vertices, bool write_vertex_offsets) const
{
  uint32_t color = getColorForPoint(current_point, point);
  float * float_buffer = internals.float_buffer;

  float x = point->position.x;
  float y = point->position.y;
  float z = point->position.z;

  for (uint32_t j = 0; j < vertices_per_point; ++j, ++internals.current_vertex_count) {
    *float_buffer++ = x;
    *float_buffer++ = y;
    *float_buffer++ = z;

    if (write_vertex_offsets) {
      *float_buffer++ = vertices[(j * 3)];
      *float_buffer++ = vertices[(j * 3) + 1];
      *float_buffer++ = vertices[(j * 3) + 2];
    }

    auto iptr = reinterpret_cast<uint32_t *>(float_buffer);
    *iptr = color;
    ++float_buffer;
  }
#ifndef NDEBUG
  size_t num_vertices = internals.rend->getBuffer()->getNumVertices();
  size_t vertex_size =
    internals.rend->getRenderOperation()->vertexData->vertexDeclaration->getVertexSize(0);
  assert(
    reinterpret_cast<uint8_t *>(float_buffer) <=
    reinterpret_cast<uint8_t *>(internals.float_buffer) + num_vertices * vertex_size);
#endif

  internals.float_buffer = float_buffer;
}

void PointCloud::popPoints(uint32_t num_points)
{
  assert(!gpu_data_);
  assert(num_points <= point_count_);

  points_.erase(points_.begin(), points_.begin() + num_points);
  point_count_ -= num_points;

  uint32_t vpp = getVerticesPerPoint();
  size_t popped_count = removePointsFromRenderables(num_points, vpp);
  (void) popped_count;

  assert(popped_count == num_points * vpp);

  resetBoundingBoxForCurrentPoints();

  if (getParentSceneNode()) {
    getParentSceneNode()->needUpdate();
  }
}

std::vector<PointCloud::Point> PointCloud::getPoints()
{
  if (gpu_data_) {
    return decodeGpuPoints();
  }
  return points_;
}

size_t PointCloud::removePointsFromRenderables(
  uint32_t number_of_points, uint32_t
  vertices_per_point)
{
  size_t popped_count = 0;
  while (popped_count < number_of_points * vertices_per_point) {
    PointCloudRenderablePtr rend = renderables_.front();
    Ogre::RenderOperation * op = rend->getRenderOperation();

    size_t popped_in_renderable = std::min(
      static_cast<size_t>(number_of_points * vertices_per_point - popped_count),
      op->vertexData->vertexCount);
    op->vertexData->vertexStart += popped_in_renderable;
    op->vertexData->vertexCount -= popped_in_renderable;

    popped_count += popped_in_renderable;

    if (op->vertexData->vertexCount == 0) {
      renderables_.pop_front();
    }
  }
  return popped_count;
}

void PointCloud::resetBoundingBoxForCurrentPoints()
{
  bounding_box_.setNull();
  for (uint32_t i = 0; i < point_count_; ++i) {
    Point & p = points_[i];
    bounding_box_.merge(p.position);
  }
}

void PointCloud::_notifyCurrentCamera(Ogre::Camera * camera)
{
  Ogre::MovableObject::_notifyCurrentCamera(camera);
  if (render_mode_ != RM_DEPTH_FADE_POINTS || !camera || !camera->getViewport()) {
    return;
  }
  const Ogre::Vector4 height(
    static_cast<float>(camera->getViewport()->getActualHeight()), 0.0f, 0.0f, 0.0f);
  for (auto & renderable : renderables_) {
    renderable->setCustomParameter(RVIZ_RENDERING_VIEWPORT_HEIGHT_PARAMETER, height);
  }
}

void PointCloud::_updateRenderQueue(Ogre::RenderQueue * queue)
{
  for (auto & renderable : renderables_) {
    queue->addRenderable(renderable.get());
  }
}

void PointCloud::_notifyAttached(Ogre::Node * parent, bool isTagPoint)
{
  Ogre::MovableObject::_notifyAttached(parent, isTagPoint);
}

void PointCloud::setPickColor(const Ogre::ColourValue & color)
{
  pick_color_ = color;
  Ogre::Vector4 pick_col(pick_color_.r, pick_color_.g, pick_color_.b, pick_color_.a);

  for (auto & renderable : renderables_) {
    renderable->setCustomParameter(RVIZ_RENDERING_PICK_COLOR_PARAMETER, pick_col);
  }
  // TODO(greimela): Add again after clearing up the module dependencies
//  getUserObjectBindings().setUserAny( "pick_handle", Ogre::Any(
// selection::colorToHandle( color )));
}

PointCloudRenderablePtr PointCloud::createRenderable(
  int num_points,
  Ogre::RenderOperation::OperationType operation_type)
{
  PointCloudRenderablePtr rend(new PointCloudRenderable(
      this, num_points, !current_mode_supports_geometry_shader_, operation_type));
  initializeRenderable(rend);
  return rend;
}

PointCloudRenderablePtr PointCloud::reuseSpareRenderable(
  uint32_t num_vertices, Ogre::RenderOperation::OperationType operation_type)
{
  for (auto it = spare_renderables_.begin(); it != spare_renderables_.end(); ++it) {
    if ((*it)->getBuffer()->getNumVertices() < num_vertices) {
      continue;
    }
    PointCloudRenderablePtr rend = *it;
    spare_renderables_.erase(it);
    Ogre::RenderOperation * op = rend->getRenderOperation();
    op->operationType = operation_type;
    op->vertexData->vertexStart = 0;
    op->vertexData->vertexCount = 0;
    rend->removeCustomParameter(RVIZ_RENDERING_AUTO_SIZE_PARAMETER);
    rend->setVisibilityFlags(Ogre::MovableObject::getDefaultVisibilityFlags());
    initializeRenderable(rend);
    return rend;
  }
  return nullptr;
}

void PointCloud::initializeRenderable(const PointCloudRenderablePtr & rend)
{
  rend->setMaterial(rend == gpu_renderable_ ? getGpuMaterial(render_mode_) : current_material_);
  Ogre::Vector4 alpha(alpha_, 0.0f, 0.0f, 0.0f);
  Ogre::Vector4 highlight(0.0f, 0.0f, 0.0f, 0.0f);
  Ogre::Vector4 pick_col(pick_color_.r, pick_color_.g, pick_color_.b, pick_color_.a);
  rend->setCustomParameter(RVIZ_RENDERING_SIZE_PARAMETER, Ogre::Vector4(point_extensions_));
  rend->setCustomParameter(RVIZ_RENDERING_ALPHA_PARAMETER, alpha);
  rend->setCustomParameter(RVIZ_RENDERING_HIGHLIGHT_PARAMETER, highlight);
  rend->setCustomParameter(RVIZ_RENDERING_PICK_COLOR_PARAMETER, pick_col);
  rend->setCustomParameter(RVIZ_RENDERING_NORMAL_PARAMETER, Ogre::Vector4(common_direction_));
  rend->setCustomParameter(RVIZ_RENDERING_UP_PARAMETER, Ogre::Vector4(common_up_vector_));
  rend->setCustomParameter(
    RVIZ_RENDERING_VIEWPORT_HEIGHT_PARAMETER, Ogre::Vector4(600.0f, 0.0f, 0.0f, 0.0f));
  if (getParentSceneNode() && !rend->isAttached()) {
    getParentSceneNode()->attachObject(rend.get());
  }
  renderables_.push_back(rend);
}

PointCloudRenderableQueue PointCloud::getRenderables()
{
  return renderables_;
}

void PointCloud::visitRenderables(Ogre::Renderable::Visitor * visitor, bool debugRenderables)
{
  (void) visitor;
  (void) debugRenderables;
}

bool PointCloud::supportsGpuPoints(RenderMode mode)
{
  const char * name = gpuMaterialName(mode);
  if (!name) {
    return false;
  }
  if (mode != RM_POINTS && mode != RM_DEPTH_FADE_POINTS) {
    Ogre::RenderSystem * render_system = Ogre::Root::getSingleton().getRenderSystem();
    if (!render_system || !render_system->getCapabilities() ||
      !render_system->getCapabilities()->hasCapability(Ogre::RSC_VERTEX_BUFFER_INSTANCE_DATA))
    {
      return false;
    }
  }
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(name);
  if (!material) {
    return false;
  }
  material->load();
  for (const char * technique_name : {"gpu_exact", "gpu"}) {
    Ogre::Technique * technique = material->getTechnique(technique_name);
    if (technique && technique->isSupported()) {
      return true;
    }
  }
  return false;
}

void PointCloud::createGpuMaterials()
{
  if (gpu_point_material_) {
    return;
  }
  static int count = 0;
  const std::string prefix = "PointCloudGpuMaterial" + std::to_string(count++);
  auto & manager = Ogre::MaterialManager::getSingleton();
  const std::pair<Ogre::MaterialPtr *, RenderMode> materials[] = {
    {&gpu_point_material_, RM_POINTS}, {&gpu_square_material_, RM_SQUARES},
    {&gpu_flat_square_material_, RM_FLAT_SQUARES}, {&gpu_sphere_material_, RM_SPHERES},
    {&gpu_depth_fade_point_material_, RM_DEPTH_FADE_POINTS}};
  for (const auto & entry : materials) {
    *entry.first = manager.getByName(gpuMaterialName(entry.second))->clone(
      prefix + std::to_string(entry.second));
    (*entry.first)->load();
  }
  setAlpha(alpha_, per_point_alpha_);
}

Ogre::HardwareVertexBufferSharedPtr PointCloud::gpuTemplate(RenderMode mode)
{
  const bool sphere = mode == RM_SPHERES;
  auto & buffer = sphere ? gpu_triangle_template_ : gpu_quad_template_;
  if (!buffer) {
    const float * vertices = sphere ? g_billboard_sphere_vertices : g_billboard_vertices;
    const size_t count = sphere ? 3 : 6;
    buffer = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
      3 * sizeof(float), count, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    buffer->writeData(0, count * 3 * sizeof(float), vertices, true);
  }
  return buffer;
}

void PointCloud::setGpuPoints(
  std::shared_ptr<const uint8_t> data, uint32_t num_points,
  const GpuLayout & layout, const GpuColor & color)
{
  setGpuPoints(std::move(data), num_points, layout, color, GpuSize());
}

void PointCloud::setGpuPoints(
  std::shared_ptr<const uint8_t> data, uint32_t num_points,
  const GpuLayout & layout, const GpuColor & color, const GpuSize & size)
{
  clear();
  V_Point().swap(points_);
  if (num_points == 0 || !data) {
    return;
  }
  createGpuMaterials();

  // One spare record: 2-byte attribute pairs of the last point may read past its end.
  const size_t capacity = static_cast<size_t>(num_points) + 1;
  if (!gpu_buffer_ || gpu_buffer_->getVertexSize() != layout.point_step ||
    gpu_buffer_->getNumVertices() < capacity || gpu_buffer_->getNumVertices() > 2 * capacity)
  {
    gpu_buffer_.reset();
    gpu_buffer_ = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
      layout.point_step, capacity, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
  }
  // On NVIDIA, map and copy takes about a tenth of the GUI thread time of glBufferData.
  const size_t bytes = static_cast<size_t>(num_points) * layout.point_step;
  std::memcpy(gpu_buffer_->lock(0, bytes, Ogre::HardwareBuffer::HBL_DISCARD), data.get(), bytes);
  gpu_buffer_->unlock();

  float min[3] = {
    std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
    std::numeric_limits<float>::max()};
  float max[3] = {-min[0], -min[1], -min[2]};
  bool any = false;
  const uint8_t * record = data.get();
  for (uint32_t i = 0; i < num_points; ++i, record += layout.point_step) {
    float p[3];
    std::memcpy(&p[0], record + layout.x_offset, sizeof(float));
    std::memcpy(&p[1], record + layout.y_offset, sizeof(float));
    std::memcpy(&p[2], record + layout.z_offset, sizeof(float));
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) {
      continue;
    }
    for (int k = 0; k < 3; ++k) {
      min[k] = std::min(min[k], p[k]);
      max[k] = std::max(max[k], p[k]);
    }
    any = true;
  }
  if (any) {
    bounding_box_.setExtents(min[0], min[1], min[2], max[0], max[1], max[2]);
  }

  gpu_data_ = std::move(data);
  gpu_layout_ = layout;
  gpu_color_ = color;
  gpu_size_ = size;
  point_count_ = num_points;
  rebuildGpuRenderable();
}

void PointCloud::rebuildGpuRenderable()
{
  if (!supportsGpuPoints(render_mode_)) {
    clear();
    return;
  }
  if (!gpu_renderable_) {
    gpu_renderable_ = std::make_shared<PointCloudRenderable>(this);
  }
  Ogre::RenderOperation * op = gpu_renderable_->getRenderOperation();
  Ogre::VertexDeclaration * declaration = op->vertexData->vertexDeclaration;
  Ogre::VertexBufferBinding * binding = op->vertexData->vertexBufferBinding;
  declaration->removeAllElements();
  binding->unsetAllBindings();

  const bool instanced = render_mode_ != RM_POINTS && render_mode_ != RM_DEPTH_FADE_POINTS;
  uint16_t source = 0;
  if (instanced) {
    Ogre::HardwareVertexBufferSharedPtr corners = gpuTemplate(render_mode_);
    declaration->addElement(0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    binding->setBinding(0, corners);
    source = 1;
    declaration->addElement(
      source, gpu_layout_.x_offset, Ogre::VET_FLOAT1, Ogre::VES_TEXTURE_COORDINATES, 1);
    op->operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
    op->vertexData->vertexCount = corners->getNumVertices();
    op->numberOfInstances = point_count_;
  } else {
    declaration->addElement(0, gpu_layout_.x_offset, Ogre::VET_FLOAT1, Ogre::VES_POSITION);
    op->operationType = Ogre::RenderOperation::OT_POINT_LIST;
    op->vertexData->vertexCount = point_count_;
    op->numberOfInstances = 1;
  }
  op->vertexData->vertexStart = 0;
  declaration->addElement(
    source, gpu_layout_.y_offset, Ogre::VET_FLOAT1, Ogre::VES_TEXTURE_COORDINATES, 2);
  declaration->addElement(
    source, gpu_layout_.z_offset, Ogre::VET_FLOAT1, Ogre::VES_TEXTURE_COORDINATES, 3);
  const bool reads_field = gpu_color_.source == GpuColor::FIELD ||
    gpu_color_.source == GpuColor::RGB || gpu_color_.source == GpuColor::RGBA;
  if (reads_field) {
    // Integer and color fields are read as two shorts and reassembled in the shader.
    const bool float_field =
      gpu_color_.source == GpuColor::FIELD && gpu_color_.field_type == GpuColor::FLOAT32;
    declaration->addElement(
      source, gpu_color_.field_offset, float_field ? Ogre::VET_FLOAT1 : Ogre::VET_SHORT2,
      Ogre::VES_TEXTURE_COORDINATES, 4);
  }
  if (gpu_size_.enabled) {
    declaration->addElement(
      source, gpu_size_.field_offset,
      gpu_size_.field_type == GpuColor::FLOAT32 ? Ogre::VET_FLOAT1 : Ogre::VET_SHORT2,
      Ogre::VES_TEXTURE_COORDINATES, 5);
  }
  gpu_buffer_->setIsInstanceData(instanced);
  if (instanced) {
    gpu_buffer_->setInstanceDataStepRate(1);
  }
  binding->setBinding(source, gpu_buffer_);
  gpu_renderable_->setBoundingBox(bounding_box_);

  if (std::find(renderables_.begin(), renderables_.end(), gpu_renderable_) == renderables_.end()) {
    initializeRenderable(gpu_renderable_);
  } else {
    gpu_renderable_->setMaterial(getGpuMaterial(render_mode_));
  }
  applyGpuColor();
  applyGpuSize();
  if (getParentSceneNode()) {
    getParentSceneNode()->needUpdate();
  }
}

void PointCloud::setGpuSize(const GpuSize & size)
{
  if (!gpu_data_) {
    return;
  }
  const bool same_attributes = size.enabled == gpu_size_.enabled &&
    size.field_type == gpu_size_.field_type && size.field_offset == gpu_size_.field_offset;
  gpu_size_ = size;
  if (same_attributes) {
    applyGpuSize();
  } else {
    rebuildGpuRenderable();
  }
}

void PointCloud::applyGpuSize()
{
  const GpuSize & s = gpu_size_;
  gpu_renderable_->setCustomParameter(
    RVIZ_RENDERING_SIZE_FIELD_PARAMETER,
    Ogre::Vector4(
      s.enabled ? 1.0f : 0.0f, static_cast<float>(s.field_type), s.min_scale, s.max_scale));
  gpu_renderable_->setCustomParameter(
    RVIZ_RENDERING_SIZE_RANGE_PARAMETER, Ogre::Vector4(s.min_value, s.value_range, 0.0f, 0.0f));
}

void PointCloud::setGpuColor(const GpuColor & color)
{
  if (!gpu_data_) {
    return;
  }
  const bool same_attributes = color.source == gpu_color_.source &&
    color.field_type == gpu_color_.field_type && color.field_offset == gpu_color_.field_offset;
  gpu_color_ = color;
  if (same_attributes) {
    applyGpuColor();
  } else {
    rebuildGpuRenderable();
  }
}

static float truncatedToByte(float value)
{
  return static_cast<float>(static_cast<uint8_t>(value * 255)) / 255.0f;
}

void PointCloud::applyGpuColor()
{
  const GpuColor & c = gpu_color_;
  Ogre::Vector4 color_a(c.min_color.r, c.min_color.g, c.min_color.b, c.min_color.a);
  if (c.source == GpuColor::FLAT) {
    // The CPU path stores colors as bytes (Ogre::ColourValue::getAsBYTE truncates).
    color_a = Ogre::Vector4(
      truncatedToByte(c.min_color.r), truncatedToByte(c.min_color.g),
      truncatedToByte(c.min_color.b), truncatedToByte(c.min_color.a));
  }
  gpu_renderable_->setCustomParameter(
    RVIZ_RENDERING_COLOR_MODE_PARAMETER,
    Ogre::Vector4(
      static_cast<float>(c.source), static_cast<float>(c.field_type), c.rainbow ? 1.0f : 0.0f,
      c.invert ? 1.0f : 0.0f));
  gpu_renderable_->setCustomParameter(
    RVIZ_RENDERING_COLOR_RANGE_PARAMETER, Ogre::Vector4(c.min_value, c.value_range, 0.0f, 0.0f));
  gpu_renderable_->setCustomParameter(RVIZ_RENDERING_COLOR_A_PARAMETER, color_a);
  gpu_renderable_->setCustomParameter(
    RVIZ_RENDERING_COLOR_B_PARAMETER,
    Ogre::Vector4(c.max_color.r, c.max_color.g, c.max_color.b, c.max_color.a));
  gpu_renderable_->setCustomParameter(RVIZ_RENDERING_COLOR_AXIS_PARAMETER, c.axis);
}

static void rainbowColor(float value, Ogre::ColourValue & color)
{
  value = std::max(std::min(value, 1.0f), 0.0f);
  float h = value * 5.0f + 1.0f;
  int i = static_cast<int>(std::floor(h));
  float f = h - static_cast<float>(i);
  if (!(i & 1)) {
    f = 1 - f;
  }
  float n = 1 - f;
  if (i <= 1) {
    color.r = n, color.g = 0, color.b = 1;
  } else if (i == 2) {
    color.r = 0, color.g = n, color.b = 1;
  } else if (i == 3) {
    color.r = 0, color.g = 1, color.b = n;
  } else if (i == 4) {
    color.r = n, color.g = 1, color.b = 0;
  } else {
    color.r = 1, color.g = n, color.b = 0;
  }
}

Ogre::ColourValue PointCloud::GpuColor::colorOf(
  const uint8_t * record, const Ogre::Vector3 & position) const
{
  Ogre::ColourValue color = Ogre::ColourValue::White;
  const uint8_t * field = record + field_offset;
  if (source == FLAT) {
    return min_color;
  }
  if (source == RGB || source == RGBA) {
    uint32_t rgb;
    std::memcpy(&rgb, field, sizeof(rgb));
    return Ogre::ColourValue(
      ((rgb >> 16) & 0xff) / 255.0f, ((rgb >> 8) & 0xff) / 255.0f, (rgb & 0xff) / 255.0f,
      source == RGBA ? (rgb >> 24) / 255.0f : 1.0f);
  }
  float value = 0.0f;
  if (source == AXIS) {
    value = axis.x * position.x + axis.y * position.y + axis.z * position.z + axis.w;
  } else if (field_type == FLOAT32) {
    std::memcpy(&value, field, sizeof(float));
  } else if (field_type == UINT8) {
    value = static_cast<float>(field[0]);
  } else if (field_type == UINT16) {
    uint16_t v;
    std::memcpy(&v, field, sizeof(v));
    value = static_cast<float>(v);
  } else {
    uint32_t v;
    std::memcpy(&v, field, sizeof(v));
    value = static_cast<float>(v);
  }
  const float t = (value - min_value) / value_range;
  if (rainbow) {
    const float value = 1.0f - t;
    rainbowColor(invert ? 1.0f - value : value, color);
  } else {
    const float n = std::min(1.0f, std::max(0.0f, t));
    color.r = max_color.r * n + min_color.r * (1.0f - n);
    color.g = max_color.g * n + min_color.g * (1.0f - n);
    color.b = max_color.b * n + min_color.b * (1.0f - n);
  }
  return color;
}

std::vector<PointCloud::Point> PointCloud::decodeGpuPoints() const
{
  std::vector<Point> points(point_count_);
  const uint8_t * record = gpu_data_.get();
  for (auto & point : points) {
    std::memcpy(&point.position.x, record + gpu_layout_.x_offset, sizeof(float));
    std::memcpy(&point.position.y, record + gpu_layout_.y_offset, sizeof(float));
    std::memcpy(&point.position.z, record + gpu_layout_.z_offset, sizeof(float));
    point.color = gpu_color_.colorOf(record, point.position);
    record += gpu_layout_.point_step;
  }
  return points;
}

}  // namespace rviz_rendering
