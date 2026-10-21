/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
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

#include "rviz_default_plugins/displays/map/swatch.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include <OgreHardwarePixelBuffer.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRenderable.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#include "rviz_rendering/custom_parameter_indices.hpp"

namespace rviz_default_plugins
{
namespace displays
{

// Helper class to set alpha parameter on all renderables.
class AlphaSetter : public Ogre::Renderable::Visitor
{
public:
  explicit AlphaSetter(float alpha)
  : alpha_vec_(alpha, alpha, alpha, alpha)
  {}

  void
  visit(Ogre::Renderable * rend, Ogre::ushort lodIndex, bool isDebug, Ogre::Any * pAny) override
  {
    (void) lodIndex;
    (void) isDebug;
    (void) pAny;

    rend->setCustomParameter(RVIZ_RENDERING_ALPHA_PARAMETER, alpha_vec_);
  }

private:
  Ogre::Vector4 alpha_vec_;
};

size_t Swatch::material_count_ = 0;
size_t Swatch::map_count_ = 0;
size_t Swatch::node_count_ = 0;
size_t Swatch::texture_count_ = 0;

Swatch::Swatch(
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode * parent_scene_node,
  size_t x, size_t y, size_t width, size_t height,
  float resolution, bool draw_under)
: scene_manager_(scene_manager),
  parent_scene_node_(parent_scene_node),
  scene_node_(nullptr),
  manual_object_(nullptr),
  x_(x), y_(y), width_(width), height_(height)
{
  setupMaterial();
  setupSceneNodeWithManualObject();

  scene_node_->setPosition(x * resolution, y * resolution, 0);
  scene_node_->setScale(width * resolution, height * resolution, 1.0);

  if (draw_under) {
    manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
  }

  // don't show map until the plugin is actually enabled
  manual_object_->setVisible(false);
}

Swatch::~Swatch()
{
  scene_manager_->destroyManualObject(manual_object_);
  scene_manager_->destroySceneNode(scene_node_);
  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_);
  }
  Ogre::MaterialManager::getSingleton().remove(material_);
}

void Swatch::updateAlpha(
  const Ogre::SceneBlendType & sceneBlending, bool depth_write, float alpha)
{
  material_->setSceneBlending(sceneBlending);
  material_->setDepthWriteEnabled(depth_write);
  if (manual_object_) {
    AlphaSetter alpha_setter(alpha);
    manual_object_->visitRenderables(&alpha_setter);
  }
}

void Swatch::updateData(const nav_msgs::msg::OccupancyGrid & map)
{
  updateData(map, x_, y_, width_, height_);
}

void Swatch::updateData(
  const nav_msgs::msg::OccupancyGrid & map, size_t x, size_t y, size_t width, size_t height)
{
  if (!texture_) {
    texture_ = Ogre::TextureManager::getSingleton().createManual(
      "MapTexture" + std::to_string(texture_count_++), "rviz_rendering", Ogre::TEX_TYPE_2D,
      static_cast<Ogre::uint>(width_), static_cast<Ogre::uint>(height_), 0, Ogre::PF_L8,
      Ogre::TU_STATIC_WRITE_ONLY);
    x = x_;
    y = y_;
    width = width_;
    height = height_;
  }
  const size_t left = std::max(x, x_);
  const size_t top = std::max(y, y_);
  const size_t right = std::min(x + width, x_ + width_);
  const size_t bottom = std::min(y + height, y_ + height_);
  if (left >= right || top >= bottom) {
    return;
  }

  const size_t map_width = map.info.width;
  auto cells = reinterpret_cast<uint8_t *>(const_cast<int8_t *>(map.data.data()));
  Ogre::PixelBox source(
    Ogre::Box(
      static_cast<uint32_t>(left), static_cast<uint32_t>(top),
      static_cast<uint32_t>(right), static_cast<uint32_t>(bottom)),
    Ogre::PF_L8, cells);
  source.rowPitch = map_width;
  source.slicePitch = map_width * map.info.height;

  // The GL upload keeps 4-byte row alignment when the copied width is a multiple of 4,
  // which only matches the map rows when they are a multiple of 4 too.
  std::vector<uint8_t> packed;
  if ((right - left) % 4 == 0 && map_width % 4 != 0) {
    const size_t row_bytes = right - left;
    packed.resize(row_bytes * (bottom - top));
    for (size_t row = top; row < bottom; ++row) {
      std::copy_n(cells + row * map_width + left, row_bytes, &packed[(row - top) * row_bytes]);
    }
    source = Ogre::PixelBox(
      static_cast<uint32_t>(right - left), static_cast<uint32_t>(bottom - top), 1, Ogre::PF_L8,
      packed.data());
  }

  texture_->getBuffer()->blitFromMemory(
    source, Ogre::Box(
      static_cast<uint32_t>(left - x_), static_cast<uint32_t>(top - y_),
      static_cast<uint32_t>(right - x_), static_cast<uint32_t>(bottom - y_)));
}

void Swatch::setVisible(bool visible)
{
  if (manual_object_) {
    manual_object_->setVisible(visible);
  }
}

void Swatch::setRenderQueueGroup(uint8_t group)
{
  if (manual_object_) {
    manual_object_->setRenderQueueGroup(group);
  }
}

void Swatch::setDepthWriteEnabled(bool depth_write_enabled)
{
  if (material_) {
    material_->setDepthWriteEnabled(depth_write_enabled);
  }
}

Ogre::Pass * Swatch::getTechniquePass()
{
  if (material_) {
    return material_->getTechnique(0)->getPass(0);
  }
  return nullptr;
}

std::string Swatch::getTextureName()
{
  if (texture_) {
    return texture_->getName();
  }
  return "";
}

void Swatch::setupMaterial()
{
  material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/Indexed8BitImage");
  material_ = material_->clone("MapMaterial" + std::to_string(material_count_++));

  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthBias(-16.0f, 0.0f);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->setDepthWriteEnabled(false);
}

void Swatch::setupSceneNodeWithManualObject()
{
  manual_object_ = scene_manager_->createManualObject("MapObject" + std::to_string(map_count_++));

  scene_node_ = parent_scene_node_->createChildSceneNode(
    "NodeObject" + std::to_string(node_count_++));
  scene_node_->attachObject(manual_object_);

  setupSquareManualObject();
}

void Swatch::setupSquareManualObject()
{
  manual_object_->begin(
    material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");

  // first triangle
  addPointWithPlaneCoordinates(0.0f, 0.0f);
  addPointWithPlaneCoordinates(1.0f, 1.0f);
  addPointWithPlaneCoordinates(0.0f, 1.0f);

  // second triangle
  addPointWithPlaneCoordinates(0.0f, 0.0f);
  addPointWithPlaneCoordinates(1.0f, 0.0f);
  addPointWithPlaneCoordinates(1.0f, 1.0f);

  manual_object_->end();
}

void Swatch::addPointWithPlaneCoordinates(float x, float y)
{
  manual_object_->position(x, y, 0.0f);
  manual_object_->textureCoord(x, y);
  manual_object_->normal(0.0f, 0.0f, 1.0f);
}

}  // namespace displays
}  // namespace rviz_default_plugins
