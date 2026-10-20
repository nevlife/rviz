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
 *     * Neither the name of the copyright holder nor the names of its
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

#include "gpu_image_converter.hpp"

#include <string>

#include <OgreCamera.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderTexture.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>

#include "rviz_common/logging.hpp"

namespace rviz_default_plugins
{
namespace displays
{

bool GpuImageConverter::supported()
{
  static const bool result = []() {
      Ogre::MaterialPtr material =
        Ogre::MaterialManager::getSingleton().getByName("rviz/ImageConvert", "rviz_rendering");
      if (!material) {
        return false;
      }
      material->load();
      Ogre::Technique * technique = material->getBestTechnique();
      if (technique) {
        RVIZ_COMMON_LOG_DEBUG_STREAM("Image conversion technique: " << technique->getName());
      }
      return technique != nullptr;
    }();
  return result;
}

GpuImageConverter::GpuImageConverter()
{
  static int count = 0;
  const std::string name = "GpuImageConverter" + std::to_string(count++);
  scene_manager_ = Ogre::Root::getSingleton().createSceneManager();
  camera_ = scene_manager_->createCamera(name);
  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(camera_);
  material_ = Ogre::MaterialManager::getSingleton().getByName(
    "rviz/ImageConvert", "rviz_rendering")->clone(name);
  material_->load();
  rectangle_ = new Ogre::Rectangle2D(true);
  rectangle_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);
  rectangle_->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
  rectangle_->setMaterial(material_);
  scene_manager_->getRootSceneNode()->attachObject(rectangle_);
}

GpuImageConverter::~GpuImageConverter()
{
  delete rectangle_;
  Ogre::Root::getSingleton().destroySceneManager(scene_manager_);
  Ogre::MaterialManager::getSingleton().remove(material_);
  for (auto texture : {&raw_, &lut_}) {
    if (*texture) {
      Ogre::TextureManager::getSingleton().remove(*texture);
    }
  }
}

void GpuImageConverter::resize(
  Ogre::TexturePtr & texture, uint32_t width, uint32_t height, Ogre::PixelFormat format,
  Ogre::TextureUsage usage)
{
  if (texture && texture->getWidth() == width && texture->getHeight() == height &&
    texture->getFormat() == format)
  {
    return;
  }
  if (texture) {
    Ogre::TextureManager::getSingleton().remove(texture);
  }
  static int count = 0;
  texture = Ogre::TextureManager::getSingleton().createManual(
    "GpuImageConverterTexture" + std::to_string(count++), "rviz_rendering", Ogre::TEX_TYPE_2D,
    width, height, 0, format, usage);
}

void GpuImageConverter::convert(
  const uint8_t * raw, uint32_t width, uint32_t height, const Params & params,
  const Ogre::TexturePtr & target)
{
  const Ogre::PixelFormat raw_format =
    params.mode == FLOAT32 ? Ogre::PF_FLOAT32_R :
    params.mode == BAYER8 ? Ogre::PF_L8 : Ogre::PF_L16;
  resize(raw_, width, height, raw_format, Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
  raw_->getBuffer()->blitFromMemory(
    Ogre::PixelBox(width, height, 1, raw_format, const_cast<uint8_t *>(raw)));

  resize(lut_, 256, 256, Ogre::PF_L8, Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
  if (params.lut) {
    const uint32_t rows = params.mode == BAYER8 ? 1 : 256;
    lut_->getBuffer()->blitFromMemory(
      Ogre::PixelBox(256, rows, 1, Ogre::PF_L8, const_cast<uint8_t *>(params.lut)),
      Ogre::Box(0, 0, 256, rows));
  }

  Ogre::RenderTexture * render_target = target->getBuffer()->getRenderTarget();
  if (render_target->getNumViewports() == 0) {
    render_target->setAutoUpdated(false);
    Ogre::Viewport * viewport = render_target->addViewport(camera_);
    viewport->setClearEveryFrame(false);
    viewport->setOverlaysEnabled(false);
  }

  Ogre::Pass * pass = material_->getBestTechnique()->getPass(0);
  pass->getTextureUnitState("raw")->setTexture(raw_);
  pass->getTextureUnitState("lut")->setTexture(lut_);
  // Floats split into a high and a low part keep the double min and range for the fp64 program.
  const float min_high = static_cast<float>(params.min);
  const float range_high = static_cast<float>(params.range);
  auto fragment = pass->getFragmentProgramParameters();
  fragment->setNamedConstant(
    "params", Ogre::Vector4(
      static_cast<float>(params.mode), static_cast<float>(params.red_col),
      static_cast<float>(params.red_row), params.lut ? 1.0f : 0.0f));
  fragment->setNamedConstant(
    "range", Ogre::Vector4(
      min_high, static_cast<float>(params.min - min_high),
      range_high, static_cast<float>(params.range - range_high)));
  render_target->update();
}

}  // namespace displays
}  // namespace rviz_default_plugins
