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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__GPU_IMAGE_CONVERTER_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__GPU_IMAGE_CONVERTER_HPP_

#include <cstdint>

#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace Ogre
{
class Camera;
class Rectangle2D;
class SceneManager;
}

namespace rviz_default_plugins
{
namespace displays
{

// Converts a raw single-channel image into 8-bit RGB with a fragment program
// (image_convert.frag), rendering into the target texture.
class GpuImageConverter
{
public:
  enum Mode {GRAY16 = 0, FLOAT32 = 1, BAYER8 = 2, BAYER16 = 3};

  struct Params
  {
    Mode mode = GRAY16;
    /// Image column and row of the red sample in each 2x2 Bayer cell.
    int red_col = 0;
    int red_row = 0;
    /// 65536 entries (GRAY16, BAYER16) or 256 (BAYER8), nullptr for none.
    const uint8_t * lut = nullptr;
    /// FLOAT32: value drawn as 0 and max - min; range <= 0 draws black.
    double min = 0.0;
    double range = 1.0;
  };

  static bool supported();

  GpuImageConverter();
  ~GpuImageConverter();

  /// Draws the packed width x height image into target, a render target of the same size.
  void convert(
    const uint8_t * raw, uint32_t width, uint32_t height, const Params & params,
    const Ogre::TexturePtr & target);

private:
  void resize(
    Ogre::TexturePtr & texture, uint32_t width, uint32_t height,
    Ogre::PixelFormat format, Ogre::TextureUsage usage);

  Ogre::SceneManager * scene_manager_;
  Ogre::Camera * camera_;
  Ogre::Rectangle2D * rectangle_;
  Ogre::MaterialPtr material_;
  Ogre::TexturePtr raw_;
  Ogre::TexturePtr lut_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__GPU_IMAGE_CONVERTER_HPP_
