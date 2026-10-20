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

#include <memory>
#include <string>
#include <vector>

#include <OgreHardwarePixelBuffer.h>
#include <OgreTextureManager.h>

#include "rviz_default_plugins/displays/map/swatch.hpp"
#include "../display_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using rviz_default_plugins::displays::Swatch;

class SwatchTestFixture : public DisplayTestFixture
{
public:
  nav_msgs::msg::OccupancyGrid map(uint32_t width, uint32_t height, int seed)
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.resolution = 1;
    for (uint32_t i = 0; i < width * height; ++i) {
      grid.data.push_back(static_cast<int8_t>((i * 7 + seed) % 101));
    }
    return grid;
  }

  std::vector<uint8_t> texels(Swatch & swatch, size_t width, size_t height)
  {
    std::vector<uint8_t> out(width * height);
    auto texture = Ogre::TextureManager::getSingleton().getByName(
      swatch.getTextureName(), "rviz_rendering");
    texture->getBuffer()->blitToMemory(
      Ogre::PixelBox(
        static_cast<uint32_t>(width), static_cast<uint32_t>(height), 1, Ogre::PF_L8, out.data()));
    return out;
  }

  std::vector<uint8_t> cells(
    const nav_msgs::msg::OccupancyGrid & grid, size_t x, size_t y, size_t width, size_t height)
  {
    std::vector<uint8_t> out;
    for (size_t row = y; row < y + height; ++row) {
      for (size_t col = x; col < x + width; ++col) {
        out.push_back(static_cast<uint8_t>(grid.data[row * grid.info.width + col]));
      }
    }
    return out;
  }

  void expectPartialUpdate(uint32_t map_width, size_t swatch_width)
  {
    const size_t x = 3, y = 2, height = 9;
    Swatch swatch(
      scene_manager_, scene_manager_->getRootSceneNode(), x, y, swatch_width, height, 1, false);
    auto first = map(map_width, 16, 0);
    swatch.updateData(first);
    const std::string name = swatch.getTextureName();
    EXPECT_THAT(texels(swatch, swatch_width, height), Eq(cells(first, x, y, swatch_width, height)));

    auto second = map(map_width, 16, 50);
    swatch.updateData(second, 5, 4, 6, 3);

    auto expected = cells(first, x, y, swatch_width, height);
    for (size_t row = 4; row < 7; ++row) {
      for (size_t col = 5; col < 11; ++col) {
        expected[(row - y) * swatch_width + col - x] =
          static_cast<uint8_t>(second.data[row * map_width + col]);
      }
    }
    EXPECT_THAT(swatch.getTextureName(), StrEq(name));
    EXPECT_THAT(texels(swatch, swatch_width, height), Eq(expected));

    swatch.updateData(second);
    EXPECT_THAT(swatch.getTextureName(), StrEq(name));
    EXPECT_THAT(
      texels(swatch, swatch_width, height), Eq(cells(second, x, y, swatch_width, height)));
  }
};

TEST_F(SwatchTestFixture, updateData_keeps_the_texture_and_uploads_only_the_given_cells) {
  expectPartialUpdate(20, 12);
}

TEST_F(SwatchTestFixture, updateData_handles_map_rows_that_are_not_a_multiple_of_four_bytes) {
  expectPartialUpdate(21, 12);
  expectPartialUpdate(21, 11);
}

TEST_F(SwatchTestFixture, updateData_ignores_cells_outside_the_swatch) {
  Swatch swatch(scene_manager_, scene_manager_->getRootSceneNode(), 0, 0, 8, 8, 1, false);
  auto first = map(16, 16, 0);
  swatch.updateData(first);

  swatch.updateData(map(16, 16, 9), 8, 8, 8, 8);

  EXPECT_THAT(texels(swatch, 8, 8), Eq(cells(first, 0, 0, 8, 8)));
}
