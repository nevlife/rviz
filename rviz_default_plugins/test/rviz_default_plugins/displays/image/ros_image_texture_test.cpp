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

#include <gmock/gmock.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Ogre.h>  // NOLINT

#include "sensor_msgs/image_encodings.hpp"

#include "../../ogre_testing_environment.hpp"

#include "rviz_default_plugins/displays/image/ros_image_texture.hpp"

using namespace rviz_default_plugins::displays;  // NOLINT
using namespace ::testing;  // NOLINT


class RosImageTextureTestFixture : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    testing_environment_ = std::make_shared<rviz_default_plugins::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();
  }

  static std::shared_ptr<rviz_default_plugins::OgreTestingEnvironment> testing_environment_;
};

std::shared_ptr<rviz_default_plugins::OgreTestingEnvironment>
RosImageTextureTestFixture::testing_environment_ = nullptr;

TEST_F(RosImageTextureTestFixture, constructor_initializes_texture_with_default_image) {
  ROSImageTexture texture;
  Ogre::TexturePtr ogreTexture = texture.getTexture();

  Ogre::Image textureImage;
  ogreTexture->convertToImage(textureImage);

  Ogre::Image expectedImage;
  expectedImage.load("no_image.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

#if OGRE_MIN_VERSION(13, 4, 3)
  ASSERT_THAT(
    std::vector<uint8_t>(textureImage.getData(), textureImage.getData() + textureImage.getSize()),
    testing::ElementsAreArray(expectedImage.getData(), expectedImage.getSize()));
#else
  // Can't compare the two images directly because the of a bug that was introduced in Ogre 1.12.10
  // and only fixed in Ogre 13.4.3
  // See https://github.com/OGRECave/ogre/pull/2519
  ASSERT_EQ(textureImage.getWidth(), expectedImage.getWidth());
  ASSERT_EQ(textureImage.getHeight(), expectedImage.getHeight());
#endif  // OGRE_MIN_VERSION(13, 4, 3)
}

TEST_F(RosImageTextureTestFixture, update_writes_new_image_to_the_texture) {
  Ogre::Image testImage;
  testImage.load("test_20x20.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  sensor_msgs::msg::Image::SharedPtr msg = std::make_shared<sensor_msgs::msg::Image>();
  msg->width = testImage.getWidth();
  msg->height = testImage.getHeight();
  msg->encoding = sensor_msgs::image_encodings::RGB8;
  msg->data = std::vector<uint8_t>(
    testImage.getData(), testImage.getData() + testImage.getSize());

  ROSImageTexture texture;
  texture.addMessage(msg);
  texture.update();

  Ogre::TexturePtr ogreTexture = texture.getTexture();
  Ogre::Image textureImage;
  ogreTexture->convertToImage(textureImage);

#if OGRE_MIN_VERSION(13, 4, 3)
  ASSERT_THAT(
    std::vector<uint8_t>(textureImage.getData(), textureImage.getData() + textureImage.getSize()),
    testing::ElementsAreArray(testImage.getData(), testImage.getSize()));
#else
  // Can't compare the two images directly because the of a bug that was introduced in Ogre 1.12.10
  // and only fixed in Ogre 13.4.3
  // See https://github.com/OGRECave/ogre/pull/2519
  ASSERT_EQ(textureImage.getWidth(), testImage.getWidth());
  ASSERT_EQ(textureImage.getHeight(), testImage.getHeight());
#endif  // OGRE_MIN_VERSION(13, 4, 3)
}

namespace
{

sensor_msgs::msg::Image::SharedPtr makeImage(
  uint32_t width, uint32_t height, const std::string & encoding, const void * data, size_t bytes)
{
  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  msg->width = width;
  msg->height = height;
  msg->encoding = encoding;
  msg->step = static_cast<uint32_t>(bytes / height);
  msg->data.assign(
    static_cast<const uint8_t *>(data), static_cast<const uint8_t *>(data) + bytes);
  return msg;
}

// RGBA bytes of the texture after update().
std::vector<uint8_t> updatedPixels(ROSImageTexture & texture, uint32_t width, uint32_t height)
{
  texture.update();
  std::vector<uint8_t> pixels(static_cast<size_t>(width) * height * 4);
  texture.getTexture()->getBuffer()->blitToMemory(
    Ogre::PixelBox(width, height, 1, Ogre::PF_BYTE_RGBA, pixels.data()));
  return pixels;
}

std::vector<uint8_t> channel(const std::vector<uint8_t> & rgba, size_t c)
{
  std::vector<uint8_t> out;
  for (size_t i = c; i < rgba.size(); i += 4) {
    out.push_back(rgba[i]);
  }
  return out;
}

}  // namespace

TEST_F(RosImageTextureTestFixture, mono16_images_are_normalized_like_the_cpu_conversion) {
  std::vector<uint16_t> values;
  for (uint32_t i = 0; i < 64 * 4; ++i) {
    values.push_back(static_cast<uint16_t>(1000 + i * 37));
  }
  ROSImageTexture texture;
  texture.setMedianFrames(1);
  texture.addMessage(
    makeImage(64, 4, sensor_msgs::image_encodings::MONO16, values.data(), values.size() * 2));

  auto pixels = updatedPixels(texture, 64, 4);

  std::vector<uint8_t> expected;
  const double range = (1000.0 + 255 * 37) - 1000.0;
  for (uint16_t v : values) {
    expected.push_back(
      static_cast<uint8_t>(std::clamp((v - 1000.0) / range, 0.0, 1.0) * 255u));
  }
  EXPECT_THAT(channel(pixels, 0), ElementsAreArray(expected));
  EXPECT_THAT(channel(pixels, 1), ElementsAreArray(expected));
  EXPECT_THAT(channel(pixels, 3), Each(Eq(255)));
}

TEST_F(RosImageTextureTestFixture, float_images_use_the_given_range) {
  std::vector<float> values;
  for (uint32_t i = 0; i < 32 * 8; ++i) {
    values.push_back(static_cast<float>(i) * 0.37f - 20.0f);
  }
  values[5] = std::nanf("");
  ROSImageTexture texture;
  texture.setNormalizeFloatImage(false, -10.0, 50.0);
  texture.addMessage(
    makeImage(32, 8, sensor_msgs::image_encodings::TYPE_32FC1, values.data(), values.size() * 4));

  auto pixels = updatedPixels(texture, 32, 8);

  std::vector<uint8_t> expected;
  for (float v : values) {
    const double t = (static_cast<double>(v) + 10.0) / 60.0;
    expected.push_back(t > 0.0 ? static_cast<uint8_t>(std::min(t, 1.0) * 255u) : 0);
  }
  EXPECT_THAT(channel(pixels, 0), ElementsAreArray(expected));
}

TEST_F(RosImageTextureTestFixture, bayer_rggb8_is_demosaiced_bilinearly) {
  // R G R G
  // G B G B
  // R G R G
  const uint8_t mosaic[12] = {
    200, 100, 220, 120,
    90, 10, 110, 30,
    180, 80, 240, 140};
  ROSImageTexture texture;
  texture.addMessage(
    makeImage(4, 3, sensor_msgs::image_encodings::BAYER_RGGB8, mosaic, sizeof(mosaic)));

  auto pixels = updatedPixels(texture, 4, 3);

  auto rgb = [&](int x, int y) {
      const size_t i = (static_cast<size_t>(y) * 4 + x) * 4;
      return std::vector<int>{pixels[i], pixels[i + 1], pixels[i + 2]};
    };
  // Interior B at (1, 1): R from 4 diagonals, G from 4 neighbours.
  EXPECT_THAT(rgb(1, 1),
    ElementsAre((200 + 220 + 180 + 240 + 2) / 4, (100 + 80 + 90 + 110 + 2) / 4, 10));
  // Interior G at (2, 1) on a B row: R from above and below, B from left and right.
  EXPECT_THAT(rgb(2, 1), ElementsAre((220 + 240 + 1) / 2, 110, (10 + 30 + 1) / 2));
  // Corner R at (0, 0): G from the 2 valid neighbours, B from the 1 valid diagonal.
  EXPECT_THAT(rgb(0, 0), ElementsAre(200, (100 + 90 + 1) / 2, 10));
}

namespace
{

// Bilinear demosaic of rviz PR #1749, written out per pixel, for the GPU result.
std::vector<int> referenceDemosaic(
  const std::vector<uint16_t> & mosaic, int width, int height, int red_col, int red_row)
{
  auto at = [&](int x, int y) {return static_cast<int>(mosaic[y * width + x]);};
  auto average = [&](int x, int y, std::vector<std::pair<int, int>> offsets) {
      int sum = 0;
      int count = 0;
      for (auto [dx, dy] : offsets) {
        if (x + dx >= 0 && x + dx < width && y + dy >= 0 && y + dy < height) {
          sum += at(x + dx, y + dy);
          ++count;
        }
      }
      return count ? (sum + count / 2) / count : 0;
    };
  std::vector<int> rgb;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int cardinal = average(x, y, {{0, -1}, {0, 1}, {-1, 0}, {1, 0}});
      const int diagonal = average(x, y, {{-1, -1}, {1, -1}, {-1, 1}, {1, 1}});
      const int horizontal = average(x, y, {{-1, 0}, {1, 0}});
      const int vertical = average(x, y, {{0, -1}, {0, 1}});
      const int xp = x & 1;
      const int yp = y & 1;
      if (xp == red_col && yp == red_row) {
        rgb.insert(rgb.end(), {at(x, y), cardinal, diagonal});
      } else if (xp != red_col && yp != red_row) {
        rgb.insert(rgb.end(), {diagonal, cardinal, at(x, y)});
      } else if (yp == red_row) {
        rgb.insert(rgb.end(), {horizontal, at(x, y), vertical});
      } else {
        rgb.insert(rgb.end(), {vertical, at(x, y), horizontal});
      }
    }
  }
  return rgb;
}

}  // namespace

TEST_F(RosImageTextureTestFixture, bayer16_matches_the_cpu_demosaic_and_quantization) {
  const int width = 37;
  const int height = 23;
  std::vector<uint16_t> mosaic;
  uint32_t state = 12345;
  for (int i = 0; i < width * height; ++i) {
    state = state * 1664525u + 1013904223u;
    mosaic.push_back(static_cast<uint16_t>(2000 + (state >> 16) % 50000));
  }
  ROSImageTexture texture;
  texture.setMedianFrames(1);
  texture.setLinearInput(true);
  texture.addMessage(
    makeImage(
      width, height, sensor_msgs::image_encodings::BAYER_GBRG16, mosaic.data(),
      mosaic.size() * 2));

  auto pixels = updatedPixels(texture, width, height);

  const auto [lo, hi] = std::minmax_element(mosaic.begin(), mosaic.end());
  const double min_value = *lo;
  const double range = static_cast<double>(*hi) - min_value;
  auto quantize = [&](int v) {
      const double scaled = std::clamp((v - min_value) * (65535.0 / range), 0.0, 65535.0);
      const double linear = static_cast<double>(std::lround(scaled)) / 65535.0;
      const double srgb = linear <= 0.0031308 ? 12.92 * linear :
        1.055 * std::pow(linear, 1.0 / 2.4) - 0.055;
      return static_cast<int>(std::lround(std::clamp(srgb, 0.0, 1.0) * 255.0));
    };
  std::vector<int> expected;
  for (int v : referenceDemosaic(mosaic, width, height, 0, 1)) {
    expected.push_back(quantize(v));
  }
  std::vector<int> actual;
  for (size_t i = 0; i < pixels.size(); i += 4) {
    actual.insert(actual.end(), {pixels[i], pixels[i + 1], pixels[i + 2]});
  }
  EXPECT_THAT(actual, ElementsAreArray(expected));
}

TEST_F(RosImageTextureTestFixture, normalized_float_images_match_the_cpu_conversion) {
  const uint32_t width = 512;
  const uint32_t height = 256;
  std::vector<float> values;
  uint32_t state = 777;
  for (uint32_t i = 0; i < width * height; ++i) {
    state = state * 1664525u + 1013904223u;
    values.push_back(static_cast<float>(state >> 8) * 1.3e-4f - 700.0f);
  }
  ROSImageTexture texture;
  texture.setMedianFrames(1);
  texture.addMessage(
    makeImage(
      width, height, sensor_msgs::image_encodings::TYPE_32FC1, values.data(),
      values.size() * 4));

  auto pixels = updatedPixels(texture, width, height);

  const auto [lo, hi] = std::minmax_element(values.begin(), values.end());
  const double min_value = *lo;
  const double range = static_cast<double>(*hi) - min_value;
  std::vector<uint8_t> expected;
  for (float v : values) {
    expected.push_back(
      static_cast<uint8_t>(std::clamp((v - min_value) / range, 0.0, 1.0) * 255u));
  }
  EXPECT_THAT(channel(pixels, 0), ElementsAreArray(expected));
}
