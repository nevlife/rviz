/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "rviz_default_plugins/displays/image/ros_image_texture.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <utility>

#include <cstring>

#include <OgreHardwarePixelBuffer.h>  // NOLINT: cpplint cannot handle include order
#include <OgrePixelFormat.h>  // NOLINT: cpplint cannot handle include order
#include <OgreTextureManager.h>  // NOLINT: cpplint cannot handle include order

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "rviz_common/logging.hpp"
#include "rviz_common/uniform_string_stream.hpp"

#include "gpu_image_converter.hpp"

namespace rviz_default_plugins
{
namespace displays
{

ROSImageTexture::ROSImageTexture()
: new_image_(false),
  texture_width_(0),
  texture_height_(0),
  texture_format_(Ogre::PF_UNKNOWN),
  width_(0),
  height_(0),
  median_frames_(5),
  linear_input_(false)
{
  empty_image_.load("no_image.png", "rviz_rendering");

  static uint32_t count = 0;
  rviz_common::UniformStringStream ss;
  ss << "ROSImageTexture" << count++;
  texture_ = Ogre::TextureManager::getSingleton().loadImage(
    ss.str(), "rviz_rendering", empty_image_,
    Ogre::TEX_TYPE_2D,
    0);

  setNormalizeFloatImage(true);
}

ROSImageTexture::~ROSImageTexture()
{
  current_image_.reset();
}

void ROSImageTexture::setLinearInput(bool linear_input)
{
  linear_input_ = linear_input;
}

void ROSImageTexture::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);

  texture_->unload();
  texture_->setUsage(Ogre::TU_DEFAULT);
  texture_->loadImage(empty_image_);
  texture_format_ = Ogre::PF_UNKNOWN;

  new_image_ = false;
  current_image_.reset();

  // Drop median history so reset/resubscribe doesn't carry stale min/max
  // into the first frame of the new stream.
  min_buffer_.clear();
  max_buffer_.clear();
}

const Ogre::String ROSImageTexture::getName() const
{
  return texture_->getName();
}

const Ogre::TexturePtr & ROSImageTexture::getTexture()
{
  return texture_;
}

const sensor_msgs::msg::Image::ConstSharedPtr ROSImageTexture::getImage()
{
  std::lock_guard<std::mutex> lock(mutex_);

  return current_image_;
}

uint32_t ROSImageTexture::getWidth() const
{
  return width_;
}

uint32_t ROSImageTexture::getHeight() const
{
  return height_;
}

void ROSImageTexture::setMedianFrames(unsigned median_frames)
{
  median_frames_ = median_frames;
}

void ROSImageTexture::setNormalizeFloatImage(bool normalize)
{
  setNormalizeFloatImage(normalize, 0.0, 1.0);
}

void ROSImageTexture::setNormalizeFloatImage(bool normalize, double min, double max)
{
  normalize_ = normalize;
  min_ = min;
  max_ = max;
}

// Bytes per pixel for encodings that have a fixed linear row layout. Returns
// 0 for encodings whose per-row layout is non-trivial (YUV 4:2:2 packed, NV12
// planar) — those are handled explicitly by their converters using stride.
static size_t bytesPerPixelForEncoding(const std::string & encoding)
{
  namespace enc = sensor_msgs::image_encodings;
  if (encoding == enc::YUV422 || encoding == enc::YUV422_YUY2 ||
    encoding == enc::UYVY || encoding == enc::YUYV ||
    encoding == enc::NV21 || encoding == enc::NV24)
  {
    return 0;
  }
  try {
    return static_cast<size_t>(enc::numChannels(encoding)) *
           enc::bitDepth(encoding) / 8;
  } catch (const std::runtime_error &) {
    return 0;  // unknown encoding — skip repack
  }
}

static double
computeMedianOfSeveralFrames(std::deque<double> & buffer, double value, unsigned median_frames)
{
  while (buffer.size() > median_frames - 1) {
    buffer.pop_back();
  }
  buffer.push_front(value);

  // Compute the median
  std::deque<double> buffer2 = buffer;
  nth_element(buffer2.begin(), buffer2.begin() + buffer2.size() / 2, buffer2.end());
  return *(buffer2.begin() + buffer2.size() / 2);
}

bool ROSImageTexture::update()
{
  std::lock_guard<std::mutex> lock(mutex_);

  sensor_msgs::msg::Image::ConstSharedPtr image = current_image_;

  if (!image || !new_image_) {
    return false;
  }

  new_image_ = false;

  if (image->data.empty()) {
    return false;
  }

  width_ = image->width;
  height_ = image->height;
  stride_ = image->step;

  // If the publisher sent rows with trailing padding (step > width * bpp),
  // repack into a contiguous buffer. Ogre::Image::loadRawData and the
  // convertTo8bit rescale loop both assume packed rows — without this,
  // each row drifts right by `padding` bytes, producing a diagonal shear.
  // YUV/NV12 encodings are left alone because their converters honor stride
  // directly.
  std::vector<uint8_t> repacked;
  const uint8_t * data_ptr = image->data.data();
  size_t data_size = image->data.size();
  const size_t bpp = bytesPerPixelForEncoding(image->encoding);
  if (bpp != 0) {
    const size_t packed_row = static_cast<size_t>(width_) * bpp;
    if (stride_ > packed_row) {
      if (image->data.size() < static_cast<size_t>(stride_) * height_) {
        RVIZ_COMMON_LOG_ERROR_STREAM(
          "Image data size " << image->data.size() <<
            " smaller than step*height (" << stride_ * height_ << ")");
        return false;
      }
      repacked.resize(packed_row * height_);
      for (uint32_t y = 0; y < height_; ++y) {
        std::memcpy(
          repacked.data() + y * packed_row,
          image->data.data() + y * stride_,
          packed_row);
      }
      data_ptr = repacked.data();
      data_size = repacked.size();
      // Reflect the repack so any downstream code reading stride_ sees the
      // new (packed) layout.
      stride_ = packed_row;
    }
  }

  if (convertOnGpu(image->encoding, data_ptr, data_size)) {
    return true;
  }

  ImageData image_data = setFormatAndNormalizeDataIfNecessary(
    image->encoding, data_ptr, data_size);

  const size_t expected_size =
    Ogre::PixelUtil::getMemorySize(width_, height_, 1, image_data.pixel_format_);
  if (image_data.size_in_bytes_ != expected_size) {
    RVIZ_COMMON_LOG_ERROR_STREAM(
      "Error loading image: data size " << image_data.size_in_bytes_ <<
        " does not match calculated image size " << expected_size);
    return false;
  }

  auto pixels = const_cast<uint8_t *>(image_data.data_ptr_);
  try {
    if (width_ == texture_width_ && height_ == texture_height_ &&
      image_data.pixel_format_ == texture_format_)
    {
      texture_->getBuffer()->blitFromMemory(
        Ogre::PixelBox(width_, height_, 1, image_data.pixel_format_, pixels));
    } else {
      Ogre::Image ogre_image;
      ogre_image.loadDynamicImage(pixels, width_, height_, 1, image_data.pixel_format_);
      texture_->unload();
      texture_->setUsage(Ogre::TU_DEFAULT);
      texture_->loadImage(ogre_image);
      texture_width_ = width_;
      texture_height_ = height_;
      texture_format_ = image_data.pixel_format_;
    }
  } catch (const Ogre::Exception & e) {
    RVIZ_COMMON_LOG_ERROR_STREAM("Error loading image: " << e.what());
    texture_format_ = Ogre::PF_UNKNOWN;
    return false;
  }

  return true;
}

struct yuyv
{
  uint8_t y0;
  uint8_t u;
  uint8_t y1;
  uint8_t v;
};

struct uyvy
{
  uint8_t u;
  uint8_t y0;
  uint8_t v;
  uint8_t y1;
};

// Function converts src_img from yuv422 format to rgb
static void imageConvertYUV422ToRGB(
  uint8_t * dst_img, uint8_t * src_img,
  int dst_start_row, int dst_end_row,
  int dst_num_cols, uint32_t stride_in_bytes)
{
  int final_y0 = 0;
  int final_u = 0;
  int final_y1 = 0;
  int final_v = 0;
  int r1 = 0;
  int b1 = 0;
  int g1 = 0;
  int r2 = 0;
  int b2 = 0;
  int g2 = 0;

  uint32_t stride_in_pixels = stride_in_bytes / 4;

  // rows in dst_img
  for (int row = dst_start_row; row < dst_end_row; row++) {
    // col iterates till num_cols / 2 since two rgb pixels processed each
    // iteration cols in dst_img
    for (int col = 0; col < dst_num_cols / 2; col++) {
      struct uyvy * src_ptr = reinterpret_cast<struct uyvy *>(src_img);
      struct uyvy * pixel = &src_ptr[col + row * stride_in_pixels];
      final_y0 = pixel->y0;
      final_u = pixel->u;
      final_y1 = pixel->y1;
      final_v = pixel->v;

      // Values generated based on this formula
      // for converting YUV to RGB
      // R = Y + 1.403V'
      // G = Y + 0.344U' - 0.714V'
      // B = Y + 1.770U'

      final_v -= 128;
      final_u -= 128;

      r1 = final_y0 + (1403 * final_v) / 1000;
      g1 = final_y0 + (344 * final_u - 714 * final_v) / 1000;
      b1 = final_y0 + (1770 * final_u) / 1000;

      r2 = final_y1 + (1403 * final_v) / 1000;
      g2 = final_y1 + (344 * final_u - 714 * final_v) / 1000;
      b2 = final_y1 + (1770 * final_u) / 1000;

      // pixel value must fit in a uint8_t
      dst_img[0] = ((r1 & 0xFFFFFF00) == 0) ? r1 : (r1 < 0) ? 0 : 0xFF;
      dst_img[1] = ((g1 & 0xFFFFFF00) == 0) ? g1 : (g1 < 0) ? 0 : 0xFF;
      dst_img[2] = ((b1 & 0xFFFFFF00) == 0) ? b1 : (b1 < 0) ? 0 : 0xFF;
      dst_img[3] = ((r2 & 0xFFFFFF00) == 0) ? r2 : (r2 < 0) ? 0 : 0xFF;
      dst_img[4] = ((g2 & 0xFFFFFF00) == 0) ? g2 : (g2 < 0) ? 0 : 0xFF;
      dst_img[5] = ((b2 & 0xFFFFFF00) == 0) ? b2 : (b2 < 0) ? 0 : 0xFF;
      dst_img += 6;
    }
  }
}

// Function converts src_img from yuv422_yuy2 format to rgb
static void imageConvertYUV422_YUY2ToRGB(
  uint8_t * dst_img, uint8_t * src_img,
  int dst_start_row, int dst_end_row,
  int dst_num_cols, uint32_t stride_in_bytes)
{
  int final_y0 = 0;
  int final_u = 0;
  int final_y1 = 0;
  int final_v = 0;
  int r1 = 0;
  int b1 = 0;
  int g1 = 0;
  int r2 = 0;
  int b2 = 0;
  int g2 = 0;

  uint32_t stride_in_pixels = stride_in_bytes / 4;

  // rows in dst_img
  for (int row = dst_start_row; row < dst_end_row; row++) {
    // col iterates till num_cols / 2 since two rgb pixels processed each
    // iteration cols in dst_img
    for (int col = 0; col < dst_num_cols / 2; col++) {
      struct yuyv * src_ptr = reinterpret_cast<struct yuyv *>(src_img);
      struct yuyv * pixel = &src_ptr[col + row * stride_in_pixels];
      final_y0 = pixel->y0;
      final_u = pixel->u;
      final_y1 = pixel->y1;
      final_v = pixel->v;

      // Values generated based on this formula
      // for converting YUV to RGB
      // R = Y + 1.403V'
      // G = Y + 0.344U' - 0.714V'
      // B = Y + 1.770U'

      final_v -= 128;
      final_u -= 128;

      r1 = final_y0 + (1403 * final_v) / 1000;
      g1 = final_y0 + (344 * final_u - 714 * final_v) / 1000;
      b1 = final_y0 + (1770 * final_u) / 1000;

      r2 = final_y1 + (1403 * final_v) / 1000;
      g2 = final_y1 + (344 * final_u - 714 * final_v) / 1000;
      b2 = final_y1 + (1770 * final_u) / 1000;

      // pixel value must fit in a uint8_t
      dst_img[0] = ((r1 & 0xFFFFFF00) == 0) ? r1 : (r1 < 0) ? 0 : 0xFF;
      dst_img[1] = ((g1 & 0xFFFFFF00) == 0) ? g1 : (g1 < 0) ? 0 : 0xFF;
      dst_img[2] = ((b1 & 0xFFFFFF00) == 0) ? b1 : (b1 < 0) ? 0 : 0xFF;
      dst_img[3] = ((r2 & 0xFFFFFF00) == 0) ? r2 : (r2 < 0) ? 0 : 0xFF;
      dst_img[4] = ((g2 & 0xFFFFFF00) == 0) ? g2 : (g2 < 0) ? 0 : 0xFF;
      dst_img[5] = ((b2 & 0xFFFFFF00) == 0) ? b2 : (b2 < 0) ? 0 : 0xFF;
      dst_img += 6;
    }
  }
}

ImageData::ImageData(
  Ogre::PixelFormat pixformat,
  const uint8_t * data_ptr,
  size_t data_size_in_bytes,
  bool take_ownership)
: pixel_format_(pixformat),
  data_ptr_(data_ptr),
  size_in_bytes_(data_size_in_bytes),
  has_ownership_(take_ownership)
{
}

ImageData::~ImageData()
{
  if (has_ownership_) {
    delete[] data_ptr_;
  }
}

template<typename T>
void
ROSImageTexture::getMinimalAndMaximalValueToNormalize(
  const T * data_ptr, size_t num_elements,
  double & min_value, double & max_value)
{
  if (normalize_) {
    // Accumulate in T to avoid per-pixel double conversions, then promote.
    T t_min = std::numeric_limits<T>::max();
    T t_max = std::numeric_limits<T>::lowest();
    const T * input_ptr = data_ptr;
    for (size_t i = 0; i < num_elements; ++i) {
      t_min = std::min(t_min, *input_ptr);
      t_max = std::max(t_max, *input_ptr);
      input_ptr++;
    }
    min_value = static_cast<double>(t_min);
    max_value = static_cast<double>(t_max);

    if (median_frames_ > 1) {
      min_value = computeMedianOfSeveralFrames(min_buffer_, min_value, this->median_frames_);
      max_value = computeMedianOfSeveralFrames(max_buffer_, max_value, this->median_frames_);
    }
  } else {
    // User-supplied min/max are doubles. Use them directly — no cast through
    // T — so a 16UC1 user can enter values up to 65535 (or above) and a
    // 32FC1 user can enter fractional values without losing precision.
    min_value = min_;
    max_value = max_;
  }
}

template<typename T>
ImageData
ROSImageTexture::convertTo8bit(const uint8_t * data_ptr, size_t data_size_in_bytes)
{
  size_t new_size_in_bytes = data_size_in_bytes / sizeof(T);

  // Zero-initialize so a degenerate range (<= 0) produces a valid all-black
  // image instead of handing uninitialized heap memory to Ogre.
  uint8_t * new_data = new uint8_t[new_size_in_bytes]();

  double min_value;
  double max_value;

  getMinimalAndMaximalValueToNormalize<T>(
    reinterpret_cast<const T *>(data_ptr), new_size_in_bytes, min_value, max_value);

  // Rescale T image and convert it to 8-bit. All arithmetic in double so
  // user-supplied min/max outside T's range (e.g. 65536 for 16UC1) still
  // produce meaningful output instead of an overflow-to-zero surprise.
  const double range = max_value - min_value;
  if (range > 0.0 && std::isfinite(range)) {
    const T * input_ptr = reinterpret_cast<const T *>(data_ptr);
    uint8_t * output_ptr = new_data;

    for (size_t i = 0; i < new_size_in_bytes; ++i, ++output_ptr, ++input_ptr) {
      double val = (static_cast<double>(*input_ptr) - min_value) / range;
      val = std::clamp(val, 0.0, 1.0);
      *output_ptr = static_cast<uint8_t>(val * 255u);
    }
  }
  // range <= 0: uniform-value frame or user-set min >= max. Leave the
  // zero-initialized buffer so we display solid black rather than garbage.

  return ImageData(Ogre::PF_BYTE_L, new_data, new_size_in_bytes, true);
}

ImageData
ROSImageTexture::convertYUV422ToRGBData(const uint8_t * data_ptr, size_t data_size_in_bytes)
{
  size_t new_size_in_bytes = data_size_in_bytes * 3 / 2;

  uint8_t * new_data = new uint8_t[new_size_in_bytes];

  imageConvertYUV422ToRGB(
    new_data, const_cast<uint8_t *>(data_ptr),
    0, height_, width_, stride_);

  return ImageData(Ogre::PF_BYTE_RGB, new_data, new_size_in_bytes, true);
}

ImageData
ROSImageTexture::convertYUV422_YUY2ToRGBData(const uint8_t * data_ptr, size_t data_size_in_bytes)
{
  size_t new_size_in_bytes = data_size_in_bytes * 3 / 2;

  uint8_t * new_data = new uint8_t[new_size_in_bytes];

  imageConvertYUV422_YUY2ToRGB(
    new_data, const_cast<uint8_t *>(data_ptr),
    0, height_, width_, stride_);

  return ImageData(Ogre::PF_BYTE_RGB, new_data, new_size_in_bytes, true);
}

ImageData
ROSImageTexture::setFormatAndNormalizeDataIfNecessary(
  const std::string & encoding, const uint8_t * data_ptr, size_t data_size_in_bytes)
{
  if (encoding == sensor_msgs::image_encodings::RGB8) {
    return ImageData(Ogre::PF_BYTE_RGB, data_ptr, data_size_in_bytes, false);
  } else if (encoding == sensor_msgs::image_encodings::RGBA8) {
    return ImageData(Ogre::PF_BYTE_RGBA, data_ptr, data_size_in_bytes, false);
  } else if (  // NOLINT enforces bracket on the same line, which makes code unreadable
    encoding == sensor_msgs::image_encodings::TYPE_8UC4 ||
    encoding == sensor_msgs::image_encodings::TYPE_8SC4 ||
    encoding == sensor_msgs::image_encodings::BGRA8)
  {
    return ImageData(Ogre::PF_BYTE_BGRA, data_ptr, data_size_in_bytes, false);
  } else if (  // NOLINT enforces bracket on the same line, which makes code unreadable
    encoding == sensor_msgs::image_encodings::TYPE_8UC3 ||
    encoding == sensor_msgs::image_encodings::TYPE_8SC3 ||
    encoding == sensor_msgs::image_encodings::BGR8)
  {
    return ImageData(Ogre::PF_BYTE_BGR, data_ptr, data_size_in_bytes, false);
  } else if (  // NOLINT enforces bracket on the same line, which makes code unreadable
    encoding == sensor_msgs::image_encodings::TYPE_8UC1 ||
    encoding == sensor_msgs::image_encodings::TYPE_8SC1 ||
    encoding == sensor_msgs::image_encodings::MONO8)
  {
    return ImageData(Ogre::PF_BYTE_L, data_ptr, data_size_in_bytes, false);
  } else if (  // NOLINT enforces bracket on the same line, which makes code unreadable
    encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
    encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
    encoding == sensor_msgs::image_encodings::MONO16)
  {
    return convertTo8bit<uint16_t>(data_ptr, data_size_in_bytes);
  } else if (encoding.find("bayer") == 0) {
    return ImageData(Ogre::PF_BYTE_L, data_ptr, data_size_in_bytes, false);
  } else if (encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    return convertTo8bit<float>(data_ptr, data_size_in_bytes);
  } else if (encoding == sensor_msgs::image_encodings::YUV422) {
    return convertYUV422ToRGBData(data_ptr, data_size_in_bytes);
  } else if (encoding == sensor_msgs::image_encodings::YUV422_YUY2) {
    return convertYUV422_YUY2ToRGBData(data_ptr, data_size_in_bytes);
  } else {
    throw UnsupportedImageEncoding(encoding);
  }
}

// sRGB transfer of a linear value in [0, 1], as in rviz PR #1749.
static double srgbEncode(double linear)
{
  if (linear <= 0.0031308) {
    return 12.92 * linear;
  }
  return 1.055 * std::pow(linear, 1.0 / 2.4) - 0.055;
}

static uint8_t srgbByte(double linear)
{
  return static_cast<uint8_t>(std::lround(std::clamp(srgbEncode(linear), 0.0, 1.0) * 255.0));
}

bool ROSImageTexture::convertOnGpu(
  const std::string & encoding, const uint8_t * data_ptr, size_t data_size_in_bytes)
{
  namespace enc = sensor_msgs::image_encodings;
  const size_t pixels = static_cast<size_t>(width_) * height_;
  const bool gray16 = encoding == enc::TYPE_16UC1 || encoding == enc::TYPE_16SC1 ||
    encoding == enc::MONO16;
  const bool float32 = encoding == enc::TYPE_32FC1;
  const bool bayer = enc::isBayer(encoding);
  if (!(gray16 || float32 || bayer) || pixels == 0 || !GpuImageConverter::supported()) {
    return false;
  }
  const size_t bytes_per_pixel = static_cast<size_t>(enc::bitDepth(encoding) / 8);
  if (data_size_in_bytes < pixels * bytes_per_pixel) {
    return false;
  }

  GpuImageConverter::Params params;
  double min_value = 0.0;
  double max_value = 0.0;
  if (float32) {
    params.mode = GpuImageConverter::FLOAT32;
    getMinimalAndMaximalValueToNormalize<float>(
      reinterpret_cast<const float *>(data_ptr), pixels, min_value, max_value);
    params.min = min_value;
    params.range = max_value - min_value;
    if (!std::isfinite(params.range)) {
      params.range = 0.0;
    }
  } else if (gray16 || bytes_per_pixel == 2) {
    getMinimalAndMaximalValueToNormalize<uint16_t>(
      reinterpret_cast<const uint16_t *>(data_ptr), pixels, min_value, max_value);
    const double range = max_value - min_value;
    lut_.assign(65536, 0);
    if (range > 0.0 && std::isfinite(range)) {
      for (size_t i = 0; i < lut_.size(); ++i) {
        const double value = static_cast<double>(i);
        if (gray16) {
          // Same arithmetic as convertTo8bit.
          lut_[i] = static_cast<uint8_t>(
            std::clamp((value - min_value) / range, 0.0, 1.0) * 255u);
        } else if (linear_input_) {
          const double scaled = std::clamp((value - min_value) * (65535.0 / range), 0.0, 65535.0);
          lut_[i] = srgbByte(static_cast<double>(std::lround(scaled)) / 65535.0);
        } else {
          lut_[i] = static_cast<uint8_t>(
            std::lround(std::clamp((value - min_value) * (255.0 / range), 0.0, 255.0)));
        }
      }
    }
    params.mode = gray16 ? GpuImageConverter::GRAY16 : GpuImageConverter::BAYER16;
    params.lut = lut_.data();
  } else {
    params.mode = GpuImageConverter::BAYER8;
    if (linear_input_) {
      static const std::array<uint8_t, 256> srgb = []() {
          std::array<uint8_t, 256> table;
          for (size_t i = 0; i < table.size(); ++i) {
            table[i] = srgbByte(static_cast<double>(i) / 255.0);
          }
          return table;
        }();
      params.lut = srgb.data();
    }
  }
  if (bayer) {
    // Position of the red sample in the 2x2 cell.
    params.red_row = encoding.find("bggr") != std::string::npos ||
      encoding.find("gbrg") != std::string::npos ? 1 : 0;
    params.red_col = encoding.find("bggr") != std::string::npos ||
      encoding.find("grbg") != std::string::npos ? 1 : 0;
  }

  if (width_ != texture_width_ || height_ != texture_height_ ||
    texture_format_ != Ogre::PF_BYTE_RGBA || texture_->getUsage() != Ogre::TU_RENDERTARGET)
  {
    // The converter renders into the texture the displays already sample.
    texture_->unload();
    texture_->setUsage(Ogre::TU_RENDERTARGET);
    texture_->setTextureType(Ogre::TEX_TYPE_2D);
    texture_->setFormat(Ogre::PF_BYTE_RGBA);
    texture_->setWidth(width_);
    texture_->setHeight(height_);
    texture_->setNumMipmaps(0);
    texture_->createInternalResources();
    texture_width_ = width_;
    texture_height_ = height_;
    texture_format_ = Ogre::PF_BYTE_RGBA;
  }
  if (!gpu_converter_) {
    gpu_converter_ = std::make_unique<GpuImageConverter>();
  }
  gpu_converter_->convert(data_ptr, width_, height_, params, texture_);
  return true;
}

void ROSImageTexture::addMessage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_image_ = msg;
  new_image_ = true;
}

}  // namespace displays
}  // namespace rviz_default_plugins
