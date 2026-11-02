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

#include <gmock/gmock.h>

#include "../../src/rviz_rendering/software_renderer.cpp"  // NOLINT (build/include)

using rviz_rendering::isSoftwareRenderer;

TEST(SoftwareRenderer, recognizes_cpu_rasterizers) {
  EXPECT_TRUE(isSoftwareRenderer("llvmpipe (LLVM 19.1.1, 256 bits)"));
  EXPECT_TRUE(isSoftwareRenderer("softpipe"));
  EXPECT_TRUE(isSoftwareRenderer("Software Rasterizer"));
  EXPECT_TRUE(isSoftwareRenderer("Google SwiftShader"));
  EXPECT_TRUE(isSoftwareRenderer("Apple Software Renderer"));
  EXPECT_TRUE(isSoftwareRenderer("Microsoft Basic Render Driver"));
}

TEST(SoftwareRenderer, keeps_hardware_renderers) {
  EXPECT_FALSE(isSoftwareRenderer("NVIDIA GeForce RTX 4070/PCIe/SSE2"));
  EXPECT_FALSE(isSoftwareRenderer("Mesa Intel(R) UHD Graphics 630 (CFL GT2)"));
  EXPECT_FALSE(isSoftwareRenderer("AMD Radeon RX 6600 (radeonsi, navi23, LLVM 17.0.6)"));
  EXPECT_FALSE(isSoftwareRenderer("NVIDIA Tegra Orin (nvgpu)/integrated"));
  EXPECT_FALSE(isSoftwareRenderer(""));
}
