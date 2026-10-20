#version 150 compatibility
#ifdef FP64
#extension GL_ARB_gpu_shader_fp64 : require
#endif

// Converts one raw image texel per output pixel into 8-bit RGB, as ROSImageTexture does on the CPU.
// mode 0: 16-bit gray through lut. 1: 32-bit float gray, (v - min) / range clamped to [0, 1].
// 2: 8-bit Bayer, 3: 16-bit Bayer, bilinear demosaic as in rviz PR #1749, values through lut
// (8-bit Bayer only when params.w is 1).
// params: x mode, y Bayer red column, z Bayer red row, w use lut for 8-bit Bayer.
// range: float mode min and max - min, each split into a high and a low float.

uniform sampler2D raw;
uniform sampler2D lut;
uniform vec4 params;
uniform vec4 range;

ivec2 size;
int mode;

int texel(ivec2 p)
{
  float v = texelFetch(raw, clamp(p, ivec2(0), size - 1), 0).r;
  return int(v * (mode == 2 ? 255.0 : 65535.0) + 0.5);
}

int lookup(int v)
{
  return int(texelFetch(lut, ivec2(v & 255, v >> 8), 0).r * 255.0 + 0.5);
}

int quantize(int v)
{
  if (mode == 3 || params.w > 0.5) {
    return lookup(v);
  }
  return clamp(v, 0, 255);
}

// Rounded mean of the valid neighbours at the given offsets.
int average(ivec2 p, ivec2 a, ivec2 b, ivec2 c, ivec2 d, int n)
{
  ivec2 offsets[4] = ivec2[4](a, b, c, d);
  int sum = 0;
  int count = 0;
  for (int i = 0; i < n; ++i) {
    ivec2 q = p + offsets[i];
    if (all(greaterThanEqual(q, ivec2(0))) && all(lessThan(q, size))) {
      sum += texel(q);
      count += 1;
    }
  }
  return count > 0 ? (sum + count / 2) / count : 0;
}

vec3 demosaic(ivec2 p)
{
  ivec2 red = ivec2(params.y, params.z);
  ivec2 parity = p & 1;
  int c = texel(p);
  int cardinal = average(p, ivec2(0, -1), ivec2(0, 1), ivec2(-1, 0), ivec2(1, 0), 4);
  int diagonal = average(p, ivec2(-1, -1), ivec2(1, -1), ivec2(-1, 1), ivec2(1, 1), 4);
  int horizontal = average(p, ivec2(-1, 0), ivec2(1, 0), ivec2(0), ivec2(0), 2);
  int vertical = average(p, ivec2(0, -1), ivec2(0, 1), ivec2(0), ivec2(0), 2);
  ivec3 rgb;
  if (parity == red) {
    rgb = ivec3(c, cardinal, diagonal);
  } else if (parity == 1 - red) {
    rgb = ivec3(diagonal, cardinal, c);
  } else if (parity.y == red.y) {
    rgb = ivec3(horizontal, c, vertical);
  } else {
    rgb = ivec3(vertical, c, horizontal);
  }
  return vec3(quantize(rgb.r), quantize(rgb.g), quantize(rgb.b)) / 255.0;
}

// Same arithmetic as ROSImageTexture::convertTo8bit, in double when the GPU has it.
float normalized(float v)
{
  // NaN is drawn black, as the CPU conversion does; comparisons alone do not catch it on NVIDIA.
  if (isnan(v) || !(range.z + range.w > 0.0)) {
    return 0.0;
  }
#ifdef FP64
  double t = (double(v) - (double(range.x) + double(range.y))) /
    (double(range.z) + double(range.w));
  if (!(t > 0.0LF)) {
    return 0.0;
  }
  return float(floor(min(t, 1.0LF) * 255.0LF)) / 255.0;
#else
  float t = (v - range.x - range.y) / (range.z + range.w);
  if (!(t > 0.0)) {
    return 0.0;
  }
  return floor(min(t, 1.0) * 255.0) / 255.0;
#endif
}

void main()
{
  size = textureSize(raw, 0);
  mode = int(params.x);
  ivec2 p = ivec2(gl_TexCoord[0].xy * vec2(size));
  vec3 color;
  if (mode == 0) {
    color = vec3(float(lookup(texel(p))) / 255.0);
  } else if (mode == 1) {
    color = vec3(normalized(texelFetch(raw, p, 0).r));
  } else {
    color = demosaic(p);
  }
  gl_FragColor = vec4(color, 1.0);
}
