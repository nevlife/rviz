#version 130
#extension GL_ARB_draw_instanced : enable
#ifdef EXACT
#extension GL_ARB_gpu_shader5 : require
#define PRECISE precise
#else
#define PRECISE
#endif

// Reads x, y, z and the color field of each point straight from the PointCloud2 bytes.
// BILLBOARD: one instance per point, "vertex" is the corner of the template quad.
// Otherwise one GL point per point and "vertex" is x.
// The position w is vertex.w (1, the attribute default), like gl_Vertex.w in the CPU-path shaders.

uniform mat4 worldviewproj_matrix;
uniform vec4 size;
uniform vec4 color_mode;
uniform vec4 color_range;
uniform vec4 color_a;
uniform vec4 color_b;
uniform vec4 color_axis;
uniform vec4 size_field;
uniform vec4 size_range;
#ifdef BILLBOARD
uniform vec4 camera_pos;
uniform vec4 auto_size;
#endif
#ifdef WITH_DEPTH
uniform mat4 worldview_matrix;
varying float depth;
#endif

attribute vec4 vertex;
#ifdef BILLBOARD
attribute vec4 uv1;
#endif
attribute vec4 uv2;
attribute vec4 uv3;
attribute vec4 uv4;
attribute vec4 uv5;

// EXACT: IEEE division and no fused multiply-add, so computed colors match the CPU path bit for bit.
float divide(float a, float b)
{
#ifdef EXACT
  precise float q = a / b;
  precise float r = fma(-q, b, a);
  precise float exact = fma(r, 1.0 / b, q);
  return exact;
#else
  return a / b;
#endif
}

// Integer fields arrive as two shorts (low, high).
float fieldValue(vec4 field, int type)
{
  if (type == 0) {
    return field.x;
  } else if (type == 1) {
    return float(int(field.x) & 0xFF);
  } else if (type == 2) {
    return float(int(field.x) & 0xFFFF);
  }
  return float(int(field.x) & 0xFFFF) + float(int(field.y) & 0xFFFF) * 65536.0;
}

float sizeScale()
{
  float v = fieldValue(uv5, int(size_field.y));
  float t = size_range.y > 0.0 ? (v - size_range.x) / size_range.y : 0.0;
  t = isnan(t) ? 0.0 : clamp(t, 0.0, 1.0);
  return mix(size_field.z, size_field.w, t);
}

vec3 rainbow(float value)
{
  value = clamp(value, 0.0, 1.0);
  PRECISE float h = value * 5.0 + 1.0;
  int i = int(floor(h));
  float f = h - float(i);
  if ((i & 1) == 0) {
    f = 1.0 - f;
  }
  float n = 1.0 - f;
  if (i <= 1) {
    return vec3(n, 0.0, 1.0);
  } else if (i == 2) {
    return vec3(0.0, n, 1.0);
  } else if (i == 3) {
    return vec3(0.0, 1.0, n);
  } else if (i == 4) {
    return vec3(n, 1.0, 0.0);
  }
  return vec3(1.0, n, 0.0);
}

// Mirrors the CPU path: computed colors are truncated to bytes like Ogre::ColourValue::getAsBYTE.
vec4 pointColor(vec3 p)
{
  int source = int(color_mode.x);
  if (source == 0) {
    return color_a;
  }
  if (source >= 3) {
    int lo = int(uv4.x) & 0xFFFF;
    int hi = int(uv4.y) & 0xFFFF;
    float a = source == 4 ? float(hi >> 8) : 255.0;
    return vec4(float(hi & 0xFF), float(lo >> 8), float(lo & 0xFF), a) / 255.0;
  }
  PRECISE float v;
  if (source == 2) {
    v = color_axis.x * p.x + color_axis.y * p.y + color_axis.z * p.z + color_axis.w;
  } else {
    v = fieldValue(uv4, int(color_mode.y));
  }
  float t = divide(v - color_range.x, color_range.y);
  PRECISE vec3 c;
  if (color_mode.z > 0.5) {
    PRECISE float value = 1.0 - t;
    if (color_mode.w > 0.5) {
      value = 1.0 - value;
    }
    c = rainbow(value);
  } else {
    float n = clamp(t, 0.0, 1.0);
    c = color_b.rgb * n + color_a.rgb * (1.0 - n);
  }
  return vec4(floor(c * 255.0) / 255.0, 1.0);
}

void main()
{
#ifdef BILLBOARD
  vec3 p = vec3(uv1.x, uv2.x, uv3.x);
#else
  vec3 p = vec3(vertex.x, uv2.x, uv3.x);
#endif

#ifdef BILLBOARD
  vec3 at = camera_pos.xyz - p;
  at = normalize(at);
  vec3 right = cross(vec3(0.0, 1.0, 0.0), at);
  vec3 up = cross(at, right);
  right = normalize(right);
  up = normalize(up);

  vec4 size_factor = (1.0 - auto_size.x + (auto_size.x * p.z)) * size;
  if (size_field.x > 0.5) {
    size_factor *= sizeScale();
  }

  vec4 s = vertex * (size_factor);
  vec3 r = s.xxx * right;
  vec3 u = s.yyy * up;

  vec4 pos = vec4(p, vertex.w) + vec4(r + u, 0.0);
  gl_TexCoord[0] = vertex + vec4(0.5, 0.5, 0.0, 0.0);
#else
  vec4 pos = vec4(p, vertex.w);
  float point_size = size.x;
  if (size_field.x > 0.5) {
    point_size *= sizeScale();
  }
  gl_PointSize = point_size;
#endif

  if (any(isnan(p)) || any(isinf(p))) {
    gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
  } else {
    gl_Position = worldviewproj_matrix * pos;
  }

#ifdef PICK_INDEX
#ifdef BILLBOARD
  int index = gl_InstanceIDARB + 1;
#else
  int index = gl_VertexID + 1;
#endif
  gl_FrontColor =
    vec4(float((index >> 16) & 0xFF), float((index >> 8) & 0xFF), float(index & 0xFF), 255.0) /
    255.0;
#elif defined(WITH_DEPTH) || defined(NO_COLOR)
  gl_FrontColor = vec4(1.0);
#else
  gl_FrontColor = pointColor(p);
#endif

#ifdef WITH_DEPTH
  depth = -(worldview_matrix * pos).z;
#endif
}
