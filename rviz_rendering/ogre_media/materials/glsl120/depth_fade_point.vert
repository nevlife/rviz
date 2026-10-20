#version 120

// Point sprites whose size follows a world size (size.x) and the distance to the camera.
// A point smaller than a pixel is drawn one pixel wide with its alpha scaled by its area;
// below MIN_ALPHA_THRESHOLD points are removed at random instead, keeping the perceived density.

uniform mat4 worldviewproj_matrix;
uniform mat4 worldview_matrix;
uniform mat4 projection_matrix;
uniform vec4 size;
uniform float alpha;
uniform float viewport_height;

const float MAX_PIXEL_SIZE = 100.0;
const float MIN_ALPHA_THRESHOLD = 0.04;

float hash21(vec3 p)
{
  p = fract(p * 0.3183099 + 0.1);
  p *= 17.0;
  return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
}

void main()
{
  gl_Position = worldviewproj_matrix * gl_Vertex;

  float depth = -(worldview_matrix * gl_Vertex).z;
  if (depth < 1e-6) {
    gl_Position.w = -1.0;
    return;
  }

  // m[3][3] is 0 for a perspective projection and 1 for an orthographic one.
  bool is_ortho = projection_matrix[3][3] > 0.5;
  float pixel_size =
    size.x * projection_matrix[1][1] / (is_ortho ? 1.0 : depth) * (viewport_height * 0.5);
  gl_PointSize = clamp(pixel_size, 1.0, MAX_PIXEL_SIZE);

  float final_alpha = gl_Color.a * alpha * clamp(pixel_size * pixel_size, 0.0, 1.0);
  if (final_alpha < MIN_ALPHA_THRESHOLD) {
    if (hash21(gl_Vertex.xyz) * MIN_ALPHA_THRESHOLD > final_alpha) {
      gl_Position.w = -1.0;
      return;
    }
    final_alpha = MIN_ALPHA_THRESHOLD;
  }

  gl_FrontColor = vec4(gl_Color.rgb, final_alpha);
}
