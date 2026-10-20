#version 150 compatibility

// Full-target quad for image_convert.frag.

uniform mat4 worldviewproj_matrix;

void main()
{
  gl_Position = worldviewproj_matrix * gl_Vertex;
  gl_TexCoord[0] = gl_MultiTexCoord0;
}
