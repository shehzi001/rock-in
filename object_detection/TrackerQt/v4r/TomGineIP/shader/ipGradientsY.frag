
uniform sampler2D frame;

vec4 gradients()
{
  vec4 a = textureOffset(frame, gl_TexCoord[0].st, ivec2( 0,-1));
  vec4 b = textureOffset(frame, gl_TexCoord[0].st, ivec2( 0, 1));

  vec4 g = b - a;
  g = (g * 0.5) + 0.5;
  g.w = 1.0;

  return g;
}

void main()
{
  gl_FragColor = gradients();
}
