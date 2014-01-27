#version 130

uniform sampler2D textures[NUM_VIEWS];

vec4 diff()
{
  vec4 vColor0 = texture2D(textures[0], gl_TexCoord[0].st);
  vec4 diff = vec4(0,0,0,0);

  int n = 0;
  for (int i = 1; i < NUM_VIEWS; i++)
  {
    diff = diff + abs(vColor0 - texture2D(textures[i], gl_TexCoord[0].st));
    n++;
  }

  diff = diff / n;
  diff.w = 1.0;

  return diff;
}

void main()
{
  gl_FragColor = diff();
}

