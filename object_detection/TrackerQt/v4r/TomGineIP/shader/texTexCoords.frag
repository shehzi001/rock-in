varying vec4 vertex;

void main(){
    vec4 texcoords = gl_TexCoord[0];
    gl_FragColor = vec4(gl_TexCoord[0].xy,1.0,1.0);
}



