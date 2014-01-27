
// handle to texture
uniform sampler2D frame;

// kernels for edge detection (to load from program)
uniform float[25] kernel;

// Threshold for removing noise
uniform float fThreshold;

vec4 dotConvolution(){
	vec3 v3Color, v3Center;
	vec4 vNull;
	float fResult;

	vNull = vec4(0.0,0.0,0.0,0.0);

	v3Center = texture(frame, gl_TexCoord[0].st).rgb;
	
	// convolute sobel kernel for each color channel
	for(int i=0; i<5; i++){
		for(int j=0; j<5; j++){
		
			v3Color = textureOffset(frame, gl_TexCoord[0].st, ivec2((i-2),(j-2))).rgb;

			int idx = i * 5 + j;
			float d = dot(v3Color, v3Center);
			fResult += kernel[idx] * d;
		}
	}
	
	if(fResult < fThreshold)
		return vNull;
			
	return vec4(fResult, fResult, fResult, 0);
}

void main(){
	gl_FragColor = dotConvolution();
}
