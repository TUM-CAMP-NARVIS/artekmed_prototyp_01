#version 330 core

// Interpolated values from the vertex shaders
in vec2 UV;
in vec2 secondUV;
in float zVS;

// Ouput data
out vec3 color;

// Values that stay constant for the whole mesh.
uniform sampler2D myTextureSampler;

uniform sampler2D depthTexSampler;

void main(){
	vec2 res = vec2(1024.0, 768.0);
	vec2 uv2 = gl_FragCoord.xy / res;
	uv2.y =  1.0 - uv2.y;
	float d = texture(depthTexSampler, uv2).r;
	if (d == 1.0) {
		color = texture2D( myTextureSampler, UV ).rgb;
	} else
		discard;
	//color = vec3(d);
//	if (-zVS > d)
//		discard;

	
	
}