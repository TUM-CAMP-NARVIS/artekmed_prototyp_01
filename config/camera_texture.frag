#version 330 core

// Interpolated values from the vertex shaders
in vec2 UV;
// Ouput data
out vec3 color2;
// Values that stay constant for the whole mesh.
uniform sampler2D myTextureSampler;
uniform sampler2D depthSampler;
void main(){
    //vec2 res = vec2(640.0, 480.0);
	//vec2 uv2 = gl_FragCoord.xy / res;
	//vec2 uv = uv2;
	//uv2.y =  1.0 - uv2.y;

	//uv2.x= uv2.x;
	//float d = texture(depthSampler, uv2).r;
	//if (d == 1.0) {
	//	color2 = texture2D( myTextureSampler, uv ).rgb;
	//} else
	//	discard;
	color2 =  texture2D( myTextureSampler, UV ).rgb;
}
