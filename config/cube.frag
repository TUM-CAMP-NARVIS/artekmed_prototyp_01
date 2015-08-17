#version 330 core

// Interpolated values from the vertex shaders
in vec2 UV;
in vec2 secondUV;
in float zVS;
uniform int isRight;
// Ouput data
out vec3 color;
// Values that stay constant for the whole mesh.
uniform sampler2D myTextureSampler;
uniform sampler2D depthTexSampler;
void main(){
    vec2 res = vec2(640.0, 480.0);
	vec2 uv2;
	if(isRight == 0)
	{
		uv2 = gl_FragCoord.xy/ res;
		uv2.x= uv2.x;
	}
	else
	{
		uv2 = gl_FragCoord.xy/(res);
		uv2.x = uv2.x - 1.0;
	}
    uv2.y =  1.0 - uv2.y;
    //uv2.x= uv2.x;
    float d = texture(depthTexSampler, uv2).r;
    if (d == 1.0) {
        discard;
    } else
		color = texture2D( myTextureSampler, uv2 ).rgb;
}

