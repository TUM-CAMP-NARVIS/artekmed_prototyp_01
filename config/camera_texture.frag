#version 330 core

// Interpolated values from the vertex shaders
in vec2 UV;

// Ouput data
out vec3 color2;

// Values that stay constant for the whole mesh.
uniform sampler2D myTextureSampler;
uniform sampler2D depthSampler;
uniform vec2 randomInc;
const float downAmount= 2.0;
const int down= 2;
const vec2 targetRes= vec2(640.0, 480.0);
float rand(vec2 co){
    return (1.0 - fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453));
}

float raisedCos(in float val){
    return pow( ((cos(val * 2.0 * 3.14159) + 1.0)/2.0), 2.0);
}
void main(){
	vec3 total = vec3(0., 0., 0.);
    int width = int(targetRes.x);
    int height = int(targetRes.y);
    vec2 pixelSize = 1.0 / targetRes;
    float x_subpix_inc = 1.0 / (targetRes.x * downAmount);
    float y_subpix_inc = 1.0 / (targetRes.y * downAmount);
    
    for (int i=0; i < down; i++) {
        for (int j=0; j < down; j++) {
            vec2 pure_sample_loc = vec2(UV.x + float(i)*x_subpix_inc, UV.y + float(j)*y_subpix_inc);
            float rand1 = rand(vec2(pure_sample_loc.x, UV.y + randomInc.y));
            float rand2 = rand(vec2(UV.x + randomInc.x, pure_sample_loc.y));
            float randScale = 20.0;
            float x_fuzz = randScale * raisedCos(UV.x);
            float y_fuzz = randScale * raisedCos(UV.y);
            vec2 pseudo_sample_loc = pure_sample_loc + vec2(x_fuzz*pixelSize.x*rand1, y_fuzz*pixelSize.y*rand2);
            //vec2 pseudo_sample_loc = vec2(texcoord.x + x_fuzz*pixelSize.x*rand1, texcoord.y + y_fuzz*pixelSize.y*rand2);
            
            total += texture2D(myTextureSampler, pseudo_sample_loc).rgb;
            // total += texture2D(myTextureSampler, pure_sample_loc).rgb;
        }
    }
    
    total = total / (floor(downAmount) * floor(downAmount));
	//UV.y= 1 - UV.y;
	color2 =  texture2D( myTextureSampler, UV ).rgb;
	//color2=total;
}