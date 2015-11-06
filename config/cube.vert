#version 330 core

// Input vertex data, different for all executions of this shader.
layout(location = 2) in vec3 vertexPosition_modelspace;
layout(location = 3) in vec2 vertexUV;
layout(location = 4) in vec2 secondVertexUV;
// Output data ; will be interpolated for each fragment.
out vec2 UV;
out vec2 secondUV;
out float zVS;
// Values that stay constant for the whole mesh.
uniform mat4 MVP;
uniform mat4 M, V, P;
void main(){
    vec4 pVS = V *  M  * vec4(vertexPosition_modelspace, 1);
    zVS = pVS.z / 1000.0;
    // Output position of the vertex, in clip space : MVP * position
	gl_Position =  P* pVS;
    // UV of the vertex. No special space for this one.
	UV = vertexUV;
    secondUV= secondVertexUV;
}

