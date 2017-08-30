uniform mat4 worldViewProj;
uniform vec4 texelOffsets;
varying vec3 oNormal;
varying float depth;
//varying vec4 oPosition;
/*float4 position : POSITION,
						float3 normal       : NORMAL,
						  
						  out float4 oPosition : POSITION,
						  out float3 oNormal  : TEXCOORD0,*/
void main()
{

    gl_Position = ftransform();

    oNormal = normalize(gl_Normal);
    vec4 tmp = gl_ModelViewMatrix * gl_Vertex;
    depth = tmp.z / tmp.w;
}



