uniform mat4 worldViewProj;
uniform vec4 texelOffsets;
  
// X = minDepth
// Y = maxDepth
// Z = depthRange
// W = 1.0 / depthRange                                                         

uniform vec4 depthRange;

varying vec2 depth;

void main()
{
  gl_Position = ftransform(); 
//gl_Position = mul(worldViewProj,position);
                                                  
  // fix pixel / texel alignment
  //gl_Position.xy += texelOffsets.zw * gl_Position.w;


  // linear depth storage
  // offset / scale range output
/*if (depthRange.x>-1)
{
 depth.x = (gl_Position.z)/(gl_Position.w);
}
else 
depth.x = 0.5;
//return depth.x;
//depth.x = (gl_Position.z - 0.1) * depthRange.w;
//depth.x = gl_Position.z/depthRange.w;
//depth.x = 0;
//depth.x= gl_Position.z*0.001;*/
}



