//uniform float fNear;
//uniform float fFar;
//varying vec2 depth;
//uniform sampler2D tex;
void main()
{
    float fDepth = gl_FragCoord.z;
/*//float fDepth= depth.x;
    float fColor = 1-smoothstep(fFar, fNear, fDepth);
if (fFar>0)
{
//gl_FragColor = vec4(vec3(fDpeth), 1.0);
//gl_FragColor = vec4(0, 0,1, 1);
gl_FragColor = vec4(depth.x/2+0.5, depth.x/2+0.5, depth.x/2+0.5, 1);
}
else 
gl_FragColor = vec4(0, 0,1, 1);*/
gl_FragColor = vec4(fDepth, fDepth, fDepth, 1);
//gl_FragColor = vec4(1, 1, 1, 1);



}

