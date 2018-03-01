varying vec3 oNormal;
// varying float depth;
uniform float fNear;
uniform float fFar;
//uniform sampler2D tex;

void main()
{
    //float fDepth = (-fFar * fNear / -depth + fFar) / (fFar - fNear);
    vec3 n = normalize(oNormal);
    if (!gl_FrontFacing)
        n = -n;

    float fDepth = gl_FragCoord.z;
    gl_FragColor = vec4(n, fDepth);
//    gl_FragColor = vec4(n * 0.5 + vec3(0.5), fDepth);

//gl_FragColor = vec4(oNormal.z/2+0.5, oNormal.y/2+0.5, oNormal.x/2+0.5,depth.x/2+0.5);
//gl_FragColor = vec4(fDepth, fDepth, fDepth,1);

}


