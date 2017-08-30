varying vec3 normal;
varying vec3 color;
varying vec2 uv;

void main()
{
    gl_Position = ftransform();
    normal = normalize(gl_NormalMatrix * gl_Normal);
    //color = gl_Color.rgb;
    color = gl_FrontMaterial.diffuse.rgb;
    uv = gl_MultiTexCoord0.xy;
}



