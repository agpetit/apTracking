uniform sampler2D tex;
varying vec3 normal;
varying vec3 color;
// varying vec2 uv;

void main()
{
    vec3 light_dir1 = normalize(vec3(1,0,0));
    vec3 light_dir2 = normalize(vec3(0,1,0));
    vec3 light_dir3 = normalize(vec3(0,0,1));
    vec3 light_dir4 = normalize(vec3(-1,0,0));
    vec3 light_dir5 = normalize(vec3(0,-1,0));
    vec3 light_dir6 = normalize(vec3(0,0,-1));
    vec3 n = normalize(normal);
    if (!gl_FrontFacing)
        n = -n;
    float l1 = max(0.0, dot(light_dir1, n));
    float l2 = max(0.0, dot(light_dir2, n));
    float l3 = max(0.0, dot(light_dir3, n));
    float l4 = max(0.0, dot(light_dir4, n));
    float l5 = max(0.0, dot(light_dir5, n));
    float l6 = max(0.0, dot(light_dir6, n));
    //gl_FragColor = vec4(l * color * texture2D(tex,uv).rgb,1.0);
    //gl_FragColor = vec4(l * color,1.0);
      gl_FragColor = vec4(l1*color + l2*color + l3*color,1.0);

}


