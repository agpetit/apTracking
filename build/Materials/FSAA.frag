uniform sampler2D tex;
uniform vec2 dp;

void main()
{
    float dx = dp.x * (1.0 / 2.0);
    float dy = dp.y * (1.0 / 2.0);
    vec4 acc = vec4(0.0, 0.0, 0.0, -1e3);
    for(float i = 0.0 ; i < 2.0 ; ++i)
        for(float j = 0.0 ; j < 2.0 ; ++j)
        {
            vec4 c = texture2D(tex, gl_TexCoord[0].st + vec2(i * dx, j * dy));
            acc.xyz += c.xyz;
            acc.w = max(acc.w, c.w);
        }
    acc.xyz *= (1.0 / 4.0);
    acc.xyz = acc.xyz * 0.5 + 0.5;
    gl_FragColor = acc;
}
