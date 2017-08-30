//define KERNEL_SIZE 9
uniform sampler2D tex;
uniform float edgeTh;
uniform float fNear;
uniform float fFar;
uniform vec2 dp;
//uniform sampler2D tex1;
const float kernel[9] = {
0, 1, 0,
1,-4, 1,
0, 1, 0
};

const float kernel_h[9] = {
-1, 0, 1,
-2, 0, 2,
-1, 0, 1
};

const float kernel_v[9] = {
-1, -2, -1,
0, 0, 0,
1, 2, 1
};

/*const float kernel_g[25] = {
2,4,5,4,2,
4,9,12,9,4,
5,12,15,12,5,
4,9,12,9,4,
2,4,5,4,2
};*/

void main()
{
    ivec2 image_size = textureSize(tex, 0);
    float step_w = 1.0/image_size[0];
    float step_h = 1.0/image_size[1];

    vec2 offset[9]={
        vec2(-step_w, -step_h), vec2(0.0, -step_h), vec2(step_w, -step_h),
        vec2(-step_w, 0.0), vec2(0.0, 0.0), vec2(step_w, 0.0),
        vec2(-step_w, step_h), vec2(0.0, step_h), vec2(step_w, step_h)
        };

    int i = 0;
    float sum = 0.0;
    vec3 nsum = vec3(0.0,0.0,0.0);
    vec4 two = vec4(2.0);
    float lum = 0.0;
    float sum_h = 0;
    float sum_v = 0;
    float ori = 0.0;
    float col = 0.0;
    float edge =0;
    float PI = 3.14159265358979323846264;
    float luml= 0.0;

    vec4 tmp0 = texture2D(tex, gl_TexCoord[0].st);
    for( i=0; i<9; i++ )
    {
        vec4 tmp=texture2D(tex, gl_TexCoord[0].st + offset[i]);
        sum += (tmp.w) * kernel[i];
        nsum += normalize(tmp.xyz - 0.5);
    }
    nsum = nsum * (1.0 / 9.0);
    if (length(nsum) < 0.95)
        sum = -1;

    if (sum*255<-edgeTh) //|| sum*255>edgeTh)
    {
        for( i=0; i<9; i++ )
        {
            vec4 tmp = texture2D(tex, gl_TexCoord[0].st + offset[i]);
            lum = tmp.w;
            sum_h += lum * kernel_h[i];
            sum_v += lum * kernel_v[i];
        }
        if ( sum_h*sum_h+sum_v*sum_v >0)
        {
            ori = -atan(sum_v/sum_h);
            col = ori/PI+0.5;
        }
        else 
        {
            col = 0.39215;
        }
    }
    else 
    {
        col = 0.39215;
    }
    gl_FragColor = vec4(col,col,col,1);
}

