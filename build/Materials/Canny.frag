//define KERNEL_SIZE 9
uniform sampler2D tex;
//uniform sampler2D tex1;
//ivec2 image_size = textureSize(tex, 0);
/*const float kernel[9] = {
0, 1, 0,
1,-4, 1,
0, 1, 0
};*/

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

//const float step_w = 1.0/image_size[0];
//const float step_h = 1.0/image_size[1];
const float step_w = 1.0/640;
const float step_h = 1.0/480;
const vec2 offset[9]={
vec2(-step_w, -step_h), vec2(0.0, -step_h), vec2(step_w, -step_h),
vec2(-step_w, 0.0), vec2(0.0, 0.0), vec2(step_w, 0.0),
vec2(-step_w, step_h), vec2(0.0, step_h), vec2(step_w, step_h)
};


void main()
{

int i = 0;
float sum = 0.0;
vec4 two = vec4(2.0);
float lum = 0.0;
float sum_h = 0;
float sum_v = 0;
//float ori = 0.0;
float mag = 0.0;
//float col = 0.0;
//float edge =0;
float PI = 3.14159265358979323846264;

vec4 tmp0 = texture2D(tex, gl_TexCoord[0].st);
//vec4 tmp01 = texture2D(tex1, gl_TexCoord[0].st);

for( i=0; i<9; i++ )
{
vec4 tmp = texture2D(tex, gl_TexCoord[0].st + offset[i]);

lum = 0.3*tmp.x + 0.3*tmp.y + 0.3*tmp.z;
sum_h += lum * kernel_h[i];
sum_v += lum * kernel_v[i];
}
if ( sum_h*sum_h+sum_v*sum_v >0.00)
{
mag = 0.5*sqrt(sum_h*sum_h + sum_v*sum_v);
//ori = atan(sum_v/sum_h);
//col = ori/PI+0.5;
//col = lum;
}
/*if (mag > 1)
{
mag=0;
}*/
/*else 
{
col = 0.39215;
//col = lum;
}*/

//gl_FragColor = vec4(mag,mag,mag,1);
gl_FragColor = vec4(mag,mag,mag,1);

}










