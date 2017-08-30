//define KERNEL_SIZE 9
uniform sampler2D tex;
ivec2 image_size = textureSize(tex, 0);

const float kernel_g[25] = {
2,4,5,4,2,
4,9,12,9,4,
5,12,15,12,5,
4,9,12,9,4,
2,4,5,4,2
};

/*const float kernel_g[9] = {
1,2,1,
2,4,2,
1,2,1
};*/

//const float step_w = 1.0/image_size[0];
//const float step_h = 1.0/image_size[1];
const float step_w = 1.0/512;
const float step_h = 1.0/512;
const vec2 offset[25]={vec2(-2*step_w,-2*step_h), vec2(-step_w,-2*step_h), vec2(0,-2*step_h), vec2(step_w,-2*step_h), vec2(2*step_w,-2*step_h),
vec2(-2*step_w,-step_h), vec2(-step_w, -step_h), vec2(0.0, -step_h), vec2(step_w, -step_h), vec2(2*step_w,-step_h), 
vec2(-2*step_w,0.0), vec2(-step_w, 0.0), vec2(0.0, 0.0), vec2(step_w, 0.0), vec2(2*step_w,0.0),  
vec2(-2*step_w, step_h), vec2(-step_w, step_h), vec2(0.0, step_h), vec2(step_w, step_h), vec2(2*step_w, step_h),
vec2(-2*step_w, 2*step_h), vec2(-step_w, 2*step_h), vec2(0.0, 2*step_h), vec2(step_w, 2*step_h), vec2(2*step_w, 2*step_h)
};

/*const vec2 offset[9]={
vec2(-step_w, -step_h), vec2(0.0, -step_h), vec2(step_w, -step_h),
vec2(-step_w, 0.0), vec2(0.0, 0.0), vec2(step_w, 0.0),
vec2(-step_w, step_h), vec2(0.0, step_h), vec2(step_w, step_h)
};*/


void main()
{

int i = 0;
float sum = 0.0;
vec4 two = vec4(2.0);
float lum = 0.0;
float sum_g = 0.0;

vec4 tmp0 = texture2D(tex, gl_TexCoord[0].st);
for( i=0; i<25; i++ )
//for( i=0; i<9; i++ )
{
vec4 tmp = texture2D(tex, gl_TexCoord[0].st + offset[i]);
lum = 0.6*tmp.x + 0.3*tmp.y + 0.1*tmp.z;

sum_g += lum * kernel_g[i];
//sum_g/=159;

}

gl_FragColor = vec4(sum_g/159,sum_g/159,sum_g/159,tmp0.w);
//gl_FragColor = vec4(sum_g/16,sum_g/16,sum_g/16,tmp0.w);
//gl_FragColor = vec4(tmp0.w,tmp0.w,tmp0.w,tmp0.w);
//gl_FragColor = vec4(sum_g,sum_g,sum_g,tmp0.w);
//gl_FragColor = vec4(0.5,0.5,0.5,1);
//gl_FragColor = sum ;
}










