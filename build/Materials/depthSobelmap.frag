//define KERNEL_SIZE 9
uniform sampler2D tex;
//ivec2 image_size = textureSize(tex, 0);
const float kernel[9] = {
0, 1, 0,
1,-4, 1,
0, 1, 0
};


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
vec4 sum = vec4(0.0);
//vec4 two = vec4(2.0);

for( i=0; i<9; i++ )
{
vec4 tmp = texture2D(tex, gl_TexCoord[0].st + offset[i]);
sum += tmp * kernel[i];
//sum /=16;

}
gl_FragColor = sum ;
//gl_FragColor = vec4(0.5,0.5,0.5,1);
}

