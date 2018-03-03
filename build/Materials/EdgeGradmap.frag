//define KERNEL_SIZE 9
uniform sampler2D tex;
//uniform float edgeTh;
//uniform sampler2D tex1;
// ivec2 image_size = textureSize(tex, 0);

// const float kernel_h[9] = {
// -1.0, 0.0, 1.0,
// -2.0, 0.0, 2.0,
// -1.0, 0.0, 1.0
// };

// const float kernel_v[9] = {
// -1.0, -2.0, -1.0,
// 0.0, 0.0, 0.0,
// 1.0, 2.0, 1.0
// };

const float step_w = 1.0/512.0;
const float step_h = 1.0/512.0;

// const vec2 offset[9]={
// 	vec2(-step_w, -step_h), vec2(0.0, -step_h), vec2(step_w, -step_h),
// 	vec2(-step_w, 0.0), vec2(0.0, 0.0), vec2(step_w, 0.0),
// 	vec2(-step_w, step_h), vec2(0.0, step_h), vec2(step_w, step_h)
// };


void main()
{

	float kernel[9];
	kernel[0] = kernel[2] = kernel[6] = kernel[8] = 0.0;
	kernel[1] = kernel[3] = kernel[5] = kernel[7] = 1.0;
	kernel[4] = -4.0;
	float kernel_h[9];
	kernel_h[1] = kernel_h[4] = kernel_h[7] = 0.0;
	kernel_h[0] = kernel_h[6] = -1.0;
	kernel_h[2] = kernel_h[8] = -1.0;
	kernel_h[3] = -2.0;
	kernel_h[5] =  2.0;

	float kernel_v[9];
	kernel_h[3] = kernel_h[4] = kernel_h[5] = 0.0;
	kernel_h[0] = kernel_h[2] = -1.0;
	kernel_h[6] = kernel_h[8] =  1.0;
	kernel_h[7] =  2.0;
	kernel_h[1] = -2.0;

	vec2 offset[9];
	offset[0] = vec2(-step_w, -step_h);
	offset[1] = vec2(0.0, -step_h);
	offset[2] = vec2(step_w, -step_h),
	offset[3] = vec2(-step_w, 0.0);
	offset[4] = vec2(0.0, 0.0);
	offset[5] = vec2(step_w, 0.0);
	offset[6] = vec2(-step_w, step_h);
	offset[7] = vec2(0.0, step_h);
	offset[8] = vec2(step_w, step_h);

	int i = 0;
	float sum = 0.0;
	vec4 two = vec4(2.0);
	float lum = 0.0;
	float sum_h = 0.0;
	float sum_v = 0.0;
	float ori = 0.0;
	float col = 0.0;
	float edge =0.0;
	float PI = 3.14159265358979323846264;

	vec4 tmp0 = texture2D(tex, gl_TexCoord[0].st);

	for( i=0; i<9; i++ )
	{
		vec4 tmp=texture2D(tex, gl_TexCoord[0].st + offset[i]);

		sum += (tmp.w) * kernel[i];

	}



	if (sum*255.0>20.0 || sum*255.0<-20.0)

	{
	for( i=0; i<9; i++ )
	{
		vec4 tmp = texture2D(tex, gl_TexCoord[0].st + offset[i]);

		//lum = 0.6*tmp.x + 0.3*tmp.y + 0.1*tmp.z;
		lum = tmp.w;
		sum_h += lum * kernel_h[i];
		sum_v += lum * kernel_v[i];
	}
	//edge = 1;

	//if ( (!sum_h==0 || !sum_v==0) && sum_h*sum_h+sum_v*sum_v >0)
	if ( sum_h*sum_h+sum_v*sum_v >0.0)
	{
		ori = atan(sum_v/sum_h);
		col = ori/PI+0.5;
		//col = lum;
	}
	else 
	{
		col = 0.39215;
		//col = lum;
	}

	//gl_FragColor = vec4(ori/3.1416+0.5);}
	}
	else 
	{
		col = 0.39215;
	}

	gl_FragColor = vec4(col,col,col,1.0);
}










