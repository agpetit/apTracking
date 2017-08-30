//define KERNEL_SIZE 9
uniform sampler2D tex;
//uniform float edgeTh;
//uniform sampler2D tex1;
ivec2 image_size = textureSize(tex, 0);
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

//const float step_w = 1.0/image_size[0];
//const float step_h = 1.0/image_size[1];
const float step_w = 1.0/512;
const float step_h = 1.0/512;
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
float ori = 0.0;
float col = 0.0;
float edge =0;
float PI = 3.14159265358979323846264;

vec4 tmp0 = texture2D(tex, gl_TexCoord[0].st);
//vec4 tmp01 = texture2D(tex1, gl_TexCoord[0].st);
//lum = 0.3*tmp0.x + 0.6*tmp0.y + 0.9*tmp0.y;

for( i=0; i<9; i++ )
{
vec4 tmp=texture2D(tex, gl_TexCoord[0].st + offset[i]);
/*vec4 tmp1 = texture2D(tex1, gl_TexCoord[0].st + offset[i]);
if ((tmp1.x*tmp01.x + tmp1.y*tmp01.y + tmp1.z*tmp01.z)/(sqrt(tmp1.x*tmp1.x + tmp1.y*tmp1.y + tmp1.z*tmp1.z)*sqrt(tmp01.x*tmp01.x + tmp01.y*tmp01.y + tmp01.z*tmp01.z)) < 0.99997)
{
edge =1;
}*/
//lum = 0.3*tmp.x + 0.6*tmp.y + 0.9*tmp.y;
sum += (tmp.w) * kernel[i];
//sum += (lum) * kernel[i];
//sum += tmp * kernel[i];
/*lum = 0.3*tmp.x + 0.6*tmp.y + 0.9*tmp.y;
sum_h += lum * kernel_h[i];
sum_v += lum * kernel_v[i];*/

/*sum_h += tmp.x * kernel_h[i];
sum_v += tmp.x * kernel_v[i];*/

}



//if (-(sum*255)>0.2)

//Trakmark
//if (-sum*255>0.2)

//Tank
//if (sum*255>1)

//if (sum*255>0.8)// || sum*255<-2)
//A400M
//if (sum*255>10)// || sum*255<-2)


//spot5
//if (sum*255>0.5)// || sum*255<-0.1)

//if (sum*255>edgeTh)

//atlantis
//if (sum*255>1.2)// || sum*255<-2)

//soyuz
//if (sum*255>5)// || sum*255<-2)

//PA
//if (sum*255>3 )//|| sum*255<-3)

if (sum*255>5 || sum*255<-5)

//Soyuz0
//if (sum*255>1)
//Soyuz1
//if (-sum*255>1)

//A380
//if (sum*255>1)

//if (sum*255>0.05 && (tmp0.x>0 || tmp0.y>0 || tmp0.z>0 ) )
//if (tmp0.w>0.5)
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
if ( sum_h*sum_h+sum_v*sum_v >0)
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

/*if (ori<-3.1415)
gl_FragColor = vec4(0.5,0.5,0.5,1);
else
gl_FragColor = vec4(0,0,0,1);*/
//gl_FragColor = vec4(ori/3.1416+0.5, ori/3.1416+0.5, ori/3.1416+0.5,edge);

gl_FragColor = vec4(col,col,col,1);
//gl_FragColor = vec4(lum,lum,lum,1);

//gl_FragColor = vec4(tmp0.w,tmp0.w,tmp0.w,1);
//gl_FragColor= vec4(gl_FragCoord.z,gl_FragCoord.z,gl_FragCoord.z,1);


//gl_FragColor = vec4(tmp0.w,tmp0.w,tmp0.w,tmp0.w);
//gl_FragColor = vec4(sum,sum,sum,sum);
//gl_FragColor = vec4(edge,edge,edge,1);
//gl_FragColor = vec4(ori, ori, ori,sum);
//gl_FragColor = sum ;
}










