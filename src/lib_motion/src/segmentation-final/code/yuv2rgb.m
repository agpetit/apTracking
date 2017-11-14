function rgb = yuv2rgb(yuv)

% yuv must be a uint8 matrix
% rgb is returned as a uint8 matrix

yuv = double(yuv) / 255;
rgb = colorspace('rgb<-yuv', yuv);
rgb = uint8(rgb * 255);
