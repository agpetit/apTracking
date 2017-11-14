function yuv = rgb2yuv(rgb)

% rgb should be a uint8 matrix
% yuv is returned as a uint8 matrix

rgb = double(rgb) / 255;
yuv = colorspace('yuv<-rgb', rgb);
yuv = uint8(yuv * 255);
