function [ LF, LB ] = ColorLikelihoodProp( frame, mask )
% color likelihood for propagation: the input frame should be in RGB color space
% the gaussian range is not mentioned in paper
    %X=-3:3;
    sig=8;
    %G33=exp(-0.5*X.^2/sig^sig);
    %G33=G33./sum(G33);
    h = fspecial('gaussian', [10, 10], sig);
    %h = h/(sum(h(:)));
    %figure, surfc(h);
    wi = 1;
    R = frame(:, :, 1).*mask;
    G = frame(:, :, 2).*mask;
    B = frame(:, :, 3).*mask;
    %calculate the mean color 
    meanR = imfilter(R, h, 'same', 'conv')*(1/wi);
    meanG = imfilter(G, h, 'same', 'conv')*(1/wi);
    meanB = imfilter(B, h, 'same', 'conv')*(1/wi);
    %calculate the variance 
    varR = imfilter(double((R-meanR).^2), h, 'same', 'conv')*(1/wi);
    varG = imfilter(double((G-meanG).^2), h, 'same', 'conv')*(1/wi);
    varB = imfilter(double((B-meanB).^2), h, 'same', 'conv')*(1/wi);
    %prevent inf value
    MAX = 10000000;
    LF = (double(varR.^2+varG.^2+varB.^2+0.000001));
    LF(LF > MAX) = MAX;
    LF = 0.5*double(((R-meanR).^2 + (G-meanG).^2 + (B-meanB).^2))./(LF);
    LF = LF/max(LF(:));
    LF(LF == 0) = eps;
        
    R = frame(:, :, 1).*(1-mask);
    G = frame(:, :, 2).*(1-mask);
    B = frame(:, :, 3).*(1-mask);
    %calculate the mean color 
    meanR = imfilter(R, h, 'same', 'conv')*(1/wi);
    meanG = imfilter(G, h, 'same', 'conv')*(1/wi);
    meanB = imfilter(B, h, 'same', 'conv')*(1/wi);
    %calculate the variance 
    varR = imfilter(double((R-meanR).^2), h, 'same', 'conv')*(1/wi);
    varG = imfilter(double((G-meanG).^2), h, 'same', 'conv')*(1/wi);
    varB = imfilter(double((B-meanB).^2), h, 'same', 'conv')*(1/wi);
    
    LB = (double(varR.^2+varG.^2+varB.^2+0.000001));
    LB(LB > MAX) = MAX;
    LB = 0.5*double(((R-meanR).^2 + (G-meanG).^2 + (B-meanB).^2))./(LB);
    LB = LB/max(LB(:));
    LB(LB == 0) = eps;
end

