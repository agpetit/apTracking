function [ BH, FH ] = postProcessColorHist( BH, FH )
% one improvement over the original approach: use Gaussian to smooth the
% color histogram
%tic;    %time takes to run this program is about 0.019 seconds
    %smoothing
    %try turn smoothing off
%if (0)
    sig=0.8;
    X=-3:3;
    G=exp(-0.5*X.^2/sig^2);
    Gx=G/sum(G);
    Gy=Gx';
    
    BH=conv2(BH, double(Gx), 'same');
    BH=conv2(BH, double(Gy), 'same');
    FH=conv2(FH, double(Gx), 'same');
    FH=conv2(FH, double(Gy), 'same');
    
%end
    %normalization
    %ss = sum(BH(:)) + sum(FH(:));
    %BH = BH/ss;
    %FH = FH/ss;
    BH=BH/sum(BH(:));
    FH=FH/sum(FH(:));
    %set a minimum value to avoid zeros
    BH(BH == 0) = eps;
    FH(FH == 0) = eps;
    %figure, surfc(FH);
    %hold on;
    %surfc(BH);
%toc;
end

