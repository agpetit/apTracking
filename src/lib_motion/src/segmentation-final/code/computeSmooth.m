function [M] = computeSmooth(A)
%   Given the 2-D mask of the previous frame segmentation result A(t-1) at 
%   time t-1, calculate the smoonthed map M(t) from A(t-1) using the
%   formula: M(t)= G(7x7)(Resize(10)(G(3x3)(Resize(1/10)(A))))

%   construct the Gaussian filters
%tic; %the time takes to run this program is about 0.025 seconds
    X=-3:3;
    sig=0.8;
    G33=exp(-0.5*X.^2/sig^sig);
    G33=G33./sum(G33);

    X=-7:7;
    sig = 2;
    G77=exp(-0.5*X.^2/sig^sig);
    G77=G77./sum(G77);

    %   compute the M
    %first padding the boundary, otherwise the default 0 padding will
    %cause error
    paddingSize = 20;
    B = padarray(A, [paddingSize, paddingSize],'replicate');
    B=double(B*255);
    M=conv2(double(0.1*B), double(G33), 'same');
    M=conv2(double(M), double(G33'), 'same');
    M=conv2(double(10*M), double(G77), 'same');
    M=conv2(double(M), double(G77'), 'same');
    M=M(paddingSize+1:end-paddingSize, paddingSize+1:end-paddingSize);
    %   normalization
    M=M/255;
%toc;
%   for testing
    %M=M.*255;
    %figure, imshow(M, []);
end

