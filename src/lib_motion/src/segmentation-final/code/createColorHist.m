function [ BH, FH ] = createColorHist( img, mask, numOfHistPerAxis )
%   this function is called with first frame's image and mask
%   it will create two histograms, both of which will be updated 
%   by another function updateColorHist

%the time to run this program is around 0.12 seconds
%tic;
    %   find the vertices that belong to foreground and background
    fgIndices = find(mask==1);
    bgIndices = find(mask==0);

    %convert the image from rgb to yuv
    img=rgb2yuv(img);

    %   bin the image's color components
    uimg=img(:,:,1);
    vimg=0.9*img(:,:,2) + 0.1*img(:,:,3);
    
    BD=double(uimg(bgIndices));
    BD=[BD,double(vimg(bgIndices))];
    
    FD=double(uimg(fgIndices));
    FD=[FD,double(vimg(fgIndices))];
    
    %build histogram
    histStep = idivide(255, numOfHistPerAxis);
    edges=cell(1,2);
    upperBound = uint16(255)+uint16(histStep-1);
    edges{1}=[uint16(0):uint16(histStep):upperBound];
    edges{2}=[uint16(0):uint16(histStep):upperBound];
    BH=hist3(BD, 'Edges', edges);
    FH=hist3(FD, 'Edges', edges);
    %figure, surfc(BH);
    %figure, surfc(FH);
%toc;
end

