function [ BH, FH ] = updateColorHist( BH, FH, img, mask, numOfHistPerAxis)
%   update the accumulative color histogram
%   we do not update all bins, but only the new bins (a bin
%   that has a value of 0)
    
%tic;    %the time takes to run this program is 0.15 seconds
    %   find the vertices that belong to foreground and background
    fgIndices = find(mask==1);
    bgIndices = find(mask==0);
    
    img=rgb2yuv(img);
    
    %   bin the image
    uimg=img(:,:,1);
    vimg=img(:,:,2)*0.9 + img(:,:,3)*0.1;
    
    BD=double(uimg(bgIndices));
    BD=[BD,double(vimg(bgIndices))];
    
    FD=double(uimg(fgIndices));
    FD=[FD,double(vimg(fgIndices))];
    
    %build new histogram
    histStep = idivide(255, numOfHistPerAxis);
    edges=cell(1,2);
    upperBound = uint16(255)+uint16(histStep-1);
    edges{1}=[uint16(0):uint16(histStep):upperBound];
    edges{2}=[uint16(0):uint16(histStep):upperBound];
    BHt=hist3(BD, 'Edges', edges);
    FHt=hist3(FD, 'Edges', edges);
    
    %update the original histogram
    bUpdateIndices=find(BH==0);
    fUpdateIndices=find(FH==0);
    
    BH(bUpdateIndices)=BHt(bUpdateIndices);
    FH(fUpdateIndices)=FHt(fUpdateIndices);

%toc;
end

