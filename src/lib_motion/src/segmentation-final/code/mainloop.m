% Real-Time Video Matting Based on Bilayer Segmentation -- Viect-Quoc Pham
% etc.
% Bilayer Segmentation: Liu Feipeng @ Apr 2011
% 
% This implementation is mainly written in Matlab with certain processing
% done in C/C++. In particular, it uses OpenCV library for finding good
% features to track, and optical flow. It also uses MRF library for energy
% minimization with GraphCut. 

% mainloop.m contains the code that glues everything together. 

inputVideoFile = '../test4/spotapproche.avi';
%inputVideoFile = '../test2/test2.avi';
%inputVideoFile = '../test3/test3.avi';

inputVideo = aviread(inputVideoFile);
numOfFrames = size(inputVideo, 2);
%numOfFrames = inputVideo.NumberOfFrames;
%%%%define the starting and ending frames for this processing
startFrameNum = 1;
endFrameNum = numOfFrames;      %or 70 if it's test video 2

%%%%%%%%%%% Input the mask for first frame
%Method 1: hand draw the mask to build bg & fg color models 
%startFrame = read(inputVideo, startFrameNum);
%startMask = drawmask(startFrame);       

%Method 2: Use Lazy Snapping to obtain a mask
%Lazy Snapping is implemented in another software. The code block below is
%just to read the Lazy Snapping result and output a mask
%uncomment and change the mask path if you want to use this method
% smaski = imread(strcat('./movieData/test3/image/', num2str(startFrameNum), '.bmp'));
% figure, imshow(smaski);
% [m, n, d] = size(smaski);
% startMask=uint8(zeros(m,n));
% for i=1:1:m
%     for j=1:1:n
%        if (smaski(i, j, 1)==0 && smaski(i, j, 2)==0 && smaski(i, j, 3)==255)
%            startMask(i, j) = 0;
%        else
%            startMask(i, j) = 1;
%        end
%     end
% end
% %save('firstmask31.mat', 'startMask');
% imwrite(startMask*255, strcat('.\MovieData\test3\mask\', num2str(startFrameNum), '.bmp'),'bmp');

%%%%%%%%%% reading mask for first frame %%%%%%%%%%%%%%%
%%% The first mask has been saved, so here we just read it from the given
%%% path. Make sure hte path is correct
%%% In case you want to start with some other frame instead of first one,
%%% you can do so by changing the startFrameNum
%%% Note that for test2 as there's a big jump in frame 71 to 72. The
%%% segmentation process should be run twice, 1~70, 71~end
startMask = imread(strcat('../test4/mask/', num2str(startFrameNum), '.bmp'));
%figure, imshow(smaski);
startMask(startMask==0) = 0;
startMask(startMask~=0) = 1;
%figure,imshow(startMask, []);

epsilon = 0.01;   %the threshold for Iprob. [epsilon~1-epsilon] will be unknown
nu = 0.1;         %the weighting coefficient for second term of data cost
weightV = 10;     %the weighting coefficient for third term of data cost

M=computeSmooth(startMask); %smooth the first mask before processing 
%figure, imshow(M, []);
%create the color histogram
%improvement over original paper: 
%1. the paper proposed 3-D histogram across RGB
%by experiment, it is found that if we work on YUV space, U and V color
%components can be blended to reduce the histogram to 2-D
%this improves the processing speed and reduce memory usage
%2. we smooth the color histogram to take care of similar colors, done in
%postProcessColorHist
%firstFrame = read(inputVideo, startFrameNum);
firstFrame = frame2im(targetvideo(1));
[m,n,d]=size(firstFrame);
%the resolution of the color histogram
%numOfHistPerAxis = uint8(85); 
numOfHistPerAxis = uint8(30);
histStep = idivide(255, numOfHistPerAxis);
[BH, FH]=createColorHist(firstFrame, startMask, numOfHistPerAxis);   %create the color histogram 
[BHt, FHt]=postProcessColorHist(BH, FH);    

stepSize=1;                     
frame = cell(1, numOfFrames);    % store each frame
mask = cell(1, numOfFrames);
mask{startFrameNum} = startMask;

prevHoleMask = startMask;       % improvement over original paper: 
                                % store the previous mask, this mask is used to padding the frame edges and apply hold filling

frame{startFrameNum} = firstFrame;
%figure, imshow(firstFrame);
tempFrame(:,:,1) = firstFrame(:,:,1).*startMask;
tempFrame(:,:,2) = firstFrame(:,:,2).*startMask;
tempFrame(:,:,3) = firstFrame(:,:,3).*startMask;
%figure, imshow(tempFrame);

prevUMask = uint8(ones(m, n));  % the unknown region mask, used for OpenCV color likelihood calculation (only for unknown area)
for frameno = startFrameNum+1: stepSize: numOfFrames
    tic;
    %calculate If 
    frame{frameno} = read(inputVideo, frameno);
    %figure, imshow(frame{frameno});
    
    %get the color likelihood for foreground pixels and background pixels:
    %for color likelihood propagation usage
    [LFPrev, LBPrev] = ColorLikelihoodProp(frame{frameno-1}, mask{frameno-1});
    %get the RGB values before conversion to YUV
    R = frame{frameno}(:,:,1);
    G = frame{frameno}(:,:,2);
    B = frame{frameno}(:,:,3);
    
    frame{frameno} = rgb2yuv(frame{frameno});
    %get YUV value for getting the P(C|B) and P(C|F) values from the color
    %histogram
    Y = frame{frameno}(:,:,1);
    U = frame{frameno}(:,:,2);
    V = frame{frameno}(:,:,3);
    
    %get the index for each pixel in the color histogram
    %lindex=uint32(uint16(idivide(U*0.9+V*0.1, histStep)-1)*86+uint16(idivide(Y, histStep))+uint16(1));
    lindex=uint32(uint16(idivide(U*0.9+V*0.1, histStep)-1)*33+uint16(idivide(Y, histStep))+uint16(1));
   
    Pf = FHt(lindex);   %Pf1 = FHt(lindex2);
    Pb = BHt(lindex);   %Pb1 = BHt(lindex2);
    If=(Pf.*M)./(Pf.*M+Pb.*(-M+1));
    Ib=(Pb.*(1-M))./(Pf.*M+Pb.*(-M+1));
    %Ib=1.0-If;
    If(If==0) = eps;
    Ib(Ib<=0) = eps;
    %figure, imshow(Pf, []);
    %figure, imshow(Pb, []);
    %figure, imshow(M, []);
    %figure,imshow(If, []);
    %figure,imshow(Ib, []);
    %%%%create a trimap with foreground, background and unknown
    mask{frameno}=uint8(ones(m,n));
    mask{frameno}=mask{frameno}*100;
    mask{frameno}(If<epsilon)=0;      %definitely background 
    mask{frameno}(If>1-epsilon)=255;  %definitely foreground
    %%%%create a mask for unknown region
    UMask = mask{frameno};
    UMask(UMask==255) = 0;
    UMask(UMask==100) = 1;
    %figure, imshow(UMask, []);
    %imwrite(UMask*255, strcat('.\MovieData\test3\unknown\', num2str(frameno), '.bmp'),'bmp');
    %figure,imshow(mask{frameno},[]);
    
    %%%%%%calculate the feature point for color likelihood propagation
    %feature point is obtained based on previous frame and optical flow
    %between previous frame and current frame
    openCVMask = reshape(prevUMask', 1, m*n);
    [resPrevX, resPrevY, resCurX, resCurY, numOfFeatures] = openCVColorLikelihoodProp(m, n, frameno-1, openCVMask, inputVideoFile);
    prevUMask = UMask;
    %openCV point is 0-based: we change it to 1 based
    resPrevX = resPrevX + 1;
    resPrevY = resPrevY + 1;
    resCurX = resCurX + 1;
    resCurY = resCurY + 1;
    %featureMask = openCVColorLikelihoodProp(m, n, frameno-1);
    %featureMask = reshape(featureMask, m, n);
    %figure, imshow(featureMask, []);
    
    %get the feature points energy
    LFCurFeature = zeros(m, n);
    LBCurFeature = zeros(m, n);
    for i=1:1:numOfFeatures
        if ((resCurY(1, i) > m) || (resCurX(1, i) > n)) %running out of frame boundary, ignore
            continue;
        end
        LFCurFeature(resCurY(1, i), resCurX(1, i)) = log(LFPrev(resPrevY(1, i), resPrevX(1, i)));
        LBCurFeature(resCurY(1, i), resCurX(1, i)) = log(LBPrev(resPrevY(1, i), resPrevX(1, i)));
    end
    LFCurFeature = LFCurFeature.*logical(mask{frameno});
    LBCurFeature = LBCurFeature.*logical(1-mask{frameno});
    
    %perform energy calculation on the unknown region
    %figure, imshow(LF, []);
    %figure, imshow(LB, []);
    
    %DEf = -1*log(Pf) + (-1*nu*log(If)) + weightV.*LF;
    %DEb = -1*log(Pb) + (-1*nu*log(If)) + weightV.*LB;
    %%%%%%%%% get the data cost for foreground %%%%%%%%%%%%%%%
    DEf1 = -1*log(Pf);
    DEf2 = -1*nu*log(If);
    DEf3 = weightV.*LFCurFeature;
    %DEf = DEf1 + DEf2 + DEf3;
    DEf = DEf1 + DEf2;
    
    %for debug
    %minDEf = min(DEf(:));
    %IDEf = DEf + (0-minDEf);
    %IDEf = double(UMask).*IDEf;
    %figure, imshow(uint8(IDEf), []);
    %DEf = (DEf1 + DEf2 + DEf3);
    %if (frameno > 0)
        %IDEf = double(UMask).*DEf;
        %figure, surfc(DEf3);
        %IDEf = double(UMask).*DEf3;
        %figure, surfc(IDEf3);
        %figure, surfc(IDEf1);
        %figure, surfc(IDEf2);
        %figure, surfc(IDEf);
    %end
    %%%%%%%%%%%%%%%% get the data cost for background %%%%%%%%%%%%%%%%%%%%%
    DEb1 = -1*log(Pb);
    DEb2 = -1*nu*log(Ib); %If is calculated for foreground, here we should use 1-If
    DEb3 = weightV.*LBCurFeature;
    %DEb = DEb1 + DEb2 + DEb3;
    DEb = DEb1 + DEb2;
    %for debug
    %minDEb = min(DEb(:));
    %IDEb = DEf + (0-minDEb);
    %IDEb = double(UMask).*IDEb;
    %figure, imshow(uint8(IDEb), []);
    %DEb = (DEb1 + DEb2 + DEb3);
    %if (frameno > 0)
        %igure, surfc(DEb3);
        %IDEb = double(UMask).*DEb3;
        %figure, surfc(DEb3);
        %figure, surfc(DEb1);
        %figure, surfc(DEb2);
        %figure, surfc(IDEb);
    %end
    %IDE = DEf-DEb;
    %IDEf = double(UMask).*IDE;
    %figure, imshow(IDEf);
    %figure, surfc(DEf+DEb);
    
    %figure, imshow(yuv2rgb(frame{frameno}));
    %imask = reshape(mask{frameno}, 1, m*n);
    imask = mask{frameno};
    %imask(1,:)=2;
    %maskU=minE(DEf, DEb, Y, U, V, imask, m, n);
    maskU=minE(DEf, DEb, R, G, B, imask, m, n);
    imask(maskU==true) = 1;
    imask(maskU==false) = 0;
    %test2 = find(imask == 1);
    %imask = reshape(imask, m, n);
    %mask{frameno} = imask;
    %figure, imshow(mask{frameno}, []);
    %padding the last row with previous segmentation result
    %allF = uint8(ones(2, n));
    holeMask = vertcat(imask, prevHoleMask(m,:));
    holeMask = vertcat(holeMask, mask{startFrameNum}(m,:));
    %holeMask = vertcat(holeMask, allF);
    %figure, imshow(holeMask, []);
    holeMask = imfill(holeMask, 'holes');
    holeMask = holeMask(1:m,:);
    %correct the isolated error foreground by looking for discontinuous
    %region: using average filter
    H = fspecial('average', [10, 10]);
    holeMask = imfilter(holeMask, H, 'replicate', 'same');
    
    prevHoleMask = holeMask;
    mask{frameno} = holeMask;
    %figure, imshow(mask{frameno}, []);
    %mask{frameno} = fillHoles(mask{frameno});
    %figure, imshow(mask{frameno}, []);
    %%%%%%%%%%%%%%%%%save the mask
    imwrite(holeMask*255, strcat('..\test1\mask\', num2str(frameno), '.bmp'),'bmp');
    %imshow(holeMask*255);
    
    tempFrame(:,:,1) = frame{frameno}(:,:,1).*holeMask;
    tempFrame(:,:,2) = frame{frameno}(:,:,2).*holeMask;
    tempFrame(:,:,3) = frame{frameno}(:,:,3).*holeMask;
    tempFrame = yuv2rgb(tempFrame);
    %figure, imshow(frame{frameno}, []);        
    imwrite(tempFrame, strcat('..\test1\image\', num2str(frameno), '.bmp'),'bmp');
    %figure, imshow(mask{frameno});
    %update the smoonth and histogram
    updateColorHist(BH, FH, frame{frameno}, mask{frameno}, numOfHistPerAxis);
    M=computeSmooth(mask{frameno});
    [BHt, FHt]=postProcessColorHist(BH, FH);
    toc;
end




















