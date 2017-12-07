function [X,Y,Z] = colorspace(Str,A,B,C)
%COLORSPACE  Convert a color image between color representations.
%   B = COLORSPACE(S,A) converts the color representation of image A
%   where S is a string specifying the conversion.  S tells the 
%   source and destination color spaces, S = 'dest<-src', or
%   alternatively, S = 'src->dest'.  Supported color spaces are
%
%      'RGB'                 standard red-green-blue
%      'HSV'/'HSB'           hue-saturation-value
%      'HLS'                 hue-luminance-saturation
%      'YIQ'/'NTSC'          NTSC YIQ luminance+chrominance
%      'YUV'/'PAL'           PAL YUV luminance+chrominance
%      'SECAM'               SECAM YUV
%      'CCIR601'             CCIR601 YUV
%
%  When RGB is the source or destination, it can be omitted. For 
%  example 'yuv<-' is short for 'yuv<-rgb'.
%
%  [Bx,By,Bz] = COLORSPACE(S,A) specifies separate output channels.
%  COLORSPACE(S,Ax,Ay,Az) specifies separate input channels.
%
%  Note: Color values outside [0,1] are clipped.  All outputs are
%  in [0,1].
%
%  Example:
%  B = colorspace('yuv<-rgb',A);  % converts RGB to YUV
%  C = colorspace('hsv<-yuv',B);  % converts YUV to HSV
%  R = colorspace('rgb<-hsv',C);  % converts HSV back to RGB
%  psnr(A,R)

% Pascal Getreuer 2005

% input parsing
if nargin < 2, error('Not enough input arguments.'); end
if ~ischar(Str), error('First argument must be a string.'); end

k = findstr(Str,'->');

if length(k) == 1
   Src = Str(1:k-1);
   Dest = Str(k+2:length(Str));
else
   k = findstr(Str,'<-');
   
   if length(k) == 1
	   Dest = Str(1:k-1);
   	Src = Str(k+2:length(Str));
   else
      error(['Error parsing conversion string ''', Str,'''.']);
   end   
end

if nargin < 4
   if size(A,3) ~= 3, error('MxNx3-size array expected.'); end
   
   A = double(A);
   C = A(:,:,3);
   B = A(:,:,2);
   A = A(:,:,1);
elseif size(A) ~= size(B) | size(A) ~= size(C)
   error('Channel sizes must match.');
else
   A = double(A);
   B = double(B);
   C = double(C);
end

% clip color values to [0,1]
if min([min(A(:)),min(B(:)),min(C(:))]) < 0 ...
      | max([max(A(:)),max(B(:)),max(C(:))]) > 1  
   A = min(max(A,0),1);
   B = min(max(B,0),1);
   C = min(max(C,0),1);
   warning('Color values clipped within [0,1].');
end

% get color matrix if possible
[S,soff] = colormat(Src);
[D,doff] = colormat(Dest);

if ischar(S)
   % nonlinear source space   
   switch S
   case 'hsv'     % HSV to RGB
      [A,B,C] = huetorgb((1 - B).*C,C,A);      
   case 'hls'		% HLS to RGB
      d = C.*min(B,1-B);
      [A,B,C] = huetorgb(B-d,B+d,A);
   end
   
   S = eye(3);
   soff = 0;
elseif ischar(D) & any(any(S ~= eye(3)))
   % linear source space, nonlinear destination space
   A = A;
   B = B - soff;
   C = C - soff;
   M = inv(S);
   tmp1 = M(1)*A + M(4)*B + M(7)*C;
   tmp2 = M(2)*A + M(5)*B + M(8)*C;
   C = M(3)*A + M(6)*B + M(9)*C;
   A = tmp1;
   B = tmp2;
end

if ischar(D)
   % nonlinear destination space
   switch D
   case 'hsv'     % RGB to HSV
      Z = max(max(A,B),C);      
      Y = (Z - min(min(A,B),C))./(Z + (Z == 0));
      X = rgbtohue(A,B,C);
   case 'hls'		% RGB to HLS
      m0 = min(min(A,B),C);      
      m2 = max(max(A,B),C);
      Y = 0.5*(m2 + m0);
      d = min(Y,1-Y);
      Z = 0.5*(m2 - m0)./(d + (d == 0));
      X = rgbtohue(A,B,C);      
   end   
   
   X = min(max(X,0),1);
   Y = min(max(Y,0),1);
	Z = min(max(Z,0),1);
   
   if nargout <= 1
      X(:,:,2) = Y;
      X(:,:,3) = Z;
   end
else
   % linear destination space
   A = A;
   B = B - soff;
   C = C - soff;
   
   if nargout <= 1
      M = S\D;	   
      X = M(1)*A + M(4)*B + M(7)*C;
      X(:,:,2) = M(2)*A + M(5)*B + M(8)*C + doff;
      X(:,:,3) = M(3)*A + M(6)*B + M(9)*C + doff;
      X = min(max(X,0),1);
   else
      X = min(max(M(1)*A + M(4)*B + M(7)*C,0),1);
      Y = min(max(M(2)*A + M(5)*B + M(8)*C + doff,0),1);
      Z = min(max(M(3)*A + M(6)*B + M(9)*C + doff,0),1);
   end   
end

return;

function [M,off] = colormat(Name)
% Color transformation matrices
if isempty(Name), Name = 'rgb'; end 
off = 0;

switch strrep(lower(Name),' ','')
case 'rgb'
   M = eye(3);
case {'hsv', 'hsb'}
   M = 'hsv';
case 'hls'
   M = 'hls';
case {'yiq', 'ntsc'}
   M = [0.299,0.587,0.114;0.21389,-0.52243,0.30854;0.59947,-0.27589,-0.32358];
   off = 0.5;
case {'yuv', 'pal'}
   M = [0.299,0.587,0.114;-0.14707,-0.28939,0.4368;0.61478,-0.5148,-0.09998];
   off = 0.5;
case 'secam'
   M = [0.299,0.587,0.114;-0.4485,-0.8805,1.329;-1.3319,1.1153,0.2166];
   off = 0.5;
case 'ccir601'
   M = [0.299,0.587,0.114;-0.169,-0.332,0.5;0.5,-0.419,-0.0813];
   off = 0.5;   
case 'xyz'
   M = [3.240479,-1.53715,-0.498535;-0.969256,1.875992,0.041556;
      0.055648,-0.204043,1.057311]/8.06;
   off = 0.5;
otherwise
   error(['Unknown color space, ''', Name, '''.']);
end
return;


function H = rgbtohue(R,G,B)
% Convert RGB to color hue, H in [0,1]

N = size(R);
R = R(:).';
G = G(:).';
B = B(:).';
M = sort([R;G;B]);
i = M(3,:) - M(2,:) > eps;

k = find(i);
F(k) = ((M(2,k) - M(1,k))./(M(3,k) - M(1,k)))/6;
k = find(i & abs(R-M(3,:)) <= eps);
H(k) = sign(G(k) - B(k)).*F(k);
k = find(i & abs(G-M(3,:)) <= eps);
H(k) = 2/6 + sign(B(k) - R(k)).*F(k);
k = find(i & abs(B-M(3,:)) <= eps);
H(k) = 4/6 + sign(R(k) - G(k)).*F(k);

k = find(~i & abs(R-G) <= eps);
H(k) = 1/6;
k = find(~i & abs(G-B) <= eps);
H(k) = 3/6;
k = find(~i & abs(B-R) <= eps);
H(k) = 5/6;
H = reshape(H + (H < 0),N);

return;


function [R,G,B] = huetorgb(m0,m2,H)
% Convert color hue to RGB 

N = size(H);
H = H(:)*6;
m0 = m0(:);
m2 = m2(:);
k = floor(H);
F = H - round(H/2)*2;
M = [m0, m0 + (m2-m0).*abs(F), m2];
Num = length(m0);
j = [2 1 0;1 2 0;0 2 1;0 1 2;1 0 2;2 0 1;2 1 0]*Num;
k = k + 1;
R = reshape(M(j(k,1)+(1:Num)'),N);
G = reshape(M(j(k,2)+(1:Num)'),N);
B = reshape(M(j(k,3)+(1:Num)'),N);

return;
