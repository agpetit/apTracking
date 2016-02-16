% This function takes an input image, allows user to select the region of
% foreground. 
% It returns the mask as output.

function handmask = drawmask(img)
    figure, image(img);
    title('Please input the number of seperate objects in the foreground'); 
    %select the foreground. When finish, press any key to continue.');
    no_f = input('Input the number of seperate objects in the foreground: ');
    close;

    mask = cell(1, no_f);
    handmask = zeros(size(img, 1), size(img, 2));
    temp = img;

    display('Please select the objects in the foreground one by one');
    display('Pressing Backspace or Delete removes the previously selected vertex.');
    display('A shift-click, right-click, or double-click adds a final vertex to the selection and then starts the fill;');
    display('pressing Return finishes the selection without adding a vertex.');

    for i = 1 : no_f
        mask{i} = roipoly(temp);
        mask{i} = uint8(mask{i});

        handmask = uint8(or(handmask, mask{i}));

        %Assume img is in RGB color space
        temp(:, :, 1) = img(:, :, 1) .* (1 - handmask);
        temp(:, :, 2) = img(:, :, 2) .* (1 - handmask);
        temp(:, :, 3) = img(:, :, 3) .* (1 - handmask);

    end

    temp(:, :, 1) = img(:, :, 1) .* handmask;
    temp(:, :, 2) = img(:, :, 2) .* handmask;
    temp(:, :, 3) = img(:, :, 3) .* handmask;
   
    image(temp);
    title('Press any key to continue');

    pause;
    close;
    drawnow;
end