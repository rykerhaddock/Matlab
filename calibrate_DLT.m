function [L, R] = calibrate_DLT(left_img, right_img, num_points)
    L_img=imread(left_img);
    R_img=imread(right_img);
    FL = [];
    gL = [];
    FR = [];
    gR = [];
    for n=1:num_points
        name = ['Calibration Point:  ', num2str(n), '/', num2str(num_points)];
        figure('Name',name,'NumberTitle','off')
        imshow(L_img)
        xlabel('Select the same calibration point for both images.')
        [uL,vL] = ginput(1);
        imshow(R_img)
        xlabel('Select the same calibration point for both images.')
        [uR,vR] = ginput(1);
        close all
        xyz = input('Enter "[X Y Z]"  : ');
        FL = [FL; xyz 1 0 0 0 0 -uL*xyz(1) -uL*xyz(2) -uL*xyz(3); ...
            0 0 0 0 xyz 1 -vL*xyz(1) -vL*xyz(2) -vL*xyz(3)];
        FR = [FR; xyz 1 0 0 0 0 -uR*xyz(1) -uR*xyz(2) -uR*xyz(3); ...
            0 0 0 0 xyz 1 -vR*xyz(1) -vR*xyz(2) -vR*xyz(3)];
        gL = [gL; uL; vL];
        gR = [gR; uR; vR];
    end
    L = inv(transpose(FL)*FL)*transpose(FL)*gL;
    R = inv(transpose(FR)*FR)*transpose(FR)*gR;
end