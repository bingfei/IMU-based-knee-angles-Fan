
%% Start of script 
addpath('Orient_algorithm');      % include algorithm library
addpath('quaternion_library');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

% the last column is run parameter. 1-3, UpPoseRef 4-6,LowPoseRef
TestData = {    
{
'test_data\MT_2020-07-10_010_00B44910.txt';% drop landing, left thigh 
'test_data\MT_2020-07-10_010_00B4490A.txt';% left shank
'test_data\Lknee trial 271.txt';
[-0.33 2.45 -0.22 -9.88 6.64 6.86] % v3d segment initial angle
}
{
'test_data\MT_2020-07-10_010_00B44910.txt';
'test_data\MT_2020-07-10_010_00B4490A.txt';
'test_data\Lknee trial 271.txt';
[-2.38 2.80 -3.70 -8.47 3.78 2.76] % leave one out mean segment initial angle
}
{
'test_data\MT_2020-07-10_010_00B44910.txt';
'test_data\MT_2020-07-10_010_00B4490A.txt';
'test_data\Lknee trial 271.txt';
[0 0 0 0 0 0] % zero segment initial angle
}
{
'test_data\MT_2020-07-10_015_00B44912.txt'; % cutting, right thigh
'test_data\MT_2020-07-10_015_00B44916.txt'; % right shank
'test_data\Rknee trial 276.txt';
[2.00 -2.36 -0.47 -11.67 -6.96 -6.24] % v3d segment initial angle
}    
{
'test_data\MT_2020-07-10_015_00B44912.txt';
'test_data\MT_2020-07-10_015_00B44916.txt';
'test_data\Rknee trial 276.txt';
[-0.24 -2.13 5.54 -9.35 -2.55 -1.09] % leave one out mean segment initial angle
}
{
'test_data\MT_2020-07-10_015_00B44912.txt';
'test_data\MT_2020-07-10_015_00B44916.txt';
'test_data\Rknee trial 276.txt';
[0 0 0 0 0 0] % zero segment initial angle
}
};

    autofiguredisable = 1; % xsens raw data plot
%     kneefiguredisable = 1; % knee angle related plot.

    testdata = TestData{1}; % change this number to load different data with different segment initial angle..
    Uppath = testdata{1};
    Lowpath = testdata{2};
    V3DKneeAnglePath = testdata{3};
    
    Knee_sign = [1,-1,-1];  %default right knee
    if contains(V3DKneeAnglePath,'Lknee') % if left knee
        Knee_sign = [-1,1,-1];  % internal rotation abduction, flexion.        
    end
    %Determine the sign of three knee angles.      
    
    runpara = testdata{4};
    
    %load parameter
    UpPoseRef = runpara(1:3);
    LowPoseRef = runpara(4:6);
    
    run Joint_angle_calc;
 
%% End of script