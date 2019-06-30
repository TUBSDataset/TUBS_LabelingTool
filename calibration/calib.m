clc; clear all; close all;    
addpath('..\classes');  
addpath('..\graphics');
% ---------------------------------------------------------------------------------------------
% This script loads point correspondeces generated with the editor's calibration mode. 
% In order to generate those correspondences, follow the instructions in the manual (see \documentation).
% ---------------------------------------------------------------------------------------------

load('Calibration_Container.mat');
calibrateCamera(clPointContainer, voPCMovableLabel, oCalib, oAxes_h);