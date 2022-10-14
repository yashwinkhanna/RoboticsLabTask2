%%LAB TASK 2 - PANCAKE CHEF - MAIN

%% Basic Environment & Device Generation

clf;
clear all;
close all;

set(0,'DefaultFigureWindowStyle','docked') %Docks figure in matlab window

hold on

%Concrete floor
surf([-2,-2;2,2],[-2,2;-2,2],[-0.7,-0.7;-0.7,-0.7],'CData',imread('concrete.jpg'),'FaceColor','texturemap');    %Loading in concrete floor
hold on;

% PlaceObject('roboticstable.ply', [-0.35,0,-0.0844]);    %Loading in table
% hold on;
% 
% PlaceObject('cooktop.ply', [-0.35,0,-0.0844]);    %Loading in cooktop
% hold on;

IRB_910sc

%Create devices
% robotChef = LinearUR3(false); %robotChef - resp for actions of cooking and serving pancake
%robotDispenser = 0; %robotDispenser - resp. for pouring liquids (batter, syrup)
