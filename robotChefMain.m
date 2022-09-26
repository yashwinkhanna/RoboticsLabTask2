%%LAB TASK 2 - PANCAKE CHEF - MAIN

%% Basic Environment & Device Generation

clf;
clear all;

set(0,'DefaultFigureWindowStyle','docked') %Docks figure in matlab window

hold on

%Concrete floor
surf([-1.8,-1.8;1.8,1.8],[-1.8,1.8;-1.8,1.8],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

%Create devices
robotChef = LinearUR3(false); %robotChef - resp for actions of cooking and serving pancake
robotDispenser = 0; %robotDispenser - resp. for pouring liquids (batter, syrup)