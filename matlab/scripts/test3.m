clear
clc
close
 
rosinit('192.168.1.102',11311);
 
sub = rossubscriber('/map','nav_msgs/OccupancyGrid');
mapMsg = receive(sub);
occGrid = readOccupancyGrid(mapMsg);
show(occGrid);
rosshutdown;