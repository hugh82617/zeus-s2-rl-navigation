clear
clc
close
 
rosinit('192.168.1.102',11311);
 
sub = rossubscriber('/scan','sensor_msgs/LaserScan');
scanMsg = receive(sub);
ranges = scanMsg.Ranges(:);
angles = readScanAngles(scanMsg);
scan = lidarScan(ranges,angles);
plot(scan)
rosshutdown;