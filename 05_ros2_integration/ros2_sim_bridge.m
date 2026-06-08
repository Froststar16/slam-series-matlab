%% ros2_sim_bridge.m  —  Publish MATLAB simulation to ROS 2
%  
%
%  Runs the MATLAB EKF SLAM simulation and publishes synthetic
%  /scan and /odom topics so ekf_slam_node.py can subscribe to them.
%  This lets you test the full ROS 2 pipeline without a real robot.
%
%  REQUIREMENTS:
%    - MATLAB R2024a with ROS Toolbox
%    - ROS 2 Humble or Jazzy running on the same machine (or network)
%    - ekf_slam_node.py already running in a ROS 2 terminal
%
%  SETUP:
%    1. In terminal A:  source ~/ros2_ws/install/setup.bash
%                       ros2 run ekf_slam_py ekf_slam_node
%    2. In terminal B:  rviz2  (add /slam/path, /slam/landmarks topics)
%    3. In MATLAB:      ros2_sim_bridge

clear; clc;
rng(42);

%% ── ROS 2 node setup ─────────────────────────────────────────────────────
ros2_domain_id = 0;   % match ROS_DOMAIN_ID in your terminals
setenv('ROS_DOMAIN_ID', num2str(ros2_domain_id));

node = ros2node('/matlab_sim_bridge');
odom_pub = ros2publisher(node, '/odom',  'nav_msgs/Odometry');
scan_pub = ros2publisher(node, '/scan',  'sensor_msgs/LaserScan');

fprintf('ROS 2 bridge started. Publishing to /odom and /scan...\n');
fprintf('Check: ros2 topic list | grep -E "odom|scan"\n\n');

%% ── Simulation parameters ────────────────────────────────────────────────
DT=0.1; N_STEPS=300; LIDAR_RANGE=3.5; WORLD_SIZE=9;
SIG_V=0.08; SIG_W=0.04; SIG_R=0.06; SIG_PHI=0.03;
N_BEAMS=360;

true_lm=[2,2;4,1;5,3;3,5;1,4;6,2;7,4;5,6;2,7;7,7;0.5,6;4,8;8,1;8,5;1,1];
walls=[1.5,1.5,1.5,4.0; 3.0,3.0,6.0,3.0; 6.0,5.0,6.0,8.0];
waypts=[1,1;7,1;7,8;1,8;1,1;4,4;7,1;1,8;7,8;1,1];
wp_idx=1;
ts=[0.5;0.5;pi/4];

%% ── Build static LaserScan message template ──────────────────────────────
beam_angles=linspace(-pi, pi, N_BEAMS+1); beam_angles(end)=[];

scan_msg=ros2message('sensor_msgs/LaserScan');
scan_msg.header.frame_id='base_scan';
scan_msg.angle_min=single(-pi);
scan_msg.angle_max=single(pi);
scan_msg.angle_increment=single(2*pi/N_BEAMS);
scan_msg.range_min=single(0.05);
scan_msg.range_max=single(LIDAR_RANGE);
scan_msg.ranges=single(zeros(1,N_BEAMS));

odom_msg=ros2message('nav_msgs/Odometry');
odom_msg.header.frame_id='odom';
odom_msg.child_frame_id='base_link';

%% ── Publish loop ─────────────────────────────────────────────────────────
rate=ros2rate(node, 1/DT);

for t=1:N_STEPS
    % Control
    [v,w]=ctrl(ts,waypts,wp_idx);
    if norm(ts(1:2)-waypts(wp_idx,:)')<0.3
        wp_idx=mod(wp_idx,size(waypts,1))+1;
    end
    ts=mmove(ts, v+randn*SIG_V, w+randn*SIG_W, DT);

    % Publish odometry
    now=ros2time(node,'now');
    odom_msg.header.stamp=now;
    odom_msg.pose.pose.position.x=ts(1);
    odom_msg.pose.pose.position.y=ts(2);
    q=eul2quat([ts(3),0,0],'ZYX');
    odom_msg.pose.pose.orientation.x=q(2);
    odom_msg.pose.pose.orientation.y=q(3);
    odom_msg.pose.pose.orientation.z=q(4);
    odom_msg.pose.pose.orientation.w=q(1);
    odom_msg.twist.twist.linear.x=v;
    odom_msg.twist.twist.angular.z=w;
    send(odom_pub, odom_msg);

    % Generate dense scan
    ranges=zeros(1,N_BEAMS,'single')+LIDAR_RANGE;
    for i=1:N_BEAMS
        beam_ang=ts(3)+beam_angles(i);
        bx=cos(beam_ang); by=sin(beam_ang);
        min_r=LIDAR_RANGE;
        % Walls
        for wi=1:size(walls,1)
            r=ray_seg(ts(1),ts(2),bx,by,walls(wi,1),walls(wi,2),walls(wi,3),walls(wi,4));
            if ~isnan(r)&&r<min_r, min_r=r; end
        end
        % Landmarks as cylinders
        for j=1:size(true_lm,1)
            dx=true_lm(j,1)-ts(1); dy=true_lm(j,2)-ts(2);
            tc=dx*bx+dy*by;
            if tc>0
                dc2=(dx-tc*bx)^2+(dy-tc*by)^2;
                if dc2<0.15^2
                    r=tc-sqrt(max(0,0.15^2-dc2));
                    if r>0&&r<min_r, min_r=r; end
                end
            end
        end
        ranges(i)=single(min_r+randn*SIG_R*0.3);
    end

    scan_msg.header.stamp=now;
    scan_msg.ranges=ranges;
    send(scan_pub, scan_msg);

    fprintf('Step %3d | pose=(%.2f, %.2f, %.1f°)\n', ...
        t, ts(1), ts(2), rad2deg(ts(3)));
    waitfor(rate);
end

fprintf('\nBridge complete.\n');
clear node;

%% ── Helpers ──────────────────────────────────────────────────────────────
function r=ray_seg(ox,oy,dx,dy,x1,y1,x2,y2)
    r=NaN; ex=x2-x1; ey=y2-y1;
    denom=dx*ey-dy*ex; if abs(denom)<1e-10, return; end
    t1=((x1-ox)*ey-(y1-oy)*ex)/denom;
    t2=((x1-ox)*dy-(y1-oy)*dx)/denom;
    if t1>0.01&&t2>=0&&t2<=1, r=t1; end
end
function s=mmove(s,v,w,dt)
    th=s(3);
    s(1)=s(1)+v*cos(th+w*dt/2)*dt;
    s(2)=s(2)+v*sin(th+w*dt/2)*dt;
    s(3)=wangle(s(3)+w*dt);
end
function [v,w]=ctrl(st,wp,i)
    dx=wp(i,1)-st(1); dy=wp(i,2)-st(2);
    v=min(0.5,sqrt(dx^2+dy^2))*0.8;
    w=wangle(atan2(dy,dx)-st(3))*2;
end
function a=wangle(a), a=mod(a+pi,2*pi)-pi; end