% FLAGS
clc; clf;close all;

show_plot = true;
publish_ros = false;

steps_ = 1e3; 
iterations_ = 1e6;
filename = 'uwrt_arm_matlab_sim_' + string(iterations_) + '.mat';

%==============================
% uwrt_arm
%==============================

urdf_lengths_ = [0.286, 0.462, 0.356, 0.228];

% MEAN
urdf_joint_limits_ = [-120, 120, ...
                       -20,  75, ...
                       0,   0, ...
                       0,    80, ...
                       -75,  75, ...
                       -135, 135, ...
                       ]*pi/180;

%      Link([Theta      d       a                   alpha])
L(1) = Link ([0         0       0                   pi/2], "revolute");
L(2) = Link ([pi/2      0       urdf_lengths_(1)    0], "revolute");
L(3) = Link ([-pi/2     0       urdf_lengths_(2)    0], "revolute");
L(4) = Link ([0         0       urdf_lengths_(3)    0], "revolute");
L(5) = Link ([0         0       0                   pi/2], "revolute");
L(6) = Link ([0         0       urdf_lengths_(4)    -pi/2], "revolute");

% check arm
uwrt_arm = SerialLink(L, 'name', 'uwrt arm', 'base', transl(0,0,0.1));
if show_plot
    plot(uwrt_arm, [0 pi/2 -pi/2 0 0 0])
    hold on;
end

%%
%==============================
% joint limits
%==============================
theta1=urdf_joint_limits_(1):((urdf_joint_limits_(1+1) - urdf_joint_limits_(1))/steps_):urdf_joint_limits_(1+1);
theta2=urdf_joint_limits_(3):((urdf_joint_limits_(3+1) - urdf_joint_limits_(3))/steps_):urdf_joint_limits_(3+1);
theta3=zeros(1, steps_);
theta4=urdf_joint_limits_(7):((urdf_joint_limits_(7+1) - urdf_joint_limits_(7))/steps_):urdf_joint_limits_(7+1);
theta5=urdf_joint_limits_(9):((urdf_joint_limits_(9+1) - urdf_joint_limits_(9))/steps_):urdf_joint_limits_(9+1);
theta6=urdf_joint_limits_(11):((urdf_joint_limits_(11+1) - urdf_joint_limits_(11))/steps_):urdf_joint_limits_(11+1);

% MAX
% theta3=zeros(1, steps_);
% theta4=zeros(1, steps_);
% theta5=zeros(1, steps_);
% theta6=zeros(1, steps_);

%%
%==============================
% ROS
%==============================
if publish_ros
    rosshutdown;
    rosinit;
    pub = rospublisher('/uwrt_arm_fwdsim', 'geometry_msgs/Vector3');
end

% pub = rospublisher('/uwrt_arm_fwdsim', 'sensor_msgs/JointState');
% msg = rosmessage('sensor_msgs/JointState')
% msg.Name = {'arm_base_turntable_joint', 'arm_shoulder_joint', 'arm_shoulder_bicep_joint', 'arm_elbow_joint', 'arm_wrist_roll_joint', 'arm_wrist_pitch_joint'}
% msg.Position = [qq(1), qq(2), qq(3), qq(4), qq(5), qq(6)]
% send(pub,msg);

%%
tic;
t_start_ = 0;
end_effector_points = zeros(iterations_, 3);
for n=1:1:iterations_
    
    t_current_ = toc;
    fprintf('Iteration: %d/%d, Time: %.2f[s]..\n', n, iterations_, t_current_ - t_start_);
    
    random_idx1 = datasample(1:1:steps_, 1);
    random_idx2 = datasample(1:1:steps_, 1);
    random_idx3 = datasample(1:1:steps_, 1);
    random_idx4 = datasample(1:1:steps_, 1);
    random_idx5 = datasample(1:1:steps_, 1);
    random_idx6 = datasample(1:1:steps_, 1);

    qq=[theta1(random_idx1) ,theta2(random_idx2) + pi/2, theta3(random_idx3) - pi/2, ...
        theta4(random_idx4), theta5(random_idx5), theta6(random_idx6)];

    end_effector_=uwrt_arm.fkine(qq);
    end_effector_points(n, 1) = end_effector_.t(1);
    end_effector_points(n, 2) = end_effector_.t(2);
    end_effector_points(n, 3) = end_effector_.t(3);

    if show_plot
        hold on;
        drawnow;

        p1 = plot3(end_effector_points(n, 1), end_effector_points(n, 2), end_effector_points(n, 3),'mx','MarkerSize',1);
        p1.Color(4) = 0.25;

        plot(uwrt_arm, qq);
        xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([-1.5 1.5]);
    end
    
    if publish_ros
        msg = rosmessage('geometry_msgs/Vector3');
        msg.X = end_effector_.t(1);
        msg.Y = end_effector_.t(2);
        msg.Z = end_effector_.t(3);
        send(pub,msg);
    end

end

if publish_ros
    rosshutdown;
end

 save(filename, 'end_effector_points')