pause on

nh = ros2node('/matlab_node');
joint_state_sub = ros2subscriber(nh,'/joint_states','sensor_msgs/JointState', @jointStateCallback);
linear_vel_sub = ros2subscriber(nh,'/linvel','geometry_msgs/Twist', @linearvelCallback);
joint_state_pub = ros2publisher(nh,'/joint_states','sensor_msgs/JointState');

joint_state_msg = ros2message('sensor_msgs/JointState');
joint_state_msg.Name = {'joint1', 'joint2', 'joint3'};
joint_state_msg.Position = [0, 0, 0];

linear_vel_msg = ros2message('geometry_msgs/Twist');
linear_vel_msg.Linear.X = 0;
linear_vel_msg.Linear.Y = 0;
linear_vel_msg.Linear.Z = 0;
linear_vel_msg.Angular.X = 0;
linear_vel_msg.Angular.Y = 0;
linear_vel_msg.Angular.Z = 0;

finger = importrobot('3dof.urdf');
home = homeConfiguration(finger);

config = struct('JointName', {'joint1', 'joint2', 'joint3'}, 'JointPosition', {0, 0, 0});

while 1
  global joint_state_msg
  global linear_vel_msg

  config.JointPosition = joint_state_msg.Position;
  jacobian = geometricJacobian(finger, config.JointPosition, 'link2');
  inverse_jacobian = pinv(jacobian);

  joint_velocities = inverse_jacobian * [linear_vel_msg.Linear.X; linear_vel_msg.Linear.Y; linear_vel_msg.Linear.Z; linear_vel_msg.Angular.X; linear_vel_msg.Angular.Y; linear_vel_msg.Angular.Z];
  joint_state_msg.Position = joint_state_msg.Position + joint_velocities * 0.01;
  send(joint_state_pub, joint_state_msg);
  pause(0.01);
  
end

function jointStateCallback(~, message)
  global joint_state_msg
  joint_state_msg.Position = message.Position;
end

function linearvelCallback(~, message)
  global linear_vel_msg
  linear_vel_msg = message;
end
