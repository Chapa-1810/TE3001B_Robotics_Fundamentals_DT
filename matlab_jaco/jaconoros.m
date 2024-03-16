pause on
%%
finger = importrobot('3dof.urdf');
home = homeConfiguration(finger);

%%
config = struct('JointName', {'joint1', 'joint2', 'joint3'}, 'JointPosition', {0.1, 0.1, 0.1});
trans = getTransform(finger, config, "link_eef");
transpose(trans(1:3,4))

%desired_config 
%desired_config(1).
global cum_err
cum_err = 0;
%dt = 0.1;
r = rateControl(120);

desired_pos = [0 0.4 0.2];

old_time = 0;

%hold on
for i = 1:100
  
  if i == 25
      desired_pos = [0 0.4 0.3];
  end
  if i == 50
      desired_pos = [0 0.4 0.4];
  end
  if i == 75
      desired_pos = [0 0.4 0.5];
  end
  show(finger, config, "PreservePlot",false);
  axis([-0.5,0.5,-0.5,0.5,-0.5,0.5])
  trans = getTransform(finger, config, "link_eef","base_link");
  itrans = getTransform(finger, config, "base_link","link_eef");
  
  time = r.TotalElapsedTime;
  dt = time - old_time; 

  ee_pos = transpose(trans(1:3,4));
  %lvel = itrans*transpose([controller(ee_pos, desired_pos),0]);
  desired_pos - ee_pos
  lvel = controller(ee_pos, desired_pos, dt)
  jacobian = geometricJacobian(finger, config, 'link_eef');
  jvel = pinv(jacobian)*[ 0; 0; 0; lvel(1);lvel(2);lvel(3)]; 

  

  config(1).JointPosition = config(1).JointPosition + jvel(1)*dt;
  config(2).JointPosition = config(2).JointPosition + jvel(2)*dt;
  config(3).JointPosition = config(3).JointPosition + jvel(3)*dt;

  old_time = time;
  %pause(dt)
  waitfor(r);
end

trans = getTransform(finger, config, "link_eef","base_link");
ee_pos = transpose(trans(1:3,4))

%hold off


%while 1
%  config(1).JointPosition = joint_state_msg.Position(1);
%  config(2).JointPosition = joint_state_msg.Position(2);
%  config(3).JointPosition = joint_state_msg.Position(3);
%  jacobian = geometricJacobian(finger, config, 'link2');
%  inverse_jacobian = pinv(jacobian);

%  joint_velocities = inverse_jacobian * [linear_vel_msg.Linear.X; linear_vel_msg.Linear.Y; linear_vel_msg.Linear.Z; linear_vel_msg.Angular.X; linear_vel_msg.Angular.Y; linear_vel_msg.Angular.Z];
%  joint_state_msg.Position = joint_state_msg.Position + joint_velocities * 0.01;
%  send(joint_state_pub, joint_state_msg);
%  pause(0.01)
%  
%end

function velocity = controller(position, desired_position, time_step)
  kp = 2;
  ki = 3;
  error = desired_position - position;
  global cum_err
  cum_err = error*time_step;
  velocity = kp*error + ki*cum_err;
 
end