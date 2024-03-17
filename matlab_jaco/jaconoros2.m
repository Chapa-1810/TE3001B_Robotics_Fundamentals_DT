pause on
%%
hand = importrobot('triple_3dof_box_cheese.urdf');
home = homeConfiguration(hand);
show(hand, home, "PreservePlot",false)

%%

s = struct('JointName',{'Box_x_joint','Box_y_joint','Box_z_joint','Box_w_joint','Box_v_joint','Box_u_joint','L_joint1','L_joint2','L_joint3','R_joint1','R_joint2','R_joint3','U_joint1','U_joint2','U_joint3'}, 'JointPosition',{0,0,0,0,0, 0,0,0,0,0 , 0,0,0,0,0});
jpos = [0;0;0;
        0;0;0;
        1.57;0.095;0.2215;
        1.57;0.095;0.2215;
        -1.57;0.095;0.2215];

s(1).JointPosition = jpos(1);
s(2).JointPosition = jpos(2);
s(3).JointPosition = jpos(3);
s(4).JointPosition = jpos(4);
s(5).JointPosition = jpos(5);
s(6).JointPosition = jpos(6);
s(7).JointPosition = jpos(7);
s(8).JointPosition = jpos(8);
s(9).JointPosition = jpos(9);
s(10).JointPosition = jpos(10);
s(11).JointPosition = jpos(11);
s(12).JointPosition = jpos(12);
s(13).JointPosition = jpos(13);
s(14).JointPosition = jpos(14);
s(15).JointPosition = jpos(15);

show(hand, s, "PreservePlot",false)

axis([-0.5,0.5,-0.5,0.5,-0.5,1.5])

%%

global cum_err_l
cum_err_l = 0;
global cum_err_r
cum_err_r = 0;
global cum_err_u
cum_err_u = 0;

r = rateControl(120);

old_time = 0;

%hold on
for i = 1:100
  
  show(hand, s, "PreservePlot",false, "Frames","off");
  axis([-0.5,0.5,-0.5,0.5,-0.5,1.5])
  view([180 2*180 3*180])
  %view([360 0 1000*720])

  s(1).JointPosition = 0.04*sin(i/10);
  %s(2).JointPosition = 0.04*sin(i/10);

  %s(3).JointPosition = -0.1*sin(i/30);
  %s(4).JointPosition = 0.3*sin(i/20);
  %s(5).JointPosition = 0.3*sin(i/20);

  ltrans = getTransform(hand, s, "L_link_eef","base_link");
  iltrans = getTransform(hand, s, "base_link","L_link_eef");

  rtrans = getTransform(hand, s, "R_link_eef","base_link");
  irtrans = getTransform(hand, s, "base_link","R_link_eef");

  utrans = getTransform(hand, s, "U_link_eef","base_link");
  iutrans = getTransform(hand, s, "base_link","U_link_eef");
  
  time = r.TotalElapsedTime;
  dt = time - old_time; 

  lee_pos = transpose(ltrans(1:3,4));
  ree_pos = transpose(rtrans(1:3,4));
  uee_pos = transpose(utrans(1:3,4));

  leetrans = getTransform(hand, s, "Box_contact_L","base_link");
  reetrans = getTransform(hand, s, "Box_contact_R","base_link");
  ueetrans = getTransform(hand, s, "Box_contact_U","base_link");
  ldesired_pos = transpose(leetrans(1:3,4));
  rdesired_pos = transpose(reetrans(1:3,4));
  udesired_pos = transpose(ueetrans(1:3,4));


  %ldesired_pos - lee_pos
  lvel = lcontroller(lee_pos, ldesired_pos, dt)
  rvel = rcontroller(ree_pos, rdesired_pos, dt)
  uvel = ucontroller(uee_pos, udesired_pos, dt)

  ljacobian = geometricJacobian(hand, s, 'L_link_eef');
  ljacobian = [ljacobian(:,7),ljacobian(:,8),ljacobian(:,9)]

  rjacobian = geometricJacobian(hand, s, 'R_link_eef');
  rjacobian = [rjacobian(:,10),rjacobian(:,11),rjacobian(:,12)]

  ujacobian = geometricJacobian(hand, s, 'U_link_eef');
  ujacobian = [ujacobian(:,13),ujacobian(:,14),ujacobian(:,15)]

  ljvel = pinv(ljacobian)*[ 0; 0; 0; lvel(1);lvel(2);lvel(3)]; 
  rjvel = pinv(rjacobian)*[ 0; 0; 0; rvel(1);rvel(2);rvel(3)]; 
  ujvel = pinv(ujacobian)*[ 0; 0; 0; uvel(1);uvel(2);uvel(3)]; 

  s(7).JointPosition = s(7).JointPosition + ljvel(1)*dt;
  s(8).JointPosition = s(8).JointPosition + ljvel(2)*dt;
  s(9).JointPosition = s(9).JointPosition + ljvel(3)*dt;

  s(10).JointPosition = s(10).JointPosition + rjvel(1)*dt;
  s(11).JointPosition = s(11).JointPosition + rjvel(2)*dt;
  s(12).JointPosition = s(12).JointPosition + rjvel(3)*dt;

  s(13).JointPosition = s(13).JointPosition + ujvel(1)*dt;
  s(14).JointPosition = s(14).JointPosition + ujvel(2)*dt;
  s(15).JointPosition = s(15).JointPosition + ujvel(3)*dt;

  old_time = time;
  %pause(dt)
  waitfor(r);
end

%trans = getTransform(hand, config, "link_eef","base_link");
%ee_pos = transpose(trans(1:3,4))

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

function velocity = lcontroller(position, desired_position, time_step)
  kp = 4;
  ki = 1;
  error = desired_position - position;
  global cum_err_l
  cum_err_l = error*time_step;
  velocity = kp*error + ki*cum_err_l;
 
end
function velocity = rcontroller(position, desired_position, time_step)
  kp = 4;
  ki = 1;
  error = desired_position - position;
  global cum_err_r
  cum_err_r = error*time_step;
  velocity = kp*error + ki*cum_err_r;
 
end
function velocity = ucontroller(position, desired_position, time_step)
  kp = 4;
  ki = 1;
  error = desired_position - position;
  global cum_err_u
  cum_err_u = error*time_step;
  velocity = kp*error + ki*cum_err_u;
 
end