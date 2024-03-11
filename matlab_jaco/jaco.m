pause on
nh = ros2node("/jacobian_matlab");
jointsub = ros2subscriber(nh,"/joint_states","sensor_msgs/JointState", @jointCallback);
lvelsub =  ros2subscriber(nh,"/left_ee_vel", "geometry_msgs/TwistStamped", @lvelCallback);
rvelsub =  ros2subscriber(nh,"/right_ee_vel","geometry_msgs/TwistStamped", @rvelCallback);
uvelsub =  ros2subscriber(nh,"/under_ee_vel","geometry_msgs/TwistStamped", @uvelCallback);
jointpub = ros2publisher( nh,"/joint_vel",   "geometry_msgs/Twist");

%%

rob = importrobot('triple_3dof.urdf');
hom = homeConfiguration(rob);

%%

velmsg = ros2message("geometry_msgs/Twist");

while 1
    jointData = receive(jointsub);
    lvelData = receive(lvelsub);
    rvelData = receive(rvelsub);
    uvelData = receive(uvelsub);
    lJ = geometricJacobian(rob,hom, "L_link2");
    rJ = geometricJacobian(rob,hom, "R_link2");
    uJ = geometricJacobian(rob,hom, "U_link2");
    
    lvel = [lvelData.linear.x; 
            lvelData.linear.y; 
            lvelData.linear.z;
            lvelData.angular.x;
            lvelData.angular.y;
            lvelData.angular.z];

    rvel = [rvelData.linear.x; 
            rvelData.linear.y; 
            rvelData.linear.z;
            rvelData.angular.x;
            rvelData.angular.y;
            rvelData.angular.z];

    uvel = [uvelData.linear.x; 
            uvelData.linear.y; 
            uvelData.linear.z;
            uvelData.angular.x;
            uvelData.angular.y;
            uvelData.angular.z];

    %Jinv = pinv(geometricJacobian(rob,s,'link2'));

    %jvel = Jinv*vel;

    %velmsg.linear.x = jvel(1);
    %velmsg.linear.y = jvel(2);
    %velmsg.linear.z = jvel(3);

    %send(jointpub, velmsg)

    

    %s(1).JointPosition = i;
    %s(2).JointPosition = i;
    %s(3).JointPosition = i;
    %pause(1)
    %geometricJacobian(rob,s,'link2')
end

%show(rob,s)
