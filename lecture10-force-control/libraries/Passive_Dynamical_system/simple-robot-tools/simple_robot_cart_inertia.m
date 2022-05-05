function mx = simple_robot_cart_inertia(robot, q)
mq = robot.inertia(q);
J = robot.jacob0(q);
J = J(1:2,1:2);
Ji = inv(J);
mx = Ji'*mq*Ji;
end