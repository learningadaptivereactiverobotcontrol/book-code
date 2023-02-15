    function q = simple_robot_ikin(robot, x)
        l1 = robot.a(1); l2 = robot.a(2);
        q2 = atan2(sqrt(1-(x'*x-l1.^2-l2.^2).^2/(2*l1*l2).^2), (x'*x-l1.^2-l2.^2)/(2*l1*l2));
        q1 = atan2(x(2), x(1)) - atan2(l2*sin(q2), l1+l2*cos(q2));
        q = [q1, q2];
    end