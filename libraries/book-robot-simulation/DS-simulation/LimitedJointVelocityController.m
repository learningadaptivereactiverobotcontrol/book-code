classdef LimitedJointVelocityController < handle
    properties
        robot
        stiffness
    end

    methods
        function self = LimitedJointVelocityController(stiffness, robot)
            self.stiffness = stiffness;
            self.robot = robot;
        end

        function controlAcceleration = computeCommand(self, desired_velocity, q_dot, full_jac)
            
            % Clamping max velocity
            if norm(desired_velocity) > 5
                desired_velocity = desired_velocity / norm(desired_velocity) * 5;
            end
            
            % Computing joint velocity from cartesian velocity
            full_jac_inv = full_jac' / (full_jac * full_jac' + 0.01 * eye(6));
            q_dot_des = (full_jac_inv * [0; 0; 0; desired_velocity]);
            
            % Controlled acceleration from desired joint speed
            controlAcceleration = self.stiffness * (q_dot_des - q_dot);
        end
    end
end