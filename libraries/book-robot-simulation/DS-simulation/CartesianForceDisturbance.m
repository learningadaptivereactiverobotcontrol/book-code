classdef CartesianForceDisturbance < handle
    properties
        values
        max_force
        iteration
        nb_active_iterations
    end

    methods
        function self = CartesianForceDisturbance(max_force, nb_active_iterations)
            self.values = zeros(3,1);
            self.max_force = max_force;
            self.iteration = 0;
            self.nb_active_iterations = nb_active_iterations;
        end

        function handleKeyEvent(self, key)
            if strcmp(key, 'd') % positive x
                self.values(1) = self.max_force;
            elseif strcmp(key, 'a') % negative x
                self.values(1) = -self.max_force;
            elseif strcmp(key, 'e') % positive y
                self.values(2) = self.max_force;
            elseif strcmp(key, 'q') % negative y
                self.values(2) = -self.max_force;
            elseif strcmp(key, 'w') % positive z
                self.values(3) = self.max_force;
            elseif strcmp(key, 's') % negative z
                self.values(3) = -self.max_force;
            end
            self.iteration = 0;
        end
    end
end

