classdef numericalDerivative
    % Numerical derivative functions for GTSAM.
    % Translated from python/gtsam/utils/numerical_derivative.py
    
    methods(Static)
        function delta_vec = local(a, b)
            if isnumeric(a)
                delta_vec = b - a;
            elseif isa(a, 'gtsam.Values')
                % Values.localCoordinates(Values) returns a VectorValues,
                % we need to convert it to a plain vector.
                delta_vec = a.localCoordinates(b).vector();
            else
                % GTSAM objects usually provide localCoordinates
                delta_vec = a.localCoordinates(b);
            end
        end
        
        function res = retract(a, xi)
            if isnumeric(a)
                res = a + xi;
            elseif isa(a, 'gtsam.Values')
                % xi is a plain column vector.
                % We need to convert it to a VectorValues object.
                % VectorValues.vector() concatenates vectors in key-sorted order,
                % so we must iterate through the keys in the same order.
                vv = gtsam.VectorValues();
                keys = a.keys();
                key_array = zeros(1, keys.size());
                for i = 0:keys.size()-1
                    key_array(i+1) = keys.at(i);
                end
                key_array = sort(key_array);
                
                zeros_vv = a.zeroVectors();
                offset = 1;
                for i = 1:length(key_array)
                    key = key_array(i);
                    d = zeros_vv.dim(key);
                    vv.insert(key, xi(offset:offset+d-1));
                    offset = offset + d;
                end
                res = a.retract(vv);
            else
                % GTSAM objects usually provide retract
                res = a.retract(xi);
            end
        end
        
        function H = numericalDerivative11(h, x, delta)
            if nargin < 3, delta = 1e-5; end
            hx = h(x);
            zeroY = gtsam.numericalDerivative.local(hx, hx);
            m = length(zeroY);
            zeroX = gtsam.numericalDerivative.local(x, x);
            n = length(zeroX);
            dx = zeros(n, 1);
            H = zeros(m, n);
            factor = 1.0 / (2.0 * delta);
            for j = 1:n
                dx(j) = delta;
                dy1 = gtsam.numericalDerivative.local(hx, h(gtsam.numericalDerivative.retract(x, dx)));
                dx(j) = -delta;
                dy2 = gtsam.numericalDerivative.local(hx, h(gtsam.numericalDerivative.retract(x, dx)));
                dx(j) = 0;
                H(:, j) = (dy1 - dy2) * factor;
            end
        end
        
        function H = numericalDerivative21(h, x1, x2, delta)
            if nargin < 4, delta = 1e-5; end
            H = gtsam.numericalDerivative.numericalDerivative11(@(x) h(x, x2), x1, delta);
        end
        
        function H = numericalDerivative22(h, x1, x2, delta)
            if nargin < 4, delta = 1e-5; end
            H = gtsam.numericalDerivative.numericalDerivative11(@(x) h(x1, x), x2, delta);
        end
        
        function H = numericalDerivative31(h, x1, x2, x3, delta)
            if nargin < 5, delta = 1e-5; end
            H = gtsam.numericalDerivative.numericalDerivative11(@(x) h(x, x2, x3), x1, delta);
        end
        
        function H = numericalDerivative32(h, x1, x2, x3, delta)
            if nargin < 5, delta = 1e-5; end
            H = gtsam.numericalDerivative.numericalDerivative11(@(x) h(x1, x, x3), x2, delta);
        end
        
        function H = numericalDerivative33(h, x1, x2, x3, delta)
            if nargin < 5, delta = 1e-5; end
            H = gtsam.numericalDerivative.numericalDerivative11(@(x) h(x1, x2, x), x3, delta);
        end
        
        % Add more as needed, but following the same pattern as python
        function H = numericalDerivative41(h, x1, x2, x3, x4, delta)
            if nargin < 6, delta = 1e-5; end
            H = gtsam.numericalDerivative.numericalDerivative11(@(x) h(x, x2, x3, x4), x1, delta);
        end
        
        function H = numericalDerivative42(h, x1, x2, x3, x4, delta)
            if nargin < 6, delta = 1e-5; end
            H = gtsam.numericalDerivative.numericalDerivative11(@(x) h(x1, x, x3, x4), x2, delta);
        end
        
        function H = numericalDerivative43(h, x1, x2, x3, x4, delta)
            if nargin < 6, delta = 1e-5; end
            H = gtsam.numericalDerivative.numericalDerivative11(@(x) h(x1, x2, x, x4), x3, delta);
        end
        
        function H = numericalDerivative44(h, x1, x2, x3, x4, delta)
            if nargin < 6, delta = 1e-5; end
            H = gtsam.numericalDerivative.numericalDerivative11(@(x) h(x1, x2, x3, x), x4, delta);
        end
    end
end
