classdef thinTree
    
    % Attributes
    properties (SetAccess = private)
        nodes = { [] };  % Array of the nodes
        depth = 0;  % Depth of the tree
        w = 0   % Number of parents for each node
        links = [];   % The matrix representing the links between the nodes
    end
    
    % Methods
    methods
        % Constructor
        function [obj root_ID] = thinTree(d, w)
            
            % If 0 input arguments, assume d = 1 and w = 1
            if nargin < 1
                [obj root_ID] = thinTree(1, 1);
                return
            end
            % If 1 input argument, assume w = 1
            if nargin < 1
                [obj root_ID] = thinTree(d, 1);
                return
            end
            % Else
            
            if w > d-1
                error('MATLAB:thinTree:thinTree', ...
                    'Cannot have %d parents on a binary tree of depth %d. You must have nParents < %d here.\n', w, d, d);
            end
            
            root_ID = 1;
            obj.nodes = cell(2^d - 1,1);    % Creation of the d^2 empty cells
            obj.depth = d;
            obj.w = w;
            obj.links = eye(2^d - 1); % Creation of the links matrix
            
            % Link the cells
            
        end
        
        % Function to add a content for a specific node
        function [obj] = addContent(obj, content, nodeID)
            obj.nodes{nodeID} = content;
            return
        end
        
        % Function to return the ID's of a node's parents
        function ids = getParents(obj, nodeID)
            % Initialisation
            node = nodeID;
            depthOfNode = obj.getNodeDepth(nodeID);
            if depthOfNode == 1
                ids = 1;
            else
                ids = zeros(1,min(obj.w, depthOfNode-1));
                % Loop on w, the number of parents associated to one node
                for i=1:min(obj.w, depthOfNode-1)
                    ids(i) = floor(node/2);
                    node = floor(node/2);
                end
            end
            % Return
            return
        end
        
        % Accessors
        function output = getDepth(obj)
            output = obj.depth;
            return
        end
        
        function output = getW(obj)
            output = obj.w;
            return
        end
        
        function output = getNumberOfElements(obj)
            output = 2^obj.depth - 1;
        end
        
        % Returns the depth of a node
        function output = getNodeDepth(obj, nodeID)
            output = ceil(log(nodeID+1)/log(2));
        end
    end     % Methods
end     % Class