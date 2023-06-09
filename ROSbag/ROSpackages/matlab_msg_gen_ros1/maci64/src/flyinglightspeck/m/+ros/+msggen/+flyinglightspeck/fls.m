
classdef fls < ros.Message
    %fls MATLAB implementation of flyinglightspeck/fls
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'flyinglightspeck/fls' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '07070340d805ea1a3fe23c4fcaeccdf4' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Duration' 'Whatispresent' 'Coordinate' 'Color' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'duration' 'whatispresent' 'coordinate' 'color' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msg.Time' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Duration
        Whatispresent
        Coordinate
        Color
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'fls', 'Header')
            obj.Header = val;
        end
        function set.Duration(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msg.Time.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msg.Time'};
            validateattributes(val, validClasses, validAttributes, 'fls', 'Duration')
            obj.Duration = val;
        end
        function set.Whatispresent(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = int8.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'fls', 'Whatispresent');
            obj.Whatispresent = int8(val);
        end
        function set.Coordinate(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = single.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'fls', 'Coordinate');
            obj.Coordinate = single(val);
        end
        function set.Color(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = int8.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'fls', 'Color');
            obj.Color = int8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.flyinglightspeck.fls.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.flyinglightspeck.fls(strObj);
        end
    end
end
