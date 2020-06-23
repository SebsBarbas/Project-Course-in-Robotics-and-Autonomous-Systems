; Auto-generated. Do not edit!


(cl:in-package milestone2-srv)


;//! \htmlinclude pathPlanning-request.msg.html

(cl:defclass <pathPlanning-request> (roslisp-msg-protocol:ros-message)
  ((x0
    :reader x0
    :initarg :x0
    :type cl:float
    :initform 0.0)
   (y0
    :reader y0
    :initarg :y0
    :type cl:float
    :initform 0.0)
   (yaw0
    :reader yaw0
    :initarg :yaw0
    :type cl:float
    :initform 0.0)
   (targetSign
    :reader targetSign
    :initarg :targetSign
    :type cl:string
    :initform ""))
)

(cl:defclass pathPlanning-request (<pathPlanning-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pathPlanning-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pathPlanning-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name milestone2-srv:<pathPlanning-request> is deprecated: use milestone2-srv:pathPlanning-request instead.")))

(cl:ensure-generic-function 'x0-val :lambda-list '(m))
(cl:defmethod x0-val ((m <pathPlanning-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader milestone2-srv:x0-val is deprecated.  Use milestone2-srv:x0 instead.")
  (x0 m))

(cl:ensure-generic-function 'y0-val :lambda-list '(m))
(cl:defmethod y0-val ((m <pathPlanning-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader milestone2-srv:y0-val is deprecated.  Use milestone2-srv:y0 instead.")
  (y0 m))

(cl:ensure-generic-function 'yaw0-val :lambda-list '(m))
(cl:defmethod yaw0-val ((m <pathPlanning-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader milestone2-srv:yaw0-val is deprecated.  Use milestone2-srv:yaw0 instead.")
  (yaw0 m))

(cl:ensure-generic-function 'targetSign-val :lambda-list '(m))
(cl:defmethod targetSign-val ((m <pathPlanning-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader milestone2-srv:targetSign-val is deprecated.  Use milestone2-srv:targetSign instead.")
  (targetSign m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pathPlanning-request>) ostream)
  "Serializes a message object of type '<pathPlanning-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'targetSign))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'targetSign))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pathPlanning-request>) istream)
  "Deserializes a message object of type '<pathPlanning-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'targetSign) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'targetSign) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pathPlanning-request>)))
  "Returns string type for a service object of type '<pathPlanning-request>"
  "milestone2/pathPlanningRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pathPlanning-request)))
  "Returns string type for a service object of type 'pathPlanning-request"
  "milestone2/pathPlanningRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pathPlanning-request>)))
  "Returns md5sum for a message object of type '<pathPlanning-request>"
  "60da1f4dc44e1c3e5e24bf52a981d0f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pathPlanning-request)))
  "Returns md5sum for a message object of type 'pathPlanning-request"
  "60da1f4dc44e1c3e5e24bf52a981d0f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pathPlanning-request>)))
  "Returns full string definition for message of type '<pathPlanning-request>"
  (cl:format cl:nil "float32 x0~%float32 y0~%float32 yaw0~%string targetSign~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pathPlanning-request)))
  "Returns full string definition for message of type 'pathPlanning-request"
  (cl:format cl:nil "float32 x0~%float32 y0~%float32 yaw0~%string targetSign~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pathPlanning-request>))
  (cl:+ 0
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'targetSign))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pathPlanning-request>))
  "Converts a ROS message object to a list"
  (cl:list 'pathPlanning-request
    (cl:cons ':x0 (x0 msg))
    (cl:cons ':y0 (y0 msg))
    (cl:cons ':yaw0 (yaw0 msg))
    (cl:cons ':targetSign (targetSign msg))
))
;//! \htmlinclude pathPlanning-response.msg.html

(cl:defclass <pathPlanning-response> (roslisp-msg-protocol:ros-message)
  ((plannedPath
    :reader plannedPath
    :initarg :plannedPath
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path)))
)

(cl:defclass pathPlanning-response (<pathPlanning-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pathPlanning-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pathPlanning-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name milestone2-srv:<pathPlanning-response> is deprecated: use milestone2-srv:pathPlanning-response instead.")))

(cl:ensure-generic-function 'plannedPath-val :lambda-list '(m))
(cl:defmethod plannedPath-val ((m <pathPlanning-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader milestone2-srv:plannedPath-val is deprecated.  Use milestone2-srv:plannedPath instead.")
  (plannedPath m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pathPlanning-response>) ostream)
  "Serializes a message object of type '<pathPlanning-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'plannedPath) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pathPlanning-response>) istream)
  "Deserializes a message object of type '<pathPlanning-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'plannedPath) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pathPlanning-response>)))
  "Returns string type for a service object of type '<pathPlanning-response>"
  "milestone2/pathPlanningResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pathPlanning-response)))
  "Returns string type for a service object of type 'pathPlanning-response"
  "milestone2/pathPlanningResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pathPlanning-response>)))
  "Returns md5sum for a message object of type '<pathPlanning-response>"
  "60da1f4dc44e1c3e5e24bf52a981d0f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pathPlanning-response)))
  "Returns md5sum for a message object of type 'pathPlanning-response"
  "60da1f4dc44e1c3e5e24bf52a981d0f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pathPlanning-response>)))
  "Returns full string definition for message of type '<pathPlanning-response>"
  (cl:format cl:nil "nav_msgs/Path plannedPath~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pathPlanning-response)))
  "Returns full string definition for message of type 'pathPlanning-response"
  (cl:format cl:nil "nav_msgs/Path plannedPath~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pathPlanning-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'plannedPath))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pathPlanning-response>))
  "Converts a ROS message object to a list"
  (cl:list 'pathPlanning-response
    (cl:cons ':plannedPath (plannedPath msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'pathPlanning)))
  'pathPlanning-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'pathPlanning)))
  'pathPlanning-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pathPlanning)))
  "Returns string type for a service object of type '<pathPlanning>"
  "milestone2/pathPlanning")