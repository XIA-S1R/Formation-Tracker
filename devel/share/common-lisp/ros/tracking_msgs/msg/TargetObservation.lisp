; Auto-generated. Do not edit!


(cl:in-package tracking_msgs-msg)


;//! \htmlinclude TargetObservation.msg.html

(cl:defclass <TargetObservation> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (image_pos
    :reader image_pos
    :initarg :image_pos
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (depth
    :reader depth
    :initarg :depth
    :type cl:float
    :initform 0.0)
   (drone_odom
    :reader drone_odom
    :initarg :drone_odom
    :type nav_msgs-msg:Odometry
    :initform (cl:make-instance 'nav_msgs-msg:Odometry)))
)

(cl:defclass TargetObservation (<TargetObservation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TargetObservation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TargetObservation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tracking_msgs-msg:<TargetObservation> is deprecated: use tracking_msgs-msg:TargetObservation instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TargetObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tracking_msgs-msg:header-val is deprecated.  Use tracking_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'image_pos-val :lambda-list '(m))
(cl:defmethod image_pos-val ((m <TargetObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tracking_msgs-msg:image_pos-val is deprecated.  Use tracking_msgs-msg:image_pos instead.")
  (image_pos m))

(cl:ensure-generic-function 'depth-val :lambda-list '(m))
(cl:defmethod depth-val ((m <TargetObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tracking_msgs-msg:depth-val is deprecated.  Use tracking_msgs-msg:depth instead.")
  (depth m))

(cl:ensure-generic-function 'drone_odom-val :lambda-list '(m))
(cl:defmethod drone_odom-val ((m <TargetObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tracking_msgs-msg:drone_odom-val is deprecated.  Use tracking_msgs-msg:drone_odom instead.")
  (drone_odom m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TargetObservation>) ostream)
  "Serializes a message object of type '<TargetObservation>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image_pos) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'drone_odom) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TargetObservation>) istream)
  "Deserializes a message object of type '<TargetObservation>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image_pos) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'depth) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'drone_odom) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TargetObservation>)))
  "Returns string type for a message object of type '<TargetObservation>"
  "tracking_msgs/TargetObservation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetObservation)))
  "Returns string type for a message object of type 'TargetObservation"
  "tracking_msgs/TargetObservation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TargetObservation>)))
  "Returns md5sum for a message object of type '<TargetObservation>"
  "1b05b044b9de45d04297a9cb0b1fd945")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TargetObservation)))
  "Returns md5sum for a message object of type 'TargetObservation"
  "1b05b044b9de45d04297a9cb0b1fd945")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TargetObservation>)))
  "Returns full string definition for message of type '<TargetObservation>"
  (cl:format cl:nil "Header header~%geometry_msgs/Point image_pos  # 目标在图像中的位置 (u,v)~%float64 depth  # 深度估计~%nav_msgs/Odometry drone_odom  # 无人机姿态~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TargetObservation)))
  "Returns full string definition for message of type 'TargetObservation"
  (cl:format cl:nil "Header header~%geometry_msgs/Point image_pos  # 目标在图像中的位置 (u,v)~%float64 depth  # 深度估计~%nav_msgs/Odometry drone_odom  # 无人机姿态~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TargetObservation>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image_pos))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'drone_odom))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TargetObservation>))
  "Converts a ROS message object to a list"
  (cl:list 'TargetObservation
    (cl:cons ':header (header msg))
    (cl:cons ':image_pos (image_pos msg))
    (cl:cons ':depth (depth msg))
    (cl:cons ':drone_odom (drone_odom msg))
))
