;; Auto-generated. Do not edit!


(when (boundp 'tracking_msgs::TargetObservation)
  (if (not (find-package "TRACKING_MSGS"))
    (make-package "TRACKING_MSGS"))
  (shadow 'TargetObservation (find-package "TRACKING_MSGS")))
(unless (find-package "TRACKING_MSGS::TARGETOBSERVATION")
  (make-package "TRACKING_MSGS::TARGETOBSERVATION"))

(in-package "ROS")
;;//! \htmlinclude TargetObservation.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tracking_msgs::TargetObservation
  :super ros::object
  :slots (_header _image_pos _depth _drone_odom ))

(defmethod tracking_msgs::TargetObservation
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:image_pos __image_pos) (instance geometry_msgs::Point :init))
    ((:depth __depth) 0.0)
    ((:drone_odom __drone_odom) (instance nav_msgs::Odometry :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _image_pos __image_pos)
   (setq _depth (float __depth))
   (setq _drone_odom __drone_odom)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:image_pos
   (&rest __image_pos)
   (if (keywordp (car __image_pos))
       (send* _image_pos __image_pos)
     (progn
       (if __image_pos (setq _image_pos (car __image_pos)))
       _image_pos)))
  (:depth
   (&optional __depth)
   (if __depth (setq _depth __depth)) _depth)
  (:drone_odom
   (&rest __drone_odom)
   (if (keywordp (car __drone_odom))
       (send* _drone_odom __drone_odom)
     (progn
       (if __drone_odom (setq _drone_odom (car __drone_odom)))
       _drone_odom)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point _image_pos
    (send _image_pos :serialization-length)
    ;; float64 _depth
    8
    ;; nav_msgs/Odometry _drone_odom
    (send _drone_odom :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point _image_pos
       (send _image_pos :serialize s)
     ;; float64 _depth
       (sys::poke _depth (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; nav_msgs/Odometry _drone_odom
       (send _drone_odom :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point _image_pos
     (send _image_pos :deserialize buf ptr-) (incf ptr- (send _image_pos :serialization-length))
   ;; float64 _depth
     (setq _depth (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; nav_msgs/Odometry _drone_odom
     (send _drone_odom :deserialize buf ptr-) (incf ptr- (send _drone_odom :serialization-length))
   ;;
   self)
  )

(setf (get tracking_msgs::TargetObservation :md5sum-) "1b05b044b9de45d04297a9cb0b1fd945")
(setf (get tracking_msgs::TargetObservation :datatype-) "tracking_msgs/TargetObservation")
(setf (get tracking_msgs::TargetObservation :definition-)
      "Header header
geometry_msgs/Point image_pos  # 目标在图像中的位置 (u,v)
float64 depth  # 深度估计
nav_msgs/Odometry drone_odom  # 无人机姿态
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: nav_msgs/Odometry
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :tracking_msgs/TargetObservation "1b05b044b9de45d04297a9cb0b1fd945")


