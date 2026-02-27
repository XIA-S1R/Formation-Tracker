;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::ReplanState)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'ReplanState (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::REPLANSTATE")
  (make-package "QUADROTOR_MSGS::REPLANSTATE"))

(in-package "ROS")
;;//! \htmlinclude ReplanState.msg.html
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::ReplanState
  :super ros::object
  :slots (_header _state _iniState _target _occmap _path _replan_stamp ))

(defmethod quadrotor_msgs::ReplanState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) 0)
    ((:iniState __iniState) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:target __target) (instance nav_msgs::Odometry :init))
    ((:occmap __occmap) (instance quadrotor_msgs::OccMap3d :init))
    ((:path __path) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:replan_stamp __replan_stamp) (instance ros::time :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _state (round __state))
   (setq _iniState __iniState)
   (setq _target __target)
   (setq _occmap __occmap)
   (setq _path __path)
   (setq _replan_stamp __replan_stamp)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:iniState
   (&optional __iniState)
   (if __iniState (setq _iniState __iniState)) _iniState)
  (:target
   (&rest __target)
   (if (keywordp (car __target))
       (send* _target __target)
     (progn
       (if __target (setq _target (car __target)))
       _target)))
  (:occmap
   (&rest __occmap)
   (if (keywordp (car __occmap))
       (send* _occmap __occmap)
     (progn
       (if __occmap (setq _occmap (car __occmap)))
       _occmap)))
  (:path
   (&optional __path)
   (if __path (setq _path __path)) _path)
  (:replan_stamp
   (&optional __replan_stamp)
   (if __replan_stamp (setq _replan_stamp __replan_stamp)) _replan_stamp)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int16 _state
    2
    ;; float64[] _iniState
    (* 8    (length _iniState)) 4
    ;; nav_msgs/Odometry _target
    (send _target :serialization-length)
    ;; quadrotor_msgs/OccMap3d _occmap
    (send _occmap :serialization-length)
    ;; float64[] _path
    (* 8    (length _path)) 4
    ;; time _replan_stamp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int16 _state
       (write-word _state s)
     ;; float64[] _iniState
     (write-long (length _iniState) s)
     (dotimes (i (length _iniState))
       (sys::poke (elt _iniState i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; nav_msgs/Odometry _target
       (send _target :serialize s)
     ;; quadrotor_msgs/OccMap3d _occmap
       (send _occmap :serialize s)
     ;; float64[] _path
     (write-long (length _path) s)
     (dotimes (i (length _path))
       (sys::poke (elt _path i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; time _replan_stamp
       (write-long (send _replan_stamp :sec) s) (write-long (send _replan_stamp :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int16 _state
     (setq _state (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64[] _iniState
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _iniState (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _iniState i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; nav_msgs/Odometry _target
     (send _target :deserialize buf ptr-) (incf ptr- (send _target :serialization-length))
   ;; quadrotor_msgs/OccMap3d _occmap
     (send _occmap :deserialize buf ptr-) (incf ptr- (send _occmap :serialization-length))
   ;; float64[] _path
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _path (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _path i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; time _replan_stamp
     (send _replan_stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _replan_stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get quadrotor_msgs::ReplanState :md5sum-) "38714edcd5022172624012c3dbfdd606")
(setf (get quadrotor_msgs::ReplanState :datatype-) "quadrotor_msgs/ReplanState")
(setf (get quadrotor_msgs::ReplanState :definition-)
      "# all information for a replan

Header header

int16 state
float64[] iniState
nav_msgs/Odometry target
quadrotor_msgs/OccMap3d occmap

# other temporal variables
float64[] path
time replan_stamp
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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

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
================================================================================
MSG: quadrotor_msgs/OccMap3d
Header header

float32 resolution
int16 size_x
int16 size_y
int16 size_z
int16 offset_x
int16 offset_y
int16 offset_z

int8[] data
")



(provide :quadrotor_msgs/ReplanState "38714edcd5022172624012c3dbfdd606")


