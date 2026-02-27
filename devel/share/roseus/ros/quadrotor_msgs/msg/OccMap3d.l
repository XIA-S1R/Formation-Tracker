;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::OccMap3d)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'OccMap3d (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::OCCMAP3D")
  (make-package "QUADROTOR_MSGS::OCCMAP3D"))

(in-package "ROS")
;;//! \htmlinclude OccMap3d.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::OccMap3d
  :super ros::object
  :slots (_header _resolution _size_x _size_y _size_z _offset_x _offset_y _offset_z _data ))

(defmethod quadrotor_msgs::OccMap3d
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:resolution __resolution) 0.0)
    ((:size_x __size_x) 0)
    ((:size_y __size_y) 0)
    ((:size_z __size_z) 0)
    ((:offset_x __offset_x) 0)
    ((:offset_y __offset_y) 0)
    ((:offset_z __offset_z) 0)
    ((:data __data) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _resolution (float __resolution))
   (setq _size_x (round __size_x))
   (setq _size_y (round __size_y))
   (setq _size_z (round __size_z))
   (setq _offset_x (round __offset_x))
   (setq _offset_y (round __offset_y))
   (setq _offset_z (round __offset_z))
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:resolution
   (&optional __resolution)
   (if __resolution (setq _resolution __resolution)) _resolution)
  (:size_x
   (&optional __size_x)
   (if __size_x (setq _size_x __size_x)) _size_x)
  (:size_y
   (&optional __size_y)
   (if __size_y (setq _size_y __size_y)) _size_y)
  (:size_z
   (&optional __size_z)
   (if __size_z (setq _size_z __size_z)) _size_z)
  (:offset_x
   (&optional __offset_x)
   (if __offset_x (setq _offset_x __offset_x)) _offset_x)
  (:offset_y
   (&optional __offset_y)
   (if __offset_y (setq _offset_y __offset_y)) _offset_y)
  (:offset_z
   (&optional __offset_z)
   (if __offset_z (setq _offset_z __offset_z)) _offset_z)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _resolution
    4
    ;; int16 _size_x
    2
    ;; int16 _size_y
    2
    ;; int16 _size_z
    2
    ;; int16 _offset_x
    2
    ;; int16 _offset_y
    2
    ;; int16 _offset_z
    2
    ;; int8[] _data
    (* 1    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _resolution
       (sys::poke _resolution (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int16 _size_x
       (write-word _size_x s)
     ;; int16 _size_y
       (write-word _size_y s)
     ;; int16 _size_z
       (write-word _size_z s)
     ;; int16 _offset_x
       (write-word _offset_x s)
     ;; int16 _offset_y
       (write-word _offset_y s)
     ;; int16 _offset_z
       (write-word _offset_z s)
     ;; int8[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (write-byte (elt _data i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _resolution
     (setq _resolution (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int16 _size_x
     (setq _size_x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _size_y
     (setq _size_y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _size_z
     (setq _size_z (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _offset_x
     (setq _offset_x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _offset_y
     (setq _offset_y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _offset_z
     (setq _offset_z (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int8[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _data i) 127) (setf (elt _data i) (- (elt _data i) 256)))
     ))
   ;;
   self)
  )

(setf (get quadrotor_msgs::OccMap3d :md5sum-) "24ed7902582cd1f7eb06c7a491c74c49")
(setf (get quadrotor_msgs::OccMap3d :datatype-) "quadrotor_msgs/OccMap3d")
(setf (get quadrotor_msgs::OccMap3d :definition-)
      "Header header

float32 resolution
int16 size_x
int16 size_y
int16 size_z
int16 offset_x
int16 offset_y
int16 offset_z

int8[] data
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

")



(provide :quadrotor_msgs/OccMap3d "24ed7902582cd1f7eb06c7a491c74c49")


