;; Auto-generated. Do not edit!


(when (boundp 'object_detection_msgs::CarPosition)
  (if (not (find-package "OBJECT_DETECTION_MSGS"))
    (make-package "OBJECT_DETECTION_MSGS"))
  (shadow 'CarPosition (find-package "OBJECT_DETECTION_MSGS")))
(unless (find-package "OBJECT_DETECTION_MSGS::CARPOSITION")
  (make-package "OBJECT_DETECTION_MSGS::CARPOSITION"))

(in-package "ROS")
;;//! \htmlinclude CarPosition.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass object_detection_msgs::CarPosition
  :super ros::object
  :slots (_header _xmean _ymean _zmean ))

(defmethod object_detection_msgs::CarPosition
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:xmean __xmean) 0.0)
    ((:ymean __ymean) 0.0)
    ((:zmean __zmean) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _xmean (float __xmean))
   (setq _ymean (float __ymean))
   (setq _zmean (float __zmean))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:xmean
   (&optional __xmean)
   (if __xmean (setq _xmean __xmean)) _xmean)
  (:ymean
   (&optional __ymean)
   (if __ymean (setq _ymean __ymean)) _ymean)
  (:zmean
   (&optional __zmean)
   (if __zmean (setq _zmean __zmean)) _zmean)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _xmean
    8
    ;; float64 _ymean
    8
    ;; float64 _zmean
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _xmean
       (sys::poke _xmean (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ymean
       (sys::poke _ymean (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _zmean
       (sys::poke _zmean (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _xmean
     (setq _xmean (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ymean
     (setq _ymean (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _zmean
     (setq _zmean (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get object_detection_msgs::CarPosition :md5sum-) "88b1fb8816b2426364b93b43e1f32170")
(setf (get object_detection_msgs::CarPosition :datatype-) "object_detection_msgs/CarPosition")
(setf (get object_detection_msgs::CarPosition :definition-)
      "Header header
float64 xmean
float64 ymean
float64 zmean

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



(provide :object_detection_msgs/CarPosition "88b1fb8816b2426364b93b43e1f32170")


