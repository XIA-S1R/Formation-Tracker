;; Auto-generated. Do not edit!


(when (boundp 'object_detection_msgs::motorAngle)
  (if (not (find-package "OBJECT_DETECTION_MSGS"))
    (make-package "OBJECT_DETECTION_MSGS"))
  (shadow 'motorAngle (find-package "OBJECT_DETECTION_MSGS")))
(unless (find-package "OBJECT_DETECTION_MSGS::MOTORANGLE")
  (make-package "OBJECT_DETECTION_MSGS::MOTORANGLE"))

(in-package "ROS")
;;//! \htmlinclude motorAngle.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass object_detection_msgs::motorAngle
  :super ros::object
  :slots (_header _angle ))

(defmethod object_detection_msgs::motorAngle
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _angle (float __angle))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get object_detection_msgs::motorAngle :md5sum-) "585a45d22eb0f9ad2f26cc1bd254f5a4")
(setf (get object_detection_msgs::motorAngle :datatype-) "object_detection_msgs/motorAngle")
(setf (get object_detection_msgs::motorAngle :definition-)
      "Header header
float32 angle


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



(provide :object_detection_msgs/motorAngle "585a45d22eb0f9ad2f26cc1bd254f5a4")


