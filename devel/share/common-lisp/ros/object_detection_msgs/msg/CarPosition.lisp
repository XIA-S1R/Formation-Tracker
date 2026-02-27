; Auto-generated. Do not edit!


(cl:in-package object_detection_msgs-msg)


;//! \htmlinclude CarPosition.msg.html

(cl:defclass <CarPosition> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (xmean
    :reader xmean
    :initarg :xmean
    :type cl:float
    :initform 0.0)
   (ymean
    :reader ymean
    :initarg :ymean
    :type cl:float
    :initform 0.0)
   (zmean
    :reader zmean
    :initarg :zmean
    :type cl:float
    :initform 0.0))
)

(cl:defclass CarPosition (<CarPosition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CarPosition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CarPosition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_detection_msgs-msg:<CarPosition> is deprecated: use object_detection_msgs-msg:CarPosition instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CarPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection_msgs-msg:header-val is deprecated.  Use object_detection_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'xmean-val :lambda-list '(m))
(cl:defmethod xmean-val ((m <CarPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection_msgs-msg:xmean-val is deprecated.  Use object_detection_msgs-msg:xmean instead.")
  (xmean m))

(cl:ensure-generic-function 'ymean-val :lambda-list '(m))
(cl:defmethod ymean-val ((m <CarPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection_msgs-msg:ymean-val is deprecated.  Use object_detection_msgs-msg:ymean instead.")
  (ymean m))

(cl:ensure-generic-function 'zmean-val :lambda-list '(m))
(cl:defmethod zmean-val ((m <CarPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection_msgs-msg:zmean-val is deprecated.  Use object_detection_msgs-msg:zmean instead.")
  (zmean m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CarPosition>) ostream)
  "Serializes a message object of type '<CarPosition>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'xmean))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ymean))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'zmean))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CarPosition>) istream)
  "Deserializes a message object of type '<CarPosition>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'xmean) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ymean) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zmean) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CarPosition>)))
  "Returns string type for a message object of type '<CarPosition>"
  "object_detection_msgs/CarPosition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CarPosition)))
  "Returns string type for a message object of type 'CarPosition"
  "object_detection_msgs/CarPosition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CarPosition>)))
  "Returns md5sum for a message object of type '<CarPosition>"
  "88b1fb8816b2426364b93b43e1f32170")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CarPosition)))
  "Returns md5sum for a message object of type 'CarPosition"
  "88b1fb8816b2426364b93b43e1f32170")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CarPosition>)))
  "Returns full string definition for message of type '<CarPosition>"
  (cl:format cl:nil "Header header~%float64 xmean~%float64 ymean~%float64 zmean~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CarPosition)))
  "Returns full string definition for message of type 'CarPosition"
  (cl:format cl:nil "Header header~%float64 xmean~%float64 ymean~%float64 zmean~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CarPosition>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CarPosition>))
  "Converts a ROS message object to a list"
  (cl:list 'CarPosition
    (cl:cons ':header (header msg))
    (cl:cons ':xmean (xmean msg))
    (cl:cons ':ymean (ymean msg))
    (cl:cons ':zmean (zmean msg))
))
