; Auto-generated. Do not edit!


(cl:in-package traj_utils-msg)


;//! \htmlinclude Assignment.msg.html

(cl:defclass <Assignment> (roslisp-msg-protocol:ros-message)
  ((assignment
    :reader assignment
    :initarg :assignment
    :type (cl:vector cl:integer)
   :initform (cl:make-array 10 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass Assignment (<Assignment>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Assignment>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Assignment)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-msg:<Assignment> is deprecated: use traj_utils-msg:Assignment instead.")))

(cl:ensure-generic-function 'assignment-val :lambda-list '(m))
(cl:defmethod assignment-val ((m <Assignment>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:assignment-val is deprecated.  Use traj_utils-msg:assignment instead.")
  (assignment m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Assignment>) ostream)
  "Serializes a message object of type '<Assignment>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'assignment))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Assignment>) istream)
  "Deserializes a message object of type '<Assignment>"
  (cl:setf (cl:slot-value msg 'assignment) (cl:make-array 10))
  (cl:let ((vals (cl:slot-value msg 'assignment)))
    (cl:dotimes (i 10)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Assignment>)))
  "Returns string type for a message object of type '<Assignment>"
  "traj_utils/Assignment")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Assignment)))
  "Returns string type for a message object of type 'Assignment"
  "traj_utils/Assignment")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Assignment>)))
  "Returns md5sum for a message object of type '<Assignment>"
  "fc09302ffe2353e20a7ca4e9c550e719")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Assignment)))
  "Returns md5sum for a message object of type 'Assignment"
  "fc09302ffe2353e20a7ca4e9c550e719")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Assignment>)))
  "Returns full string definition for message of type '<Assignment>"
  (cl:format cl:nil "uint32[10] assignment~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Assignment)))
  "Returns full string definition for message of type 'Assignment"
  (cl:format cl:nil "uint32[10] assignment~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Assignment>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'assignment) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Assignment>))
  "Converts a ROS message object to a list"
  (cl:list 'Assignment
    (cl:cons ':assignment (assignment msg))
))
