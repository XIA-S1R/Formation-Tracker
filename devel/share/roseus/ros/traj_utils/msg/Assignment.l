;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::Assignment)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'Assignment (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::ASSIGNMENT")
  (make-package "TRAJ_UTILS::ASSIGNMENT"))

(in-package "ROS")
;;//! \htmlinclude Assignment.msg.html


(defclass traj_utils::Assignment
  :super ros::object
  :slots (_assignment ))

(defmethod traj_utils::Assignment
  (:init
   (&key
    ((:assignment __assignment) (make-array 10 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _assignment __assignment)
   self)
  (:assignment
   (&optional __assignment)
   (if __assignment (setq _assignment __assignment)) _assignment)
  (:serialization-length
   ()
   (+
    ;; uint32[10] _assignment
    (* 4    10)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32[10] _assignment
     (dotimes (i 10)
       (write-long (elt _assignment i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32[10] _assignment
   (dotimes (i (length _assignment))
     (setf (elt _assignment i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get traj_utils::Assignment :md5sum-) "fc09302ffe2353e20a7ca4e9c550e719")
(setf (get traj_utils::Assignment :datatype-) "traj_utils/Assignment")
(setf (get traj_utils::Assignment :definition-)
      "uint32[10] assignment

")



(provide :traj_utils/Assignment "fc09302ffe2353e20a7ca4e9c550e719")


