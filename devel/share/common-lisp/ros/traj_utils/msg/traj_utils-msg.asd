
(cl:in-package :asdf)

(defsystem "traj_utils-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Assignment" :depends-on ("_package_Assignment"))
    (:file "_package_Assignment" :depends-on ("_package"))
    (:file "DataDisp" :depends-on ("_package_DataDisp"))
    (:file "_package_DataDisp" :depends-on ("_package"))
    (:file "PolyTraj" :depends-on ("_package_PolyTraj"))
    (:file "_package_PolyTraj" :depends-on ("_package"))
  ))