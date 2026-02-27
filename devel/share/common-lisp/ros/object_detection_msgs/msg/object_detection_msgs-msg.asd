
(cl:in-package :asdf)

(defsystem "object_detection_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BoundingBox" :depends-on ("_package_BoundingBox"))
    (:file "_package_BoundingBox" :depends-on ("_package"))
    (:file "BoundingBoxes" :depends-on ("_package_BoundingBoxes"))
    (:file "_package_BoundingBoxes" :depends-on ("_package"))
    (:file "CarPosition" :depends-on ("_package_CarPosition"))
    (:file "_package_CarPosition" :depends-on ("_package"))
    (:file "ObjectCount" :depends-on ("_package_ObjectCount"))
    (:file "_package_ObjectCount" :depends-on ("_package"))
    (:file "motorAngle" :depends-on ("_package_motorAngle"))
    (:file "_package_motorAngle" :depends-on ("_package"))
  ))