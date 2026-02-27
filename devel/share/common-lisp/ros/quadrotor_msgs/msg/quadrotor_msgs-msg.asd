
(cl:in-package :asdf)

(defsystem "quadrotor_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AuxCommand" :depends-on ("_package_AuxCommand"))
    (:file "_package_AuxCommand" :depends-on ("_package"))
    (:file "OccMap3d" :depends-on ("_package_OccMap3d"))
    (:file "_package_OccMap3d" :depends-on ("_package"))
    (:file "PolyTraj" :depends-on ("_package_PolyTraj"))
    (:file "_package_PolyTraj" :depends-on ("_package"))
    (:file "PositionCommand" :depends-on ("_package_PositionCommand"))
    (:file "_package_PositionCommand" :depends-on ("_package"))
    (:file "Px4ctrlDebug" :depends-on ("_package_Px4ctrlDebug"))
    (:file "_package_Px4ctrlDebug" :depends-on ("_package"))
    (:file "ReplanState" :depends-on ("_package_ReplanState"))
    (:file "_package_ReplanState" :depends-on ("_package"))
    (:file "SO3Command" :depends-on ("_package_SO3Command"))
    (:file "_package_SO3Command" :depends-on ("_package"))
    (:file "TakeoffLand" :depends-on ("_package_TakeoffLand"))
    (:file "_package_TakeoffLand" :depends-on ("_package"))
  ))