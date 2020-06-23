
(cl:in-package :asdf)

(defsystem "milestone2-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "pathPlanning" :depends-on ("_package_pathPlanning"))
    (:file "_package_pathPlanning" :depends-on ("_package"))
  ))