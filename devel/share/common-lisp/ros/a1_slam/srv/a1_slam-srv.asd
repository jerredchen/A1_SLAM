
(cl:in-package :asdf)

(defsystem "a1_slam-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GtsamResults" :depends-on ("_package_GtsamResults"))
    (:file "_package_GtsamResults" :depends-on ("_package"))
  ))