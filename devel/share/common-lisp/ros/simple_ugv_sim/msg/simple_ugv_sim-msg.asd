
(cl:in-package :asdf)

(defsystem "simple_ugv_sim-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "UGVState" :depends-on ("_package_UGVState"))
    (:file "_package_UGVState" :depends-on ("_package"))
  ))