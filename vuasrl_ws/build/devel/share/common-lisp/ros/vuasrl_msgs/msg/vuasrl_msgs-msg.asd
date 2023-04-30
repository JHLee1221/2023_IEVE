
(cl:in-package :asdf)

(defsystem "vuasrl_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "vuasrl_motor" :depends-on ("_package_vuasrl_motor"))
    (:file "_package_vuasrl_motor" :depends-on ("_package"))
    (:file "vuasrl_ultrasounds" :depends-on ("_package_vuasrl_ultrasounds"))
    (:file "_package_vuasrl_ultrasounds" :depends-on ("_package"))
  ))