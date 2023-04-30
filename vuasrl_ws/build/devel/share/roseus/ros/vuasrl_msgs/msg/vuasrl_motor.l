;; Auto-generated. Do not edit!


(when (boundp 'vuasrl_msgs::vuasrl_motor)
  (if (not (find-package "VUASRL_MSGS"))
    (make-package "VUASRL_MSGS"))
  (shadow 'vuasrl_motor (find-package "VUASRL_MSGS")))
(unless (find-package "VUASRL_MSGS::VUASRL_MOTOR")
  (make-package "VUASRL_MSGS::VUASRL_MOTOR"))

(in-package "ROS")
;;//! \htmlinclude vuasrl_motor.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass vuasrl_msgs::vuasrl_motor
  :super ros::object
  :slots (_header _angle _speed ))

(defmethod vuasrl_msgs::vuasrl_motor
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:angle __angle) 0)
    ((:speed __speed) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _angle (round __angle))
   (setq _speed (round __speed))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _angle
    4
    ;; int32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _angle
       (write-long _angle s)
     ;; int32 _speed
       (write-long _speed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _angle
     (setq _angle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _speed
     (setq _speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get vuasrl_msgs::vuasrl_motor :md5sum-) "776d1e41053a9ee231920fdd113d2d0a")
(setf (get vuasrl_msgs::vuasrl_motor :datatype-) "vuasrl_msgs/vuasrl_motor")
(setf (get vuasrl_msgs::vuasrl_motor :definition-)
      "Header header
int32 angle
int32 speed


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :vuasrl_msgs/vuasrl_motor "776d1e41053a9ee231920fdd113d2d0a")


