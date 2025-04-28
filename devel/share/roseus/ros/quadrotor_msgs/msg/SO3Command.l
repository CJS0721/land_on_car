;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::SO3Command)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'SO3Command (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::SO3COMMAND")
  (make-package "QUADROTOR_MSGS::SO3COMMAND"))

(in-package "ROS")
;;//! \htmlinclude SO3Command.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::SO3Command
  :super ros::object
  :slots (_header _force _orientation _kR _kOm _aux _omega_d _omega_d_dot ))

(defmethod quadrotor_msgs::SO3Command
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:force __force) (instance geometry_msgs::Vector3 :init))
    ((:orientation __orientation) (instance geometry_msgs::Quaternion :init))
    ((:kR __kR) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:kOm __kOm) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:aux __aux) (instance quadrotor_msgs::AuxCommand :init))
    ((:omega_d __omega_d) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:omega_d_dot __omega_d_dot) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _force __force)
   (setq _orientation __orientation)
   (setq _kR __kR)
   (setq _kOm __kOm)
   (setq _aux __aux)
   (setq _omega_d __omega_d)
   (setq _omega_d_dot __omega_d_dot)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:force
   (&rest __force)
   (if (keywordp (car __force))
       (send* _force __force)
     (progn
       (if __force (setq _force (car __force)))
       _force)))
  (:orientation
   (&rest __orientation)
   (if (keywordp (car __orientation))
       (send* _orientation __orientation)
     (progn
       (if __orientation (setq _orientation (car __orientation)))
       _orientation)))
  (:kR
   (&optional __kR)
   (if __kR (setq _kR __kR)) _kR)
  (:kOm
   (&optional __kOm)
   (if __kOm (setq _kOm __kOm)) _kOm)
  (:aux
   (&rest __aux)
   (if (keywordp (car __aux))
       (send* _aux __aux)
     (progn
       (if __aux (setq _aux (car __aux)))
       _aux)))
  (:omega_d
   (&optional __omega_d)
   (if __omega_d (setq _omega_d __omega_d)) _omega_d)
  (:omega_d_dot
   (&optional __omega_d_dot)
   (if __omega_d_dot (setq _omega_d_dot __omega_d_dot)) _omega_d_dot)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _force
    (send _force :serialization-length)
    ;; geometry_msgs/Quaternion _orientation
    (send _orientation :serialization-length)
    ;; float64[3] _kR
    (* 8    3)
    ;; float64[3] _kOm
    (* 8    3)
    ;; quadrotor_msgs/AuxCommand _aux
    (send _aux :serialization-length)
    ;; float64[3] _omega_d
    (* 8    3)
    ;; float64[3] _omega_d_dot
    (* 8    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _force
       (send _force :serialize s)
     ;; geometry_msgs/Quaternion _orientation
       (send _orientation :serialize s)
     ;; float64[3] _kR
     (dotimes (i 3)
       (sys::poke (elt _kR i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _kOm
     (dotimes (i 3)
       (sys::poke (elt _kOm i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; quadrotor_msgs/AuxCommand _aux
       (send _aux :serialize s)
     ;; float64[3] _omega_d
     (dotimes (i 3)
       (sys::poke (elt _omega_d i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _omega_d_dot
     (dotimes (i 3)
       (sys::poke (elt _omega_d_dot i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _force
     (send _force :deserialize buf ptr-) (incf ptr- (send _force :serialization-length))
   ;; geometry_msgs/Quaternion _orientation
     (send _orientation :deserialize buf ptr-) (incf ptr- (send _orientation :serialization-length))
   ;; float64[3] _kR
   (dotimes (i (length _kR))
     (setf (elt _kR i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _kOm
   (dotimes (i (length _kOm))
     (setf (elt _kOm i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; quadrotor_msgs/AuxCommand _aux
     (send _aux :deserialize buf ptr-) (incf ptr- (send _aux :serialization-length))
   ;; float64[3] _omega_d
   (dotimes (i (length _omega_d))
     (setf (elt _omega_d i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _omega_d_dot
   (dotimes (i (length _omega_d_dot))
     (setf (elt _omega_d_dot i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get quadrotor_msgs::SO3Command :md5sum-) "c82859493dd48c4c93d5b2ac92d97819")
(setf (get quadrotor_msgs::SO3Command :datatype-) "quadrotor_msgs/SO3Command")
(setf (get quadrotor_msgs::SO3Command :definition-)
      "Header header
geometry_msgs/Vector3 force
geometry_msgs/Quaternion orientation
float64[3] kR
float64[3] kOm
quadrotor_msgs/AuxCommand aux
float64[3] omega_d
float64[3] omega_d_dot
# geometry_msgs/Vector3 jerk   # 添加的三维jerk向量
# geometry_msgs/Vector3 snap   # 添加的三维snap向量
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

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: quadrotor_msgs/AuxCommand
float64 current_yaw
float64 kf_correction
float64[2] angle_corrections# Trims for roll, pitch
bool enable_motors
bool use_external_yaw

")



(provide :quadrotor_msgs/SO3Command "c82859493dd48c4c93d5b2ac92d97819")


