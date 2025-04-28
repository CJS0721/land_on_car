;; Auto-generated. Do not edit!


(when (boundp 'simple_ugv_sim::UGVState)
  (if (not (find-package "SIMPLE_UGV_SIM"))
    (make-package "SIMPLE_UGV_SIM"))
  (shadow 'UGVState (find-package "SIMPLE_UGV_SIM")))
(unless (find-package "SIMPLE_UGV_SIM::UGVSTATE")
  (make-package "SIMPLE_UGV_SIM::UGVSTATE"))

(in-package "ROS")
;;//! \htmlinclude UGVState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass simple_ugv_sim::UGVState
  :super ros::object
  :slots (_header _x _y _z _yaw _v_linear _v_angular _dir_x _dir_y ))

(defmethod simple_ugv_sim::UGVState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:yaw __yaw) 0.0)
    ((:v_linear __v_linear) 0.0)
    ((:v_angular __v_angular) 0.0)
    ((:dir_x __dir_x) 0.0)
    ((:dir_y __dir_y) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _yaw (float __yaw))
   (setq _v_linear (float __v_linear))
   (setq _v_angular (float __v_angular))
   (setq _dir_x (float __dir_x))
   (setq _dir_y (float __dir_y))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:v_linear
   (&optional __v_linear)
   (if __v_linear (setq _v_linear __v_linear)) _v_linear)
  (:v_angular
   (&optional __v_angular)
   (if __v_angular (setq _v_angular __v_angular)) _v_angular)
  (:dir_x
   (&optional __dir_x)
   (if __dir_x (setq _dir_x __dir_x)) _dir_x)
  (:dir_y
   (&optional __dir_y)
   (if __dir_y (setq _dir_y __dir_y)) _dir_y)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _yaw
    8
    ;; float64 _v_linear
    8
    ;; float64 _v_angular
    8
    ;; float64 _dir_x
    8
    ;; float64 _dir_y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v_linear
       (sys::poke _v_linear (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v_angular
       (sys::poke _v_angular (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _dir_x
       (sys::poke _dir_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _dir_y
       (sys::poke _dir_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v_linear
     (setq _v_linear (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v_angular
     (setq _v_angular (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _dir_x
     (setq _dir_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _dir_y
     (setq _dir_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get simple_ugv_sim::UGVState :md5sum-) "5aa9d6604c104759394b83817d0df5c3")
(setf (get simple_ugv_sim::UGVState :datatype-) "simple_ugv_sim/UGVState")
(setf (get simple_ugv_sim::UGVState :definition-)
      "Header header

# 位置信息
float64 x
float64 y
float64 z
float64 yaw

# 速度信息
float64 v_linear
float64 v_angular

# 方向向量
float64 dir_x
float64 dir_y

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



(provide :simple_ugv_sim/UGVState "5aa9d6604c104759394b83817d0df5c3")


