;; Auto-generated. Do not edit!


(when (boundp 'unitree_legged_msgs::IMU)
  (if (not (find-package "UNITREE_LEGGED_MSGS"))
    (make-package "UNITREE_LEGGED_MSGS"))
  (shadow 'IMU (find-package "UNITREE_LEGGED_MSGS")))
(unless (find-package "UNITREE_LEGGED_MSGS::IMU")
  (make-package "UNITREE_LEGGED_MSGS::IMU"))

(in-package "ROS")
;;//! \htmlinclude IMU.msg.html


(defclass unitree_legged_msgs::IMU
  :super ros::object
  :slots (_quaternion _gyroscope _accelerometer _temperature ))

(defmethod unitree_legged_msgs::IMU
  (:init
   (&key
    ((:quaternion __quaternion) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:gyroscope __gyroscope) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:accelerometer __accelerometer) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:temperature __temperature) 0)
    )
   (send-super :init)
   (setq _quaternion __quaternion)
   (setq _gyroscope __gyroscope)
   (setq _accelerometer __accelerometer)
   (setq _temperature (round __temperature))
   self)
  (:quaternion
   (&optional __quaternion)
   (if __quaternion (setq _quaternion __quaternion)) _quaternion)
  (:gyroscope
   (&optional __gyroscope)
   (if __gyroscope (setq _gyroscope __gyroscope)) _gyroscope)
  (:accelerometer
   (&optional __accelerometer)
   (if __accelerometer (setq _accelerometer __accelerometer)) _accelerometer)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:serialization-length
   ()
   (+
    ;; float32[4] _quaternion
    (* 4    4)
    ;; float32[3] _gyroscope
    (* 4    3)
    ;; float32[3] _accelerometer
    (* 4    3)
    ;; int8 _temperature
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[4] _quaternion
     (dotimes (i 4)
       (sys::poke (elt _quaternion i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _gyroscope
     (dotimes (i 3)
       (sys::poke (elt _gyroscope i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _accelerometer
     (dotimes (i 3)
       (sys::poke (elt _accelerometer i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int8 _temperature
       (write-byte _temperature s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[4] _quaternion
   (dotimes (i (length _quaternion))
     (setf (elt _quaternion i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _gyroscope
   (dotimes (i (length _gyroscope))
     (setf (elt _gyroscope i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _accelerometer
   (dotimes (i (length _accelerometer))
     (setf (elt _accelerometer i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; int8 _temperature
     (setq _temperature (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _temperature 127) (setq _temperature (- _temperature 256)))
   ;;
   self)
  )

(setf (get unitree_legged_msgs::IMU :md5sum-) "dd4bb4e42aa2f15aa1fb1b6a3c3752cb")
(setf (get unitree_legged_msgs::IMU :datatype-) "unitree_legged_msgs/IMU")
(setf (get unitree_legged_msgs::IMU :definition-)
      "float32[4] quaternion
float32[3] gyroscope
float32[3] accelerometer
int8 temperature
")



(provide :unitree_legged_msgs/IMU "dd4bb4e42aa2f15aa1fb1b6a3c3752cb")


