;; Auto-generated. Do not edit!


(when (boundp 'unitree_legged_msgs::MotorCmd)
  (if (not (find-package "UNITREE_LEGGED_MSGS"))
    (make-package "UNITREE_LEGGED_MSGS"))
  (shadow 'MotorCmd (find-package "UNITREE_LEGGED_MSGS")))
(unless (find-package "UNITREE_LEGGED_MSGS::MOTORCMD")
  (make-package "UNITREE_LEGGED_MSGS::MOTORCMD"))

(in-package "ROS")
;;//! \htmlinclude MotorCmd.msg.html


(defclass unitree_legged_msgs::MotorCmd
  :super ros::object
  :slots (_mode _q _dq _tau _Kp _Kd _reserve ))

(defmethod unitree_legged_msgs::MotorCmd
  (:init
   (&key
    ((:mode __mode) 0)
    ((:q __q) 0.0)
    ((:dq __dq) 0.0)
    ((:tau __tau) 0.0)
    ((:Kp __Kp) 0.0)
    ((:Kd __Kd) 0.0)
    ((:reserve __reserve) (make-array 3 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _q (float __q))
   (setq _dq (float __dq))
   (setq _tau (float __tau))
   (setq _Kp (float __Kp))
   (setq _Kd (float __Kd))
   (setq _reserve __reserve)
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:q
   (&optional __q)
   (if __q (setq _q __q)) _q)
  (:dq
   (&optional __dq)
   (if __dq (setq _dq __dq)) _dq)
  (:tau
   (&optional __tau)
   (if __tau (setq _tau __tau)) _tau)
  (:Kp
   (&optional __Kp)
   (if __Kp (setq _Kp __Kp)) _Kp)
  (:Kd
   (&optional __Kd)
   (if __Kd (setq _Kd __Kd)) _Kd)
  (:reserve
   (&optional __reserve)
   (if __reserve (setq _reserve __reserve)) _reserve)
  (:serialization-length
   ()
   (+
    ;; uint8 _mode
    1
    ;; float32 _q
    4
    ;; float32 _dq
    4
    ;; float32 _tau
    4
    ;; float32 _Kp
    4
    ;; float32 _Kd
    4
    ;; uint32[3] _reserve
    (* 4    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _mode
       (write-byte _mode s)
     ;; float32 _q
       (sys::poke _q (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dq
       (sys::poke _dq (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tau
       (sys::poke _tau (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Kp
       (sys::poke _Kp (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Kd
       (sys::poke _Kd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32[3] _reserve
     (dotimes (i 3)
       (write-long (elt _reserve i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _q
     (setq _q (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dq
     (setq _dq (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tau
     (setq _tau (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Kp
     (setq _Kp (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Kd
     (setq _Kd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32[3] _reserve
   (dotimes (i (length _reserve))
     (setf (elt _reserve i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get unitree_legged_msgs::MotorCmd :md5sum-) "bbb3b7d91319c3a1b99055f0149ba221")
(setf (get unitree_legged_msgs::MotorCmd :datatype-) "unitree_legged_msgs/MotorCmd")
(setf (get unitree_legged_msgs::MotorCmd :definition-)
      "uint8 mode           # motor target mode
float32 q            # motor target position
float32 dq           # motor target velocity
float32 tau          # motor target torque
float32 Kp           # motor spring stiffness coefficient
float32 Kd           # motor damper coefficient
uint32[3] reserve    # motor target torque
")



(provide :unitree_legged_msgs/MotorCmd "bbb3b7d91319c3a1b99055f0149ba221")


