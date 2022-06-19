;; Auto-generated. Do not edit!


(when (boundp 'unitree_legged_msgs::MotorState)
  (if (not (find-package "UNITREE_LEGGED_MSGS"))
    (make-package "UNITREE_LEGGED_MSGS"))
  (shadow 'MotorState (find-package "UNITREE_LEGGED_MSGS")))
(unless (find-package "UNITREE_LEGGED_MSGS::MOTORSTATE")
  (make-package "UNITREE_LEGGED_MSGS::MOTORSTATE"))

(in-package "ROS")
;;//! \htmlinclude MotorState.msg.html


(defclass unitree_legged_msgs::MotorState
  :super ros::object
  :slots (_mode _q _dq _ddq _tauEst _q_raw _dq_raw _ddq_raw _temperature _reserve ))

(defmethod unitree_legged_msgs::MotorState
  (:init
   (&key
    ((:mode __mode) 0)
    ((:q __q) 0.0)
    ((:dq __dq) 0.0)
    ((:ddq __ddq) 0.0)
    ((:tauEst __tauEst) 0.0)
    ((:q_raw __q_raw) 0.0)
    ((:dq_raw __dq_raw) 0.0)
    ((:ddq_raw __ddq_raw) 0.0)
    ((:temperature __temperature) 0)
    ((:reserve __reserve) (make-array 2 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _q (float __q))
   (setq _dq (float __dq))
   (setq _ddq (float __ddq))
   (setq _tauEst (float __tauEst))
   (setq _q_raw (float __q_raw))
   (setq _dq_raw (float __dq_raw))
   (setq _ddq_raw (float __ddq_raw))
   (setq _temperature (round __temperature))
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
  (:ddq
   (&optional __ddq)
   (if __ddq (setq _ddq __ddq)) _ddq)
  (:tauEst
   (&optional __tauEst)
   (if __tauEst (setq _tauEst __tauEst)) _tauEst)
  (:q_raw
   (&optional __q_raw)
   (if __q_raw (setq _q_raw __q_raw)) _q_raw)
  (:dq_raw
   (&optional __dq_raw)
   (if __dq_raw (setq _dq_raw __dq_raw)) _dq_raw)
  (:ddq_raw
   (&optional __ddq_raw)
   (if __ddq_raw (setq _ddq_raw __ddq_raw)) _ddq_raw)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
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
    ;; float32 _ddq
    4
    ;; float32 _tauEst
    4
    ;; float32 _q_raw
    4
    ;; float32 _dq_raw
    4
    ;; float32 _ddq_raw
    4
    ;; int8 _temperature
    1
    ;; uint32[2] _reserve
    (* 4    2)
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
     ;; float32 _ddq
       (sys::poke _ddq (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tauEst
       (sys::poke _tauEst (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _q_raw
       (sys::poke _q_raw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dq_raw
       (sys::poke _dq_raw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ddq_raw
       (sys::poke _ddq_raw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int8 _temperature
       (write-byte _temperature s)
     ;; uint32[2] _reserve
     (dotimes (i 2)
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
   ;; float32 _ddq
     (setq _ddq (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tauEst
     (setq _tauEst (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _q_raw
     (setq _q_raw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dq_raw
     (setq _dq_raw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ddq_raw
     (setq _ddq_raw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int8 _temperature
     (setq _temperature (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _temperature 127) (setq _temperature (- _temperature 256)))
   ;; uint32[2] _reserve
   (dotimes (i (length _reserve))
     (setf (elt _reserve i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get unitree_legged_msgs::MotorState :md5sum-) "94c55ee3b7852be2bd437b20ce12a254")
(setf (get unitree_legged_msgs::MotorState :datatype-) "unitree_legged_msgs/MotorState")
(setf (get unitree_legged_msgs::MotorState :definition-)
      "uint8 mode           # motor current mode 
float32 q            # motor current position（rad）
float32 dq           # motor current speed（rad/s）
float32 ddq          # motor current speed（rad/s）
float32 tauEst       # current estimated output torque（N*m）
float32 q_raw        # motor current position（rad）
float32 dq_raw       # motor current speed（rad/s）
float32 ddq_raw      # motor current speed（rad/s）
int8 temperature     # motor temperature（slow conduction of temperature leads to lag）
uint32[2] reserve
")



(provide :unitree_legged_msgs/MotorState "94c55ee3b7852be2bd437b20ce12a254")


