;; Auto-generated. Do not edit!


(when (boundp 'unitree_legged_msgs::LowState)
  (if (not (find-package "UNITREE_LEGGED_MSGS"))
    (make-package "UNITREE_LEGGED_MSGS"))
  (shadow 'LowState (find-package "UNITREE_LEGGED_MSGS")))
(unless (find-package "UNITREE_LEGGED_MSGS::LOWSTATE")
  (make-package "UNITREE_LEGGED_MSGS::LOWSTATE"))

(in-package "ROS")
;;//! \htmlinclude LowState.msg.html


(defclass unitree_legged_msgs::LowState
  :super ros::object
  :slots (_levelFlag _commVersion _robotID _SN _bandWidth _imu _motorState _footForce _footForceEst _tick _wirelessRemote _reserve _crc _eeForceRaw _eeForce _position _velocity _velocity_w ))

(defmethod unitree_legged_msgs::LowState
  (:init
   (&key
    ((:levelFlag __levelFlag) 0)
    ((:commVersion __commVersion) 0)
    ((:robotID __robotID) 0)
    ((:SN __SN) 0)
    ((:bandWidth __bandWidth) 0)
    ((:imu __imu) (instance unitree_legged_msgs::IMU :init))
    ((:motorState __motorState) (let (r) (dotimes (i 20) (push (instance unitree_legged_msgs::MotorState :init) r)) r))
    ((:footForce __footForce) (make-array 4 :initial-element 0 :element-type :integer))
    ((:footForceEst __footForceEst) (make-array 4 :initial-element 0 :element-type :integer))
    ((:tick __tick) 0)
    ((:wirelessRemote __wirelessRemote) (make-array 40 :initial-element 0 :element-type :char))
    ((:reserve __reserve) 0)
    ((:crc __crc) 0)
    ((:eeForceRaw __eeForceRaw) (let (r) (dotimes (i 4) (push (instance unitree_legged_msgs::Cartesian :init) r)) r))
    ((:eeForce __eeForce) (let (r) (dotimes (i 4) (push (instance unitree_legged_msgs::Cartesian :init) r)) r))
    ((:position __position) (instance unitree_legged_msgs::Cartesian :init))
    ((:velocity __velocity) (instance unitree_legged_msgs::Cartesian :init))
    ((:velocity_w __velocity_w) (instance unitree_legged_msgs::Cartesian :init))
    )
   (send-super :init)
   (setq _levelFlag (round __levelFlag))
   (setq _commVersion (round __commVersion))
   (setq _robotID (round __robotID))
   (setq _SN (round __SN))
   (setq _bandWidth (round __bandWidth))
   (setq _imu __imu)
   (setq _motorState __motorState)
   (setq _footForce __footForce)
   (setq _footForceEst __footForceEst)
   (setq _tick (round __tick))
   (setq _wirelessRemote __wirelessRemote)
   (setq _reserve (round __reserve))
   (setq _crc (round __crc))
   (setq _eeForceRaw __eeForceRaw)
   (setq _eeForce __eeForce)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _velocity_w __velocity_w)
   self)
  (:levelFlag
   (&optional __levelFlag)
   (if __levelFlag (setq _levelFlag __levelFlag)) _levelFlag)
  (:commVersion
   (&optional __commVersion)
   (if __commVersion (setq _commVersion __commVersion)) _commVersion)
  (:robotID
   (&optional __robotID)
   (if __robotID (setq _robotID __robotID)) _robotID)
  (:SN
   (&optional __SN)
   (if __SN (setq _SN __SN)) _SN)
  (:bandWidth
   (&optional __bandWidth)
   (if __bandWidth (setq _bandWidth __bandWidth)) _bandWidth)
  (:imu
   (&rest __imu)
   (if (keywordp (car __imu))
       (send* _imu __imu)
     (progn
       (if __imu (setq _imu (car __imu)))
       _imu)))
  (:motorState
   (&rest __motorState)
   (if (keywordp (car __motorState))
       (send* _motorState __motorState)
     (progn
       (if __motorState (setq _motorState (car __motorState)))
       _motorState)))
  (:footForce
   (&optional __footForce)
   (if __footForce (setq _footForce __footForce)) _footForce)
  (:footForceEst
   (&optional __footForceEst)
   (if __footForceEst (setq _footForceEst __footForceEst)) _footForceEst)
  (:tick
   (&optional __tick)
   (if __tick (setq _tick __tick)) _tick)
  (:wirelessRemote
   (&optional __wirelessRemote)
   (if __wirelessRemote (setq _wirelessRemote __wirelessRemote)) _wirelessRemote)
  (:reserve
   (&optional __reserve)
   (if __reserve (setq _reserve __reserve)) _reserve)
  (:crc
   (&optional __crc)
   (if __crc (setq _crc __crc)) _crc)
  (:eeForceRaw
   (&rest __eeForceRaw)
   (if (keywordp (car __eeForceRaw))
       (send* _eeForceRaw __eeForceRaw)
     (progn
       (if __eeForceRaw (setq _eeForceRaw (car __eeForceRaw)))
       _eeForceRaw)))
  (:eeForce
   (&rest __eeForce)
   (if (keywordp (car __eeForce))
       (send* _eeForce __eeForce)
     (progn
       (if __eeForce (setq _eeForce (car __eeForce)))
       _eeForce)))
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:velocity_w
   (&rest __velocity_w)
   (if (keywordp (car __velocity_w))
       (send* _velocity_w __velocity_w)
     (progn
       (if __velocity_w (setq _velocity_w (car __velocity_w)))
       _velocity_w)))
  (:serialization-length
   ()
   (+
    ;; uint8 _levelFlag
    1
    ;; uint16 _commVersion
    2
    ;; uint16 _robotID
    2
    ;; uint32 _SN
    4
    ;; uint8 _bandWidth
    1
    ;; unitree_legged_msgs/IMU _imu
    (send _imu :serialization-length)
    ;; unitree_legged_msgs/MotorState[20] _motorState
    (apply #'+ (send-all _motorState :serialization-length))
    ;; int16[4] _footForce
    (* 2    4)
    ;; int16[4] _footForceEst
    (* 2    4)
    ;; uint32 _tick
    4
    ;; uint8[40] _wirelessRemote
    (* 1    40)
    ;; uint32 _reserve
    4
    ;; uint32 _crc
    4
    ;; unitree_legged_msgs/Cartesian[4] _eeForceRaw
    (apply #'+ (send-all _eeForceRaw :serialization-length))
    ;; unitree_legged_msgs/Cartesian[4] _eeForce
    (apply #'+ (send-all _eeForce :serialization-length))
    ;; unitree_legged_msgs/Cartesian _position
    (send _position :serialization-length)
    ;; unitree_legged_msgs/Cartesian _velocity
    (send _velocity :serialization-length)
    ;; unitree_legged_msgs/Cartesian _velocity_w
    (send _velocity_w :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _levelFlag
       (write-byte _levelFlag s)
     ;; uint16 _commVersion
       (write-word _commVersion s)
     ;; uint16 _robotID
       (write-word _robotID s)
     ;; uint32 _SN
       (write-long _SN s)
     ;; uint8 _bandWidth
       (write-byte _bandWidth s)
     ;; unitree_legged_msgs/IMU _imu
       (send _imu :serialize s)
     ;; unitree_legged_msgs/MotorState[20] _motorState
     (dolist (elem _motorState)
       (send elem :serialize s)
       )
     ;; int16[4] _footForce
     (dotimes (i 4)
       (write-word (elt _footForce i) s)
       )
     ;; int16[4] _footForceEst
     (dotimes (i 4)
       (write-word (elt _footForceEst i) s)
       )
     ;; uint32 _tick
       (write-long _tick s)
     ;; uint8[40] _wirelessRemote
     (princ _wirelessRemote s)
     ;; uint32 _reserve
       (write-long _reserve s)
     ;; uint32 _crc
       (write-long _crc s)
     ;; unitree_legged_msgs/Cartesian[4] _eeForceRaw
     (dolist (elem _eeForceRaw)
       (send elem :serialize s)
       )
     ;; unitree_legged_msgs/Cartesian[4] _eeForce
     (dolist (elem _eeForce)
       (send elem :serialize s)
       )
     ;; unitree_legged_msgs/Cartesian _position
       (send _position :serialize s)
     ;; unitree_legged_msgs/Cartesian _velocity
       (send _velocity :serialize s)
     ;; unitree_legged_msgs/Cartesian _velocity_w
       (send _velocity_w :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _levelFlag
     (setq _levelFlag (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _commVersion
     (setq _commVersion (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _robotID
     (setq _robotID (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint32 _SN
     (setq _SN (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _bandWidth
     (setq _bandWidth (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; unitree_legged_msgs/IMU _imu
     (send _imu :deserialize buf ptr-) (incf ptr- (send _imu :serialization-length))
   ;; unitree_legged_msgs/MotorState[20] _motorState
   (dotimes (i 20)
     (send (elt _motorState i) :deserialize buf ptr-) (incf ptr- (send (elt _motorState i) :serialization-length))
     )
   ;; int16[4] _footForce
   (dotimes (i (length _footForce))
     (setf (elt _footForce i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; int16[4] _footForceEst
   (dotimes (i (length _footForceEst))
     (setf (elt _footForceEst i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint32 _tick
     (setq _tick (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8[40] _wirelessRemote
   (setq _wirelessRemote (make-array 40 :element-type :char))
   (replace _wirelessRemote buf :start2 ptr-) (incf ptr- 40)
   ;; uint32 _reserve
     (setq _reserve (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _crc
     (setq _crc (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; unitree_legged_msgs/Cartesian[4] _eeForceRaw
   (dotimes (i 4)
     (send (elt _eeForceRaw i) :deserialize buf ptr-) (incf ptr- (send (elt _eeForceRaw i) :serialization-length))
     )
   ;; unitree_legged_msgs/Cartesian[4] _eeForce
   (dotimes (i 4)
     (send (elt _eeForce i) :deserialize buf ptr-) (incf ptr- (send (elt _eeForce i) :serialization-length))
     )
   ;; unitree_legged_msgs/Cartesian _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; unitree_legged_msgs/Cartesian _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; unitree_legged_msgs/Cartesian _velocity_w
     (send _velocity_w :deserialize buf ptr-) (incf ptr- (send _velocity_w :serialization-length))
   ;;
   self)
  )

(setf (get unitree_legged_msgs::LowState :md5sum-) "cef9d4f6b5a89bd6330896affb1bca88")
(setf (get unitree_legged_msgs::LowState :datatype-) "unitree_legged_msgs/LowState")
(setf (get unitree_legged_msgs::LowState :definition-)
      "uint8 levelFlag
uint16 commVersion              # Old version Aliengo does not have
uint16 robotID                  # Old version Aliengo does not have
uint32 SN                       # Old version Aliengo does not have
uint8 bandWidth                 # Old version Aliengo does not have
IMU imu
MotorState[20] motorState
int16[4] footForce              # force sensors     # Old version Aliengo is different
int16[4] footForceEst           # force sensors     # Old version Aliengo does not have
uint32 tick                     # reference real-time from motion controller (unit: us)
uint8[40] wirelessRemote        # wireless commands
uint32 reserve                  # Old version Aliengo does not have
uint32 crc

# Old version Aliengo does not have:
Cartesian[4] eeForceRaw
Cartesian[4] eeForce          #it's a 1-DOF force infact, but we use 3-DOF here just for visualization 
Cartesian position            # will delete
Cartesian velocity            # will delete
Cartesian velocity_w            # will delete

================================================================================
MSG: unitree_legged_msgs/IMU
float32[4] quaternion
float32[3] gyroscope
float32[3] accelerometer
int8 temperature
================================================================================
MSG: unitree_legged_msgs/MotorState
uint8 mode           # motor current mode 
float32 q            # motor current position（rad）
float32 dq           # motor current speed（rad/s）
float32 ddq          # motor current speed（rad/s）
float32 tauEst       # current estimated output torque（N*m）
float32 q_raw        # motor current position（rad）
float32 dq_raw       # motor current speed（rad/s）
float32 ddq_raw      # motor current speed（rad/s）
int8 temperature     # motor temperature（slow conduction of temperature leads to lag）
uint32[2] reserve
================================================================================
MSG: unitree_legged_msgs/Cartesian
float32 x
float32 y
float32 z
")



(provide :unitree_legged_msgs/LowState "cef9d4f6b5a89bd6330896affb1bca88")


