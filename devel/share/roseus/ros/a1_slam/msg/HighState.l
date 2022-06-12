;; Auto-generated. Do not edit!


(when (boundp 'a1_slam::HighState)
  (if (not (find-package "A1_SLAM"))
    (make-package "A1_SLAM"))
  (shadow 'HighState (find-package "A1_SLAM")))
(unless (find-package "A1_SLAM::HIGHSTATE")
  (make-package "A1_SLAM::HIGHSTATE"))

(in-package "ROS")
;;//! \htmlinclude HighState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass a1_slam::HighState
  :super ros::object
  :slots (_header _levelFlag _commVersion _robotID _SN _bandWidth _mode _imu _forwardSpeed _sideSpeed _rotateSpeed _bodyHeight _updownSpeed _forwardPosition _sidePosition _footPosition2Body _footSpeed2Body _footForce _footForceEst _tick _wirelessRemote _reserve _crc _eeForce _jointP ))

(defmethod a1_slam::HighState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:levelFlag __levelFlag) 0)
    ((:commVersion __commVersion) 0)
    ((:robotID __robotID) 0)
    ((:SN __SN) 0)
    ((:bandWidth __bandWidth) 0)
    ((:mode __mode) 0)
    ((:imu __imu) (instance a1_slam::IMU :init))
    ((:forwardSpeed __forwardSpeed) 0.0)
    ((:sideSpeed __sideSpeed) 0.0)
    ((:rotateSpeed __rotateSpeed) 0.0)
    ((:bodyHeight __bodyHeight) 0.0)
    ((:updownSpeed __updownSpeed) 0.0)
    ((:forwardPosition __forwardPosition) 0.0)
    ((:sidePosition __sidePosition) 0.0)
    ((:footPosition2Body __footPosition2Body) (let (r) (dotimes (i 4) (push (instance a1_slam::Cartesian :init) r)) r))
    ((:footSpeed2Body __footSpeed2Body) (let (r) (dotimes (i 4) (push (instance a1_slam::Cartesian :init) r)) r))
    ((:footForce __footForce) (make-array 4 :initial-element 0 :element-type :integer))
    ((:footForceEst __footForceEst) (make-array 4 :initial-element 0 :element-type :integer))
    ((:tick __tick) 0)
    ((:wirelessRemote __wirelessRemote) (make-array 40 :initial-element 0 :element-type :char))
    ((:reserve __reserve) 0)
    ((:crc __crc) 0)
    ((:eeForce __eeForce) (let (r) (dotimes (i 4) (push (instance a1_slam::Cartesian :init) r)) r))
    ((:jointP __jointP) (make-array 12 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _levelFlag (round __levelFlag))
   (setq _commVersion (round __commVersion))
   (setq _robotID (round __robotID))
   (setq _SN (round __SN))
   (setq _bandWidth (round __bandWidth))
   (setq _mode (round __mode))
   (setq _imu __imu)
   (setq _forwardSpeed (float __forwardSpeed))
   (setq _sideSpeed (float __sideSpeed))
   (setq _rotateSpeed (float __rotateSpeed))
   (setq _bodyHeight (float __bodyHeight))
   (setq _updownSpeed (float __updownSpeed))
   (setq _forwardPosition (float __forwardPosition))
   (setq _sidePosition (float __sidePosition))
   (setq _footPosition2Body __footPosition2Body)
   (setq _footSpeed2Body __footSpeed2Body)
   (setq _footForce __footForce)
   (setq _footForceEst __footForceEst)
   (setq _tick (round __tick))
   (setq _wirelessRemote __wirelessRemote)
   (setq _reserve (round __reserve))
   (setq _crc (round __crc))
   (setq _eeForce __eeForce)
   (setq _jointP __jointP)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
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
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:imu
   (&rest __imu)
   (if (keywordp (car __imu))
       (send* _imu __imu)
     (progn
       (if __imu (setq _imu (car __imu)))
       _imu)))
  (:forwardSpeed
   (&optional __forwardSpeed)
   (if __forwardSpeed (setq _forwardSpeed __forwardSpeed)) _forwardSpeed)
  (:sideSpeed
   (&optional __sideSpeed)
   (if __sideSpeed (setq _sideSpeed __sideSpeed)) _sideSpeed)
  (:rotateSpeed
   (&optional __rotateSpeed)
   (if __rotateSpeed (setq _rotateSpeed __rotateSpeed)) _rotateSpeed)
  (:bodyHeight
   (&optional __bodyHeight)
   (if __bodyHeight (setq _bodyHeight __bodyHeight)) _bodyHeight)
  (:updownSpeed
   (&optional __updownSpeed)
   (if __updownSpeed (setq _updownSpeed __updownSpeed)) _updownSpeed)
  (:forwardPosition
   (&optional __forwardPosition)
   (if __forwardPosition (setq _forwardPosition __forwardPosition)) _forwardPosition)
  (:sidePosition
   (&optional __sidePosition)
   (if __sidePosition (setq _sidePosition __sidePosition)) _sidePosition)
  (:footPosition2Body
   (&rest __footPosition2Body)
   (if (keywordp (car __footPosition2Body))
       (send* _footPosition2Body __footPosition2Body)
     (progn
       (if __footPosition2Body (setq _footPosition2Body (car __footPosition2Body)))
       _footPosition2Body)))
  (:footSpeed2Body
   (&rest __footSpeed2Body)
   (if (keywordp (car __footSpeed2Body))
       (send* _footSpeed2Body __footSpeed2Body)
     (progn
       (if __footSpeed2Body (setq _footSpeed2Body (car __footSpeed2Body)))
       _footSpeed2Body)))
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
  (:eeForce
   (&rest __eeForce)
   (if (keywordp (car __eeForce))
       (send* _eeForce __eeForce)
     (progn
       (if __eeForce (setq _eeForce (car __eeForce)))
       _eeForce)))
  (:jointP
   (&optional __jointP)
   (if __jointP (setq _jointP __jointP)) _jointP)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
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
    ;; uint8 _mode
    1
    ;; a1_slam/IMU _imu
    (send _imu :serialization-length)
    ;; float32 _forwardSpeed
    4
    ;; float32 _sideSpeed
    4
    ;; float32 _rotateSpeed
    4
    ;; float32 _bodyHeight
    4
    ;; float32 _updownSpeed
    4
    ;; float32 _forwardPosition
    4
    ;; float32 _sidePosition
    4
    ;; a1_slam/Cartesian[4] _footPosition2Body
    (apply #'+ (send-all _footPosition2Body :serialization-length))
    ;; a1_slam/Cartesian[4] _footSpeed2Body
    (apply #'+ (send-all _footSpeed2Body :serialization-length))
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
    ;; a1_slam/Cartesian[4] _eeForce
    (apply #'+ (send-all _eeForce :serialization-length))
    ;; float32[12] _jointP
    (* 4    12)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
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
     ;; uint8 _mode
       (write-byte _mode s)
     ;; a1_slam/IMU _imu
       (send _imu :serialize s)
     ;; float32 _forwardSpeed
       (sys::poke _forwardSpeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _sideSpeed
       (sys::poke _sideSpeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rotateSpeed
       (sys::poke _rotateSpeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bodyHeight
       (sys::poke _bodyHeight (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _updownSpeed
       (sys::poke _updownSpeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _forwardPosition
       (sys::poke _forwardPosition (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _sidePosition
       (sys::poke _sidePosition (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; a1_slam/Cartesian[4] _footPosition2Body
     (dolist (elem _footPosition2Body)
       (send elem :serialize s)
       )
     ;; a1_slam/Cartesian[4] _footSpeed2Body
     (dolist (elem _footSpeed2Body)
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
     ;; a1_slam/Cartesian[4] _eeForce
     (dolist (elem _eeForce)
       (send elem :serialize s)
       )
     ;; float32[12] _jointP
     (dotimes (i 12)
       (sys::poke (elt _jointP i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
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
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; a1_slam/IMU _imu
     (send _imu :deserialize buf ptr-) (incf ptr- (send _imu :serialization-length))
   ;; float32 _forwardSpeed
     (setq _forwardSpeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _sideSpeed
     (setq _sideSpeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rotateSpeed
     (setq _rotateSpeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bodyHeight
     (setq _bodyHeight (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _updownSpeed
     (setq _updownSpeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _forwardPosition
     (setq _forwardPosition (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _sidePosition
     (setq _sidePosition (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; a1_slam/Cartesian[4] _footPosition2Body
   (dotimes (i 4)
     (send (elt _footPosition2Body i) :deserialize buf ptr-) (incf ptr- (send (elt _footPosition2Body i) :serialization-length))
     )
   ;; a1_slam/Cartesian[4] _footSpeed2Body
   (dotimes (i 4)
     (send (elt _footSpeed2Body i) :deserialize buf ptr-) (incf ptr- (send (elt _footSpeed2Body i) :serialization-length))
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
   ;; a1_slam/Cartesian[4] _eeForce
   (dotimes (i 4)
     (send (elt _eeForce i) :deserialize buf ptr-) (incf ptr- (send (elt _eeForce i) :serialization-length))
     )
   ;; float32[12] _jointP
   (dotimes (i (length _jointP))
     (setf (elt _jointP i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get a1_slam::HighState :md5sum-) "e509eef5597723d817fac8bddca34ac0")
(setf (get a1_slam::HighState :datatype-) "a1_slam/HighState")
(setf (get a1_slam::HighState :definition-)
      "Header header
uint8 levelFlag
uint16 commVersion                  # Old version Aliengo does not have
uint16 robotID                      # Old version Aliengo does not have
uint32 SN                           # Old version Aliengo does not have
uint8 bandWidth                     # Old version Aliengo does not have
uint8 mode
IMU imu
float32 forwardSpeed
float32 sideSpeed
float32 rotateSpeed
float32 bodyHeight
float32 updownSpeed
float32 forwardPosition       # (will be float type next version)   # Old version Aliengo is different
float32 sidePosition          # (will be float type next version)   # Old version Aliengo is different
Cartesian[4] footPosition2Body
Cartesian[4] footSpeed2Body
int16[4] footForce                  # Old version Aliengo is different
int16[4] footForceEst               # Old version Aliengo does not have
uint32 tick               
uint8[40] wirelessRemote
uint32 reserve                      # Old version Aliengo does not have
uint32 crc

# Under are not defined in SDK yet. # Old version Aliengo does not have
Cartesian[4] eeForce            # It's a 1-DOF force in real robot, but 3-DOF is better for visualization.
float32[12] jointP              # for visualization
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
MSG: a1_slam/IMU
float32[4] quaternion
float32[3] gyroscope
float32[3] accelerometer
int8 temperature
================================================================================
MSG: a1_slam/Cartesian
float32 x
float32 y
float32 z
")



(provide :a1_slam/HighState "e509eef5597723d817fac8bddca34ac0")


