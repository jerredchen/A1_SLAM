;; Auto-generated. Do not edit!


(when (boundp 'unitree_legged_msgs::HighCmd)
  (if (not (find-package "UNITREE_LEGGED_MSGS"))
    (make-package "UNITREE_LEGGED_MSGS"))
  (shadow 'HighCmd (find-package "UNITREE_LEGGED_MSGS")))
(unless (find-package "UNITREE_LEGGED_MSGS::HIGHCMD")
  (make-package "UNITREE_LEGGED_MSGS::HIGHCMD"))

(in-package "ROS")
;;//! \htmlinclude HighCmd.msg.html


(defclass unitree_legged_msgs::HighCmd
  :super ros::object
  :slots (_levelFlag _commVersion _robotID _SN _bandWidth _mode _forwardSpeed _sideSpeed _rotateSpeed _bodyHeight _footRaiseHeight _yaw _pitch _roll _led _wirelessRemote _AppRemote _reserve _crc ))

(defmethod unitree_legged_msgs::HighCmd
  (:init
   (&key
    ((:levelFlag __levelFlag) 0)
    ((:commVersion __commVersion) 0)
    ((:robotID __robotID) 0)
    ((:SN __SN) 0)
    ((:bandWidth __bandWidth) 0)
    ((:mode __mode) 0)
    ((:forwardSpeed __forwardSpeed) 0.0)
    ((:sideSpeed __sideSpeed) 0.0)
    ((:rotateSpeed __rotateSpeed) 0.0)
    ((:bodyHeight __bodyHeight) 0.0)
    ((:footRaiseHeight __footRaiseHeight) 0.0)
    ((:yaw __yaw) 0.0)
    ((:pitch __pitch) 0.0)
    ((:roll __roll) 0.0)
    ((:led __led) (let (r) (dotimes (i 4) (push (instance unitree_legged_msgs::LED :init) r)) r))
    ((:wirelessRemote __wirelessRemote) (make-array 40 :initial-element 0 :element-type :char))
    ((:AppRemote __AppRemote) (make-array 40 :initial-element 0 :element-type :char))
    ((:reserve __reserve) 0)
    ((:crc __crc) 0)
    )
   (send-super :init)
   (setq _levelFlag (round __levelFlag))
   (setq _commVersion (round __commVersion))
   (setq _robotID (round __robotID))
   (setq _SN (round __SN))
   (setq _bandWidth (round __bandWidth))
   (setq _mode (round __mode))
   (setq _forwardSpeed (float __forwardSpeed))
   (setq _sideSpeed (float __sideSpeed))
   (setq _rotateSpeed (float __rotateSpeed))
   (setq _bodyHeight (float __bodyHeight))
   (setq _footRaiseHeight (float __footRaiseHeight))
   (setq _yaw (float __yaw))
   (setq _pitch (float __pitch))
   (setq _roll (float __roll))
   (setq _led __led)
   (setq _wirelessRemote __wirelessRemote)
   (setq _AppRemote __AppRemote)
   (setq _reserve (round __reserve))
   (setq _crc (round __crc))
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
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
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
  (:footRaiseHeight
   (&optional __footRaiseHeight)
   (if __footRaiseHeight (setq _footRaiseHeight __footRaiseHeight)) _footRaiseHeight)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:led
   (&rest __led)
   (if (keywordp (car __led))
       (send* _led __led)
     (progn
       (if __led (setq _led (car __led)))
       _led)))
  (:wirelessRemote
   (&optional __wirelessRemote)
   (if __wirelessRemote (setq _wirelessRemote __wirelessRemote)) _wirelessRemote)
  (:AppRemote
   (&optional __AppRemote)
   (if __AppRemote (setq _AppRemote __AppRemote)) _AppRemote)
  (:reserve
   (&optional __reserve)
   (if __reserve (setq _reserve __reserve)) _reserve)
  (:crc
   (&optional __crc)
   (if __crc (setq _crc __crc)) _crc)
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
    ;; uint8 _mode
    1
    ;; float32 _forwardSpeed
    4
    ;; float32 _sideSpeed
    4
    ;; float32 _rotateSpeed
    4
    ;; float32 _bodyHeight
    4
    ;; float32 _footRaiseHeight
    4
    ;; float32 _yaw
    4
    ;; float32 _pitch
    4
    ;; float32 _roll
    4
    ;; unitree_legged_msgs/LED[4] _led
    (apply #'+ (send-all _led :serialization-length))
    ;; uint8[40] _wirelessRemote
    (* 1    40)
    ;; uint8[40] _AppRemote
    (* 1    40)
    ;; uint32 _reserve
    4
    ;; int32 _crc
    4
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
     ;; uint8 _mode
       (write-byte _mode s)
     ;; float32 _forwardSpeed
       (sys::poke _forwardSpeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _sideSpeed
       (sys::poke _sideSpeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rotateSpeed
       (sys::poke _rotateSpeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bodyHeight
       (sys::poke _bodyHeight (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _footRaiseHeight
       (sys::poke _footRaiseHeight (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; unitree_legged_msgs/LED[4] _led
     (dolist (elem _led)
       (send elem :serialize s)
       )
     ;; uint8[40] _wirelessRemote
     (princ _wirelessRemote s)
     ;; uint8[40] _AppRemote
     (princ _AppRemote s)
     ;; uint32 _reserve
       (write-long _reserve s)
     ;; int32 _crc
       (write-long _crc s)
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
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _forwardSpeed
     (setq _forwardSpeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _sideSpeed
     (setq _sideSpeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rotateSpeed
     (setq _rotateSpeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bodyHeight
     (setq _bodyHeight (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _footRaiseHeight
     (setq _footRaiseHeight (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll
     (setq _roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; unitree_legged_msgs/LED[4] _led
   (dotimes (i 4)
     (send (elt _led i) :deserialize buf ptr-) (incf ptr- (send (elt _led i) :serialization-length))
     )
   ;; uint8[40] _wirelessRemote
   (setq _wirelessRemote (make-array 40 :element-type :char))
   (replace _wirelessRemote buf :start2 ptr-) (incf ptr- 40)
   ;; uint8[40] _AppRemote
   (setq _AppRemote (make-array 40 :element-type :char))
   (replace _AppRemote buf :start2 ptr-) (incf ptr- 40)
   ;; uint32 _reserve
     (setq _reserve (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _crc
     (setq _crc (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get unitree_legged_msgs::HighCmd :md5sum-) "1a655499a3f64905db59ceed65ca774a")
(setf (get unitree_legged_msgs::HighCmd :datatype-) "unitree_legged_msgs/HighCmd")
(setf (get unitree_legged_msgs::HighCmd :definition-)
      "uint8 levelFlag
uint16 commVersion              # Old version Aliengo does not have
uint16 robotID                  # Old version Aliengo does not have
uint32 SN                       # Old version Aliengo does not have
uint8 bandWidth                 # Old version Aliengo does not have
uint8 mode
float32 forwardSpeed
float32 sideSpeed
float32 rotateSpeed	
float32 bodyHeight
float32 footRaiseHeight
float32 yaw
float32 pitch
float32 roll
LED[4] led
uint8[40] wirelessRemote
uint8[40] AppRemote             # Old version Aliengo does not have
uint32 reserve                  # Old version Aliengo does not have
int32 crc
================================================================================
MSG: unitree_legged_msgs/LED
uint8 r
uint8 g
uint8 b
")



(provide :unitree_legged_msgs/HighCmd "1a655499a3f64905db59ceed65ca774a")


