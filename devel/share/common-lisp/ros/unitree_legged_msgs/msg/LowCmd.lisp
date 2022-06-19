; Auto-generated. Do not edit!


(cl:in-package unitree_legged_msgs-msg)


;//! \htmlinclude LowCmd.msg.html

(cl:defclass <LowCmd> (roslisp-msg-protocol:ros-message)
  ((levelFlag
    :reader levelFlag
    :initarg :levelFlag
    :type cl:fixnum
    :initform 0)
   (commVersion
    :reader commVersion
    :initarg :commVersion
    :type cl:fixnum
    :initform 0)
   (robotID
    :reader robotID
    :initarg :robotID
    :type cl:fixnum
    :initform 0)
   (SN
    :reader SN
    :initarg :SN
    :type cl:integer
    :initform 0)
   (bandWidth
    :reader bandWidth
    :initarg :bandWidth
    :type cl:fixnum
    :initform 0)
   (motorCmd
    :reader motorCmd
    :initarg :motorCmd
    :type (cl:vector unitree_legged_msgs-msg:MotorCmd)
   :initform (cl:make-array 20 :element-type 'unitree_legged_msgs-msg:MotorCmd :initial-element (cl:make-instance 'unitree_legged_msgs-msg:MotorCmd)))
   (led
    :reader led
    :initarg :led
    :type (cl:vector unitree_legged_msgs-msg:LED)
   :initform (cl:make-array 4 :element-type 'unitree_legged_msgs-msg:LED :initial-element (cl:make-instance 'unitree_legged_msgs-msg:LED)))
   (wirelessRemote
    :reader wirelessRemote
    :initarg :wirelessRemote
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 40 :element-type 'cl:fixnum :initial-element 0))
   (reserve
    :reader reserve
    :initarg :reserve
    :type cl:integer
    :initform 0)
   (crc
    :reader crc
    :initarg :crc
    :type cl:integer
    :initform 0)
   (ff
    :reader ff
    :initarg :ff
    :type (cl:vector unitree_legged_msgs-msg:Cartesian)
   :initform (cl:make-array 4 :element-type 'unitree_legged_msgs-msg:Cartesian :initial-element (cl:make-instance 'unitree_legged_msgs-msg:Cartesian))))
)

(cl:defclass LowCmd (<LowCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LowCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LowCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name unitree_legged_msgs-msg:<LowCmd> is deprecated: use unitree_legged_msgs-msg:LowCmd instead.")))

(cl:ensure-generic-function 'levelFlag-val :lambda-list '(m))
(cl:defmethod levelFlag-val ((m <LowCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:levelFlag-val is deprecated.  Use unitree_legged_msgs-msg:levelFlag instead.")
  (levelFlag m))

(cl:ensure-generic-function 'commVersion-val :lambda-list '(m))
(cl:defmethod commVersion-val ((m <LowCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:commVersion-val is deprecated.  Use unitree_legged_msgs-msg:commVersion instead.")
  (commVersion m))

(cl:ensure-generic-function 'robotID-val :lambda-list '(m))
(cl:defmethod robotID-val ((m <LowCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:robotID-val is deprecated.  Use unitree_legged_msgs-msg:robotID instead.")
  (robotID m))

(cl:ensure-generic-function 'SN-val :lambda-list '(m))
(cl:defmethod SN-val ((m <LowCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:SN-val is deprecated.  Use unitree_legged_msgs-msg:SN instead.")
  (SN m))

(cl:ensure-generic-function 'bandWidth-val :lambda-list '(m))
(cl:defmethod bandWidth-val ((m <LowCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:bandWidth-val is deprecated.  Use unitree_legged_msgs-msg:bandWidth instead.")
  (bandWidth m))

(cl:ensure-generic-function 'motorCmd-val :lambda-list '(m))
(cl:defmethod motorCmd-val ((m <LowCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:motorCmd-val is deprecated.  Use unitree_legged_msgs-msg:motorCmd instead.")
  (motorCmd m))

(cl:ensure-generic-function 'led-val :lambda-list '(m))
(cl:defmethod led-val ((m <LowCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:led-val is deprecated.  Use unitree_legged_msgs-msg:led instead.")
  (led m))

(cl:ensure-generic-function 'wirelessRemote-val :lambda-list '(m))
(cl:defmethod wirelessRemote-val ((m <LowCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:wirelessRemote-val is deprecated.  Use unitree_legged_msgs-msg:wirelessRemote instead.")
  (wirelessRemote m))

(cl:ensure-generic-function 'reserve-val :lambda-list '(m))
(cl:defmethod reserve-val ((m <LowCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:reserve-val is deprecated.  Use unitree_legged_msgs-msg:reserve instead.")
  (reserve m))

(cl:ensure-generic-function 'crc-val :lambda-list '(m))
(cl:defmethod crc-val ((m <LowCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:crc-val is deprecated.  Use unitree_legged_msgs-msg:crc instead.")
  (crc m))

(cl:ensure-generic-function 'ff-val :lambda-list '(m))
(cl:defmethod ff-val ((m <LowCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:ff-val is deprecated.  Use unitree_legged_msgs-msg:ff instead.")
  (ff m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LowCmd>) ostream)
  "Serializes a message object of type '<LowCmd>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'levelFlag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'commVersion)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'commVersion)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robotID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'robotID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'SN)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'SN)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'SN)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'SN)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bandWidth)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'motorCmd))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'led))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'wirelessRemote))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reserve)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reserve)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'reserve)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'reserve)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'crc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'crc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'crc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'crc)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'ff))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LowCmd>) istream)
  "Deserializes a message object of type '<LowCmd>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'levelFlag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'commVersion)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'commVersion)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robotID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'robotID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'SN)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'SN)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'SN)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'SN)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bandWidth)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'motorCmd) (cl:make-array 20))
  (cl:let ((vals (cl:slot-value msg 'motorCmd)))
    (cl:dotimes (i 20)
    (cl:setf (cl:aref vals i) (cl:make-instance 'unitree_legged_msgs-msg:MotorCmd))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'led) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'led)))
    (cl:dotimes (i 4)
    (cl:setf (cl:aref vals i) (cl:make-instance 'unitree_legged_msgs-msg:LED))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'wirelessRemote) (cl:make-array 40))
  (cl:let ((vals (cl:slot-value msg 'wirelessRemote)))
    (cl:dotimes (i 40)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reserve)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reserve)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'reserve)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'reserve)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'crc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'crc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'crc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'crc)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ff) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'ff)))
    (cl:dotimes (i 4)
    (cl:setf (cl:aref vals i) (cl:make-instance 'unitree_legged_msgs-msg:Cartesian))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LowCmd>)))
  "Returns string type for a message object of type '<LowCmd>"
  "unitree_legged_msgs/LowCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LowCmd)))
  "Returns string type for a message object of type 'LowCmd"
  "unitree_legged_msgs/LowCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LowCmd>)))
  "Returns md5sum for a message object of type '<LowCmd>"
  "357432b2562edd0a8e89b9c9f5fc4821")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LowCmd)))
  "Returns md5sum for a message object of type 'LowCmd"
  "357432b2562edd0a8e89b9c9f5fc4821")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LowCmd>)))
  "Returns full string definition for message of type '<LowCmd>"
  (cl:format cl:nil "uint8 levelFlag                 ~%uint16 commVersion              # Old version Aliengo does not have~%uint16 robotID                  # Old version Aliengo does not have~%uint32 SN                       # Old version Aliengo does not have~%uint8 bandWidth                 # Old version Aliengo does not have~%MotorCmd[20] motorCmd~%LED[4] led~%uint8[40] wirelessRemote~%uint32 reserve                  # Old version Aliengo does not have~%uint32 crc~%~%Cartesian[4] ff               # will delete # Old version Aliengo does not have~%================================================================================~%MSG: unitree_legged_msgs/MotorCmd~%uint8 mode           # motor target mode~%float32 q            # motor target position~%float32 dq           # motor target velocity~%float32 tau          # motor target torque~%float32 Kp           # motor spring stiffness coefficient~%float32 Kd           # motor damper coefficient~%uint32[3] reserve    # motor target torque~%================================================================================~%MSG: unitree_legged_msgs/LED~%uint8 r~%uint8 g~%uint8 b~%================================================================================~%MSG: unitree_legged_msgs/Cartesian~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LowCmd)))
  "Returns full string definition for message of type 'LowCmd"
  (cl:format cl:nil "uint8 levelFlag                 ~%uint16 commVersion              # Old version Aliengo does not have~%uint16 robotID                  # Old version Aliengo does not have~%uint32 SN                       # Old version Aliengo does not have~%uint8 bandWidth                 # Old version Aliengo does not have~%MotorCmd[20] motorCmd~%LED[4] led~%uint8[40] wirelessRemote~%uint32 reserve                  # Old version Aliengo does not have~%uint32 crc~%~%Cartesian[4] ff               # will delete # Old version Aliengo does not have~%================================================================================~%MSG: unitree_legged_msgs/MotorCmd~%uint8 mode           # motor target mode~%float32 q            # motor target position~%float32 dq           # motor target velocity~%float32 tau          # motor target torque~%float32 Kp           # motor spring stiffness coefficient~%float32 Kd           # motor damper coefficient~%uint32[3] reserve    # motor target torque~%================================================================================~%MSG: unitree_legged_msgs/LED~%uint8 r~%uint8 g~%uint8 b~%================================================================================~%MSG: unitree_legged_msgs/Cartesian~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LowCmd>))
  (cl:+ 0
     1
     2
     2
     4
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'motorCmd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'led) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'wirelessRemote) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'ff) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LowCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'LowCmd
    (cl:cons ':levelFlag (levelFlag msg))
    (cl:cons ':commVersion (commVersion msg))
    (cl:cons ':robotID (robotID msg))
    (cl:cons ':SN (SN msg))
    (cl:cons ':bandWidth (bandWidth msg))
    (cl:cons ':motorCmd (motorCmd msg))
    (cl:cons ':led (led msg))
    (cl:cons ':wirelessRemote (wirelessRemote msg))
    (cl:cons ':reserve (reserve msg))
    (cl:cons ':crc (crc msg))
    (cl:cons ':ff (ff msg))
))
