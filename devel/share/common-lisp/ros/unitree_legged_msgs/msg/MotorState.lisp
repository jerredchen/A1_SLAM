; Auto-generated. Do not edit!


(cl:in-package unitree_legged_msgs-msg)


;//! \htmlinclude MotorState.msg.html

(cl:defclass <MotorState> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (q
    :reader q
    :initarg :q
    :type cl:float
    :initform 0.0)
   (dq
    :reader dq
    :initarg :dq
    :type cl:float
    :initform 0.0)
   (ddq
    :reader ddq
    :initarg :ddq
    :type cl:float
    :initform 0.0)
   (tauEst
    :reader tauEst
    :initarg :tauEst
    :type cl:float
    :initform 0.0)
   (q_raw
    :reader q_raw
    :initarg :q_raw
    :type cl:float
    :initform 0.0)
   (dq_raw
    :reader dq_raw
    :initarg :dq_raw
    :type cl:float
    :initform 0.0)
   (ddq_raw
    :reader ddq_raw
    :initarg :ddq_raw
    :type cl:float
    :initform 0.0)
   (temperature
    :reader temperature
    :initarg :temperature
    :type cl:fixnum
    :initform 0)
   (reserve
    :reader reserve
    :initarg :reserve
    :type (cl:vector cl:integer)
   :initform (cl:make-array 2 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass MotorState (<MotorState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name unitree_legged_msgs-msg:<MotorState> is deprecated: use unitree_legged_msgs-msg:MotorState instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <MotorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:mode-val is deprecated.  Use unitree_legged_msgs-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'q-val :lambda-list '(m))
(cl:defmethod q-val ((m <MotorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:q-val is deprecated.  Use unitree_legged_msgs-msg:q instead.")
  (q m))

(cl:ensure-generic-function 'dq-val :lambda-list '(m))
(cl:defmethod dq-val ((m <MotorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:dq-val is deprecated.  Use unitree_legged_msgs-msg:dq instead.")
  (dq m))

(cl:ensure-generic-function 'ddq-val :lambda-list '(m))
(cl:defmethod ddq-val ((m <MotorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:ddq-val is deprecated.  Use unitree_legged_msgs-msg:ddq instead.")
  (ddq m))

(cl:ensure-generic-function 'tauEst-val :lambda-list '(m))
(cl:defmethod tauEst-val ((m <MotorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:tauEst-val is deprecated.  Use unitree_legged_msgs-msg:tauEst instead.")
  (tauEst m))

(cl:ensure-generic-function 'q_raw-val :lambda-list '(m))
(cl:defmethod q_raw-val ((m <MotorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:q_raw-val is deprecated.  Use unitree_legged_msgs-msg:q_raw instead.")
  (q_raw m))

(cl:ensure-generic-function 'dq_raw-val :lambda-list '(m))
(cl:defmethod dq_raw-val ((m <MotorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:dq_raw-val is deprecated.  Use unitree_legged_msgs-msg:dq_raw instead.")
  (dq_raw m))

(cl:ensure-generic-function 'ddq_raw-val :lambda-list '(m))
(cl:defmethod ddq_raw-val ((m <MotorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:ddq_raw-val is deprecated.  Use unitree_legged_msgs-msg:ddq_raw instead.")
  (ddq_raw m))

(cl:ensure-generic-function 'temperature-val :lambda-list '(m))
(cl:defmethod temperature-val ((m <MotorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:temperature-val is deprecated.  Use unitree_legged_msgs-msg:temperature instead.")
  (temperature m))

(cl:ensure-generic-function 'reserve-val :lambda-list '(m))
(cl:defmethod reserve-val ((m <MotorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:reserve-val is deprecated.  Use unitree_legged_msgs-msg:reserve instead.")
  (reserve m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorState>) ostream)
  "Serializes a message object of type '<MotorState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'q))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dq))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ddq))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tauEst))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'q_raw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dq_raw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ddq_raw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'temperature)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'reserve))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorState>) istream)
  "Deserializes a message object of type '<MotorState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'q) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dq) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ddq) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tauEst) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'q_raw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dq_raw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ddq_raw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'temperature) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:setf (cl:slot-value msg 'reserve) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'reserve)))
    (cl:dotimes (i 2)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorState>)))
  "Returns string type for a message object of type '<MotorState>"
  "unitree_legged_msgs/MotorState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorState)))
  "Returns string type for a message object of type 'MotorState"
  "unitree_legged_msgs/MotorState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorState>)))
  "Returns md5sum for a message object of type '<MotorState>"
  "94c55ee3b7852be2bd437b20ce12a254")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorState)))
  "Returns md5sum for a message object of type 'MotorState"
  "94c55ee3b7852be2bd437b20ce12a254")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorState>)))
  "Returns full string definition for message of type '<MotorState>"
  (cl:format cl:nil "uint8 mode           # motor current mode ~%float32 q            # motor current position（rad）~%float32 dq           # motor current speed（rad/s）~%float32 ddq          # motor current speed（rad/s）~%float32 tauEst       # current estimated output torque（N*m）~%float32 q_raw        # motor current position（rad）~%float32 dq_raw       # motor current speed（rad/s）~%float32 ddq_raw      # motor current speed（rad/s）~%int8 temperature     # motor temperature（slow conduction of temperature leads to lag）~%uint32[2] reserve~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorState)))
  "Returns full string definition for message of type 'MotorState"
  (cl:format cl:nil "uint8 mode           # motor current mode ~%float32 q            # motor current position（rad）~%float32 dq           # motor current speed（rad/s）~%float32 ddq          # motor current speed（rad/s）~%float32 tauEst       # current estimated output torque（N*m）~%float32 q_raw        # motor current position（rad）~%float32 dq_raw       # motor current speed（rad/s）~%float32 ddq_raw      # motor current speed（rad/s）~%int8 temperature     # motor temperature（slow conduction of temperature leads to lag）~%uint32[2] reserve~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorState>))
  (cl:+ 0
     1
     4
     4
     4
     4
     4
     4
     4
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'reserve) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorState>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorState
    (cl:cons ':mode (mode msg))
    (cl:cons ':q (q msg))
    (cl:cons ':dq (dq msg))
    (cl:cons ':ddq (ddq msg))
    (cl:cons ':tauEst (tauEst msg))
    (cl:cons ':q_raw (q_raw msg))
    (cl:cons ':dq_raw (dq_raw msg))
    (cl:cons ':ddq_raw (ddq_raw msg))
    (cl:cons ':temperature (temperature msg))
    (cl:cons ':reserve (reserve msg))
))
