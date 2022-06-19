; Auto-generated. Do not edit!


(cl:in-package unitree_legged_msgs-msg)


;//! \htmlinclude MotorCmd.msg.html

(cl:defclass <MotorCmd> (roslisp-msg-protocol:ros-message)
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
   (tau
    :reader tau
    :initarg :tau
    :type cl:float
    :initform 0.0)
   (Kp
    :reader Kp
    :initarg :Kp
    :type cl:float
    :initform 0.0)
   (Kd
    :reader Kd
    :initarg :Kd
    :type cl:float
    :initform 0.0)
   (reserve
    :reader reserve
    :initarg :reserve
    :type (cl:vector cl:integer)
   :initform (cl:make-array 3 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass MotorCmd (<MotorCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name unitree_legged_msgs-msg:<MotorCmd> is deprecated: use unitree_legged_msgs-msg:MotorCmd instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <MotorCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:mode-val is deprecated.  Use unitree_legged_msgs-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'q-val :lambda-list '(m))
(cl:defmethod q-val ((m <MotorCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:q-val is deprecated.  Use unitree_legged_msgs-msg:q instead.")
  (q m))

(cl:ensure-generic-function 'dq-val :lambda-list '(m))
(cl:defmethod dq-val ((m <MotorCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:dq-val is deprecated.  Use unitree_legged_msgs-msg:dq instead.")
  (dq m))

(cl:ensure-generic-function 'tau-val :lambda-list '(m))
(cl:defmethod tau-val ((m <MotorCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:tau-val is deprecated.  Use unitree_legged_msgs-msg:tau instead.")
  (tau m))

(cl:ensure-generic-function 'Kp-val :lambda-list '(m))
(cl:defmethod Kp-val ((m <MotorCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:Kp-val is deprecated.  Use unitree_legged_msgs-msg:Kp instead.")
  (Kp m))

(cl:ensure-generic-function 'Kd-val :lambda-list '(m))
(cl:defmethod Kd-val ((m <MotorCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:Kd-val is deprecated.  Use unitree_legged_msgs-msg:Kd instead.")
  (Kd m))

(cl:ensure-generic-function 'reserve-val :lambda-list '(m))
(cl:defmethod reserve-val ((m <MotorCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:reserve-val is deprecated.  Use unitree_legged_msgs-msg:reserve instead.")
  (reserve m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorCmd>) ostream)
  "Serializes a message object of type '<MotorCmd>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tau))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Kp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Kd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'reserve))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorCmd>) istream)
  "Deserializes a message object of type '<MotorCmd>"
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
    (cl:setf (cl:slot-value msg 'tau) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Kp) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Kd) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'reserve) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'reserve)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorCmd>)))
  "Returns string type for a message object of type '<MotorCmd>"
  "unitree_legged_msgs/MotorCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorCmd)))
  "Returns string type for a message object of type 'MotorCmd"
  "unitree_legged_msgs/MotorCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorCmd>)))
  "Returns md5sum for a message object of type '<MotorCmd>"
  "bbb3b7d91319c3a1b99055f0149ba221")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorCmd)))
  "Returns md5sum for a message object of type 'MotorCmd"
  "bbb3b7d91319c3a1b99055f0149ba221")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorCmd>)))
  "Returns full string definition for message of type '<MotorCmd>"
  (cl:format cl:nil "uint8 mode           # motor target mode~%float32 q            # motor target position~%float32 dq           # motor target velocity~%float32 tau          # motor target torque~%float32 Kp           # motor spring stiffness coefficient~%float32 Kd           # motor damper coefficient~%uint32[3] reserve    # motor target torque~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorCmd)))
  "Returns full string definition for message of type 'MotorCmd"
  (cl:format cl:nil "uint8 mode           # motor target mode~%float32 q            # motor target position~%float32 dq           # motor target velocity~%float32 tau          # motor target torque~%float32 Kp           # motor spring stiffness coefficient~%float32 Kd           # motor damper coefficient~%uint32[3] reserve    # motor target torque~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorCmd>))
  (cl:+ 0
     1
     4
     4
     4
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'reserve) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorCmd
    (cl:cons ':mode (mode msg))
    (cl:cons ':q (q msg))
    (cl:cons ':dq (dq msg))
    (cl:cons ':tau (tau msg))
    (cl:cons ':Kp (Kp msg))
    (cl:cons ':Kd (Kd msg))
    (cl:cons ':reserve (reserve msg))
))
