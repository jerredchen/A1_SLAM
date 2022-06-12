; Auto-generated. Do not edit!


(cl:in-package a1_pose_slam-msg)


;//! \htmlinclude HighState.msg.html

(cl:defclass <HighState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (levelFlag
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
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (imu
    :reader imu
    :initarg :imu
    :type a1_pose_slam-msg:IMU
    :initform (cl:make-instance 'a1_pose_slam-msg:IMU))
   (forwardSpeed
    :reader forwardSpeed
    :initarg :forwardSpeed
    :type cl:float
    :initform 0.0)
   (sideSpeed
    :reader sideSpeed
    :initarg :sideSpeed
    :type cl:float
    :initform 0.0)
   (rotateSpeed
    :reader rotateSpeed
    :initarg :rotateSpeed
    :type cl:float
    :initform 0.0)
   (bodyHeight
    :reader bodyHeight
    :initarg :bodyHeight
    :type cl:float
    :initform 0.0)
   (updownSpeed
    :reader updownSpeed
    :initarg :updownSpeed
    :type cl:float
    :initform 0.0)
   (forwardPosition
    :reader forwardPosition
    :initarg :forwardPosition
    :type cl:float
    :initform 0.0)
   (sidePosition
    :reader sidePosition
    :initarg :sidePosition
    :type cl:float
    :initform 0.0)
   (footPosition2Body
    :reader footPosition2Body
    :initarg :footPosition2Body
    :type (cl:vector a1_pose_slam-msg:Cartesian)
   :initform (cl:make-array 4 :element-type 'a1_pose_slam-msg:Cartesian :initial-element (cl:make-instance 'a1_pose_slam-msg:Cartesian)))
   (footSpeed2Body
    :reader footSpeed2Body
    :initarg :footSpeed2Body
    :type (cl:vector a1_pose_slam-msg:Cartesian)
   :initform (cl:make-array 4 :element-type 'a1_pose_slam-msg:Cartesian :initial-element (cl:make-instance 'a1_pose_slam-msg:Cartesian)))
   (footForce
    :reader footForce
    :initarg :footForce
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (footForceEst
    :reader footForceEst
    :initarg :footForceEst
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (tick
    :reader tick
    :initarg :tick
    :type cl:integer
    :initform 0)
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
   (eeForce
    :reader eeForce
    :initarg :eeForce
    :type (cl:vector a1_pose_slam-msg:Cartesian)
   :initform (cl:make-array 4 :element-type 'a1_pose_slam-msg:Cartesian :initial-element (cl:make-instance 'a1_pose_slam-msg:Cartesian)))
   (jointP
    :reader jointP
    :initarg :jointP
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass HighState (<HighState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HighState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HighState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name a1_pose_slam-msg:<HighState> is deprecated: use a1_pose_slam-msg:HighState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:header-val is deprecated.  Use a1_pose_slam-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'levelFlag-val :lambda-list '(m))
(cl:defmethod levelFlag-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:levelFlag-val is deprecated.  Use a1_pose_slam-msg:levelFlag instead.")
  (levelFlag m))

(cl:ensure-generic-function 'commVersion-val :lambda-list '(m))
(cl:defmethod commVersion-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:commVersion-val is deprecated.  Use a1_pose_slam-msg:commVersion instead.")
  (commVersion m))

(cl:ensure-generic-function 'robotID-val :lambda-list '(m))
(cl:defmethod robotID-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:robotID-val is deprecated.  Use a1_pose_slam-msg:robotID instead.")
  (robotID m))

(cl:ensure-generic-function 'SN-val :lambda-list '(m))
(cl:defmethod SN-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:SN-val is deprecated.  Use a1_pose_slam-msg:SN instead.")
  (SN m))

(cl:ensure-generic-function 'bandWidth-val :lambda-list '(m))
(cl:defmethod bandWidth-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:bandWidth-val is deprecated.  Use a1_pose_slam-msg:bandWidth instead.")
  (bandWidth m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:mode-val is deprecated.  Use a1_pose_slam-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'imu-val :lambda-list '(m))
(cl:defmethod imu-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:imu-val is deprecated.  Use a1_pose_slam-msg:imu instead.")
  (imu m))

(cl:ensure-generic-function 'forwardSpeed-val :lambda-list '(m))
(cl:defmethod forwardSpeed-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:forwardSpeed-val is deprecated.  Use a1_pose_slam-msg:forwardSpeed instead.")
  (forwardSpeed m))

(cl:ensure-generic-function 'sideSpeed-val :lambda-list '(m))
(cl:defmethod sideSpeed-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:sideSpeed-val is deprecated.  Use a1_pose_slam-msg:sideSpeed instead.")
  (sideSpeed m))

(cl:ensure-generic-function 'rotateSpeed-val :lambda-list '(m))
(cl:defmethod rotateSpeed-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:rotateSpeed-val is deprecated.  Use a1_pose_slam-msg:rotateSpeed instead.")
  (rotateSpeed m))

(cl:ensure-generic-function 'bodyHeight-val :lambda-list '(m))
(cl:defmethod bodyHeight-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:bodyHeight-val is deprecated.  Use a1_pose_slam-msg:bodyHeight instead.")
  (bodyHeight m))

(cl:ensure-generic-function 'updownSpeed-val :lambda-list '(m))
(cl:defmethod updownSpeed-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:updownSpeed-val is deprecated.  Use a1_pose_slam-msg:updownSpeed instead.")
  (updownSpeed m))

(cl:ensure-generic-function 'forwardPosition-val :lambda-list '(m))
(cl:defmethod forwardPosition-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:forwardPosition-val is deprecated.  Use a1_pose_slam-msg:forwardPosition instead.")
  (forwardPosition m))

(cl:ensure-generic-function 'sidePosition-val :lambda-list '(m))
(cl:defmethod sidePosition-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:sidePosition-val is deprecated.  Use a1_pose_slam-msg:sidePosition instead.")
  (sidePosition m))

(cl:ensure-generic-function 'footPosition2Body-val :lambda-list '(m))
(cl:defmethod footPosition2Body-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:footPosition2Body-val is deprecated.  Use a1_pose_slam-msg:footPosition2Body instead.")
  (footPosition2Body m))

(cl:ensure-generic-function 'footSpeed2Body-val :lambda-list '(m))
(cl:defmethod footSpeed2Body-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:footSpeed2Body-val is deprecated.  Use a1_pose_slam-msg:footSpeed2Body instead.")
  (footSpeed2Body m))

(cl:ensure-generic-function 'footForce-val :lambda-list '(m))
(cl:defmethod footForce-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:footForce-val is deprecated.  Use a1_pose_slam-msg:footForce instead.")
  (footForce m))

(cl:ensure-generic-function 'footForceEst-val :lambda-list '(m))
(cl:defmethod footForceEst-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:footForceEst-val is deprecated.  Use a1_pose_slam-msg:footForceEst instead.")
  (footForceEst m))

(cl:ensure-generic-function 'tick-val :lambda-list '(m))
(cl:defmethod tick-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:tick-val is deprecated.  Use a1_pose_slam-msg:tick instead.")
  (tick m))

(cl:ensure-generic-function 'wirelessRemote-val :lambda-list '(m))
(cl:defmethod wirelessRemote-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:wirelessRemote-val is deprecated.  Use a1_pose_slam-msg:wirelessRemote instead.")
  (wirelessRemote m))

(cl:ensure-generic-function 'reserve-val :lambda-list '(m))
(cl:defmethod reserve-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:reserve-val is deprecated.  Use a1_pose_slam-msg:reserve instead.")
  (reserve m))

(cl:ensure-generic-function 'crc-val :lambda-list '(m))
(cl:defmethod crc-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:crc-val is deprecated.  Use a1_pose_slam-msg:crc instead.")
  (crc m))

(cl:ensure-generic-function 'eeForce-val :lambda-list '(m))
(cl:defmethod eeForce-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:eeForce-val is deprecated.  Use a1_pose_slam-msg:eeForce instead.")
  (eeForce m))

(cl:ensure-generic-function 'jointP-val :lambda-list '(m))
(cl:defmethod jointP-val ((m <HighState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_pose_slam-msg:jointP-val is deprecated.  Use a1_pose_slam-msg:jointP instead.")
  (jointP m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HighState>) ostream)
  "Serializes a message object of type '<HighState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'imu) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'forwardSpeed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sideSpeed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rotateSpeed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bodyHeight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'updownSpeed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'forwardPosition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sidePosition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'footPosition2Body))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'footSpeed2Body))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'footForce))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'footForceEst))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tick)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tick)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tick)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tick)) ostream)
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
   (cl:slot-value msg 'eeForce))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'jointP))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HighState>) istream)
  "Deserializes a message object of type '<HighState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
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
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'imu) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'forwardSpeed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sideSpeed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotateSpeed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bodyHeight) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'updownSpeed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'forwardPosition) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sidePosition) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'footPosition2Body) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'footPosition2Body)))
    (cl:dotimes (i 4)
    (cl:setf (cl:aref vals i) (cl:make-instance 'a1_pose_slam-msg:Cartesian))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'footSpeed2Body) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'footSpeed2Body)))
    (cl:dotimes (i 4)
    (cl:setf (cl:aref vals i) (cl:make-instance 'a1_pose_slam-msg:Cartesian))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'footForce) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'footForce)))
    (cl:dotimes (i 4)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'footForceEst) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'footForceEst)))
    (cl:dotimes (i 4)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tick)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tick)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tick)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tick)) (cl:read-byte istream))
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
  (cl:setf (cl:slot-value msg 'eeForce) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'eeForce)))
    (cl:dotimes (i 4)
    (cl:setf (cl:aref vals i) (cl:make-instance 'a1_pose_slam-msg:Cartesian))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (cl:setf (cl:slot-value msg 'jointP) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'jointP)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HighState>)))
  "Returns string type for a message object of type '<HighState>"
  "a1_pose_slam/HighState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HighState)))
  "Returns string type for a message object of type 'HighState"
  "a1_pose_slam/HighState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HighState>)))
  "Returns md5sum for a message object of type '<HighState>"
  "e509eef5597723d817fac8bddca34ac0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HighState)))
  "Returns md5sum for a message object of type 'HighState"
  "e509eef5597723d817fac8bddca34ac0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HighState>)))
  "Returns full string definition for message of type '<HighState>"
  (cl:format cl:nil "Header header~%uint8 levelFlag~%uint16 commVersion                  # Old version Aliengo does not have~%uint16 robotID                      # Old version Aliengo does not have~%uint32 SN                           # Old version Aliengo does not have~%uint8 bandWidth                     # Old version Aliengo does not have~%uint8 mode~%IMU imu~%float32 forwardSpeed~%float32 sideSpeed~%float32 rotateSpeed~%float32 bodyHeight~%float32 updownSpeed~%float32 forwardPosition       # (will be float type next version)   # Old version Aliengo is different~%float32 sidePosition          # (will be float type next version)   # Old version Aliengo is different~%Cartesian[4] footPosition2Body~%Cartesian[4] footSpeed2Body~%int16[4] footForce                  # Old version Aliengo is different~%int16[4] footForceEst               # Old version Aliengo does not have~%uint32 tick               ~%uint8[40] wirelessRemote~%uint32 reserve                      # Old version Aliengo does not have~%uint32 crc~%~%# Under are not defined in SDK yet. # Old version Aliengo does not have~%Cartesian[4] eeForce            # It's a 1-DOF force in real robot, but 3-DOF is better for visualization.~%float32[12] jointP              # for visualization~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: a1_pose_slam/IMU~%float32[4] quaternion~%float32[3] gyroscope~%float32[3] accelerometer~%int8 temperature~%================================================================================~%MSG: a1_pose_slam/Cartesian~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HighState)))
  "Returns full string definition for message of type 'HighState"
  (cl:format cl:nil "Header header~%uint8 levelFlag~%uint16 commVersion                  # Old version Aliengo does not have~%uint16 robotID                      # Old version Aliengo does not have~%uint32 SN                           # Old version Aliengo does not have~%uint8 bandWidth                     # Old version Aliengo does not have~%uint8 mode~%IMU imu~%float32 forwardSpeed~%float32 sideSpeed~%float32 rotateSpeed~%float32 bodyHeight~%float32 updownSpeed~%float32 forwardPosition       # (will be float type next version)   # Old version Aliengo is different~%float32 sidePosition          # (will be float type next version)   # Old version Aliengo is different~%Cartesian[4] footPosition2Body~%Cartesian[4] footSpeed2Body~%int16[4] footForce                  # Old version Aliengo is different~%int16[4] footForceEst               # Old version Aliengo does not have~%uint32 tick               ~%uint8[40] wirelessRemote~%uint32 reserve                      # Old version Aliengo does not have~%uint32 crc~%~%# Under are not defined in SDK yet. # Old version Aliengo does not have~%Cartesian[4] eeForce            # It's a 1-DOF force in real robot, but 3-DOF is better for visualization.~%float32[12] jointP              # for visualization~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: a1_pose_slam/IMU~%float32[4] quaternion~%float32[3] gyroscope~%float32[3] accelerometer~%int8 temperature~%================================================================================~%MSG: a1_pose_slam/Cartesian~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HighState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     2
     2
     4
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'imu))
     4
     4
     4
     4
     4
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'footPosition2Body) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'footSpeed2Body) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'footForce) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'footForceEst) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'wirelessRemote) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'eeForce) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'jointP) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HighState>))
  "Converts a ROS message object to a list"
  (cl:list 'HighState
    (cl:cons ':header (header msg))
    (cl:cons ':levelFlag (levelFlag msg))
    (cl:cons ':commVersion (commVersion msg))
    (cl:cons ':robotID (robotID msg))
    (cl:cons ':SN (SN msg))
    (cl:cons ':bandWidth (bandWidth msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':imu (imu msg))
    (cl:cons ':forwardSpeed (forwardSpeed msg))
    (cl:cons ':sideSpeed (sideSpeed msg))
    (cl:cons ':rotateSpeed (rotateSpeed msg))
    (cl:cons ':bodyHeight (bodyHeight msg))
    (cl:cons ':updownSpeed (updownSpeed msg))
    (cl:cons ':forwardPosition (forwardPosition msg))
    (cl:cons ':sidePosition (sidePosition msg))
    (cl:cons ':footPosition2Body (footPosition2Body msg))
    (cl:cons ':footSpeed2Body (footSpeed2Body msg))
    (cl:cons ':footForce (footForce msg))
    (cl:cons ':footForceEst (footForceEst msg))
    (cl:cons ':tick (tick msg))
    (cl:cons ':wirelessRemote (wirelessRemote msg))
    (cl:cons ':reserve (reserve msg))
    (cl:cons ':crc (crc msg))
    (cl:cons ':eeForce (eeForce msg))
    (cl:cons ':jointP (jointP msg))
))
