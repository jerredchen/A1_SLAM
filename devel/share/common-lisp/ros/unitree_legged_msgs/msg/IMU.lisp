; Auto-generated. Do not edit!


(cl:in-package unitree_legged_msgs-msg)


;//! \htmlinclude IMU.msg.html

(cl:defclass <IMU> (roslisp-msg-protocol:ros-message)
  ((quaternion
    :reader quaternion
    :initarg :quaternion
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (gyroscope
    :reader gyroscope
    :initarg :gyroscope
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (accelerometer
    :reader accelerometer
    :initarg :accelerometer
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (temperature
    :reader temperature
    :initarg :temperature
    :type cl:fixnum
    :initform 0))
)

(cl:defclass IMU (<IMU>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IMU>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IMU)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name unitree_legged_msgs-msg:<IMU> is deprecated: use unitree_legged_msgs-msg:IMU instead.")))

(cl:ensure-generic-function 'quaternion-val :lambda-list '(m))
(cl:defmethod quaternion-val ((m <IMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:quaternion-val is deprecated.  Use unitree_legged_msgs-msg:quaternion instead.")
  (quaternion m))

(cl:ensure-generic-function 'gyroscope-val :lambda-list '(m))
(cl:defmethod gyroscope-val ((m <IMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:gyroscope-val is deprecated.  Use unitree_legged_msgs-msg:gyroscope instead.")
  (gyroscope m))

(cl:ensure-generic-function 'accelerometer-val :lambda-list '(m))
(cl:defmethod accelerometer-val ((m <IMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:accelerometer-val is deprecated.  Use unitree_legged_msgs-msg:accelerometer instead.")
  (accelerometer m))

(cl:ensure-generic-function 'temperature-val :lambda-list '(m))
(cl:defmethod temperature-val ((m <IMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unitree_legged_msgs-msg:temperature-val is deprecated.  Use unitree_legged_msgs-msg:temperature instead.")
  (temperature m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IMU>) ostream)
  "Serializes a message object of type '<IMU>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'quaternion))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'gyroscope))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'accelerometer))
  (cl:let* ((signed (cl:slot-value msg 'temperature)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IMU>) istream)
  "Deserializes a message object of type '<IMU>"
  (cl:setf (cl:slot-value msg 'quaternion) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'quaternion)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'gyroscope) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'gyroscope)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'accelerometer) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'accelerometer)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'temperature) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IMU>)))
  "Returns string type for a message object of type '<IMU>"
  "unitree_legged_msgs/IMU")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IMU)))
  "Returns string type for a message object of type 'IMU"
  "unitree_legged_msgs/IMU")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IMU>)))
  "Returns md5sum for a message object of type '<IMU>"
  "dd4bb4e42aa2f15aa1fb1b6a3c3752cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IMU)))
  "Returns md5sum for a message object of type 'IMU"
  "dd4bb4e42aa2f15aa1fb1b6a3c3752cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IMU>)))
  "Returns full string definition for message of type '<IMU>"
  (cl:format cl:nil "float32[4] quaternion~%float32[3] gyroscope~%float32[3] accelerometer~%int8 temperature~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IMU)))
  "Returns full string definition for message of type 'IMU"
  (cl:format cl:nil "float32[4] quaternion~%float32[3] gyroscope~%float32[3] accelerometer~%int8 temperature~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IMU>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'quaternion) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'gyroscope) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'accelerometer) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IMU>))
  "Converts a ROS message object to a list"
  (cl:list 'IMU
    (cl:cons ':quaternion (quaternion msg))
    (cl:cons ':gyroscope (gyroscope msg))
    (cl:cons ':accelerometer (accelerometer msg))
    (cl:cons ':temperature (temperature msg))
))
