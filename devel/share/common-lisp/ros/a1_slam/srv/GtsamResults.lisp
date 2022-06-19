; Auto-generated. Do not edit!


(cl:in-package a1_slam-srv)


;//! \htmlinclude GtsamResults-request.msg.html

(cl:defclass <GtsamResults-request> (roslisp-msg-protocol:ros-message)
  ((factor_type
    :reader factor_type
    :initarg :factor_type
    :type cl:string
    :initform "")
   (factor
    :reader factor
    :initarg :factor
    :type cl:string
    :initform "")
   (key
    :reader key
    :initarg :key
    :type cl:integer
    :initform 0)
   (init_estimate
    :reader init_estimate
    :initarg :init_estimate
    :type cl:string
    :initform ""))
)

(cl:defclass GtsamResults-request (<GtsamResults-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GtsamResults-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GtsamResults-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name a1_slam-srv:<GtsamResults-request> is deprecated: use a1_slam-srv:GtsamResults-request instead.")))

(cl:ensure-generic-function 'factor_type-val :lambda-list '(m))
(cl:defmethod factor_type-val ((m <GtsamResults-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_slam-srv:factor_type-val is deprecated.  Use a1_slam-srv:factor_type instead.")
  (factor_type m))

(cl:ensure-generic-function 'factor-val :lambda-list '(m))
(cl:defmethod factor-val ((m <GtsamResults-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_slam-srv:factor-val is deprecated.  Use a1_slam-srv:factor instead.")
  (factor m))

(cl:ensure-generic-function 'key-val :lambda-list '(m))
(cl:defmethod key-val ((m <GtsamResults-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_slam-srv:key-val is deprecated.  Use a1_slam-srv:key instead.")
  (key m))

(cl:ensure-generic-function 'init_estimate-val :lambda-list '(m))
(cl:defmethod init_estimate-val ((m <GtsamResults-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_slam-srv:init_estimate-val is deprecated.  Use a1_slam-srv:init_estimate instead.")
  (init_estimate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GtsamResults-request>) ostream)
  "Serializes a message object of type '<GtsamResults-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'factor_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'factor_type))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'factor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'factor))
  (cl:let* ((signed (cl:slot-value msg 'key)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'init_estimate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'init_estimate))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GtsamResults-request>) istream)
  "Deserializes a message object of type '<GtsamResults-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'factor_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'factor_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'factor) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'factor) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'key) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'init_estimate) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'init_estimate) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GtsamResults-request>)))
  "Returns string type for a service object of type '<GtsamResults-request>"
  "a1_slam/GtsamResultsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GtsamResults-request)))
  "Returns string type for a service object of type 'GtsamResults-request"
  "a1_slam/GtsamResultsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GtsamResults-request>)))
  "Returns md5sum for a message object of type '<GtsamResults-request>"
  "0e35ec748ec070bc1eff3fd9921a52fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GtsamResults-request)))
  "Returns md5sum for a message object of type 'GtsamResults-request"
  "0e35ec748ec070bc1eff3fd9921a52fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GtsamResults-request>)))
  "Returns full string definition for message of type '<GtsamResults-request>"
  (cl:format cl:nil "string factor_type~%string factor~%int64 key~%string init_estimate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GtsamResults-request)))
  "Returns full string definition for message of type 'GtsamResults-request"
  (cl:format cl:nil "string factor_type~%string factor~%int64 key~%string init_estimate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GtsamResults-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'factor_type))
     4 (cl:length (cl:slot-value msg 'factor))
     8
     4 (cl:length (cl:slot-value msg 'init_estimate))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GtsamResults-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GtsamResults-request
    (cl:cons ':factor_type (factor_type msg))
    (cl:cons ':factor (factor msg))
    (cl:cons ':key (key msg))
    (cl:cons ':init_estimate (init_estimate msg))
))
;//! \htmlinclude GtsamResults-response.msg.html

(cl:defclass <GtsamResults-response> (roslisp-msg-protocol:ros-message)
  ((results
    :reader results
    :initarg :results
    :type cl:string
    :initform ""))
)

(cl:defclass GtsamResults-response (<GtsamResults-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GtsamResults-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GtsamResults-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name a1_slam-srv:<GtsamResults-response> is deprecated: use a1_slam-srv:GtsamResults-response instead.")))

(cl:ensure-generic-function 'results-val :lambda-list '(m))
(cl:defmethod results-val ((m <GtsamResults-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_slam-srv:results-val is deprecated.  Use a1_slam-srv:results instead.")
  (results m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GtsamResults-response>) ostream)
  "Serializes a message object of type '<GtsamResults-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'results))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'results))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GtsamResults-response>) istream)
  "Deserializes a message object of type '<GtsamResults-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'results) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'results) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GtsamResults-response>)))
  "Returns string type for a service object of type '<GtsamResults-response>"
  "a1_slam/GtsamResultsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GtsamResults-response)))
  "Returns string type for a service object of type 'GtsamResults-response"
  "a1_slam/GtsamResultsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GtsamResults-response>)))
  "Returns md5sum for a message object of type '<GtsamResults-response>"
  "0e35ec748ec070bc1eff3fd9921a52fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GtsamResults-response)))
  "Returns md5sum for a message object of type 'GtsamResults-response"
  "0e35ec748ec070bc1eff3fd9921a52fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GtsamResults-response>)))
  "Returns full string definition for message of type '<GtsamResults-response>"
  (cl:format cl:nil "string results~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GtsamResults-response)))
  "Returns full string definition for message of type 'GtsamResults-response"
  (cl:format cl:nil "string results~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GtsamResults-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'results))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GtsamResults-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GtsamResults-response
    (cl:cons ':results (results msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GtsamResults)))
  'GtsamResults-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GtsamResults)))
  'GtsamResults-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GtsamResults)))
  "Returns string type for a service object of type '<GtsamResults>"
  "a1_slam/GtsamResults")