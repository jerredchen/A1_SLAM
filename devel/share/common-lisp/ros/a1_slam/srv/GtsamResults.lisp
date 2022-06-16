; Auto-generated. Do not edit!


(cl:in-package a1_slam-srv)


;//! \htmlinclude GtsamResults-request.msg.html

(cl:defclass <GtsamResults-request> (roslisp-msg-protocol:ros-message)
  ((req
    :reader req
    :initarg :req
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GtsamResults-request (<GtsamResults-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GtsamResults-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GtsamResults-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name a1_slam-srv:<GtsamResults-request> is deprecated: use a1_slam-srv:GtsamResults-request instead.")))

(cl:ensure-generic-function 'req-val :lambda-list '(m))
(cl:defmethod req-val ((m <GtsamResults-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_slam-srv:req-val is deprecated.  Use a1_slam-srv:req instead.")
  (req m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GtsamResults-request>) ostream)
  "Serializes a message object of type '<GtsamResults-request>"
  (cl:let* ((signed (cl:slot-value msg 'req)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GtsamResults-request>) istream)
  "Deserializes a message object of type '<GtsamResults-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'req) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
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
  "7189306d182a38b37d781470320bb653")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GtsamResults-request)))
  "Returns md5sum for a message object of type 'GtsamResults-request"
  "7189306d182a38b37d781470320bb653")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GtsamResults-request>)))
  "Returns full string definition for message of type '<GtsamResults-request>"
  (cl:format cl:nil "int8 req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GtsamResults-request)))
  "Returns full string definition for message of type 'GtsamResults-request"
  (cl:format cl:nil "int8 req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GtsamResults-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GtsamResults-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GtsamResults-request
    (cl:cons ':req (req msg))
))
;//! \htmlinclude GtsamResults-response.msg.html

(cl:defclass <GtsamResults-response> (roslisp-msg-protocol:ros-message)
  ((str
    :reader str
    :initarg :str
    :type cl:string
    :initform ""))
)

(cl:defclass GtsamResults-response (<GtsamResults-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GtsamResults-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GtsamResults-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name a1_slam-srv:<GtsamResults-response> is deprecated: use a1_slam-srv:GtsamResults-response instead.")))

(cl:ensure-generic-function 'str-val :lambda-list '(m))
(cl:defmethod str-val ((m <GtsamResults-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_slam-srv:str-val is deprecated.  Use a1_slam-srv:str instead.")
  (str m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GtsamResults-response>) ostream)
  "Serializes a message object of type '<GtsamResults-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'str))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'str))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GtsamResults-response>) istream)
  "Deserializes a message object of type '<GtsamResults-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'str) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'str) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
  "7189306d182a38b37d781470320bb653")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GtsamResults-response)))
  "Returns md5sum for a message object of type 'GtsamResults-response"
  "7189306d182a38b37d781470320bb653")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GtsamResults-response>)))
  "Returns full string definition for message of type '<GtsamResults-response>"
  (cl:format cl:nil "string str~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GtsamResults-response)))
  "Returns full string definition for message of type 'GtsamResults-response"
  (cl:format cl:nil "string str~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GtsamResults-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'str))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GtsamResults-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GtsamResults-response
    (cl:cons ':str (str msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GtsamResults)))
  'GtsamResults-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GtsamResults)))
  'GtsamResults-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GtsamResults)))
  "Returns string type for a service object of type '<GtsamResults>"
  "a1_slam/GtsamResults")