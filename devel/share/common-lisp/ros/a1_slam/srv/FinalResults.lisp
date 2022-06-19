; Auto-generated. Do not edit!


(cl:in-package a1_slam-srv)


;//! \htmlinclude FinalResults-request.msg.html

(cl:defclass <FinalResults-request> (roslisp-msg-protocol:ros-message)
  ((req
    :reader req
    :initarg :req
    :type cl:fixnum
    :initform 0))
)

(cl:defclass FinalResults-request (<FinalResults-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FinalResults-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FinalResults-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name a1_slam-srv:<FinalResults-request> is deprecated: use a1_slam-srv:FinalResults-request instead.")))

(cl:ensure-generic-function 'req-val :lambda-list '(m))
(cl:defmethod req-val ((m <FinalResults-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_slam-srv:req-val is deprecated.  Use a1_slam-srv:req instead.")
  (req m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FinalResults-request>) ostream)
  "Serializes a message object of type '<FinalResults-request>"
  (cl:let* ((signed (cl:slot-value msg 'req)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FinalResults-request>) istream)
  "Deserializes a message object of type '<FinalResults-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'req) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FinalResults-request>)))
  "Returns string type for a service object of type '<FinalResults-request>"
  "a1_slam/FinalResultsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FinalResults-request)))
  "Returns string type for a service object of type 'FinalResults-request"
  "a1_slam/FinalResultsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FinalResults-request>)))
  "Returns md5sum for a message object of type '<FinalResults-request>"
  "da3ecb06e58699211b37ea0e285d071a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FinalResults-request)))
  "Returns md5sum for a message object of type 'FinalResults-request"
  "da3ecb06e58699211b37ea0e285d071a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FinalResults-request>)))
  "Returns full string definition for message of type '<FinalResults-request>"
  (cl:format cl:nil "int8 req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FinalResults-request)))
  "Returns full string definition for message of type 'FinalResults-request"
  (cl:format cl:nil "int8 req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FinalResults-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FinalResults-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FinalResults-request
    (cl:cons ':req (req msg))
))
;//! \htmlinclude FinalResults-response.msg.html

(cl:defclass <FinalResults-response> (roslisp-msg-protocol:ros-message)
  ((results
    :reader results
    :initarg :results
    :type cl:string
    :initform ""))
)

(cl:defclass FinalResults-response (<FinalResults-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FinalResults-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FinalResults-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name a1_slam-srv:<FinalResults-response> is deprecated: use a1_slam-srv:FinalResults-response instead.")))

(cl:ensure-generic-function 'results-val :lambda-list '(m))
(cl:defmethod results-val ((m <FinalResults-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_slam-srv:results-val is deprecated.  Use a1_slam-srv:results instead.")
  (results m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FinalResults-response>) ostream)
  "Serializes a message object of type '<FinalResults-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'results))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'results))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FinalResults-response>) istream)
  "Deserializes a message object of type '<FinalResults-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FinalResults-response>)))
  "Returns string type for a service object of type '<FinalResults-response>"
  "a1_slam/FinalResultsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FinalResults-response)))
  "Returns string type for a service object of type 'FinalResults-response"
  "a1_slam/FinalResultsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FinalResults-response>)))
  "Returns md5sum for a message object of type '<FinalResults-response>"
  "da3ecb06e58699211b37ea0e285d071a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FinalResults-response)))
  "Returns md5sum for a message object of type 'FinalResults-response"
  "da3ecb06e58699211b37ea0e285d071a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FinalResults-response>)))
  "Returns full string definition for message of type '<FinalResults-response>"
  (cl:format cl:nil "string results~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FinalResults-response)))
  "Returns full string definition for message of type 'FinalResults-response"
  (cl:format cl:nil "string results~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FinalResults-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'results))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FinalResults-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FinalResults-response
    (cl:cons ':results (results msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FinalResults)))
  'FinalResults-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FinalResults)))
  'FinalResults-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FinalResults)))
  "Returns string type for a service object of type '<FinalResults>"
  "a1_slam/FinalResults")