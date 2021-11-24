; Auto-generated. Do not edit!


(cl:in-package armbot_move-srv)


;//! \htmlinclude SavePosition-request.msg.html

(cl:defclass <SavePosition-request> (roslisp-msg-protocol:ros-message)
  ((save
    :reader save
    :initarg :save
    :type cl:string
    :initform ""))
)

(cl:defclass SavePosition-request (<SavePosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SavePosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SavePosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name armbot_move-srv:<SavePosition-request> is deprecated: use armbot_move-srv:SavePosition-request instead.")))

(cl:ensure-generic-function 'save-val :lambda-list '(m))
(cl:defmethod save-val ((m <SavePosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader armbot_move-srv:save-val is deprecated.  Use armbot_move-srv:save instead.")
  (save m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SavePosition-request>) ostream)
  "Serializes a message object of type '<SavePosition-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'save))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'save))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SavePosition-request>) istream)
  "Deserializes a message object of type '<SavePosition-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'save) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'save) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SavePosition-request>)))
  "Returns string type for a service object of type '<SavePosition-request>"
  "armbot_move/SavePositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SavePosition-request)))
  "Returns string type for a service object of type 'SavePosition-request"
  "armbot_move/SavePositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SavePosition-request>)))
  "Returns md5sum for a message object of type '<SavePosition-request>"
  "61b51a87ebe23475b436f7b122d27fe9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SavePosition-request)))
  "Returns md5sum for a message object of type 'SavePosition-request"
  "61b51a87ebe23475b436f7b122d27fe9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SavePosition-request>)))
  "Returns full string definition for message of type '<SavePosition-request>"
  (cl:format cl:nil "string save~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SavePosition-request)))
  "Returns full string definition for message of type 'SavePosition-request"
  (cl:format cl:nil "string save~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SavePosition-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'save))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SavePosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SavePosition-request
    (cl:cons ':save (save msg))
))
;//! \htmlinclude SavePosition-response.msg.html

(cl:defclass <SavePosition-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass SavePosition-response (<SavePosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SavePosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SavePosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name armbot_move-srv:<SavePosition-response> is deprecated: use armbot_move-srv:SavePosition-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SavePosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader armbot_move-srv:result-val is deprecated.  Use armbot_move-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SavePosition-response>) ostream)
  "Serializes a message object of type '<SavePosition-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SavePosition-response>) istream)
  "Deserializes a message object of type '<SavePosition-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SavePosition-response>)))
  "Returns string type for a service object of type '<SavePosition-response>"
  "armbot_move/SavePositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SavePosition-response)))
  "Returns string type for a service object of type 'SavePosition-response"
  "armbot_move/SavePositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SavePosition-response>)))
  "Returns md5sum for a message object of type '<SavePosition-response>"
  "61b51a87ebe23475b436f7b122d27fe9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SavePosition-response)))
  "Returns md5sum for a message object of type 'SavePosition-response"
  "61b51a87ebe23475b436f7b122d27fe9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SavePosition-response>)))
  "Returns full string definition for message of type '<SavePosition-response>"
  (cl:format cl:nil "string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SavePosition-response)))
  "Returns full string definition for message of type 'SavePosition-response"
  (cl:format cl:nil "string result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SavePosition-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SavePosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SavePosition-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SavePosition)))
  'SavePosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SavePosition)))
  'SavePosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SavePosition)))
  "Returns string type for a service object of type '<SavePosition>"
  "armbot_move/SavePosition")