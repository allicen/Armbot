; Auto-generated. Do not edit!


(cl:in-package armbot_move-msg)


;//! \htmlinclude move_position.msg.html

(cl:defclass <move_position> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type cl:string
    :initform "")
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0))
)

(cl:defclass move_position (<move_position>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <move_position>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'move_position)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name armbot_move-msg:<move_position> is deprecated: use armbot_move-msg:move_position instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <move_position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader armbot_move-msg:position-val is deprecated.  Use armbot_move-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <move_position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader armbot_move-msg:x-val is deprecated.  Use armbot_move-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <move_position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader armbot_move-msg:y-val is deprecated.  Use armbot_move-msg:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <move_position>) ostream)
  "Serializes a message object of type '<move_position>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'position))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <move_position>) istream)
  "Deserializes a message object of type '<move_position>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'position) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'position) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<move_position>)))
  "Returns string type for a message object of type '<move_position>"
  "armbot_move/move_position")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'move_position)))
  "Returns string type for a message object of type 'move_position"
  "armbot_move/move_position")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<move_position>)))
  "Returns md5sum for a message object of type '<move_position>"
  "4286d84b6e49763290a7e754b908f274")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'move_position)))
  "Returns md5sum for a message object of type 'move_position"
  "4286d84b6e49763290a7e754b908f274")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<move_position>)))
  "Returns full string definition for message of type '<move_position>"
  (cl:format cl:nil "string position~%float32 x~%float32 y~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'move_position)))
  "Returns full string definition for message of type 'move_position"
  (cl:format cl:nil "string position~%float32 x~%float32 y~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <move_position>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'position))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <move_position>))
  "Converts a ROS message object to a list"
  (cl:list 'move_position
    (cl:cons ':position (position msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
