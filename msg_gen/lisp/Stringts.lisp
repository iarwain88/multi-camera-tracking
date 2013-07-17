; Auto-generated. Do not edit!


(cl:in-package tutorialROSOpenCV-msg)


;//! \htmlinclude Stringts.msg.html

(cl:defclass <Stringts> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:string
    :initform "")
   (stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0))
)

(cl:defclass Stringts (<Stringts>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Stringts>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Stringts)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tutorialROSOpenCV-msg:<Stringts> is deprecated: use tutorialROSOpenCV-msg:Stringts instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Stringts>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tutorialROSOpenCV-msg:data-val is deprecated.  Use tutorialROSOpenCV-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <Stringts>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tutorialROSOpenCV-msg:stamp-val is deprecated.  Use tutorialROSOpenCV-msg:stamp instead.")
  (stamp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Stringts>) ostream)
  "Serializes a message object of type '<Stringts>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Stringts>) istream)
  "Deserializes a message object of type '<Stringts>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Stringts>)))
  "Returns string type for a message object of type '<Stringts>"
  "tutorialROSOpenCV/Stringts")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Stringts)))
  "Returns string type for a message object of type 'Stringts"
  "tutorialROSOpenCV/Stringts")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Stringts>)))
  "Returns md5sum for a message object of type '<Stringts>"
  "d1fd498755e0380859e3dd5ec7217be4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Stringts)))
  "Returns md5sum for a message object of type 'Stringts"
  "d1fd498755e0380859e3dd5ec7217be4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Stringts>)))
  "Returns full string definition for message of type '<Stringts>"
  (cl:format cl:nil "string data~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%time stamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Stringts)))
  "Returns full string definition for message of type 'Stringts"
  (cl:format cl:nil "string data~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%time stamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Stringts>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'data))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Stringts>))
  "Converts a ROS message object to a list"
  (cl:list 'Stringts
    (cl:cons ':data (data msg))
    (cl:cons ':stamp (stamp msg))
))
