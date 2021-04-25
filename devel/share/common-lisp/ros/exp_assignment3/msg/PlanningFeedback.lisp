; Auto-generated. Do not edit!


(cl:in-package exp_assignment3-msg)


;//! \htmlinclude PlanningFeedback.msg.html

(cl:defclass <PlanningFeedback> (roslisp-msg-protocol:ros-message)
  ((stat
    :reader stat
    :initarg :stat
    :type cl:string
    :initform "")
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass PlanningFeedback (<PlanningFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanningFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanningFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name exp_assignment3-msg:<PlanningFeedback> is deprecated: use exp_assignment3-msg:PlanningFeedback instead.")))

(cl:ensure-generic-function 'stat-val :lambda-list '(m))
(cl:defmethod stat-val ((m <PlanningFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader exp_assignment3-msg:stat-val is deprecated.  Use exp_assignment3-msg:stat instead.")
  (stat m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <PlanningFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader exp_assignment3-msg:position-val is deprecated.  Use exp_assignment3-msg:position instead.")
  (position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanningFeedback>) ostream)
  "Serializes a message object of type '<PlanningFeedback>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'stat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'stat))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanningFeedback>) istream)
  "Deserializes a message object of type '<PlanningFeedback>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stat) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'stat) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanningFeedback>)))
  "Returns string type for a message object of type '<PlanningFeedback>"
  "exp_assignment3/PlanningFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanningFeedback)))
  "Returns string type for a message object of type 'PlanningFeedback"
  "exp_assignment3/PlanningFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanningFeedback>)))
  "Returns md5sum for a message object of type '<PlanningFeedback>"
  "436d418e7944afa3067a7cb1612a2ab0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanningFeedback)))
  "Returns md5sum for a message object of type 'PlanningFeedback"
  "436d418e7944afa3067a7cb1612a2ab0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanningFeedback>)))
  "Returns full string definition for message of type '<PlanningFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%string stat~%geometry_msgs/Pose position~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanningFeedback)))
  "Returns full string definition for message of type 'PlanningFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%string stat~%geometry_msgs/Pose position~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanningFeedback>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'stat))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanningFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanningFeedback
    (cl:cons ':stat (stat msg))
    (cl:cons ':position (position msg))
))