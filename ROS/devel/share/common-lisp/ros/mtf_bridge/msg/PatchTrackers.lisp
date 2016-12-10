; Auto-generated. Do not edit!


(cl:in-package mtf_bridge-msg)


;//! \htmlinclude PatchTrackers.msg.html

(cl:defclass <PatchTrackers> (roslisp-msg-protocol:ros-message)
  ((trackers
    :reader trackers
    :initarg :trackers
    :type (cl:vector mtf_bridge-msg:Patch)
   :initform (cl:make-array 0 :element-type 'mtf_bridge-msg:Patch :initial-element (cl:make-instance 'mtf_bridge-msg:Patch))))
)

(cl:defclass PatchTrackers (<PatchTrackers>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PatchTrackers>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PatchTrackers)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mtf_bridge-msg:<PatchTrackers> is deprecated: use mtf_bridge-msg:PatchTrackers instead.")))

(cl:ensure-generic-function 'trackers-val :lambda-list '(m))
(cl:defmethod trackers-val ((m <PatchTrackers>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtf_bridge-msg:trackers-val is deprecated.  Use mtf_bridge-msg:trackers instead.")
  (trackers m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PatchTrackers>) ostream)
  "Serializes a message object of type '<PatchTrackers>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'trackers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'trackers))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PatchTrackers>) istream)
  "Deserializes a message object of type '<PatchTrackers>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'trackers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'trackers)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'mtf_bridge-msg:Patch))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PatchTrackers>)))
  "Returns string type for a message object of type '<PatchTrackers>"
  "mtf_bridge/PatchTrackers")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PatchTrackers)))
  "Returns string type for a message object of type 'PatchTrackers"
  "mtf_bridge/PatchTrackers")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PatchTrackers>)))
  "Returns md5sum for a message object of type '<PatchTrackers>"
  "2b617595ec9fb84f8e093a4f20db41ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PatchTrackers)))
  "Returns md5sum for a message object of type 'PatchTrackers"
  "2b617595ec9fb84f8e093a4f20db41ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PatchTrackers>)))
  "Returns full string definition for message of type '<PatchTrackers>"
  (cl:format cl:nil "Patch[] trackers~%~%================================================================================~%MSG: mtf_bridge/Patch~%Point[4] corners~%Point center~%~%================================================================================~%MSG: mtf_bridge/Point~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PatchTrackers)))
  "Returns full string definition for message of type 'PatchTrackers"
  (cl:format cl:nil "Patch[] trackers~%~%================================================================================~%MSG: mtf_bridge/Patch~%Point[4] corners~%Point center~%~%================================================================================~%MSG: mtf_bridge/Point~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PatchTrackers>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trackers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PatchTrackers>))
  "Converts a ROS message object to a list"
  (cl:list 'PatchTrackers
    (cl:cons ':trackers (trackers msg))
))
