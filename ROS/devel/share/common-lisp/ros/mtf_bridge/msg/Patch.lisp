; Auto-generated. Do not edit!


(cl:in-package mtf_bridge-msg)


;//! \htmlinclude Patch.msg.html

(cl:defclass <Patch> (roslisp-msg-protocol:ros-message)
  ((corners
    :reader corners
    :initarg :corners
    :type (cl:vector mtf_bridge-msg:Point)
   :initform (cl:make-array 4 :element-type 'mtf_bridge-msg:Point :initial-element (cl:make-instance 'mtf_bridge-msg:Point)))
   (center
    :reader center
    :initarg :center
    :type mtf_bridge-msg:Point
    :initform (cl:make-instance 'mtf_bridge-msg:Point)))
)

(cl:defclass Patch (<Patch>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Patch>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Patch)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mtf_bridge-msg:<Patch> is deprecated: use mtf_bridge-msg:Patch instead.")))

(cl:ensure-generic-function 'corners-val :lambda-list '(m))
(cl:defmethod corners-val ((m <Patch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtf_bridge-msg:corners-val is deprecated.  Use mtf_bridge-msg:corners instead.")
  (corners m))

(cl:ensure-generic-function 'center-val :lambda-list '(m))
(cl:defmethod center-val ((m <Patch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtf_bridge-msg:center-val is deprecated.  Use mtf_bridge-msg:center instead.")
  (center m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Patch>) ostream)
  "Serializes a message object of type '<Patch>"
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'corners))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'center) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Patch>) istream)
  "Deserializes a message object of type '<Patch>"
  (cl:setf (cl:slot-value msg 'corners) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'corners)))
    (cl:dotimes (i 4)
    (cl:setf (cl:aref vals i) (cl:make-instance 'mtf_bridge-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'center) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Patch>)))
  "Returns string type for a message object of type '<Patch>"
  "mtf_bridge/Patch")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Patch)))
  "Returns string type for a message object of type 'Patch"
  "mtf_bridge/Patch")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Patch>)))
  "Returns md5sum for a message object of type '<Patch>"
  "1fe35727486600b70962648847a432de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Patch)))
  "Returns md5sum for a message object of type 'Patch"
  "1fe35727486600b70962648847a432de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Patch>)))
  "Returns full string definition for message of type '<Patch>"
  (cl:format cl:nil "Point[4] corners~%Point center~%~%================================================================================~%MSG: mtf_bridge/Point~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Patch)))
  "Returns full string definition for message of type 'Patch"
  (cl:format cl:nil "Point[4] corners~%Point center~%~%================================================================================~%MSG: mtf_bridge/Point~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Patch>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'corners) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'center))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Patch>))
  "Converts a ROS message object to a list"
  (cl:list 'Patch
    (cl:cons ':corners (corners msg))
    (cl:cons ':center (center msg))
))
