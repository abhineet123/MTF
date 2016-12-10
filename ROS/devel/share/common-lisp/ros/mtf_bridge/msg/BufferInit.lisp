; Auto-generated. Do not edit!


(cl:in-package mtf_bridge-msg)


;//! \htmlinclude BufferInit.msg.html

(cl:defclass <BufferInit> (roslisp-msg-protocol:ros-message)
  ((height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (channels
    :reader channels
    :initarg :channels
    :type cl:integer
    :initform 0)
   (buffer_count
    :reader buffer_count
    :initarg :buffer_count
    :type cl:integer
    :initform 0)
   (frame_size
    :reader frame_size
    :initarg :frame_size
    :type cl:integer
    :initform 0)
   (shm_num
    :reader shm_num
    :initarg :shm_num
    :type cl:integer
    :initform 0)
   (init_id
    :reader init_id
    :initarg :init_id
    :type cl:integer
    :initform 0))
)

(cl:defclass BufferInit (<BufferInit>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BufferInit>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BufferInit)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mtf_bridge-msg:<BufferInit> is deprecated: use mtf_bridge-msg:BufferInit instead.")))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <BufferInit>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtf_bridge-msg:height-val is deprecated.  Use mtf_bridge-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <BufferInit>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtf_bridge-msg:width-val is deprecated.  Use mtf_bridge-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'channels-val :lambda-list '(m))
(cl:defmethod channels-val ((m <BufferInit>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtf_bridge-msg:channels-val is deprecated.  Use mtf_bridge-msg:channels instead.")
  (channels m))

(cl:ensure-generic-function 'buffer_count-val :lambda-list '(m))
(cl:defmethod buffer_count-val ((m <BufferInit>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtf_bridge-msg:buffer_count-val is deprecated.  Use mtf_bridge-msg:buffer_count instead.")
  (buffer_count m))

(cl:ensure-generic-function 'frame_size-val :lambda-list '(m))
(cl:defmethod frame_size-val ((m <BufferInit>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtf_bridge-msg:frame_size-val is deprecated.  Use mtf_bridge-msg:frame_size instead.")
  (frame_size m))

(cl:ensure-generic-function 'shm_num-val :lambda-list '(m))
(cl:defmethod shm_num-val ((m <BufferInit>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtf_bridge-msg:shm_num-val is deprecated.  Use mtf_bridge-msg:shm_num instead.")
  (shm_num m))

(cl:ensure-generic-function 'init_id-val :lambda-list '(m))
(cl:defmethod init_id-val ((m <BufferInit>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mtf_bridge-msg:init_id-val is deprecated.  Use mtf_bridge-msg:init_id instead.")
  (init_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BufferInit>) ostream)
  "Serializes a message object of type '<BufferInit>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'channels)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'channels)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'channels)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'channels)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'buffer_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'buffer_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'buffer_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'buffer_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'shm_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'shm_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'shm_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'shm_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'init_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'init_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'init_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'init_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BufferInit>) istream)
  "Deserializes a message object of type '<BufferInit>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'channels)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'channels)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'channels)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'channels)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'buffer_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'buffer_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'buffer_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'buffer_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'shm_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'shm_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'shm_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'shm_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'init_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'init_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'init_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'init_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BufferInit>)))
  "Returns string type for a message object of type '<BufferInit>"
  "mtf_bridge/BufferInit")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BufferInit)))
  "Returns string type for a message object of type 'BufferInit"
  "mtf_bridge/BufferInit")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BufferInit>)))
  "Returns md5sum for a message object of type '<BufferInit>"
  "7e7b7c3b3633bfc4b67478213b87074a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BufferInit)))
  "Returns md5sum for a message object of type 'BufferInit"
  "7e7b7c3b3633bfc4b67478213b87074a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BufferInit>)))
  "Returns full string definition for message of type '<BufferInit>"
  (cl:format cl:nil "uint32 height~%uint32 width~%uint32 channels~%uint32 buffer_count~%uint32 frame_size~%uint32 shm_num~%uint32 init_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BufferInit)))
  "Returns full string definition for message of type 'BufferInit"
  (cl:format cl:nil "uint32 height~%uint32 width~%uint32 channels~%uint32 buffer_count~%uint32 frame_size~%uint32 shm_num~%uint32 init_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BufferInit>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BufferInit>))
  "Converts a ROS message object to a list"
  (cl:list 'BufferInit
    (cl:cons ':height (height msg))
    (cl:cons ':width (width msg))
    (cl:cons ':channels (channels msg))
    (cl:cons ':buffer_count (buffer_count msg))
    (cl:cons ':frame_size (frame_size msg))
    (cl:cons ':shm_num (shm_num msg))
    (cl:cons ':init_id (init_id msg))
))
