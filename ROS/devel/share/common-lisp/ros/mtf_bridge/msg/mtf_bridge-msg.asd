
(cl:in-package :asdf)

(defsystem "mtf_bridge-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "BufferInit" :depends-on ("_package_BufferInit"))
    (:file "_package_BufferInit" :depends-on ("_package"))
    (:file "Error" :depends-on ("_package_Error"))
    (:file "_package_Error" :depends-on ("_package"))
    (:file "Noise" :depends-on ("_package_Noise"))
    (:file "_package_Noise" :depends-on ("_package"))
    (:file "Patch" :depends-on ("_package_Patch"))
    (:file "_package_Patch" :depends-on ("_package"))
    (:file "PatchTrackers" :depends-on ("_package_PatchTrackers"))
    (:file "_package_PatchTrackers" :depends-on ("_package"))
    (:file "Point" :depends-on ("_package_Point"))
    (:file "_package_Point" :depends-on ("_package"))
  ))