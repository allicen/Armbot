
(cl:in-package :asdf)

(defsystem "armbot_move-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SavePosition" :depends-on ("_package_SavePosition"))
    (:file "_package_SavePosition" :depends-on ("_package"))
    (:file "SetPosition" :depends-on ("_package_SetPosition"))
    (:file "_package_SetPosition" :depends-on ("_package"))
  ))