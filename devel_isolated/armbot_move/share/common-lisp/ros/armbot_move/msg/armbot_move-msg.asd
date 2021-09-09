
(cl:in-package :asdf)

(defsystem "armbot_move-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "move_position" :depends-on ("_package_move_position"))
    (:file "_package_move_position" :depends-on ("_package"))
  ))