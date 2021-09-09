;; Auto-generated. Do not edit!


(when (boundp 'armbot_move::move_position)
  (if (not (find-package "ARMBOT_MOVE"))
    (make-package "ARMBOT_MOVE"))
  (shadow 'move_position (find-package "ARMBOT_MOVE")))
(unless (find-package "ARMBOT_MOVE::MOVE_POSITION")
  (make-package "ARMBOT_MOVE::MOVE_POSITION"))

(in-package "ROS")
;;//! \htmlinclude move_position.msg.html


(defclass armbot_move::move_position
  :super ros::object
  :slots (_position _x _y ))

(defmethod armbot_move::move_position
  (:init
   (&key
    ((:position __position) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    )
   (send-super :init)
   (setq _position (string __position))
   (setq _x (float __x))
   (setq _y (float __y))
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; string _position
    4 (length _position)
    ;; float32 _x
    4
    ;; float32 _y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _position
       (write-long (length _position) s) (princ _position s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _position
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _position (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get armbot_move::move_position :md5sum-) "4286d84b6e49763290a7e754b908f274")
(setf (get armbot_move::move_position :datatype-) "armbot_move/move_position")
(setf (get armbot_move::move_position :definition-)
      "string position
float32 x
float32 y
")



(provide :armbot_move/move_position "4286d84b6e49763290a7e754b908f274")


