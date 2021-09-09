;; Auto-generated. Do not edit!


(when (boundp 'armbot_move::SetPosition)
  (if (not (find-package "ARMBOT_MOVE"))
    (make-package "ARMBOT_MOVE"))
  (shadow 'SetPosition (find-package "ARMBOT_MOVE")))
(unless (find-package "ARMBOT_MOVE::SETPOSITION")
  (make-package "ARMBOT_MOVE::SETPOSITION"))
(unless (find-package "ARMBOT_MOVE::SETPOSITIONREQUEST")
  (make-package "ARMBOT_MOVE::SETPOSITIONREQUEST"))
(unless (find-package "ARMBOT_MOVE::SETPOSITIONRESPONSE")
  (make-package "ARMBOT_MOVE::SETPOSITIONRESPONSE"))

(in-package "ROS")





(defclass armbot_move::SetPositionRequest
  :super ros::object
  :slots (_position _x _y ))

(defmethod armbot_move::SetPositionRequest
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

(defclass armbot_move::SetPositionResponse
  :super ros::object
  :slots (_result ))

(defmethod armbot_move::SetPositionResponse
  (:init
   (&key
    ((:result __result) "")
    )
   (send-super :init)
   (setq _result (string __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; string _result
    4 (length _result)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _result
       (write-long (length _result) s) (princ _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _result
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _result (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass armbot_move::SetPosition
  :super ros::object
  :slots ())

(setf (get armbot_move::SetPosition :md5sum-) "44ff66b996c0035936f8e48c32854dc3")
(setf (get armbot_move::SetPosition :datatype-) "armbot_move/SetPosition")
(setf (get armbot_move::SetPosition :request) armbot_move::SetPositionRequest)
(setf (get armbot_move::SetPosition :response) armbot_move::SetPositionResponse)

(defmethod armbot_move::SetPositionRequest
  (:response () (instance armbot_move::SetPositionResponse :init)))

(setf (get armbot_move::SetPositionRequest :md5sum-) "44ff66b996c0035936f8e48c32854dc3")
(setf (get armbot_move::SetPositionRequest :datatype-) "armbot_move/SetPositionRequest")
(setf (get armbot_move::SetPositionRequest :definition-)
      "string position
float32 x
float32 y
---
string result
")

(setf (get armbot_move::SetPositionResponse :md5sum-) "44ff66b996c0035936f8e48c32854dc3")
(setf (get armbot_move::SetPositionResponse :datatype-) "armbot_move/SetPositionResponse")
(setf (get armbot_move::SetPositionResponse :definition-)
      "string position
float32 x
float32 y
---
string result
")



(provide :armbot_move/SetPosition "44ff66b996c0035936f8e48c32854dc3")


