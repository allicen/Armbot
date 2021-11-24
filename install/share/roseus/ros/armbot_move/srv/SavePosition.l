;; Auto-generated. Do not edit!


(when (boundp 'armbot_move::SavePosition)
  (if (not (find-package "ARMBOT_MOVE"))
    (make-package "ARMBOT_MOVE"))
  (shadow 'SavePosition (find-package "ARMBOT_MOVE")))
(unless (find-package "ARMBOT_MOVE::SAVEPOSITION")
  (make-package "ARMBOT_MOVE::SAVEPOSITION"))
(unless (find-package "ARMBOT_MOVE::SAVEPOSITIONREQUEST")
  (make-package "ARMBOT_MOVE::SAVEPOSITIONREQUEST"))
(unless (find-package "ARMBOT_MOVE::SAVEPOSITIONRESPONSE")
  (make-package "ARMBOT_MOVE::SAVEPOSITIONRESPONSE"))

(in-package "ROS")





(defclass armbot_move::SavePositionRequest
  :super ros::object
  :slots (_save ))

(defmethod armbot_move::SavePositionRequest
  (:init
   (&key
    ((:save __save) "")
    )
   (send-super :init)
   (setq _save (string __save))
   self)
  (:save
   (&optional __save)
   (if __save (setq _save __save)) _save)
  (:serialization-length
   ()
   (+
    ;; string _save
    4 (length _save)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _save
       (write-long (length _save) s) (princ _save s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _save
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _save (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass armbot_move::SavePositionResponse
  :super ros::object
  :slots (_result ))

(defmethod armbot_move::SavePositionResponse
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

(defclass armbot_move::SavePosition
  :super ros::object
  :slots ())

(setf (get armbot_move::SavePosition :md5sum-) "61b51a87ebe23475b436f7b122d27fe9")
(setf (get armbot_move::SavePosition :datatype-) "armbot_move/SavePosition")
(setf (get armbot_move::SavePosition :request) armbot_move::SavePositionRequest)
(setf (get armbot_move::SavePosition :response) armbot_move::SavePositionResponse)

(defmethod armbot_move::SavePositionRequest
  (:response () (instance armbot_move::SavePositionResponse :init)))

(setf (get armbot_move::SavePositionRequest :md5sum-) "61b51a87ebe23475b436f7b122d27fe9")
(setf (get armbot_move::SavePositionRequest :datatype-) "armbot_move/SavePositionRequest")
(setf (get armbot_move::SavePositionRequest :definition-)
      "string save
---
string result
")

(setf (get armbot_move::SavePositionResponse :md5sum-) "61b51a87ebe23475b436f7b122d27fe9")
(setf (get armbot_move::SavePositionResponse :datatype-) "armbot_move/SavePositionResponse")
(setf (get armbot_move::SavePositionResponse :definition-)
      "string save
---
string result
")



(provide :armbot_move/SavePosition "61b51a87ebe23475b436f7b122d27fe9")


