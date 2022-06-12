;; Auto-generated. Do not edit!


(when (boundp 'a1_pose_slam::LED)
  (if (not (find-package "A1_POSE_SLAM"))
    (make-package "A1_POSE_SLAM"))
  (shadow 'LED (find-package "A1_POSE_SLAM")))
(unless (find-package "A1_POSE_SLAM::LED")
  (make-package "A1_POSE_SLAM::LED"))

(in-package "ROS")
;;//! \htmlinclude LED.msg.html


(defclass a1_pose_slam::LED
  :super ros::object
  :slots (_r _g _b ))

(defmethod a1_pose_slam::LED
  (:init
   (&key
    ((:r __r) 0)
    ((:g __g) 0)
    ((:b __b) 0)
    )
   (send-super :init)
   (setq _r (round __r))
   (setq _g (round __g))
   (setq _b (round __b))
   self)
  (:r
   (&optional __r)
   (if __r (setq _r __r)) _r)
  (:g
   (&optional __g)
   (if __g (setq _g __g)) _g)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:serialization-length
   ()
   (+
    ;; uint8 _r
    1
    ;; uint8 _g
    1
    ;; uint8 _b
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _r
       (write-byte _r s)
     ;; uint8 _g
       (write-byte _g s)
     ;; uint8 _b
       (write-byte _b s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _r
     (setq _r (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _g
     (setq _g (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _b
     (setq _b (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get a1_pose_slam::LED :md5sum-) "353891e354491c51aabe32df673fb446")
(setf (get a1_pose_slam::LED :datatype-) "a1_pose_slam/LED")
(setf (get a1_pose_slam::LED :definition-)
      "uint8 r
uint8 g
uint8 b
")



(provide :a1_pose_slam/LED "353891e354491c51aabe32df673fb446")


