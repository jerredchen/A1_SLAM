;; Auto-generated. Do not edit!


(when (boundp 'a1_slam::GtsamResults)
  (if (not (find-package "A1_SLAM"))
    (make-package "A1_SLAM"))
  (shadow 'GtsamResults (find-package "A1_SLAM")))
(unless (find-package "A1_SLAM::GTSAMRESULTS")
  (make-package "A1_SLAM::GTSAMRESULTS"))
(unless (find-package "A1_SLAM::GTSAMRESULTSREQUEST")
  (make-package "A1_SLAM::GTSAMRESULTSREQUEST"))
(unless (find-package "A1_SLAM::GTSAMRESULTSRESPONSE")
  (make-package "A1_SLAM::GTSAMRESULTSRESPONSE"))

(in-package "ROS")





(defclass a1_slam::GtsamResultsRequest
  :super ros::object
  :slots (_req ))

(defmethod a1_slam::GtsamResultsRequest
  (:init
   (&key
    ((:req __req) 0)
    )
   (send-super :init)
   (setq _req (round __req))
   self)
  (:req
   (&optional __req)
   (if __req (setq _req __req)) _req)
  (:serialization-length
   ()
   (+
    ;; int8 _req
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _req
       (write-byte _req s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _req
     (setq _req (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _req 127) (setq _req (- _req 256)))
   ;;
   self)
  )

(defclass a1_slam::GtsamResultsResponse
  :super ros::object
  :slots (_str ))

(defmethod a1_slam::GtsamResultsResponse
  (:init
   (&key
    ((:str __str) "")
    )
   (send-super :init)
   (setq _str (string __str))
   self)
  (:str
   (&optional __str)
   (if __str (setq _str __str)) _str)
  (:serialization-length
   ()
   (+
    ;; string _str
    4 (length _str)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _str
       (write-long (length _str) s) (princ _str s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _str
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _str (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass a1_slam::GtsamResults
  :super ros::object
  :slots ())

(setf (get a1_slam::GtsamResults :md5sum-) "7189306d182a38b37d781470320bb653")
(setf (get a1_slam::GtsamResults :datatype-) "a1_slam/GtsamResults")
(setf (get a1_slam::GtsamResults :request) a1_slam::GtsamResultsRequest)
(setf (get a1_slam::GtsamResults :response) a1_slam::GtsamResultsResponse)

(defmethod a1_slam::GtsamResultsRequest
  (:response () (instance a1_slam::GtsamResultsResponse :init)))

(setf (get a1_slam::GtsamResultsRequest :md5sum-) "7189306d182a38b37d781470320bb653")
(setf (get a1_slam::GtsamResultsRequest :datatype-) "a1_slam/GtsamResultsRequest")
(setf (get a1_slam::GtsamResultsRequest :definition-)
      "int8 req
---
string str
")

(setf (get a1_slam::GtsamResultsResponse :md5sum-) "7189306d182a38b37d781470320bb653")
(setf (get a1_slam::GtsamResultsResponse :datatype-) "a1_slam/GtsamResultsResponse")
(setf (get a1_slam::GtsamResultsResponse :definition-)
      "int8 req
---
string str
")



(provide :a1_slam/GtsamResults "7189306d182a38b37d781470320bb653")


