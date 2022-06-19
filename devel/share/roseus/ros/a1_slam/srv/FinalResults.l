;; Auto-generated. Do not edit!


(when (boundp 'a1_slam::FinalResults)
  (if (not (find-package "A1_SLAM"))
    (make-package "A1_SLAM"))
  (shadow 'FinalResults (find-package "A1_SLAM")))
(unless (find-package "A1_SLAM::FINALRESULTS")
  (make-package "A1_SLAM::FINALRESULTS"))
(unless (find-package "A1_SLAM::FINALRESULTSREQUEST")
  (make-package "A1_SLAM::FINALRESULTSREQUEST"))
(unless (find-package "A1_SLAM::FINALRESULTSRESPONSE")
  (make-package "A1_SLAM::FINALRESULTSRESPONSE"))

(in-package "ROS")





(defclass a1_slam::FinalResultsRequest
  :super ros::object
  :slots (_req ))

(defmethod a1_slam::FinalResultsRequest
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

(defclass a1_slam::FinalResultsResponse
  :super ros::object
  :slots (_results ))

(defmethod a1_slam::FinalResultsResponse
  (:init
   (&key
    ((:results __results) "")
    )
   (send-super :init)
   (setq _results (string __results))
   self)
  (:results
   (&optional __results)
   (if __results (setq _results __results)) _results)
  (:serialization-length
   ()
   (+
    ;; string _results
    4 (length _results)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _results
       (write-long (length _results) s) (princ _results s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _results
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _results (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass a1_slam::FinalResults
  :super ros::object
  :slots ())

(setf (get a1_slam::FinalResults :md5sum-) "da3ecb06e58699211b37ea0e285d071a")
(setf (get a1_slam::FinalResults :datatype-) "a1_slam/FinalResults")
(setf (get a1_slam::FinalResults :request) a1_slam::FinalResultsRequest)
(setf (get a1_slam::FinalResults :response) a1_slam::FinalResultsResponse)

(defmethod a1_slam::FinalResultsRequest
  (:response () (instance a1_slam::FinalResultsResponse :init)))

(setf (get a1_slam::FinalResultsRequest :md5sum-) "da3ecb06e58699211b37ea0e285d071a")
(setf (get a1_slam::FinalResultsRequest :datatype-) "a1_slam/FinalResultsRequest")
(setf (get a1_slam::FinalResultsRequest :definition-)
      "int8 req
---
string results
")

(setf (get a1_slam::FinalResultsResponse :md5sum-) "da3ecb06e58699211b37ea0e285d071a")
(setf (get a1_slam::FinalResultsResponse :datatype-) "a1_slam/FinalResultsResponse")
(setf (get a1_slam::FinalResultsResponse :definition-)
      "int8 req
---
string results
")



(provide :a1_slam/FinalResults "da3ecb06e58699211b37ea0e285d071a")


