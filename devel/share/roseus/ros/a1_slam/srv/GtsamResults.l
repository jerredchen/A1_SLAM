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
  :slots (_factor_type _factor _key _init_estimate ))

(defmethod a1_slam::GtsamResultsRequest
  (:init
   (&key
    ((:factor_type __factor_type) "")
    ((:factor __factor) "")
    ((:key __key) 0)
    ((:init_estimate __init_estimate) "")
    )
   (send-super :init)
   (setq _factor_type (string __factor_type))
   (setq _factor (string __factor))
   (setq _key (round __key))
   (setq _init_estimate (string __init_estimate))
   self)
  (:factor_type
   (&optional __factor_type)
   (if __factor_type (setq _factor_type __factor_type)) _factor_type)
  (:factor
   (&optional __factor)
   (if __factor (setq _factor __factor)) _factor)
  (:key
   (&optional __key)
   (if __key (setq _key __key)) _key)
  (:init_estimate
   (&optional __init_estimate)
   (if __init_estimate (setq _init_estimate __init_estimate)) _init_estimate)
  (:serialization-length
   ()
   (+
    ;; string _factor_type
    4 (length _factor_type)
    ;; string _factor
    4 (length _factor)
    ;; int64 _key
    8
    ;; string _init_estimate
    4 (length _init_estimate)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _factor_type
       (write-long (length _factor_type) s) (princ _factor_type s)
     ;; string _factor
       (write-long (length _factor) s) (princ _factor s)
     ;; int64 _key
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _key (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _key) (= (length (_key . bv)) 2)) ;; bignum
              (write-long (ash (elt (_key . bv) 0) 0) s)
              (write-long (ash (elt (_key . bv) 1) -1) s))
             ((and (class _key) (= (length (_key . bv)) 1)) ;; big1
              (write-long (elt (_key . bv) 0) s)
              (write-long (if (>= _key 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _key s)(write-long (if (>= _key 0) 0 #xffffffff) s)))
     ;; string _init_estimate
       (write-long (length _init_estimate) s) (princ _init_estimate s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _factor_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _factor_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _factor
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _factor (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int64 _key
#+(or :alpha :irix6 :x86_64)
      (setf _key (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _key (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; string _init_estimate
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _init_estimate (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass a1_slam::GtsamResultsResponse
  :super ros::object
  :slots (_results ))

(defmethod a1_slam::GtsamResultsResponse
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

(defclass a1_slam::GtsamResults
  :super ros::object
  :slots ())

(setf (get a1_slam::GtsamResults :md5sum-) "0e35ec748ec070bc1eff3fd9921a52fc")
(setf (get a1_slam::GtsamResults :datatype-) "a1_slam/GtsamResults")
(setf (get a1_slam::GtsamResults :request) a1_slam::GtsamResultsRequest)
(setf (get a1_slam::GtsamResults :response) a1_slam::GtsamResultsResponse)

(defmethod a1_slam::GtsamResultsRequest
  (:response () (instance a1_slam::GtsamResultsResponse :init)))

(setf (get a1_slam::GtsamResultsRequest :md5sum-) "0e35ec748ec070bc1eff3fd9921a52fc")
(setf (get a1_slam::GtsamResultsRequest :datatype-) "a1_slam/GtsamResultsRequest")
(setf (get a1_slam::GtsamResultsRequest :definition-)
      "string factor_type
string factor
int64 key
string init_estimate
---
string results
")

(setf (get a1_slam::GtsamResultsResponse :md5sum-) "0e35ec748ec070bc1eff3fd9921a52fc")
(setf (get a1_slam::GtsamResultsResponse :datatype-) "a1_slam/GtsamResultsResponse")
(setf (get a1_slam::GtsamResultsResponse :definition-)
      "string factor_type
string factor
int64 key
string init_estimate
---
string results
")



(provide :a1_slam/GtsamResults "0e35ec748ec070bc1eff3fd9921a52fc")


