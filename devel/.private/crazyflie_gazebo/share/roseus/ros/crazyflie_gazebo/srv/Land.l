;; Auto-generated. Do not edit!


(when (boundp 'crazyflie_gazebo::Land)
  (if (not (find-package "CRAZYFLIE_GAZEBO"))
    (make-package "CRAZYFLIE_GAZEBO"))
  (shadow 'Land (find-package "CRAZYFLIE_GAZEBO")))
(unless (find-package "CRAZYFLIE_GAZEBO::LAND")
  (make-package "CRAZYFLIE_GAZEBO::LAND"))
(unless (find-package "CRAZYFLIE_GAZEBO::LANDREQUEST")
  (make-package "CRAZYFLIE_GAZEBO::LANDREQUEST"))
(unless (find-package "CRAZYFLIE_GAZEBO::LANDRESPONSE")
  (make-package "CRAZYFLIE_GAZEBO::LANDRESPONSE"))

(in-package "ROS")





(defclass crazyflie_gazebo::LandRequest
  :super ros::object
  :slots (_groupMask _height _duration ))

(defmethod crazyflie_gazebo::LandRequest
  (:init
   (&key
    ((:groupMask __groupMask) 0)
    ((:height __height) 0.0)
    ((:duration __duration) (instance ros::time :init))
    )
   (send-super :init)
   (setq _groupMask (round __groupMask))
   (setq _height (float __height))
   (setq _duration __duration)
   self)
  (:groupMask
   (&optional __groupMask)
   (if __groupMask (setq _groupMask __groupMask)) _groupMask)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; uint8 _groupMask
    1
    ;; float32 _height
    4
    ;; duration _duration
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _groupMask
       (write-byte _groupMask s)
     ;; float32 _height
       (sys::poke _height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; duration _duration
       (write-long (send _duration :sec) s) (write-long (send _duration :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _groupMask
     (setq _groupMask (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _height
     (setq _height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; duration _duration
     (send _duration :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _duration :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass crazyflie_gazebo::LandResponse
  :super ros::object
  :slots ())

(defmethod crazyflie_gazebo::LandResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass crazyflie_gazebo::Land
  :super ros::object
  :slots ())

(setf (get crazyflie_gazebo::Land :md5sum-) "b665b6c83a196e4774268cc26329b159")
(setf (get crazyflie_gazebo::Land :datatype-) "crazyflie_gazebo/Land")
(setf (get crazyflie_gazebo::Land :request) crazyflie_gazebo::LandRequest)
(setf (get crazyflie_gazebo::Land :response) crazyflie_gazebo::LandResponse)

(defmethod crazyflie_gazebo::LandRequest
  (:response () (instance crazyflie_gazebo::LandResponse :init)))

(setf (get crazyflie_gazebo::LandRequest :md5sum-) "b665b6c83a196e4774268cc26329b159")
(setf (get crazyflie_gazebo::LandRequest :datatype-) "crazyflie_gazebo/LandRequest")
(setf (get crazyflie_gazebo::LandRequest :definition-)
      "uint8 groupMask
float32 height
duration duration
---

")

(setf (get crazyflie_gazebo::LandResponse :md5sum-) "b665b6c83a196e4774268cc26329b159")
(setf (get crazyflie_gazebo::LandResponse :datatype-) "crazyflie_gazebo/LandResponse")
(setf (get crazyflie_gazebo::LandResponse :definition-)
      "uint8 groupMask
float32 height
duration duration
---

")



(provide :crazyflie_gazebo/Land "b665b6c83a196e4774268cc26329b159")


