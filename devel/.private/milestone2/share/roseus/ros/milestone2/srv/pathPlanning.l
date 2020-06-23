;; Auto-generated. Do not edit!


(when (boundp 'milestone2::pathPlanning)
  (if (not (find-package "MILESTONE2"))
    (make-package "MILESTONE2"))
  (shadow 'pathPlanning (find-package "MILESTONE2")))
(unless (find-package "MILESTONE2::PATHPLANNING")
  (make-package "MILESTONE2::PATHPLANNING"))
(unless (find-package "MILESTONE2::PATHPLANNINGREQUEST")
  (make-package "MILESTONE2::PATHPLANNINGREQUEST"))
(unless (find-package "MILESTONE2::PATHPLANNINGRESPONSE")
  (make-package "MILESTONE2::PATHPLANNINGRESPONSE"))

(in-package "ROS")



(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass milestone2::pathPlanningRequest
  :super ros::object
  :slots (_x0 _y0 _yaw0 _targetSign ))

(defmethod milestone2::pathPlanningRequest
  (:init
   (&key
    ((:x0 __x0) 0.0)
    ((:y0 __y0) 0.0)
    ((:yaw0 __yaw0) 0.0)
    ((:targetSign __targetSign) "")
    )
   (send-super :init)
   (setq _x0 (float __x0))
   (setq _y0 (float __y0))
   (setq _yaw0 (float __yaw0))
   (setq _targetSign (string __targetSign))
   self)
  (:x0
   (&optional __x0)
   (if __x0 (setq _x0 __x0)) _x0)
  (:y0
   (&optional __y0)
   (if __y0 (setq _y0 __y0)) _y0)
  (:yaw0
   (&optional __yaw0)
   (if __yaw0 (setq _yaw0 __yaw0)) _yaw0)
  (:targetSign
   (&optional __targetSign)
   (if __targetSign (setq _targetSign __targetSign)) _targetSign)
  (:serialization-length
   ()
   (+
    ;; float32 _x0
    4
    ;; float32 _y0
    4
    ;; float32 _yaw0
    4
    ;; string _targetSign
    4 (length _targetSign)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x0
       (sys::poke _x0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y0
       (sys::poke _y0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw0
       (sys::poke _yaw0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _targetSign
       (write-long (length _targetSign) s) (princ _targetSign s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x0
     (setq _x0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y0
     (setq _y0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw0
     (setq _yaw0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _targetSign
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _targetSign (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass milestone2::pathPlanningResponse
  :super ros::object
  :slots (_plannedPath ))

(defmethod milestone2::pathPlanningResponse
  (:init
   (&key
    ((:plannedPath __plannedPath) (instance nav_msgs::Path :init))
    )
   (send-super :init)
   (setq _plannedPath __plannedPath)
   self)
  (:plannedPath
   (&rest __plannedPath)
   (if (keywordp (car __plannedPath))
       (send* _plannedPath __plannedPath)
     (progn
       (if __plannedPath (setq _plannedPath (car __plannedPath)))
       _plannedPath)))
  (:serialization-length
   ()
   (+
    ;; nav_msgs/Path _plannedPath
    (send _plannedPath :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/Path _plannedPath
       (send _plannedPath :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/Path _plannedPath
     (send _plannedPath :deserialize buf ptr-) (incf ptr- (send _plannedPath :serialization-length))
   ;;
   self)
  )

(defclass milestone2::pathPlanning
  :super ros::object
  :slots ())

(setf (get milestone2::pathPlanning :md5sum-) "60da1f4dc44e1c3e5e24bf52a981d0f9")
(setf (get milestone2::pathPlanning :datatype-) "milestone2/pathPlanning")
(setf (get milestone2::pathPlanning :request) milestone2::pathPlanningRequest)
(setf (get milestone2::pathPlanning :response) milestone2::pathPlanningResponse)

(defmethod milestone2::pathPlanningRequest
  (:response () (instance milestone2::pathPlanningResponse :init)))

(setf (get milestone2::pathPlanningRequest :md5sum-) "60da1f4dc44e1c3e5e24bf52a981d0f9")
(setf (get milestone2::pathPlanningRequest :datatype-) "milestone2/pathPlanningRequest")
(setf (get milestone2::pathPlanningRequest :definition-)
      "float32 x0
float32 y0
float32 yaw0
string targetSign
---
nav_msgs/Path plannedPath

================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get milestone2::pathPlanningResponse :md5sum-) "60da1f4dc44e1c3e5e24bf52a981d0f9")
(setf (get milestone2::pathPlanningResponse :datatype-) "milestone2/pathPlanningResponse")
(setf (get milestone2::pathPlanningResponse :definition-)
      "float32 x0
float32 y0
float32 yaw0
string targetSign
---
nav_msgs/Path plannedPath

================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :milestone2/pathPlanning "60da1f4dc44e1c3e5e24bf52a981d0f9")


