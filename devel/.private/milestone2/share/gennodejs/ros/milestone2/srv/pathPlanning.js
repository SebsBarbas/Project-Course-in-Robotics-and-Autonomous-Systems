// Auto-generated. Do not edit!

// (in-package milestone2.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let nav_msgs = _finder('nav_msgs');

//-----------------------------------------------------------

class pathPlanningRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x0 = null;
      this.y0 = null;
      this.yaw0 = null;
      this.targetSign = null;
    }
    else {
      if (initObj.hasOwnProperty('x0')) {
        this.x0 = initObj.x0
      }
      else {
        this.x0 = 0.0;
      }
      if (initObj.hasOwnProperty('y0')) {
        this.y0 = initObj.y0
      }
      else {
        this.y0 = 0.0;
      }
      if (initObj.hasOwnProperty('yaw0')) {
        this.yaw0 = initObj.yaw0
      }
      else {
        this.yaw0 = 0.0;
      }
      if (initObj.hasOwnProperty('targetSign')) {
        this.targetSign = initObj.targetSign
      }
      else {
        this.targetSign = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pathPlanningRequest
    // Serialize message field [x0]
    bufferOffset = _serializer.float32(obj.x0, buffer, bufferOffset);
    // Serialize message field [y0]
    bufferOffset = _serializer.float32(obj.y0, buffer, bufferOffset);
    // Serialize message field [yaw0]
    bufferOffset = _serializer.float32(obj.yaw0, buffer, bufferOffset);
    // Serialize message field [targetSign]
    bufferOffset = _serializer.string(obj.targetSign, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pathPlanningRequest
    let len;
    let data = new pathPlanningRequest(null);
    // Deserialize message field [x0]
    data.x0 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y0]
    data.y0 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw0]
    data.yaw0 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [targetSign]
    data.targetSign = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.targetSign.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'milestone2/pathPlanningRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4b9c7c668b24c524f238db36f3c23676';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 x0
    float32 y0
    float32 yaw0
    string targetSign
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new pathPlanningRequest(null);
    if (msg.x0 !== undefined) {
      resolved.x0 = msg.x0;
    }
    else {
      resolved.x0 = 0.0
    }

    if (msg.y0 !== undefined) {
      resolved.y0 = msg.y0;
    }
    else {
      resolved.y0 = 0.0
    }

    if (msg.yaw0 !== undefined) {
      resolved.yaw0 = msg.yaw0;
    }
    else {
      resolved.yaw0 = 0.0
    }

    if (msg.targetSign !== undefined) {
      resolved.targetSign = msg.targetSign;
    }
    else {
      resolved.targetSign = ''
    }

    return resolved;
    }
};

class pathPlanningResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.plannedPath = null;
    }
    else {
      if (initObj.hasOwnProperty('plannedPath')) {
        this.plannedPath = initObj.plannedPath
      }
      else {
        this.plannedPath = new nav_msgs.msg.Path();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pathPlanningResponse
    // Serialize message field [plannedPath]
    bufferOffset = nav_msgs.msg.Path.serialize(obj.plannedPath, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pathPlanningResponse
    let len;
    let data = new pathPlanningResponse(null);
    // Deserialize message field [plannedPath]
    data.plannedPath = nav_msgs.msg.Path.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += nav_msgs.msg.Path.getMessageSize(object.plannedPath);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'milestone2/pathPlanningResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2ed23ddc4e8131abb32610932f89b487';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new pathPlanningResponse(null);
    if (msg.plannedPath !== undefined) {
      resolved.plannedPath = nav_msgs.msg.Path.Resolve(msg.plannedPath)
    }
    else {
      resolved.plannedPath = new nav_msgs.msg.Path()
    }

    return resolved;
    }
};

module.exports = {
  Request: pathPlanningRequest,
  Response: pathPlanningResponse,
  md5sum() { return '60da1f4dc44e1c3e5e24bf52a981d0f9'; },
  datatype() { return 'milestone2/pathPlanning'; }
};
