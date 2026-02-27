// Auto-generated. Do not edit!

// (in-package quadrotor_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let OccMap3d = require('./OccMap3d.js');
let nav_msgs = _finder('nav_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ReplanState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.state = null;
      this.iniState = null;
      this.target = null;
      this.occmap = null;
      this.path = null;
      this.replan_stamp = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('iniState')) {
        this.iniState = initObj.iniState
      }
      else {
        this.iniState = [];
      }
      if (initObj.hasOwnProperty('target')) {
        this.target = initObj.target
      }
      else {
        this.target = new nav_msgs.msg.Odometry();
      }
      if (initObj.hasOwnProperty('occmap')) {
        this.occmap = initObj.occmap
      }
      else {
        this.occmap = new OccMap3d();
      }
      if (initObj.hasOwnProperty('path')) {
        this.path = initObj.path
      }
      else {
        this.path = [];
      }
      if (initObj.hasOwnProperty('replan_stamp')) {
        this.replan_stamp = initObj.replan_stamp
      }
      else {
        this.replan_stamp = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReplanState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.int16(obj.state, buffer, bufferOffset);
    // Serialize message field [iniState]
    bufferOffset = _arraySerializer.float64(obj.iniState, buffer, bufferOffset, null);
    // Serialize message field [target]
    bufferOffset = nav_msgs.msg.Odometry.serialize(obj.target, buffer, bufferOffset);
    // Serialize message field [occmap]
    bufferOffset = OccMap3d.serialize(obj.occmap, buffer, bufferOffset);
    // Serialize message field [path]
    bufferOffset = _arraySerializer.float64(obj.path, buffer, bufferOffset, null);
    // Serialize message field [replan_stamp]
    bufferOffset = _serializer.time(obj.replan_stamp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReplanState
    let len;
    let data = new ReplanState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [iniState]
    data.iniState = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [target]
    data.target = nav_msgs.msg.Odometry.deserialize(buffer, bufferOffset);
    // Deserialize message field [occmap]
    data.occmap = OccMap3d.deserialize(buffer, bufferOffset);
    // Deserialize message field [path]
    data.path = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [replan_stamp]
    data.replan_stamp = _deserializer.time(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.iniState.length;
    length += nav_msgs.msg.Odometry.getMessageSize(object.target);
    length += OccMap3d.getMessageSize(object.occmap);
    length += 8 * object.path.length;
    return length + 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/ReplanState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '38714edcd5022172624012c3dbfdd606';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # all information for a replan
    
    Header header
    
    int16 state
    float64[] iniState
    nav_msgs/Odometry target
    quadrotor_msgs/OccMap3d occmap
    
    # other temporal variables
    float64[] path
    time replan_stamp
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
    MSG: nav_msgs/Odometry
    # This represents an estimate of a position and velocity in free space.  
    # The pose in this message should be specified in the coordinate frame given by header.frame_id.
    # The twist in this message should be specified in the coordinate frame given by the child_frame_id
    Header header
    string child_frame_id
    geometry_msgs/PoseWithCovariance pose
    geometry_msgs/TwistWithCovariance twist
    
    ================================================================================
    MSG: geometry_msgs/PoseWithCovariance
    # This represents a pose in free space with uncertainty.
    
    Pose pose
    
    # Row-major representation of the 6x6 covariance matrix
    # The orientation parameters use a fixed-axis representation.
    # In order, the parameters are:
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    float64[36] covariance
    
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
    
    ================================================================================
    MSG: geometry_msgs/TwistWithCovariance
    # This expresses velocity in free space with uncertainty.
    
    Twist twist
    
    # Row-major representation of the 6x6 covariance matrix
    # The orientation parameters use a fixed-axis representation.
    # In order, the parameters are:
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    float64[36] covariance
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: quadrotor_msgs/OccMap3d
    Header header
    
    float32 resolution
    int16 size_x
    int16 size_y
    int16 size_z
    int16 offset_x
    int16 offset_y
    int16 offset_z
    
    int8[] data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ReplanState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.iniState !== undefined) {
      resolved.iniState = msg.iniState;
    }
    else {
      resolved.iniState = []
    }

    if (msg.target !== undefined) {
      resolved.target = nav_msgs.msg.Odometry.Resolve(msg.target)
    }
    else {
      resolved.target = new nav_msgs.msg.Odometry()
    }

    if (msg.occmap !== undefined) {
      resolved.occmap = OccMap3d.Resolve(msg.occmap)
    }
    else {
      resolved.occmap = new OccMap3d()
    }

    if (msg.path !== undefined) {
      resolved.path = msg.path;
    }
    else {
      resolved.path = []
    }

    if (msg.replan_stamp !== undefined) {
      resolved.replan_stamp = msg.replan_stamp;
    }
    else {
      resolved.replan_stamp = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

module.exports = ReplanState;
