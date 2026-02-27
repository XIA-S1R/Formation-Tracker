// Auto-generated. Do not edit!

// (in-package object_detection_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CarPosition {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.xmean = null;
      this.ymean = null;
      this.zmean = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('xmean')) {
        this.xmean = initObj.xmean
      }
      else {
        this.xmean = 0.0;
      }
      if (initObj.hasOwnProperty('ymean')) {
        this.ymean = initObj.ymean
      }
      else {
        this.ymean = 0.0;
      }
      if (initObj.hasOwnProperty('zmean')) {
        this.zmean = initObj.zmean
      }
      else {
        this.zmean = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CarPosition
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [xmean]
    bufferOffset = _serializer.float64(obj.xmean, buffer, bufferOffset);
    // Serialize message field [ymean]
    bufferOffset = _serializer.float64(obj.ymean, buffer, bufferOffset);
    // Serialize message field [zmean]
    bufferOffset = _serializer.float64(obj.zmean, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CarPosition
    let len;
    let data = new CarPosition(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [xmean]
    data.xmean = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ymean]
    data.ymean = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [zmean]
    data.zmean = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_detection_msgs/CarPosition';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '88b1fb8816b2426364b93b43e1f32170';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 xmean
    float64 ymean
    float64 zmean
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CarPosition(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.xmean !== undefined) {
      resolved.xmean = msg.xmean;
    }
    else {
      resolved.xmean = 0.0
    }

    if (msg.ymean !== undefined) {
      resolved.ymean = msg.ymean;
    }
    else {
      resolved.ymean = 0.0
    }

    if (msg.zmean !== undefined) {
      resolved.zmean = msg.zmean;
    }
    else {
      resolved.zmean = 0.0
    }

    return resolved;
    }
};

module.exports = CarPosition;
