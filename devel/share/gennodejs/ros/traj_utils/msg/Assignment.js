// Auto-generated. Do not edit!

// (in-package traj_utils.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Assignment {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.assignment = null;
    }
    else {
      if (initObj.hasOwnProperty('assignment')) {
        this.assignment = initObj.assignment
      }
      else {
        this.assignment = new Array(10).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Assignment
    // Check that the constant length array field [assignment] has the right length
    if (obj.assignment.length !== 10) {
      throw new Error('Unable to serialize array field assignment - length must be 10')
    }
    // Serialize message field [assignment]
    bufferOffset = _arraySerializer.uint32(obj.assignment, buffer, bufferOffset, 10);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Assignment
    let len;
    let data = new Assignment(null);
    // Deserialize message field [assignment]
    data.assignment = _arrayDeserializer.uint32(buffer, bufferOffset, 10)
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_utils/Assignment';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fc09302ffe2353e20a7ca4e9c550e719';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32[10] assignment
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Assignment(null);
    if (msg.assignment !== undefined) {
      resolved.assignment = msg.assignment;
    }
    else {
      resolved.assignment = new Array(10).fill(0)
    }

    return resolved;
    }
};

module.exports = Assignment;
