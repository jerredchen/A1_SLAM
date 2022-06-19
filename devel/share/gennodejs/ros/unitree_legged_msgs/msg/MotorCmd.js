// Auto-generated. Do not edit!

// (in-package unitree_legged_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MotorCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
      this.q = null;
      this.dq = null;
      this.tau = null;
      this.Kp = null;
      this.Kd = null;
      this.reserve = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('q')) {
        this.q = initObj.q
      }
      else {
        this.q = 0.0;
      }
      if (initObj.hasOwnProperty('dq')) {
        this.dq = initObj.dq
      }
      else {
        this.dq = 0.0;
      }
      if (initObj.hasOwnProperty('tau')) {
        this.tau = initObj.tau
      }
      else {
        this.tau = 0.0;
      }
      if (initObj.hasOwnProperty('Kp')) {
        this.Kp = initObj.Kp
      }
      else {
        this.Kp = 0.0;
      }
      if (initObj.hasOwnProperty('Kd')) {
        this.Kd = initObj.Kd
      }
      else {
        this.Kd = 0.0;
      }
      if (initObj.hasOwnProperty('reserve')) {
        this.reserve = initObj.reserve
      }
      else {
        this.reserve = new Array(3).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorCmd
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [q]
    bufferOffset = _serializer.float32(obj.q, buffer, bufferOffset);
    // Serialize message field [dq]
    bufferOffset = _serializer.float32(obj.dq, buffer, bufferOffset);
    // Serialize message field [tau]
    bufferOffset = _serializer.float32(obj.tau, buffer, bufferOffset);
    // Serialize message field [Kp]
    bufferOffset = _serializer.float32(obj.Kp, buffer, bufferOffset);
    // Serialize message field [Kd]
    bufferOffset = _serializer.float32(obj.Kd, buffer, bufferOffset);
    // Check that the constant length array field [reserve] has the right length
    if (obj.reserve.length !== 3) {
      throw new Error('Unable to serialize array field reserve - length must be 3')
    }
    // Serialize message field [reserve]
    bufferOffset = _arraySerializer.uint32(obj.reserve, buffer, bufferOffset, 3);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorCmd
    let len;
    let data = new MotorCmd(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [q]
    data.q = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dq]
    data.dq = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tau]
    data.tau = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Kp]
    data.Kp = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Kd]
    data.Kd = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [reserve]
    data.reserve = _arrayDeserializer.uint32(buffer, bufferOffset, 3)
    return data;
  }

  static getMessageSize(object) {
    return 33;
  }

  static datatype() {
    // Returns string type for a message object
    return 'unitree_legged_msgs/MotorCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bbb3b7d91319c3a1b99055f0149ba221';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 mode           # motor target mode
    float32 q            # motor target position
    float32 dq           # motor target velocity
    float32 tau          # motor target torque
    float32 Kp           # motor spring stiffness coefficient
    float32 Kd           # motor damper coefficient
    uint32[3] reserve    # motor target torque
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorCmd(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.q !== undefined) {
      resolved.q = msg.q;
    }
    else {
      resolved.q = 0.0
    }

    if (msg.dq !== undefined) {
      resolved.dq = msg.dq;
    }
    else {
      resolved.dq = 0.0
    }

    if (msg.tau !== undefined) {
      resolved.tau = msg.tau;
    }
    else {
      resolved.tau = 0.0
    }

    if (msg.Kp !== undefined) {
      resolved.Kp = msg.Kp;
    }
    else {
      resolved.Kp = 0.0
    }

    if (msg.Kd !== undefined) {
      resolved.Kd = msg.Kd;
    }
    else {
      resolved.Kd = 0.0
    }

    if (msg.reserve !== undefined) {
      resolved.reserve = msg.reserve;
    }
    else {
      resolved.reserve = new Array(3).fill(0)
    }

    return resolved;
    }
};

module.exports = MotorCmd;
