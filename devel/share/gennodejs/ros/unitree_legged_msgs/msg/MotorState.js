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

class MotorState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
      this.q = null;
      this.dq = null;
      this.ddq = null;
      this.tauEst = null;
      this.q_raw = null;
      this.dq_raw = null;
      this.ddq_raw = null;
      this.temperature = null;
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
      if (initObj.hasOwnProperty('ddq')) {
        this.ddq = initObj.ddq
      }
      else {
        this.ddq = 0.0;
      }
      if (initObj.hasOwnProperty('tauEst')) {
        this.tauEst = initObj.tauEst
      }
      else {
        this.tauEst = 0.0;
      }
      if (initObj.hasOwnProperty('q_raw')) {
        this.q_raw = initObj.q_raw
      }
      else {
        this.q_raw = 0.0;
      }
      if (initObj.hasOwnProperty('dq_raw')) {
        this.dq_raw = initObj.dq_raw
      }
      else {
        this.dq_raw = 0.0;
      }
      if (initObj.hasOwnProperty('ddq_raw')) {
        this.ddq_raw = initObj.ddq_raw
      }
      else {
        this.ddq_raw = 0.0;
      }
      if (initObj.hasOwnProperty('temperature')) {
        this.temperature = initObj.temperature
      }
      else {
        this.temperature = 0;
      }
      if (initObj.hasOwnProperty('reserve')) {
        this.reserve = initObj.reserve
      }
      else {
        this.reserve = new Array(2).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorState
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [q]
    bufferOffset = _serializer.float32(obj.q, buffer, bufferOffset);
    // Serialize message field [dq]
    bufferOffset = _serializer.float32(obj.dq, buffer, bufferOffset);
    // Serialize message field [ddq]
    bufferOffset = _serializer.float32(obj.ddq, buffer, bufferOffset);
    // Serialize message field [tauEst]
    bufferOffset = _serializer.float32(obj.tauEst, buffer, bufferOffset);
    // Serialize message field [q_raw]
    bufferOffset = _serializer.float32(obj.q_raw, buffer, bufferOffset);
    // Serialize message field [dq_raw]
    bufferOffset = _serializer.float32(obj.dq_raw, buffer, bufferOffset);
    // Serialize message field [ddq_raw]
    bufferOffset = _serializer.float32(obj.ddq_raw, buffer, bufferOffset);
    // Serialize message field [temperature]
    bufferOffset = _serializer.int8(obj.temperature, buffer, bufferOffset);
    // Check that the constant length array field [reserve] has the right length
    if (obj.reserve.length !== 2) {
      throw new Error('Unable to serialize array field reserve - length must be 2')
    }
    // Serialize message field [reserve]
    bufferOffset = _arraySerializer.uint32(obj.reserve, buffer, bufferOffset, 2);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorState
    let len;
    let data = new MotorState(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [q]
    data.q = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dq]
    data.dq = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ddq]
    data.ddq = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tauEst]
    data.tauEst = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [q_raw]
    data.q_raw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dq_raw]
    data.dq_raw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ddq_raw]
    data.ddq_raw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [temperature]
    data.temperature = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [reserve]
    data.reserve = _arrayDeserializer.uint32(buffer, bufferOffset, 2)
    return data;
  }

  static getMessageSize(object) {
    return 38;
  }

  static datatype() {
    // Returns string type for a message object
    return 'unitree_legged_msgs/MotorState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '94c55ee3b7852be2bd437b20ce12a254';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 mode           # motor current mode 
    float32 q            # motor current position（rad）
    float32 dq           # motor current speed（rad/s）
    float32 ddq          # motor current speed（rad/s）
    float32 tauEst       # current estimated output torque（N*m）
    float32 q_raw        # motor current position（rad）
    float32 dq_raw       # motor current speed（rad/s）
    float32 ddq_raw      # motor current speed（rad/s）
    int8 temperature     # motor temperature（slow conduction of temperature leads to lag）
    uint32[2] reserve
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorState(null);
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

    if (msg.ddq !== undefined) {
      resolved.ddq = msg.ddq;
    }
    else {
      resolved.ddq = 0.0
    }

    if (msg.tauEst !== undefined) {
      resolved.tauEst = msg.tauEst;
    }
    else {
      resolved.tauEst = 0.0
    }

    if (msg.q_raw !== undefined) {
      resolved.q_raw = msg.q_raw;
    }
    else {
      resolved.q_raw = 0.0
    }

    if (msg.dq_raw !== undefined) {
      resolved.dq_raw = msg.dq_raw;
    }
    else {
      resolved.dq_raw = 0.0
    }

    if (msg.ddq_raw !== undefined) {
      resolved.ddq_raw = msg.ddq_raw;
    }
    else {
      resolved.ddq_raw = 0.0
    }

    if (msg.temperature !== undefined) {
      resolved.temperature = msg.temperature;
    }
    else {
      resolved.temperature = 0
    }

    if (msg.reserve !== undefined) {
      resolved.reserve = msg.reserve;
    }
    else {
      resolved.reserve = new Array(2).fill(0)
    }

    return resolved;
    }
};

module.exports = MotorState;
