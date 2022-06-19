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

class IMU {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.quaternion = null;
      this.gyroscope = null;
      this.accelerometer = null;
      this.temperature = null;
    }
    else {
      if (initObj.hasOwnProperty('quaternion')) {
        this.quaternion = initObj.quaternion
      }
      else {
        this.quaternion = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('gyroscope')) {
        this.gyroscope = initObj.gyroscope
      }
      else {
        this.gyroscope = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('accelerometer')) {
        this.accelerometer = initObj.accelerometer
      }
      else {
        this.accelerometer = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('temperature')) {
        this.temperature = initObj.temperature
      }
      else {
        this.temperature = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IMU
    // Check that the constant length array field [quaternion] has the right length
    if (obj.quaternion.length !== 4) {
      throw new Error('Unable to serialize array field quaternion - length must be 4')
    }
    // Serialize message field [quaternion]
    bufferOffset = _arraySerializer.float32(obj.quaternion, buffer, bufferOffset, 4);
    // Check that the constant length array field [gyroscope] has the right length
    if (obj.gyroscope.length !== 3) {
      throw new Error('Unable to serialize array field gyroscope - length must be 3')
    }
    // Serialize message field [gyroscope]
    bufferOffset = _arraySerializer.float32(obj.gyroscope, buffer, bufferOffset, 3);
    // Check that the constant length array field [accelerometer] has the right length
    if (obj.accelerometer.length !== 3) {
      throw new Error('Unable to serialize array field accelerometer - length must be 3')
    }
    // Serialize message field [accelerometer]
    bufferOffset = _arraySerializer.float32(obj.accelerometer, buffer, bufferOffset, 3);
    // Serialize message field [temperature]
    bufferOffset = _serializer.int8(obj.temperature, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IMU
    let len;
    let data = new IMU(null);
    // Deserialize message field [quaternion]
    data.quaternion = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [gyroscope]
    data.gyroscope = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [accelerometer]
    data.accelerometer = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [temperature]
    data.temperature = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 41;
  }

  static datatype() {
    // Returns string type for a message object
    return 'unitree_legged_msgs/IMU';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dd4bb4e42aa2f15aa1fb1b6a3c3752cb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[4] quaternion
    float32[3] gyroscope
    float32[3] accelerometer
    int8 temperature
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IMU(null);
    if (msg.quaternion !== undefined) {
      resolved.quaternion = msg.quaternion;
    }
    else {
      resolved.quaternion = new Array(4).fill(0)
    }

    if (msg.gyroscope !== undefined) {
      resolved.gyroscope = msg.gyroscope;
    }
    else {
      resolved.gyroscope = new Array(3).fill(0)
    }

    if (msg.accelerometer !== undefined) {
      resolved.accelerometer = msg.accelerometer;
    }
    else {
      resolved.accelerometer = new Array(3).fill(0)
    }

    if (msg.temperature !== undefined) {
      resolved.temperature = msg.temperature;
    }
    else {
      resolved.temperature = 0
    }

    return resolved;
    }
};

module.exports = IMU;
