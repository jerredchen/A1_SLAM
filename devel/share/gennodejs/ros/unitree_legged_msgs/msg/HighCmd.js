// Auto-generated. Do not edit!

// (in-package unitree_legged_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let LED = require('./LED.js');

//-----------------------------------------------------------

class HighCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.levelFlag = null;
      this.commVersion = null;
      this.robotID = null;
      this.SN = null;
      this.bandWidth = null;
      this.mode = null;
      this.forwardSpeed = null;
      this.sideSpeed = null;
      this.rotateSpeed = null;
      this.bodyHeight = null;
      this.footRaiseHeight = null;
      this.yaw = null;
      this.pitch = null;
      this.roll = null;
      this.led = null;
      this.wirelessRemote = null;
      this.AppRemote = null;
      this.reserve = null;
      this.crc = null;
    }
    else {
      if (initObj.hasOwnProperty('levelFlag')) {
        this.levelFlag = initObj.levelFlag
      }
      else {
        this.levelFlag = 0;
      }
      if (initObj.hasOwnProperty('commVersion')) {
        this.commVersion = initObj.commVersion
      }
      else {
        this.commVersion = 0;
      }
      if (initObj.hasOwnProperty('robotID')) {
        this.robotID = initObj.robotID
      }
      else {
        this.robotID = 0;
      }
      if (initObj.hasOwnProperty('SN')) {
        this.SN = initObj.SN
      }
      else {
        this.SN = 0;
      }
      if (initObj.hasOwnProperty('bandWidth')) {
        this.bandWidth = initObj.bandWidth
      }
      else {
        this.bandWidth = 0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('forwardSpeed')) {
        this.forwardSpeed = initObj.forwardSpeed
      }
      else {
        this.forwardSpeed = 0.0;
      }
      if (initObj.hasOwnProperty('sideSpeed')) {
        this.sideSpeed = initObj.sideSpeed
      }
      else {
        this.sideSpeed = 0.0;
      }
      if (initObj.hasOwnProperty('rotateSpeed')) {
        this.rotateSpeed = initObj.rotateSpeed
      }
      else {
        this.rotateSpeed = 0.0;
      }
      if (initObj.hasOwnProperty('bodyHeight')) {
        this.bodyHeight = initObj.bodyHeight
      }
      else {
        this.bodyHeight = 0.0;
      }
      if (initObj.hasOwnProperty('footRaiseHeight')) {
        this.footRaiseHeight = initObj.footRaiseHeight
      }
      else {
        this.footRaiseHeight = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('led')) {
        this.led = initObj.led
      }
      else {
        this.led = new Array(4).fill(new LED());
      }
      if (initObj.hasOwnProperty('wirelessRemote')) {
        this.wirelessRemote = initObj.wirelessRemote
      }
      else {
        this.wirelessRemote = new Array(40).fill(0);
      }
      if (initObj.hasOwnProperty('AppRemote')) {
        this.AppRemote = initObj.AppRemote
      }
      else {
        this.AppRemote = new Array(40).fill(0);
      }
      if (initObj.hasOwnProperty('reserve')) {
        this.reserve = initObj.reserve
      }
      else {
        this.reserve = 0;
      }
      if (initObj.hasOwnProperty('crc')) {
        this.crc = initObj.crc
      }
      else {
        this.crc = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HighCmd
    // Serialize message field [levelFlag]
    bufferOffset = _serializer.uint8(obj.levelFlag, buffer, bufferOffset);
    // Serialize message field [commVersion]
    bufferOffset = _serializer.uint16(obj.commVersion, buffer, bufferOffset);
    // Serialize message field [robotID]
    bufferOffset = _serializer.uint16(obj.robotID, buffer, bufferOffset);
    // Serialize message field [SN]
    bufferOffset = _serializer.uint32(obj.SN, buffer, bufferOffset);
    // Serialize message field [bandWidth]
    bufferOffset = _serializer.uint8(obj.bandWidth, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [forwardSpeed]
    bufferOffset = _serializer.float32(obj.forwardSpeed, buffer, bufferOffset);
    // Serialize message field [sideSpeed]
    bufferOffset = _serializer.float32(obj.sideSpeed, buffer, bufferOffset);
    // Serialize message field [rotateSpeed]
    bufferOffset = _serializer.float32(obj.rotateSpeed, buffer, bufferOffset);
    // Serialize message field [bodyHeight]
    bufferOffset = _serializer.float32(obj.bodyHeight, buffer, bufferOffset);
    // Serialize message field [footRaiseHeight]
    bufferOffset = _serializer.float32(obj.footRaiseHeight, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float32(obj.pitch, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float32(obj.roll, buffer, bufferOffset);
    // Check that the constant length array field [led] has the right length
    if (obj.led.length !== 4) {
      throw new Error('Unable to serialize array field led - length must be 4')
    }
    // Serialize message field [led]
    obj.led.forEach((val) => {
      bufferOffset = LED.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [wirelessRemote] has the right length
    if (obj.wirelessRemote.length !== 40) {
      throw new Error('Unable to serialize array field wirelessRemote - length must be 40')
    }
    // Serialize message field [wirelessRemote]
    bufferOffset = _arraySerializer.uint8(obj.wirelessRemote, buffer, bufferOffset, 40);
    // Check that the constant length array field [AppRemote] has the right length
    if (obj.AppRemote.length !== 40) {
      throw new Error('Unable to serialize array field AppRemote - length must be 40')
    }
    // Serialize message field [AppRemote]
    bufferOffset = _arraySerializer.uint8(obj.AppRemote, buffer, bufferOffset, 40);
    // Serialize message field [reserve]
    bufferOffset = _serializer.uint32(obj.reserve, buffer, bufferOffset);
    // Serialize message field [crc]
    bufferOffset = _serializer.int32(obj.crc, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HighCmd
    let len;
    let data = new HighCmd(null);
    // Deserialize message field [levelFlag]
    data.levelFlag = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [commVersion]
    data.commVersion = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [robotID]
    data.robotID = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [SN]
    data.SN = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [bandWidth]
    data.bandWidth = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [forwardSpeed]
    data.forwardSpeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [sideSpeed]
    data.sideSpeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rotateSpeed]
    data.rotateSpeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bodyHeight]
    data.bodyHeight = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [footRaiseHeight]
    data.footRaiseHeight = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [led]
    len = 4;
    data.led = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.led[i] = LED.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [wirelessRemote]
    data.wirelessRemote = _arrayDeserializer.uint8(buffer, bufferOffset, 40)
    // Deserialize message field [AppRemote]
    data.AppRemote = _arrayDeserializer.uint8(buffer, bufferOffset, 40)
    // Deserialize message field [reserve]
    data.reserve = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [crc]
    data.crc = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 134;
  }

  static datatype() {
    // Returns string type for a message object
    return 'unitree_legged_msgs/HighCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1a655499a3f64905db59ceed65ca774a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 levelFlag
    uint16 commVersion              # Old version Aliengo does not have
    uint16 robotID                  # Old version Aliengo does not have
    uint32 SN                       # Old version Aliengo does not have
    uint8 bandWidth                 # Old version Aliengo does not have
    uint8 mode
    float32 forwardSpeed
    float32 sideSpeed
    float32 rotateSpeed	
    float32 bodyHeight
    float32 footRaiseHeight
    float32 yaw
    float32 pitch
    float32 roll
    LED[4] led
    uint8[40] wirelessRemote
    uint8[40] AppRemote             # Old version Aliengo does not have
    uint32 reserve                  # Old version Aliengo does not have
    int32 crc
    ================================================================================
    MSG: unitree_legged_msgs/LED
    uint8 r
    uint8 g
    uint8 b
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HighCmd(null);
    if (msg.levelFlag !== undefined) {
      resolved.levelFlag = msg.levelFlag;
    }
    else {
      resolved.levelFlag = 0
    }

    if (msg.commVersion !== undefined) {
      resolved.commVersion = msg.commVersion;
    }
    else {
      resolved.commVersion = 0
    }

    if (msg.robotID !== undefined) {
      resolved.robotID = msg.robotID;
    }
    else {
      resolved.robotID = 0
    }

    if (msg.SN !== undefined) {
      resolved.SN = msg.SN;
    }
    else {
      resolved.SN = 0
    }

    if (msg.bandWidth !== undefined) {
      resolved.bandWidth = msg.bandWidth;
    }
    else {
      resolved.bandWidth = 0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.forwardSpeed !== undefined) {
      resolved.forwardSpeed = msg.forwardSpeed;
    }
    else {
      resolved.forwardSpeed = 0.0
    }

    if (msg.sideSpeed !== undefined) {
      resolved.sideSpeed = msg.sideSpeed;
    }
    else {
      resolved.sideSpeed = 0.0
    }

    if (msg.rotateSpeed !== undefined) {
      resolved.rotateSpeed = msg.rotateSpeed;
    }
    else {
      resolved.rotateSpeed = 0.0
    }

    if (msg.bodyHeight !== undefined) {
      resolved.bodyHeight = msg.bodyHeight;
    }
    else {
      resolved.bodyHeight = 0.0
    }

    if (msg.footRaiseHeight !== undefined) {
      resolved.footRaiseHeight = msg.footRaiseHeight;
    }
    else {
      resolved.footRaiseHeight = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.led !== undefined) {
      resolved.led = new Array(4)
      for (let i = 0; i < resolved.led.length; ++i) {
        if (msg.led.length > i) {
          resolved.led[i] = LED.Resolve(msg.led[i]);
        }
        else {
          resolved.led[i] = new LED();
        }
      }
    }
    else {
      resolved.led = new Array(4).fill(new LED())
    }

    if (msg.wirelessRemote !== undefined) {
      resolved.wirelessRemote = msg.wirelessRemote;
    }
    else {
      resolved.wirelessRemote = new Array(40).fill(0)
    }

    if (msg.AppRemote !== undefined) {
      resolved.AppRemote = msg.AppRemote;
    }
    else {
      resolved.AppRemote = new Array(40).fill(0)
    }

    if (msg.reserve !== undefined) {
      resolved.reserve = msg.reserve;
    }
    else {
      resolved.reserve = 0
    }

    if (msg.crc !== undefined) {
      resolved.crc = msg.crc;
    }
    else {
      resolved.crc = 0
    }

    return resolved;
    }
};

module.exports = HighCmd;
