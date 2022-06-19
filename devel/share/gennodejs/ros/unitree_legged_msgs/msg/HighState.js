// Auto-generated. Do not edit!

// (in-package unitree_legged_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let IMU = require('./IMU.js');
let Cartesian = require('./Cartesian.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class HighState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.levelFlag = null;
      this.commVersion = null;
      this.robotID = null;
      this.SN = null;
      this.bandWidth = null;
      this.mode = null;
      this.imu = null;
      this.forwardSpeed = null;
      this.sideSpeed = null;
      this.rotateSpeed = null;
      this.bodyHeight = null;
      this.updownSpeed = null;
      this.forwardPosition = null;
      this.sidePosition = null;
      this.footPosition2Body = null;
      this.footSpeed2Body = null;
      this.footForce = null;
      this.footForceEst = null;
      this.tick = null;
      this.wirelessRemote = null;
      this.reserve = null;
      this.crc = null;
      this.eeForce = null;
      this.jointP = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
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
      if (initObj.hasOwnProperty('imu')) {
        this.imu = initObj.imu
      }
      else {
        this.imu = new IMU();
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
      if (initObj.hasOwnProperty('updownSpeed')) {
        this.updownSpeed = initObj.updownSpeed
      }
      else {
        this.updownSpeed = 0.0;
      }
      if (initObj.hasOwnProperty('forwardPosition')) {
        this.forwardPosition = initObj.forwardPosition
      }
      else {
        this.forwardPosition = 0.0;
      }
      if (initObj.hasOwnProperty('sidePosition')) {
        this.sidePosition = initObj.sidePosition
      }
      else {
        this.sidePosition = 0.0;
      }
      if (initObj.hasOwnProperty('footPosition2Body')) {
        this.footPosition2Body = initObj.footPosition2Body
      }
      else {
        this.footPosition2Body = new Array(4).fill(new Cartesian());
      }
      if (initObj.hasOwnProperty('footSpeed2Body')) {
        this.footSpeed2Body = initObj.footSpeed2Body
      }
      else {
        this.footSpeed2Body = new Array(4).fill(new Cartesian());
      }
      if (initObj.hasOwnProperty('footForce')) {
        this.footForce = initObj.footForce
      }
      else {
        this.footForce = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('footForceEst')) {
        this.footForceEst = initObj.footForceEst
      }
      else {
        this.footForceEst = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('tick')) {
        this.tick = initObj.tick
      }
      else {
        this.tick = 0;
      }
      if (initObj.hasOwnProperty('wirelessRemote')) {
        this.wirelessRemote = initObj.wirelessRemote
      }
      else {
        this.wirelessRemote = new Array(40).fill(0);
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
      if (initObj.hasOwnProperty('eeForce')) {
        this.eeForce = initObj.eeForce
      }
      else {
        this.eeForce = new Array(4).fill(new Cartesian());
      }
      if (initObj.hasOwnProperty('jointP')) {
        this.jointP = initObj.jointP
      }
      else {
        this.jointP = new Array(12).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HighState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
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
    // Serialize message field [imu]
    bufferOffset = IMU.serialize(obj.imu, buffer, bufferOffset);
    // Serialize message field [forwardSpeed]
    bufferOffset = _serializer.float32(obj.forwardSpeed, buffer, bufferOffset);
    // Serialize message field [sideSpeed]
    bufferOffset = _serializer.float32(obj.sideSpeed, buffer, bufferOffset);
    // Serialize message field [rotateSpeed]
    bufferOffset = _serializer.float32(obj.rotateSpeed, buffer, bufferOffset);
    // Serialize message field [bodyHeight]
    bufferOffset = _serializer.float32(obj.bodyHeight, buffer, bufferOffset);
    // Serialize message field [updownSpeed]
    bufferOffset = _serializer.float32(obj.updownSpeed, buffer, bufferOffset);
    // Serialize message field [forwardPosition]
    bufferOffset = _serializer.float32(obj.forwardPosition, buffer, bufferOffset);
    // Serialize message field [sidePosition]
    bufferOffset = _serializer.float32(obj.sidePosition, buffer, bufferOffset);
    // Check that the constant length array field [footPosition2Body] has the right length
    if (obj.footPosition2Body.length !== 4) {
      throw new Error('Unable to serialize array field footPosition2Body - length must be 4')
    }
    // Serialize message field [footPosition2Body]
    obj.footPosition2Body.forEach((val) => {
      bufferOffset = Cartesian.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [footSpeed2Body] has the right length
    if (obj.footSpeed2Body.length !== 4) {
      throw new Error('Unable to serialize array field footSpeed2Body - length must be 4')
    }
    // Serialize message field [footSpeed2Body]
    obj.footSpeed2Body.forEach((val) => {
      bufferOffset = Cartesian.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [footForce] has the right length
    if (obj.footForce.length !== 4) {
      throw new Error('Unable to serialize array field footForce - length must be 4')
    }
    // Serialize message field [footForce]
    bufferOffset = _arraySerializer.int16(obj.footForce, buffer, bufferOffset, 4);
    // Check that the constant length array field [footForceEst] has the right length
    if (obj.footForceEst.length !== 4) {
      throw new Error('Unable to serialize array field footForceEst - length must be 4')
    }
    // Serialize message field [footForceEst]
    bufferOffset = _arraySerializer.int16(obj.footForceEst, buffer, bufferOffset, 4);
    // Serialize message field [tick]
    bufferOffset = _serializer.uint32(obj.tick, buffer, bufferOffset);
    // Check that the constant length array field [wirelessRemote] has the right length
    if (obj.wirelessRemote.length !== 40) {
      throw new Error('Unable to serialize array field wirelessRemote - length must be 40')
    }
    // Serialize message field [wirelessRemote]
    bufferOffset = _arraySerializer.uint8(obj.wirelessRemote, buffer, bufferOffset, 40);
    // Serialize message field [reserve]
    bufferOffset = _serializer.uint32(obj.reserve, buffer, bufferOffset);
    // Serialize message field [crc]
    bufferOffset = _serializer.uint32(obj.crc, buffer, bufferOffset);
    // Check that the constant length array field [eeForce] has the right length
    if (obj.eeForce.length !== 4) {
      throw new Error('Unable to serialize array field eeForce - length must be 4')
    }
    // Serialize message field [eeForce]
    obj.eeForce.forEach((val) => {
      bufferOffset = Cartesian.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [jointP] has the right length
    if (obj.jointP.length !== 12) {
      throw new Error('Unable to serialize array field jointP - length must be 12')
    }
    // Serialize message field [jointP]
    bufferOffset = _arraySerializer.float32(obj.jointP, buffer, bufferOffset, 12);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HighState
    let len;
    let data = new HighState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
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
    // Deserialize message field [imu]
    data.imu = IMU.deserialize(buffer, bufferOffset);
    // Deserialize message field [forwardSpeed]
    data.forwardSpeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [sideSpeed]
    data.sideSpeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rotateSpeed]
    data.rotateSpeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bodyHeight]
    data.bodyHeight = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [updownSpeed]
    data.updownSpeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [forwardPosition]
    data.forwardPosition = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [sidePosition]
    data.sidePosition = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [footPosition2Body]
    len = 4;
    data.footPosition2Body = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.footPosition2Body[i] = Cartesian.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [footSpeed2Body]
    len = 4;
    data.footSpeed2Body = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.footSpeed2Body[i] = Cartesian.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [footForce]
    data.footForce = _arrayDeserializer.int16(buffer, bufferOffset, 4)
    // Deserialize message field [footForceEst]
    data.footForceEst = _arrayDeserializer.int16(buffer, bufferOffset, 4)
    // Deserialize message field [tick]
    data.tick = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [wirelessRemote]
    data.wirelessRemote = _arrayDeserializer.uint8(buffer, bufferOffset, 40)
    // Deserialize message field [reserve]
    data.reserve = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [crc]
    data.crc = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [eeForce]
    len = 4;
    data.eeForce = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.eeForce[i] = Cartesian.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [jointP]
    data.jointP = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 340;
  }

  static datatype() {
    // Returns string type for a message object
    return 'unitree_legged_msgs/HighState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e509eef5597723d817fac8bddca34ac0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    uint8 levelFlag
    uint16 commVersion                  # Old version Aliengo does not have
    uint16 robotID                      # Old version Aliengo does not have
    uint32 SN                           # Old version Aliengo does not have
    uint8 bandWidth                     # Old version Aliengo does not have
    uint8 mode
    IMU imu
    float32 forwardSpeed
    float32 sideSpeed
    float32 rotateSpeed
    float32 bodyHeight
    float32 updownSpeed
    float32 forwardPosition       # (will be float type next version)   # Old version Aliengo is different
    float32 sidePosition          # (will be float type next version)   # Old version Aliengo is different
    Cartesian[4] footPosition2Body
    Cartesian[4] footSpeed2Body
    int16[4] footForce                  # Old version Aliengo is different
    int16[4] footForceEst               # Old version Aliengo does not have
    uint32 tick               
    uint8[40] wirelessRemote
    uint32 reserve                      # Old version Aliengo does not have
    uint32 crc
    
    # Under are not defined in SDK yet. # Old version Aliengo does not have
    Cartesian[4] eeForce            # It's a 1-DOF force in real robot, but 3-DOF is better for visualization.
    float32[12] jointP              # for visualization
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
    MSG: unitree_legged_msgs/IMU
    float32[4] quaternion
    float32[3] gyroscope
    float32[3] accelerometer
    int8 temperature
    ================================================================================
    MSG: unitree_legged_msgs/Cartesian
    float32 x
    float32 y
    float32 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HighState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

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

    if (msg.imu !== undefined) {
      resolved.imu = IMU.Resolve(msg.imu)
    }
    else {
      resolved.imu = new IMU()
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

    if (msg.updownSpeed !== undefined) {
      resolved.updownSpeed = msg.updownSpeed;
    }
    else {
      resolved.updownSpeed = 0.0
    }

    if (msg.forwardPosition !== undefined) {
      resolved.forwardPosition = msg.forwardPosition;
    }
    else {
      resolved.forwardPosition = 0.0
    }

    if (msg.sidePosition !== undefined) {
      resolved.sidePosition = msg.sidePosition;
    }
    else {
      resolved.sidePosition = 0.0
    }

    if (msg.footPosition2Body !== undefined) {
      resolved.footPosition2Body = new Array(4)
      for (let i = 0; i < resolved.footPosition2Body.length; ++i) {
        if (msg.footPosition2Body.length > i) {
          resolved.footPosition2Body[i] = Cartesian.Resolve(msg.footPosition2Body[i]);
        }
        else {
          resolved.footPosition2Body[i] = new Cartesian();
        }
      }
    }
    else {
      resolved.footPosition2Body = new Array(4).fill(new Cartesian())
    }

    if (msg.footSpeed2Body !== undefined) {
      resolved.footSpeed2Body = new Array(4)
      for (let i = 0; i < resolved.footSpeed2Body.length; ++i) {
        if (msg.footSpeed2Body.length > i) {
          resolved.footSpeed2Body[i] = Cartesian.Resolve(msg.footSpeed2Body[i]);
        }
        else {
          resolved.footSpeed2Body[i] = new Cartesian();
        }
      }
    }
    else {
      resolved.footSpeed2Body = new Array(4).fill(new Cartesian())
    }

    if (msg.footForce !== undefined) {
      resolved.footForce = msg.footForce;
    }
    else {
      resolved.footForce = new Array(4).fill(0)
    }

    if (msg.footForceEst !== undefined) {
      resolved.footForceEst = msg.footForceEst;
    }
    else {
      resolved.footForceEst = new Array(4).fill(0)
    }

    if (msg.tick !== undefined) {
      resolved.tick = msg.tick;
    }
    else {
      resolved.tick = 0
    }

    if (msg.wirelessRemote !== undefined) {
      resolved.wirelessRemote = msg.wirelessRemote;
    }
    else {
      resolved.wirelessRemote = new Array(40).fill(0)
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

    if (msg.eeForce !== undefined) {
      resolved.eeForce = new Array(4)
      for (let i = 0; i < resolved.eeForce.length; ++i) {
        if (msg.eeForce.length > i) {
          resolved.eeForce[i] = Cartesian.Resolve(msg.eeForce[i]);
        }
        else {
          resolved.eeForce[i] = new Cartesian();
        }
      }
    }
    else {
      resolved.eeForce = new Array(4).fill(new Cartesian())
    }

    if (msg.jointP !== undefined) {
      resolved.jointP = msg.jointP;
    }
    else {
      resolved.jointP = new Array(12).fill(0)
    }

    return resolved;
    }
};

module.exports = HighState;
