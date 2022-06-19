// Auto-generated. Do not edit!

// (in-package a1_slam.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GtsamResultsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.factor_type = null;
      this.factor = null;
      this.key = null;
      this.init_estimate = null;
    }
    else {
      if (initObj.hasOwnProperty('factor_type')) {
        this.factor_type = initObj.factor_type
      }
      else {
        this.factor_type = '';
      }
      if (initObj.hasOwnProperty('factor')) {
        this.factor = initObj.factor
      }
      else {
        this.factor = '';
      }
      if (initObj.hasOwnProperty('key')) {
        this.key = initObj.key
      }
      else {
        this.key = 0;
      }
      if (initObj.hasOwnProperty('init_estimate')) {
        this.init_estimate = initObj.init_estimate
      }
      else {
        this.init_estimate = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GtsamResultsRequest
    // Serialize message field [factor_type]
    bufferOffset = _serializer.string(obj.factor_type, buffer, bufferOffset);
    // Serialize message field [factor]
    bufferOffset = _serializer.string(obj.factor, buffer, bufferOffset);
    // Serialize message field [key]
    bufferOffset = _serializer.int64(obj.key, buffer, bufferOffset);
    // Serialize message field [init_estimate]
    bufferOffset = _serializer.string(obj.init_estimate, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GtsamResultsRequest
    let len;
    let data = new GtsamResultsRequest(null);
    // Deserialize message field [factor_type]
    data.factor_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [factor]
    data.factor = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [key]
    data.key = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [init_estimate]
    data.init_estimate = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.factor_type);
    length += _getByteLength(object.factor);
    length += _getByteLength(object.init_estimate);
    return length + 20;
  }

  static datatype() {
    // Returns string type for a service object
    return 'a1_slam/GtsamResultsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0daaec3a4a618092d7b420d174078c0f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string factor_type
    string factor
    int64 key
    string init_estimate
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GtsamResultsRequest(null);
    if (msg.factor_type !== undefined) {
      resolved.factor_type = msg.factor_type;
    }
    else {
      resolved.factor_type = ''
    }

    if (msg.factor !== undefined) {
      resolved.factor = msg.factor;
    }
    else {
      resolved.factor = ''
    }

    if (msg.key !== undefined) {
      resolved.key = msg.key;
    }
    else {
      resolved.key = 0
    }

    if (msg.init_estimate !== undefined) {
      resolved.init_estimate = msg.init_estimate;
    }
    else {
      resolved.init_estimate = ''
    }

    return resolved;
    }
};

class GtsamResultsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.results = null;
    }
    else {
      if (initObj.hasOwnProperty('results')) {
        this.results = initObj.results
      }
      else {
        this.results = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GtsamResultsResponse
    // Serialize message field [results]
    bufferOffset = _serializer.string(obj.results, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GtsamResultsResponse
    let len;
    let data = new GtsamResultsResponse(null);
    // Deserialize message field [results]
    data.results = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.results);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'a1_slam/GtsamResultsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '050d718085d2969bac3160f48f51460c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string results
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GtsamResultsResponse(null);
    if (msg.results !== undefined) {
      resolved.results = msg.results;
    }
    else {
      resolved.results = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: GtsamResultsRequest,
  Response: GtsamResultsResponse,
  md5sum() { return '0e35ec748ec070bc1eff3fd9921a52fc'; },
  datatype() { return 'a1_slam/GtsamResults'; }
};
