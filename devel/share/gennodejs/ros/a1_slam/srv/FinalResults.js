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

class FinalResultsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.req = null;
    }
    else {
      if (initObj.hasOwnProperty('req')) {
        this.req = initObj.req
      }
      else {
        this.req = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FinalResultsRequest
    // Serialize message field [req]
    bufferOffset = _serializer.int8(obj.req, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FinalResultsRequest
    let len;
    let data = new FinalResultsRequest(null);
    // Deserialize message field [req]
    data.req = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'a1_slam/FinalResultsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0813fe7539f367d34151c5a6cfd4dacd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 req
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FinalResultsRequest(null);
    if (msg.req !== undefined) {
      resolved.req = msg.req;
    }
    else {
      resolved.req = 0
    }

    return resolved;
    }
};

class FinalResultsResponse {
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
    // Serializes a message object of type FinalResultsResponse
    // Serialize message field [results]
    bufferOffset = _serializer.string(obj.results, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FinalResultsResponse
    let len;
    let data = new FinalResultsResponse(null);
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
    return 'a1_slam/FinalResultsResponse';
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
    const resolved = new FinalResultsResponse(null);
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
  Request: FinalResultsRequest,
  Response: FinalResultsResponse,
  md5sum() { return 'da3ecb06e58699211b37ea0e285d071a'; },
  datatype() { return 'a1_slam/FinalResults'; }
};
