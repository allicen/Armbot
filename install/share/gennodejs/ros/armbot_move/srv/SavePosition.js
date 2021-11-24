// Auto-generated. Do not edit!

// (in-package armbot_move.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SavePositionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.save = null;
    }
    else {
      if (initObj.hasOwnProperty('save')) {
        this.save = initObj.save
      }
      else {
        this.save = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SavePositionRequest
    // Serialize message field [save]
    bufferOffset = _serializer.string(obj.save, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SavePositionRequest
    let len;
    let data = new SavePositionRequest(null);
    // Deserialize message field [save]
    data.save = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.save.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'armbot_move/SavePositionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e104100265146051383c5a794b30ab04';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string save
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SavePositionRequest(null);
    if (msg.save !== undefined) {
      resolved.save = msg.save;
    }
    else {
      resolved.save = ''
    }

    return resolved;
    }
};

class SavePositionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SavePositionResponse
    // Serialize message field [result]
    bufferOffset = _serializer.string(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SavePositionResponse
    let len;
    let data = new SavePositionResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.result.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'armbot_move/SavePositionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c22f2a1ed8654a0b365f1bb3f7ff2c0f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string result
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SavePositionResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: SavePositionRequest,
  Response: SavePositionResponse,
  md5sum() { return '61b51a87ebe23475b436f7b122d27fe9'; },
  datatype() { return 'armbot_move/SavePosition'; }
};
