// Auto-generated. Do not edit!

// (in-package simple_ugv_sim.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class UGVState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.x = null;
      this.y = null;
      this.z = null;
      this.yaw = null;
      this.v_linear = null;
      this.v_angular = null;
      this.dir_x = null;
      this.dir_y = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('v_linear')) {
        this.v_linear = initObj.v_linear
      }
      else {
        this.v_linear = 0.0;
      }
      if (initObj.hasOwnProperty('v_angular')) {
        this.v_angular = initObj.v_angular
      }
      else {
        this.v_angular = 0.0;
      }
      if (initObj.hasOwnProperty('dir_x')) {
        this.dir_x = initObj.dir_x
      }
      else {
        this.dir_x = 0.0;
      }
      if (initObj.hasOwnProperty('dir_y')) {
        this.dir_y = initObj.dir_y
      }
      else {
        this.dir_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UGVState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float64(obj.z, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float64(obj.yaw, buffer, bufferOffset);
    // Serialize message field [v_linear]
    bufferOffset = _serializer.float64(obj.v_linear, buffer, bufferOffset);
    // Serialize message field [v_angular]
    bufferOffset = _serializer.float64(obj.v_angular, buffer, bufferOffset);
    // Serialize message field [dir_x]
    bufferOffset = _serializer.float64(obj.dir_x, buffer, bufferOffset);
    // Serialize message field [dir_y]
    bufferOffset = _serializer.float64(obj.dir_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UGVState
    let len;
    let data = new UGVState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [v_linear]
    data.v_linear = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [v_angular]
    data.v_angular = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [dir_x]
    data.dir_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [dir_y]
    data.dir_y = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'simple_ugv_sim/UGVState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5aa9d6604c104759394b83817d0df5c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    # 位置信息
    float64 x
    float64 y
    float64 z
    float64 yaw
    
    # 速度信息
    float64 v_linear
    float64 v_angular
    
    # 方向向量
    float64 dir_x
    float64 dir_y
    
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
    const resolved = new UGVState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.v_linear !== undefined) {
      resolved.v_linear = msg.v_linear;
    }
    else {
      resolved.v_linear = 0.0
    }

    if (msg.v_angular !== undefined) {
      resolved.v_angular = msg.v_angular;
    }
    else {
      resolved.v_angular = 0.0
    }

    if (msg.dir_x !== undefined) {
      resolved.dir_x = msg.dir_x;
    }
    else {
      resolved.dir_x = 0.0
    }

    if (msg.dir_y !== undefined) {
      resolved.dir_y = msg.dir_y;
    }
    else {
      resolved.dir_y = 0.0
    }

    return resolved;
    }
};

module.exports = UGVState;
