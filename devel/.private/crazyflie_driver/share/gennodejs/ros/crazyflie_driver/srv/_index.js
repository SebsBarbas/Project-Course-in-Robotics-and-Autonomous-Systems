
"use strict";

let Land = require('./Land.js')
let Takeoff = require('./Takeoff.js')
let sendPacket = require('./sendPacket.js')
let GoTo = require('./GoTo.js')
let SetGroupMask = require('./SetGroupMask.js')
let Stop = require('./Stop.js')
let UploadTrajectory = require('./UploadTrajectory.js')
let UpdateParams = require('./UpdateParams.js')
let StartTrajectory = require('./StartTrajectory.js')
let RemoveCrazyflie = require('./RemoveCrazyflie.js')
let AddCrazyflie = require('./AddCrazyflie.js')

module.exports = {
  Land: Land,
  Takeoff: Takeoff,
  sendPacket: sendPacket,
  GoTo: GoTo,
  SetGroupMask: SetGroupMask,
  Stop: Stop,
  UploadTrajectory: UploadTrajectory,
  UpdateParams: UpdateParams,
  StartTrajectory: StartTrajectory,
  RemoveCrazyflie: RemoveCrazyflie,
  AddCrazyflie: AddCrazyflie,
};
