// Press the refresh button over there ---->>>
// Importing graphing library please ignore :)
import { uPlot } from "./uplot";

// This version has gain scheduling in the PID loop. Change it if u want something different.

// PLAY AROUND WITH THESE VALUES BELOW!
// Default settings may not be perfect for
// your purposes. Take some time to change
// the variables below and see how the rocket
// reacts!

// Simulation total time
const simTimeSeconds = 10; //seconds

// Set this to true to control x-position using PID
// Set to false to control ONLY rotation.
const controlPosition = false;
// The horizontal position setpoint
const x_setpoint = 9; // m
// PID Gains for position PID
const kp_pos = .15;
const ki_pos = 0.1;
const kd_pos = 0.05;
// Range of rocket angles allowed by PID
const rocketAngleRange = 30; // +/- deg

// What angle should the rocket start at?
const initialRocketAngle = 0; // deg

// If controlPosition == false
// This will set the rocket setpoint
let rocketAngleSetpoint = 0; // deg

// PID GAINS for rotation PID
// My E12 Gimbal test use: 0.4, 0.1, 0.1
const kp_tvc = .55;
const ki_tvc = 0.4;
const kd_tvc = 0.2;

// Info about your rocket.
// Ideally you'd measure these in various ways

//distance between center of mass and TVC pivot
//doesn't affect rocket dimensions
const tvc_to_COM = 0.152; // meters

// More info on how to measure this here:
// https://youtu.be/nwgd1CV__rs?t=93
// const momentOfInertia = 0.00804; //  kg*m^2
const momentOfInertia = 0.015533; //  kg*m^2

// Mass of rocket used in height calculations
const mass = 0.666; // kg

//max angle range of your TVC plus and minus
// So 5 deg would be +/- 5 deg
const tvcAngleRange = 5; // deg

// Ratio of servo movement / gimbal movement
const servoLinkageRatio = 2;

// Measure how many degrees per second your gimbal can move
// 62.5 deg/sec measured from my servo
const tvc_deg_per_second_max = 257.5; // deg/sec

// THRUST PROPERTIES
//Set to false if you want to use constant thrust
const useThrustCurve = true;
// Only applicable if useThrustCurve is false
const constantThrust = 12; // newtons
// default 2.64
//Thrust Data for Estes F15 Motor
// Feel free to add in your own thrust data
// Data is of the form [seconds, newtons]
//Flame rectangle changes height in proportion to thrust
const C6EstesThrust = [
  [0, 0],
  [0.014, 0.633],
  [0.026, 1.533],
  [0.067, 2.726],
  [0.099, 5.136],
  [0.15, 9.103],
  [0.183, 11.465],
  [0.207, 11.635],
  [0.219, 11.391],
  [0.262, 6.377],
  [0.333, 5.014],
  [0.349, 5.209],
  [0.392, 4.722],
  [0.475, 4.771],
  [0.653, 4.746],
  [0.913, 4.673],
  [1.366, 4.625],
  [1.607, 4.625],
  [1.745, 4.868],
  [1.978, 4.795],
  [2.023, 0.828],
  [2.024, 0.0]
];

const E12EstesThrust = [
  [0.0, 0.0],
  [0.052, 5.045],
  [0.096, 9.91],
  [0.196, 24.144],
  [0.251, 31.351],
  [0.287, 32.973],
  [0.3, 29.91],
  [0.344, 17.117],
  [0.37, 14.414],
  [0.4, 12.973],
  [0.5, 11.712],
  [0.6, 11.171],
  [0.7, 10.631],
  [0.8, 10.09],
  [0.9, 9.73],
  [1.0, 9.55],
  [1.101, 9.91],
  [1.2, 9.55],
  [1.3, 9.73],
  [1.4, 9.73],
  [1.5, 9.73],
  [1.6, 9.73],
  [1.7, 9.55],
  [1.8, 9.73],
  [1.9, 9.73],
  [2.0, 9.55],
  [2.1, 9.55],
  [2.2, 9.73],
  [2.3, 9.19],
  [2.375, 9.37],
  [2.4, 5.95],
  [2.44, 0.0]
];

const F15EstesThrust = [
  [0, 0],
  [0.063, 2.127],
  [0.118, 4.407],
  [0.158, 8.359],
  [0.228, 13.68],
  [0.34, 20.82],
  [0.386, 26.75],
  [0.425, 25.38],
  [0.481, 22.19],
  [0.583, 17.93],
  [0.883, 16.11],
  [1.191, 14.59],
  [1.364, 15.35],
  [1.569, 15.65],
  [1.727, 14.74],
  [2.0, 14.28],
  [2.39, 13.68],
  [2.68, 13.08],
  [2.96, 13.07],
  [3.25, 13.05],
  [3.35, 13.0],
  [3.39, 7.3],
  [3.4, 0.0]
];

const E9EstesThrust = [
  [0, 0],
  [0.046, 1.913],
  [0.235, 16.696],
  [0.273, 18.435],
  [0.326, 14.957],
  [0.38, 12.174],
  [0.44, 10.435],
  [0.835, 9.043],
  [1.093, 8.87],
  [1.496, 8.696],
  [1.997, 8.696],
  [2.498, 8.696],
  [3.014, 9.217],
  [3.037, 5.043],
  [3.067, 1.217],
  [3.09, 0.0]
];

const D12EstesThrust = [
  [0,0],
  [0.116, 9.369],
  [0.184, 17.275],
  [0.237, 24.258],
  [0.282, 29.73],
  [0.297, 27.01],
  [0.311, 22.589],
  [0.322, 17.99],
  [0.348, 14.126],
  [0.386, 12.099],
  [0.442, 10.808],
  [0.546, 9.876],
  [0.718, 9.306],
  [0.879, 9.105],
  [1.066, 8.901],
  [1.257, 8.698],
  [1.436, 8.31],
  [1.59, 8.294],
  [1.612, 4.613],
  [1.65, 0]
];

const F10ApogeeThrust = [
  [0, 0],
  [0.015, 28.22],
  [0.077, 26.082],
  [0.201, 24.934],
  [0.31, 22.806],
  [0.464, 20.183],
  [0.573, 17.886],
  [0.789, 16.075],
  [1.068, 13.946],
  [1.393, 12.63],
  [1.718, 11.155],
  [2.166, 9.844],
  [2.677, 9.515],
  [3.311, 9.187],
  [3.683, 8.859],
  [3.791, 9.679],
  [4.101, 9.679],
  [4.658, 9.515],
  [5.168, 9.023],
  [5.725, 9.023],
  [6.112, 8.531],
  [6.329, 8.859],
  [6.499, 7.546],
  [6.685, 5.742],
  [6.778, 4.921],
  [6.917, 2.625],
  [7.025, 1.312],
  [7.13, 0]
];

// Set this to the motor variable above
const thrustCurveData = D12EstesThrust;

//initial motor Mass. E12 is 62 grams F15 is 95 grams
const motorMassStart = 0.0371; //kg
const motorMassLossRatio = 0.0211; // how much Mass motor looses
const motorMassEnd = motorMassStart - motorMassStart * motorMassLossRatio;

// Air resistance
// Set drag coefficient to neglect drag
// Only affects vertical and horizontal results
const dragCoefficient = 0;
const dragArea = 0; // m ^ 2
const airDensity = 1.225; // kg/m^3

// Function that calculates drag force
const getDragForce = (velocity) => {
  return dragCoefficient * airDensity * dragArea * Math.pow(velocity, 2);
};

// Flight Computer Properties

// Servo Delay in seconds
// Measure this in real life by filming an LED
// turning on right before you command your servo
// to move. See how long that command takes.
const servoDelaySec = 0.075; // sec

// Servo linkages for TVC can flex a bit
// This variable affects the amplitude
// of random noise applied to the TVC angle
// to simulate tvc linkage flex effects
const tvcNoiseAmplitude = 0; // deg

// Seconds between each PID call.
const PIDLoopTime = 0.01111; // sec

// Use these to add noise and offset to the angle measurement
// of the rocket.
const angleSensorNoiseAmplitude = 0.1; // deg
const angleSensorBias = 0; //deg

// Initial values
// Only initial velocity will have a noticeable effect

const initialTVCAngle = 0; // deg
const initialAccel = 0; // rad/s^2
const initialVelocity = 0; // rad/s

// Torque offset and noise
// A crude way of simulating wind disturbances
const torqueNoiseAmplitude = 0; // Nm
let constantTorqueBias = 0; // N*m

// Used to calculate wind torque.
// This is special because it applies a sinusoidal value of torque
// to the rocket. Look at the last graph.
const windSinAmplitude = 0; // N*m
const windSinFrequency = 0; // hz

// Simulation stuff

// Time step in seconds
// The smaller the more accurate the sim. 0.001 is good
// If you do lower than 0.001, you'll need to turn off
// The infinite loop protection in code sandbox options
const deltaTSeconds = 0.0001; //seconds

//************************************************************************* */
// From here on out, there be dragons....
// But further down is the simulation function
// And the PID control function!
const maxDegPerTimestep = tvc_deg_per_second_max * deltaTSeconds;
const canvas = document.getElementById("simCanvas");
const ctx = canvas.getContext("2d");
const replayButton = document.getElementById("replay");

const servoAngleRange = servoLinkageRatio * tvcAngleRange;

// Set sizes of rocket and stuff
const rocketWidth = 40;
const rocketHeight = 130;
const tvcWidth = 15;
const tvcHeight = 30;
const flameWidth = 15;

const setCanvasSize = () => {
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight - 400;
};

setCanvasSize();

const centerX = canvas.width / 2 - rocketWidth / 2;
const centerY = canvas.height / 2 - rocketHeight / 2;

// Initialize data arrays
let sensedRocketAngle = [initialRocketAngle]; // deg
let tvcAngle = [initialTVCAngle]; // deg
let accel = [initialAccel]; // rad/s^2
let velocity = [initialVelocity]; // rad/s
const actualRocketAngle = [initialRocketAngle];
// Total amount of iterations
const simSteps = simTimeSeconds / deltaTSeconds;

// Interpolates thrust array linearly to get more datapoints
const interpolateThrust = (thrust) => {
  const interpThrust = [];
  for (let i = 0; i < thrust.length - 1; i += 1) {
    const timeDiff = thrust[i + 1][0] - thrust[i][0];
    const steps = Math.floor(timeDiff / deltaTSeconds);
    const startThrust = thrust[i][1];
    const endThrust = thrust[i + 1][1];
    for (let k = 0; k < steps; k += 1) {
      const ratio = k / steps;
      interpThrust.push(startThrust + ratio * (endThrust - startThrust));
    }
  }
  const stepsLeft = simSteps - interpThrust.length;

  for (let i = 0; i < stepsLeft; i += 1) {
    interpThrust.push(0);
  }
  return interpThrust;
};

// Checks if constant thrust or interpolated and fills array accordingly
let thrust = [];
if (useThrustCurve) {
  thrust = interpolateThrust(thrustCurveData);
} else {
  for (let i = 0; i < simSteps; i += 1) {
    thrust[i] = constantThrust;
  }
}

// Torque produced by wind. Not random noise but a sin wave.
const windTorque = [];

for (let i = 0; i < simSteps; i += 1) {
  windTorque[i] =
    windSinAmplitude *
    Math.sin(windSinFrequency * deltaTSeconds * i * 2 * Math.PI);
}

// Used so the rocket uses the shortest route between initial and setpoint
let setpoint =
  Math.abs(rocketAngleSetpoint - initialRocketAngle) > 180
    ? -(360 - rocketAngleSetpoint)
    : rocketAngleSetpoint; // deg

// *******************************************//

const initPID = (kp, ki, kd, range) => {
  let lastErr = 0;
  let firstStep = true;
  let ITerm = 0;
  const pErrs = [0];
  const dErrs = [0];
  const ITerms = [0, 0];

  return {
    pErrs,
    dErrs,
    ITerms,
    compute: (input, setpoint, angularVel) => {
      const error = setpoint - input;

      let newKp = kp;
      let newkD = kd;
      let newKi = ki;

      ITerm += newKi * error * simStepsBetweenPIDCalls * deltaTSeconds;
      if (ITerm > range) {
        ITerm = range;
      }
      if (ITerm < -range) {
        ITerm = -range;
      }

      if (firstStep === true) {
        lastErr = error;
        firstStep = false;
      }
      let dErr;
      if (angularVel) {
        dErr = angularVel / (simStepsBetweenPIDCalls * deltaTSeconds);
      } else {
        dErr = (error - lastErr) / (simStepsBetweenPIDCalls * deltaTSeconds);
      }

      const output = Math.floor(newKp * error + ITerm + newkD * dErr);
      lastErr = error;

      // Push the error terms to an array just for logging
      // You can ignore these when writing PID code
      // in C++
      pErrs.push(error);
      dErrs.push(dErr);
      ITerms.push(ITerm);

      if (-output > range) {
        return range;
      }
      if (-output < -range) {
        return -range;
      }
      return -output;
    }
  };
};

const initPIDPOM = (kp, ki, kd, range) => {
  let lastErr = 0;
  let firstStep = true;
  let ITerm = 0;
  const pErrs = [0];
  const dErrs = [0];
  const ITerms = [0, 0];
  let initInput;

  return {
    pErrs,
    dErrs,
    ITerms,
    compute: (input, setpoint) => {
      const error = setpoint - input;
      const dt = simStepsBetweenPIDCalls * deltaTSeconds;
      if (firstStep === true) {
        initInput = input;
        lastErr = error;
        firstStep = false;
      }

      ITerm += ki * error * dt;
      if (ITerm > range) {
        ITerm = range;
      }
      if (ITerm < -range) {
        ITerm = -range;
      }

      const dErr = (error - lastErr) / dt;
      const pErr = -kp * (input - initInput);
      const output = pErr + ITerm + kd * dErr;
      lastErr = error;

      // Push the error terms to an array just for logging
      // You can ignore these when writing PID code
      // in C++
      pErrs.push(pErr);
      dErrs.push(dErr);
      ITerms.push(ITerm);

      if (-output > range) {
        return range;
      }
      if (-output < -range) {
        return -range;
      }
      return -output;
    }
  };
};

// Helper variables
const RAD_TO_DEG = 180 / Math.PI;
const DEG_TO_RAD = Math.PI / 180;

// Limits TVC angle change speed
const limitTVCAngle = (newTVCAngle, prevTVCAngle) => {
  const tvcAngleSign = Math.sign(newTVCAngle - prevTVCAngle);
  const angleDelta = Math.abs(newTVCAngle - prevTVCAngle);

  if (angleDelta > maxDegPerTimestep) {
    const allowedAngle = prevTVCAngle + tvcAngleSign * maxDegPerTimestep;
    if (allowedAngle > tvcAngleRange) {
      return tvcAngleRange;
    }
    if (allowedAngle < -tvcAngleRange) {
      return tvcAngleRange;
    }
    return allowedAngle;
  } else {
    return newTVCAngle;
  }
};

// Creates noise in the torque imparted on the rocket.
const getRandomTorqueNoise = () => {
  return torqueNoiseAmplitude * Math.random() - torqueNoiseAmplitude / 2;
};

// Number of timesteps to wait between servo command being sent and
// servo actually actuating
const tvcStepWait = Math.ceil(servoDelaySec / deltaTSeconds);

// List of angle commands that the PID outputs
const tvcAngleCommands = [0];

// Arrays for height simulation
const y_position = [0];
const y_velocity = [0];
const y_accel = [0];

// Arrays for horizontal simulation
const x_position = [0];
const x_velocity = [0];
const x_accel = [0];

const torques = [0];

// Number of simulation steps between each PID call
const simStepsBetweenPIDCalls = Math.ceil(PIDLoopTime / deltaTSeconds);

//Initialize PID
const TVCPID = initPID(kp_tvc, ki_tvc, kd_tvc, servoAngleRange);
const xPositionPID = initPID(kp_pos, ki_pos, kd_pos, rocketAngleRange);

// **************************************************//
// Physics Simulation Code
// **************************************************//

const endThrustTime = thrustCurveData[thrustCurveData.length - 1][0];
const thrustStepNum = Math.floor(endThrustTime / deltaTSeconds);

const motorMassDelta = (motorMassEnd - motorMassStart) / thrustStepNum;
let motorMassSumLoss = 0;

const motorMass = [motorMassStart];

const simulate = () => {
  for (let i = 1; i < simSteps; i += 1) {
    const prevRocketAngle = sensedRocketAngle[i - 1];
    let perpendicularThrust = 0;

    if (i % simStepsBetweenPIDCalls === 0) {
      if (controlPosition === true) {
        const neededAngle = -xPositionPID.compute(
          x_position[i - 1],
          x_setpoint
        );
        tvcAngleCommands[i] =
          TVCPID.compute(prevRocketAngle, neededAngle) / servoLinkageRatio;
      } else {
        tvcAngleCommands[i] =
          TVCPID.compute(prevRocketAngle, setpoint) / servoLinkageRatio;
      }
    } else {
      tvcAngleCommands[i] = tvcAngleCommands[i - 1];
    }

    if (i > tvcStepWait) {
      tvcAngle[i] = limitTVCAngle(
        tvcAngleCommands[i - tvcStepWait],
        tvcAngle[i - 1]
      );
      tvcAngle[i] += tvcNoiseAmplitude * Math.random() - tvcNoiseAmplitude / 2;
      perpendicularThrust = -thrust[i] * Math.sin(DEG_TO_RAD * tvcAngle[i]);
    } else {
      tvcAngle[i] =
        initialTVCAngle +
        (tvcNoiseAmplitude * Math.random() - tvcNoiseAmplitude / 2);
      perpendicularThrust = -thrust[i] * Math.sin(DEG_TO_RAD * tvcAngle[i]);
    }

    let torque =
      perpendicularThrust * tvc_to_COM -
      constantTorqueBias -
      getRandomTorqueNoise() +
      windTorque[i];

    torques.push(torque);

    if (y_position[i - 1] <= 0) {
      torque = 0;
    }

    accel[i] = torque / momentOfInertia;
    velocity[i] = velocity[i - 1] + accel[i - 1] * deltaTSeconds;
    let exactAngle =
      RAD_TO_DEG *
      (actualRocketAngle[i - 1] * (Math.PI / 180) +
        velocity[i - 1] * deltaTSeconds);
    actualRocketAngle[i] = exactAngle;

    // if (y_position <= 0) {
    //   exactAngle = actualRocketAngle[i - 1];
    // }

    sensedRocketAngle[i] =
      exactAngle +
      (angleSensorNoiseAmplitude * Math.random() -
        angleSensorNoiseAmplitude / 2) +
      angleSensorBias;

    const resultantVelocity = Math.sqrt(
      Math.pow(y_velocity[i - 1], 2) + Math.pow(x_velocity[i - 1], 2)
    );

    // Height simulation

    let dragForce = 0;

    if (y_velocity[i - 1] > 0) {
      dragForce =
        getDragForce(resultantVelocity) * Math.cos(DEG_TO_RAD * exactAngle);
    } else if (y_velocity[i - 1] < 0) {
      dragForce =
        -getDragForce(resultantVelocity) * Math.cos(DEG_TO_RAD * exactAngle);
    }

    let totalMass = mass + motorMassSumLoss;

    const y_force =
      thrust[i] *
        Math.cos(DEG_TO_RAD * tvcAngle[i]) *
        Math.cos(DEG_TO_RAD * exactAngle) -
      dragForce;
    y_accel[i] = y_force / totalMass - 9.81;
    if (y_position[i - 1] <= 0 && y_accel[i] <= 0) {
      y_accel[i] = 0;
    }
    y_velocity[i] = y_velocity[i - 1] + y_accel[i] * deltaTSeconds;
    y_position[i] = y_position[i - 1] + y_velocity[i] * deltaTSeconds;

    const x_force =
      thrust[i] *
        Math.cos(DEG_TO_RAD * tvcAngle[i]) *
        Math.sin(DEG_TO_RAD * exactAngle) -
      dragForce * Math.sin(DEG_TO_RAD * exactAngle);
    x_accel[i] = x_force / totalMass;
    x_velocity[i] =
      x_velocity[i - 1] +
      (1 / 2) * (x_accel[i] + x_accel[i - 1]) * deltaTSeconds;
    x_position[i] =
      x_position[i - 1] +
      (1 / 2) * (x_velocity[i] + x_velocity[i - 1]) * deltaTSeconds;

    if (y_position[i] <= 0) {
      y_position[i] = 0;
      y_velocity[i] = 0;
      x_velocity[i] = 0;
      x_position[i] = 0;
      x_accel[i] = 0;
    }

    if (i < thrustStepNum) {
      motorMassSumLoss += motorMassDelta;
    }
    motorMass.push(motorMassStart + motorMassSumLoss);
  }
};

console.log(motorMassDelta);

// Do not move this variable. Moving this crashes the program
// for some reason :0
let i = 0;

simulate();

// Draws the rocket onto the canvas. No need to change this
const drawRocket = (deg, tvc, flameLength) => {
  setCanvasSize();
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.beginPath();
  ctx.save();
  // translate to center
  ctx.translate(centerX + tvcWidth / 2, centerY + tvcHeight / 2);
  if (deg) {
    ctx.fillText("Rocket Angle: " + deg.toFixed(2), 0, 120);
  }
  if (tvcAngle[i]) {
    ctx.fillText("TVC Angle: " + tvcAngle[i].toFixed(2), 0, 130);
  }

  //rotate rocket
  ctx.rotate((deg * Math.PI) / 180);
  //translate motor down
  ctx.translate(0, rocketHeight / 2);
  //unrotate rocket
  ctx.rotate(-(deg * Math.PI) / 180);
  //rotate motor
  ctx.rotate((tvc * Math.PI) / 180);
  // translate motor flame down
  ctx.translate(0, tvcHeight / 2);
  // draw flame
  ctx.fillStyle = "#fca103";
  ctx.fillRect(-flameWidth / 2, 0, flameWidth, flameLength * 4);
  //un translate
  ctx.translate(0, -tvcHeight / 2);
  // draw motor
  ctx.rect(-tvcWidth / 2, -tvcHeight / 2, tvcWidth, tvcHeight);
  // un rotate motor
  ctx.rotate(-(tvc * Math.PI) / 180);
  //un rotate rocket
  ctx.rotate((deg * Math.PI) / 180);
  // un translate motor
  ctx.translate(0, -rocketHeight / 2);
  //draw rocket
  ctx.rect(-rocketWidth / 2, -rocketHeight / 2, rocketWidth, rocketHeight);
  ctx.stroke();
  ctx.restore();
};

// Runs through one animation iteration on each frame of your monitor
const animate = () => {
  const tvc = tvcAngle[i] + actualRocketAngle[i];
  drawRocket(actualRocketAngle[i], tvc, thrust[i]);
  requestAnimationFrame(animate);
};

// Keep at 60. Just affects animation fps
const fps = 60;
const stepsPerFrame = Math.floor(1 / fps / deltaTSeconds);
setInterval(() => {
  i += stepsPerFrame;
}, 1000 / fps);
animate();

replayButton.onclick = () => {
  i = 0;
  animate();
};

// This part is for graphing purposes.
// Because the PID gets upated sparsely, the PID values
// don't have the same number of datapoints as the
// full simulation. So here we just fill in the blanks
// assuming constant values between PID calls.
// Thus giving a stepped look to the graph if PID interval is high
const totalPIDSteps = Math.floor(simTimeSeconds / PIDLoopTime);
const simStepsBetweenPIDSteps = Math.floor(PIDLoopTime / deltaTSeconds);

const newIterms = [];
const newPErrs = [];
const newDErrs = [];

for (let i = 0; i < totalPIDSteps; i += 1) {
  newIterms[i] = TVCPID.ITerms[i];
  newPErrs[i] = TVCPID.pErrs[i];
  newDErrs[i] = TVCPID.dErrs[i];
  for (let k = 0; k < simStepsBetweenPIDSteps; k += 1) {
    newIterms.push(TVCPID.ITerms[i]);
    newPErrs.push(TVCPID.pErrs[i]);
    newDErrs.push(TVCPID.dErrs[i]);
  }
}

// Setup and initialize plots
// You can add your own plots as well
new uPlot(
  {
    title: "PID Values ",
    id: "chart1",
    class: "my-chart",
    width: 500,
    height: 200,
    scales: {
      x: {
        time: false
      },
      y: {
        range: [-40, 40]
      }
    },
    series: [
      {
        value: (self, rawValue) => `${rawValue}s`
      },
      {
        label: "Actual Rocket Angle (deg)",
        stroke: "rgba(255, 0, 0, 1)",
        value: (self, rawValue) => rawValue.toFixed(2)
      },
      {
        label: "Sensed Rocket Angle (deg)",
        stroke: "rgba(255, 0, 0, 1)",
        value: (self, rawValue) => rawValue.toFixed(2)
      },
      {
        label: "I Term ",
        stroke: "rgba(0, 0, 255, 1)",
        value: (self, rawValue) => rawValue.toFixed(2)
      }
    ]
  },
  [
    actualRocketAngle.map((x, i) => i * deltaTSeconds),
    actualRocketAngle,
    sensedRocketAngle,
    newIterms
  ],
  document.getElementById("chart")
);

new uPlot(
  {
    title: "TVC Angle (deg)",
    id: "chart1",
    class: "my-chart",
    width: 500,
    height: 200,
    scales: {
      x: {
        time: false
      }
    },
    series: [
      {
        value: (self, rawValue) => `${rawValue}s`
      },
      {
        label: "Actual Angle (deg)",
        stroke: "rgba(255, 0, 0, 1)",
        value: (self, rawValue) => rawValue.toFixed(2)
      },
      {
        label: "Commanded Angle (deg)",
        stroke: "rgba(0,0, 0, 1)",
        value: (self, rawValue) => rawValue.toFixed(2)
      },
      {
        label: "Torque (Nm)",
        stroke: "rgba(0,0, 0, 0)",
        value: (self, rawValue) => rawValue.toFixed(2)
      }
    ]
  },
  [
    actualRocketAngle.map((x, i) => i * deltaTSeconds),
    tvcAngle,
    tvcAngleCommands,
    torques
  ],
  document.getElementById("chart")
);

new uPlot(
  {
    title: "Torque (Nm)",
    id: "chart1",
    class: "my-chart",
    width: 500,
    height: 200,
    scales: {
      x: {
        time: false
      },
      y: {
        range: [-0.3, 0.3]
      }
    },
    series: [
      {
        value: (self, rawValue) => `${rawValue}s`
      },
      {
        label: "Torque (Nm)",
        stroke: "rgba(0,0, 0, 1)",
        value: (self, rawValue) => rawValue.toFixed(2)
      }
    ]
  },
  [torques.map((x, i) => i * deltaTSeconds), torques],
  document.getElementById("chart")
);

new uPlot(
  {
    title: "Thrust (N)",
    id: "chart1",
    class: "my-chart",
    width: 500,
    height: 400,
    scales: {
      x: {
        time: false
      }
    },
    series: [
      {
        value: (self, rawValue) => `${rawValue}s`
      },
      {
        label: "Thrust (N)",
        stroke: "rgba(255, 0, 0, 1)",
        value: (self, rawValue) => rawValue.toFixed(2)
      }
    ]
  },
  [actualRocketAngle.map((x, i) => i * deltaTSeconds), thrust],
  document.getElementById("chart")
);

new uPlot(
  {
    title: "Vertical Values",
    id: "chart1",
    class: "my-chart",
    width: 500,
    height: 400,
    scales: {
      x: {
        time: false
      }
    },
    series: [
      {
        value: (self, rawValue) => `${rawValue}s`
      },
      {
        label: "Height (m)",
        stroke: "rgba(0, 0, 0, 1)",
        value: (self, rawValue) => rawValue.toFixed(4)
      },
      {
        label: "Acceleration (m/s^2)",
        stroke: "rgba(255, 0, 0, 1)",
        value: (self, rawValue) => rawValue.toFixed(4)
      },
      {
        label: "Velocity (m/s)",
        stroke: "rgba(0, 0, 255, 1)",
        value: (self, rawValue) => rawValue.toFixed(4)
      },
      {
        label: "Thrust (N)",
        stroke: "rgba(0, 0, 255, 1)",
        value: (self, rawValue) => rawValue.toFixed(2)
      }
    ]
  },
  [
    y_position.map((x, i) => i * deltaTSeconds),
    y_position,
    y_accel,
    y_velocity,
    thrust
  ],
  document.getElementById("chart")
);

new uPlot(
  {
    title: "Horizontal Values",
    id: "chart1",
    class: "my-chart",
    width: 500,
    height: 400,
    scales: {
      x: {
        time: false
      }
    },
    series: [
      {
        value: (self, rawValue) => `${rawValue}s`
      },
      {
        label: "X Position (m)",
        stroke: "rgba(0, 0, 0, 1)",
        value: (self, rawValue) => rawValue.toFixed(4)
      },
      {
        label: "Acceleration (m/s^2)",
        stroke: "rgba(255, 0, 0, 1)",
        value: (self, rawValue) => rawValue.toFixed(4)
      },
      {
        label: "Velocity (m/s)",
        stroke: "rgba(0, 0, 255, 1)",
        value: (self, rawValue) => rawValue.toFixed(4)
      }
    ]
  },
  [
    x_position.map((x, i) => i * deltaTSeconds),
    x_position,
    x_accel,
    x_velocity
  ],
  document.getElementById("chart")
);

var list = [];
for (var j = 0; j < y_position.length; j++)
  list.push({ x: x_position[j], y: y_position[j] });

//2) sort:
list.sort(function (a, b) {
  return a.x < b.x ? -1 : a.x === b.x ? 0 : 1;
  //Sort could be modified to, for example, sort on the age
  // if the name is the same.
});

const x_pos_sorted = [];
const y_pos_sorted = [];

//3) separate them back out:
for (var k = 0; k < list.length; k++) {
  x_pos_sorted[k] = list[k].x;
  y_pos_sorted[k] = list[k].y;
}

new uPlot(
  {
    title: "X-Y Plot",
    id: "chart1",
    class: "my-chart",
    width: 500,
    height: 400,
    scales: {
      x: {
        time: false
      }
    },
    series: [
      {
        label: "X (m)",
        value: (self, rawValue) => rawValue.toFixed(2)
      },
      {
        label: "Y (m)",
        value: (self, rawValue) => rawValue.toFixed(2),
        points: {
          space: 0,
          fill: "red"
        }
      }
    ]
  },
  [x_pos_sorted.map((x, i) => x), y_pos_sorted],
  document.getElementById("chart")
);

new uPlot(
  {
    title: "Wind Torque Effect",
    id: "chart1",
    class: "my-chart",
    width: 500,
    height: 400,
    scales: {
      x: {
        time: false
      }
    },
    series: [
      {
        value: (self, rawValue) => `${rawValue}s`
      },
      {
        label: "Wind Amplitude (N*m)",
        value: (self, rawValue) => rawValue.toFixed(2),
        points: {
          space: 0,
          fill: "red"
        }
      }
    ]
  },
  [windTorque.map((x, i) => i * deltaTSeconds), windTorque],
  document.getElementById("chart")
);

new uPlot(
  {
    title: "Motor Mass (kg)",
    id: "chart1",
    class: "my-chart",
    width: 500,
    height: 400,
    scales: {
      x: {
        time: false
      }
    },
    series: [
      {
        value: (self, rawValue) => `${rawValue}s`
      },
      {
        label: "Motor Mass (kg)",
        value: (self, rawValue) => rawValue.toFixed(2),
        points: {
          space: 0,
          fill: "red"
        }
      }
    ]
  },
  [motorMass.map((x, i) => i * deltaTSeconds), motorMass],
  document.getElementById("chart")
);
