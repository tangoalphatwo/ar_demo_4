// pose.js
let cameraMatrix = null;
let distCoeffs = null;
let cvModule = null;
let worldOrigin = null;
let rawPose = null;
let lastPose = null;

const SMOOTHING = 0.85; // closer to 1 = smoother

function smoothValue(prev, next) {
  return prev * SMOOTHING + next * (1 - SMOOTHING);
}

function matFromArray(rows, cols, type, array) {
  const mat = new cvModule.Mat(rows, cols, type);
  // Validate length
  const expected = rows * cols;
  if (array.length !== expected) {
    throw new RangeError(`matFromArray: expected ${expected} elements (rows*cols=${rows}*${cols}), got ${array.length}`);
  }

  // Prefer 64F backing for numeric safety; set underlying buffer
  if (mat.data64F) mat.data64F.set(array);
  else if (mat.data32F) mat.data32F.set(array);
  else if (mat.data) mat.data.set(array);
  else throw new Error('Unsupported matrix backing type');

  return mat;
}

function normalizeTranslation(tvec) {
  return {
    x:  tvec[0],
    y: -tvec[1], // flip Y
    z:  tvec[2]
  };
}

function rotationMatrixToEuler(m) {
  const r00 = m[0], r01 = m[1], r02 = m[2];
  const r10 = m[3], r11 = m[4], r12 = m[5];
  const r20 = m[6], r21 = m[7], r22 = m[8];

  const pitch = Math.asin(-r20);
  const roll  = Math.atan2(r21, r22);
  const yaw   = Math.atan2(r10, r00);

  return {
    yaw,
    pitch,
    roll
  };
}

export function initPose(video, cv) {
  // Accept a cv instance (modular builds) or fall back to global window.cv
  cvModule = cv || (typeof window !== 'undefined' ? window.cv : null);
  if (!cvModule || !cvModule.Mat) {
    throw new Error('OpenCV cv object not provided or not initialized');
  }

  const width  = video.videoWidth;
  const height = video.videoHeight;

  const focalLength = width;

  cameraMatrix = matFromArray(3, 3, cvModule.CV_64F, [
    focalLength, 0, width / 2,
    0, focalLength, height / 2,
    0, 0, 1
  ]);

  distCoeffs = cvModule.Mat.zeros(4, 1, cvModule.CV_64F);
}

function toCvPoint2f(points) {
  const data = [];
  points.forEach(p => data.push(p.x, p.y));
  // Use Nx2 CV_64F mat so underlying data64F is available and length matches rows*cols
  return matFromArray(points.length, 2, cvModule.CV_64F, data);
}

function toCvPoint3f(points) {
  const data = [];
  points.forEach(p => data.push(p.x, p.y, p.z));
  // Use Nx3 CV_64F mat for object points
  return matFromArray(points.length, 3, cvModule.CV_64F, data);
}

export function setWorldOriginFromPlane(planePoint) {
  if (!planePoint || planePoint.x === undefined) {
    console.warn("setWorldOriginFromPlane called with invalid planePoint:", planePoint);
    return;
  }

  worldOrigin = {
    position: {
      x: planePoint.x,
      y: planePoint.y,
      z: planePoint.z
    },
    rotation: {
      yaw: 0,
      pitch: 0,
      roll: 0
    }
  };

  console.log("World origin set from plane:", worldOrigin);
}

export function setWorldOrigin() {
  if (!rawPose) return;

  worldOrigin = {
    position: { ...rawPose.position },
    rotation: { ...rawPose.rotation }
  };

  console.log("World origin set:", worldOrigin);
}

function applyWorldZero(pose) {
  if (!worldOrigin) return pose;

  return {
    position: {
      x: pose.position.x - worldOrigin.position.x,
      y: pose.position.y - worldOrigin.position.y,
      z: pose.position.z - worldOrigin.position.z
    },
    rotation: {
      yaw:   pose.rotation.yaw   - worldOrigin.rotation.yaw,
      pitch: pose.rotation.pitch - worldOrigin.rotation.pitch,
      roll:  pose.rotation.roll  - worldOrigin.rotation.roll
    }
  };
}

export function estimatePose(imagePoints, objectPoints, cv) {
  const cvLocal = cv || cvModule;
  if (!cvLocal || !cameraMatrix || !distCoeffs) {
    console.warn('Pose system not initialized');
    return null;
  }

  // Ensure cvModule is set for helper functions
  cvModule = cvModule || cvLocal;

  const imgPts = toCvPoint2f(imagePoints);
  const objPts = toCvPoint3f(objectPoints);

  const rvec = new cvModule.Mat();
  const tvec = new cvModule.Mat();

  const success = cvModule.solvePnP(
    objPts,
    imgPts,
    cameraMatrix,
    distCoeffs,
    rvec,
    tvec,
    false,
    cvModule.SOLVEPNP_ITERATIVE
  );

  imgPts.delete();
  objPts.delete();

  if (!success) {
    rvec.delete();
    tvec.delete();
    return null;
  }

  const rotationMatrix = new cvModule.Mat();
  cvModule.Rodrigues(rvec, rotationMatrix);

  // Extract numeric arrays from mats (prefer 64F, then 32F, then generic data)
  const tArr = (tvec.data64F && Array.from(tvec.data64F)) || (tvec.data32F && Array.from(tvec.data32F)) || (tvec.data && Array.from(tvec.data));
  const rotArr = (rotationMatrix.data64F && Array.from(rotationMatrix.data64F)) || (rotationMatrix.data32F && Array.from(rotationMatrix.data32F)) || (rotationMatrix.data && Array.from(rotationMatrix.data));

  // Normalize translation and convert rotation matrix to Euler
  const t = normalizeTranslation(tArr);
  const r = rotationMatrixToEuler(rotArr);

  rawPose = {
    position: { ...t },
    rotation: { ...r },
    rotationMatrix: rotArr.slice() // keep numeric rotation matrix around
    };

  // Build pose state and smooth against lastPose
  let poseState = {
    position: { ...t },
    rotation: { ...r },
    rotationMatrix: rotArr.slice()
  };

  // Clean up mats
  rvec.delete();
  tvec.delete();
  rotationMatrix.delete();

  if (lastPose) {
    poseState.position.x = smoothValue(lastPose.position.x, poseState.position.x);
    poseState.position.y = smoothValue(lastPose.position.y, poseState.position.y);
    poseState.position.z = smoothValue(lastPose.position.z, poseState.position.z);

    poseState.rotation.yaw   = smoothValue(lastPose.rotation.yaw,   poseState.rotation.yaw);
    poseState.rotation.pitch = smoothValue(lastPose.rotation.pitch, poseState.rotation.pitch);
    poseState.rotation.roll  = smoothValue(lastPose.rotation.roll,  poseState.rotation.roll);
  }

  lastPose = poseState;
  const worldPose = applyWorldZero(poseState);
  // lastPose = worldPose;
  // Preserve numeric rotation matrix in worldPose if available
  if (poseState.rotationMatrix) {
    worldPose.rotationMatrix = poseState.rotationMatrix.slice();
  }

  return worldPose;
}
