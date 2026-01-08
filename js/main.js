// main.js
import { CameraManager } from './camera.js';
import { ARRenderer } from './renderer.js';
import { SlamCore } from './slam_core.js';
import { PlaneEstimator } from './plane_estimator.js';
import { initPose, estimatePose, setWorldOriginFromPlane } from "./pose.js";

window.addEventListener('load', () => {
  const videoEl = document.getElementById('camera');
  const cvCanvas = document.getElementById('cvCanvas');
  const threeCanvas = document.getElementById('threeCanvas');
  const startBtn = document.getElementById('startBtn');
  const statusEl = document.getElementById('status');
  const debugToggle = document.getElementById('debugToggle');
  const debugInfo = document.getElementById('debugInfo');

  // Debug overlay canvas (screen-space visualization)
  const debugCanvas = document.createElement('canvas');
  debugCanvas.style.position = 'fixed';
  debugCanvas.style.top = '0';
  debugCanvas.style.left = '0';
  debugCanvas.style.pointerEvents = 'none';
  debugCanvas.style.zIndex = '30';
  document.body.appendChild(debugCanvas);

  function resizeDebugCanvas() {
    const dpr = window.devicePixelRatio || 1;
    const w = window.innerWidth;
    const h = window.innerHeight;
    debugCanvas.width = Math.round(w * dpr);
    debugCanvas.height = Math.round(h * dpr);
    debugCanvas.style.width = w + 'px';
    debugCanvas.style.height = h + 'px';
    const dc = debugCanvas.getContext('2d');
    dc.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  window.addEventListener('resize', resizeDebugCanvas);
  resizeDebugCanvas();
  const debugCtx = debugCanvas.getContext('2d');

  const camera = new CameraManager(videoEl, cvCanvas);
  const renderer = new ARRenderer(threeCanvas);

  let debugFeaturePoints = [];
  let showDebug = false;
  let latestPose = null;

  let lastPlaneHit = null;
  let latestPlaneEstimate = null; // { plane, inlierIndices, inlierCount, centroid, inlierPoints } (updated each frame when available) 

  // Plane anchoring state
  let candidatePlane = null;
  let candidateCount = 0;
  let anchoredPlane = null;
  let anchoredReplaceCount = 0;

  // Anchoring thresholds (tweakable)
  const ANCHOR_ANGLE_RAD = 12 * Math.PI / 180; // 12 degrees
  const ANCHOR_DIST = 0.15; // meters
  const ANCHOR_MIN_STABLE = 4; // frames required to lock
  const ANCHOR_REPLACE_MIN = 6; // frames required to replace existing anchor
  const ANCHOR_REPLACE_ANGLE = 20 * Math.PI / 180; // replace threshold
  const ANCHOR_REPLACE_DIST = 0.3;
  const PLANE_SMOOTH_ALPHA = 0.2; // smoothing when accumulating candidate

  // Inlier-based checks
  const MIN_INLIERS = 50; // minimum inliers required to consider anchoring
  const INLIER_OVERLAP_THRESH = 0.6; // fraction of overlap between consecutive candidate inliers
  const INLIER_MATCH_DIST = 0.05; // meters: distance threshold to consider same inlier between frames

  function projectToScreen(pos) {
    const focal = 600; // legacy fallback
    const z = Math.max(pos.z, 0.05); // prevent explosion

    return {
      x: debugCanvas.clientWidth  / 2 + (pos.x / z) * focal,
      y: debugCanvas.clientWidth / 2 - (pos.y / z) * focal
    };
  }

  // Improved projection: transform a world-space point into camera space using
  // the pose's numeric rotation matrix (if available), then project using
  // camera intrinsics derived from the video size and map to debug canvas.
  function projectWorldToScreen(worldPos, pose) {
    const p = pose || latestPose;
    if (!worldPos) return null;

    if (p && p.rotationMatrix && p.position) {
      const rot = p.rotationMatrix; // row-major 3x3 array

      const dx = worldPos.x - p.position.x;
      const dy = worldPos.y - p.position.y;
      const dz = worldPos.z - p.position.z;

      // p_cam = R * (X_world - cam_pos)
      const X = rot[0] * dx + rot[1] * dy + rot[2] * dz;
      const Y = rot[3] * dx + rot[4] * dy + rot[5] * dz;
      const Z = rot[6] * dx + rot[7] * dy + rot[8] * dz;

      if (Z <= 0.05) return null;

      const fx = videoEl.videoWidth || 600;
      const fy = videoEl.videoWidth || 600;
      const cx = (videoEl.videoWidth || 640) / 2;
      const cy = (videoEl.videoHeight || 480) / 2;

      const x_img = (fx * X / Z) + cx;
      const y_img = (fy * Y / Z) + cy;

      const sx = debugCanvas.clientWidth / (videoEl.videoWidth || debugCanvas.clientWidth);
      const sy = debugCanvas.clientHeight / (videoEl.videoHeight || debugCanvas.clientHeight);

      return { x: x_img * sx, y: y_img * sy };
    }

    // Fallback to the original simple projection
    return projectToScreen(worldPos);
  }

  // Wire up debug toggle UI
  const setZeroBtn = document.getElementById('setZeroBtn');

  if (debugToggle) {
    debugToggle.addEventListener('click', () => {
      showDebug = !showDebug;
      debugToggle.textContent = showDebug ? 'Hide Debug' : 'Show Debug';
      debugToggle.setAttribute('aria-pressed', String(showDebug));
      if (!debugInfo) return;
      debugInfo.hidden = !showDebug;
      debugInfo.setAttribute('aria-hidden', String(!showDebug));
    });
  }

  // Toast helper
  const toastEl = document.getElementById('toast');
  let toastTimer = null;
  function showToast(msg, duration = 1500) {
    if (!toastEl) return;
    toastEl.textContent = msg;
    toastEl.hidden = false;
    toastEl.setAttribute('aria-hidden', 'false');
    toastEl.classList.add('show');
    if (toastTimer) clearTimeout(toastTimer);
    toastTimer = setTimeout(() => {
      toastEl.classList.remove('show');
      toastEl.setAttribute('aria-hidden', 'true');
      toastTimer = setTimeout(() => { toastEl.hidden = true; toastTimer = null; }, 180);
    }, duration);
  }

  // Wire up Set World Zero button
  if (setZeroBtn) {
    setZeroBtn.addEventListener('click', () => {
      const planePoint = (anchoredPlane && anchoredPlane.center) ? anchoredPlane.center : lastPlaneHit;
      if (!planePoint) {
        console.warn('Set World Zero pressed, but no plane hit yet');
        showToast('No surface detected yet');
        return;
      }

      setWorldOriginFromPlane(planePoint);
      showToast('World zero set to surface');
    });
  }

  let running = false;
  let slam = null;
  let planeEstimator = null;

  let cvReady = false;
  let cvInstance = null;

  let prevGray = null;
  let prevPoints = null;

  // Track if loadedmetadata fired before OpenCV became available
  let videoMetadataPending = false;
  videoEl.addEventListener('loadedmetadata', () => {
    if (cvInstance) {
      try {
        initPose(videoEl, cvInstance);
      } catch (e) {
        console.warn('initPose failed from loadedmetadata:', e);
      }
    } else {
      videoMetadataPending = true;
    }
  });
  
  async function initOpenCV() {
    console.log("Initializing OpenCV");

    // cv is a Promise in modularized builds
    if (window.cv instanceof Promise) {
      const cvInstance = await window.cv;
      console.log("OpenCV ready (awaited Promise)");
      return cvInstance;
    }
  
    // Fallback (non-modularized build)
    if (window.cv && window.cv.Mat) {
      console.log("OpenCV ready (non-modularized)");
      return window.cv;
    }
  
    throw new Error("OpenCV not found");
  }

  startBtn.addEventListener("click", async () => {
    if (running) return; // prevent double-starts

    statusEl.textContent = "Starting camera...";
    await camera.start();

    await videoEl.play();

    const videoW = videoEl.videoWidth;
    const videoH = videoEl.videoHeight;

    // Fit the canvas to the viewport while preserving the video's aspect ratio
    const maxW = window.innerWidth;
    const maxH = window.innerHeight;
    const scale = Math.min(maxW / videoW, maxH / videoH, 1); // don't upscale beyond native
    const displayW = Math.round(videoW * scale);
    const displayH = Math.round(videoH * scale);

    const dpr = window.devicePixelRatio || 1;

    // Backing buffer uses device pixels; CSS size uses CSS pixels
    cvCanvas.width = Math.round(displayW * dpr);
    cvCanvas.height = Math.round(displayH * dpr);
    cvCanvas.style.width = `${displayW}px`;
    cvCanvas.style.height = `${displayH}px`;

    // Ensure 2D context maps CSS pixels correctly onto the backing buffer
    const cvCtx = cvCanvas.getContext('2d');
    cvCtx.setTransform(dpr, 0, 0, dpr, 0, 0);

    cvInstance = await initOpenCV();

    // If loadedmetadata happened earlier, initialize pose now; otherwise initialize immediately
    try {
      if (videoMetadataPending || videoEl.readyState >= 1) {
        initPose(videoEl, cvInstance);
      }
    } catch (e) {
      console.warn('initPose failed (OpenCV may not be ready):', e);
    }

    slam = new SlamCore(cvInstance);
    // Create a plane estimator instance (will need 3D points to estimate planes)
    planeEstimator = new PlaneEstimator();

    running = true;
    loop(); // START THE FRAME LOOP

    // Reveal the debug controls only after entering the AR experience
    if (debugToggle) {
      debugToggle.hidden = false;
      debugToggle.setAttribute('aria-pressed', 'false');
      debugToggle.textContent = 'Show Debug';
    }
    if (debugInfo) {
      debugInfo.hidden = true;
      debugInfo.setAttribute('aria-hidden', 'true');
    }

    // Reveal/set up the Set World Zero button when AR starts
    if (setZeroBtn) {
      setZeroBtn.hidden = false;
      setZeroBtn.disabled = true; // disabled until a valid pose is available
      setZeroBtn.textContent = 'Set World Zero';
    }

    // Hide the Start button after AR begins to avoid accidental re-starts
    if (startBtn) {
      startBtn.hidden = true;
    }

    // (toast helper moved to outer scope)
  });

  function drawFeatures(points) {
    if (!points || points.length === 0) return;

    const ctx = cvCanvas.getContext("2d");

    ctx.save();
    ctx.fillStyle = "lime";

    for (let i = 0; i < points.length; i++) {
      const p = points[i];

      // For now we fake projection: screen-space debug
      const x = (p.x !== undefined) ? p.x : null;
      const y = (p.y !== undefined) ? p.y : null;

      if (x === null || y === null) continue;

      ctx.beginPath();
      ctx.arc(x, y, 2, 0, Math.PI * 2);
      ctx.fill();
    }

    ctx.restore();
  }

  function drawVideoPreserveAspect(ctx, video, canvas) {
    const videoAspect = video.videoWidth / video.videoHeight;
    // Use CSS pixels for layout calculations (clientWidth/clientHeight)
    const canvasCssW = canvas.clientWidth;
    const canvasCssH = canvas.clientHeight;
    const canvasAspect = canvasCssW / canvasCssH;

    let drawWidth, drawHeight, offsetX, offsetY;

    if (canvasAspect > videoAspect) {
      // Canvas is wider than video → pillarbox
      drawHeight = canvasCssH;
      drawWidth = drawHeight * videoAspect;
      offsetX = (canvasCssW - drawWidth) / 2;
      offsetY = 0;
    } else {
      // Canvas is taller than video → letterbox
      drawWidth = canvasCssW;
      drawHeight = drawWidth / videoAspect;
      offsetX = 0;
      offsetY = (canvasCssH - drawHeight) / 2;
    }

    // Clear the full backing buffer
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    // drawImage coordinates are in CSS pixels because the context transform maps them
    ctx.drawImage(video, offsetX, offsetY, drawWidth, drawHeight);

    return { offsetX, offsetY, drawWidth, drawHeight };
  }

  // Return four image points (in processing coords) to try as correspondences.
  // This is a heuristic placeholder: picks 4 points nearest to extreme corners
  // of the current detected feature set. Replace with marker detection for
  // robust pose estimation.
  function getDetectedPoints() {
    const pts = debugFeaturePoints;
    if (!pts || pts.length < 4) return null;

    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const p of pts) {
      if (p.x < minX) minX = p.x;
      if (p.x > maxX) maxX = p.x;
      if (p.y < minY) minY = p.y;
      if (p.y > maxY) maxY = p.y;
    }

    function closest(targetX, targetY) {
      let best = null;
      let bd = Infinity;
      for (const p of pts) {
        const dx = p.x - targetX;
        const dy = p.y - targetY;
        const d = dx * dx + dy * dy;
        if (d < bd) {
          bd = d;
          best = p;
        }
      }
      return best;
    }

    const tl = closest(minX, minY);
    const tr = closest(maxX, minY);
    const br = closest(maxX, maxY);
    const bl = closest(minX, maxY);

    if (!tl || !tr || !br || !bl) return null;

    return [ { x: tl.x, y: tl.y }, { x: tr.x, y: tr.y }, { x: br.x, y: br.y }, { x: bl.x, y: bl.y } ];
  }

  // Helper math utilities for plane anchoring
  function len(v) { return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }
  function normalize(v) { const L = len(v) || 1; return { x: v.x / L, y: v.y / L, z: v.z / L }; }
  function dot(a,b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
  function angleBetween(a,b) { const d = Math.max(-1, Math.min(1, dot(normalize(a), normalize(b)))); return Math.acos(d); }
  function dist(a,b) { const dx = a.x-b.x, dy = a.y-b.y, dz = a.z-b.z; return Math.sqrt(dx*dx+dy*dy+dz*dz); }

  function updatePlaneCandidate(candidate) {
    if (!candidate || !candidate.center || !candidate.normal) {
      candidatePlane = null;
      candidateCount = 0;
      return;
    }

    // Convert to normalized representation
    const c = { x: candidate.center.x, y: candidate.center.y, z: candidate.center.z };
    const n = normalize(candidate.normal);

    // If we have an anchored plane, consider replacement logic
    if (anchoredPlane) {
      const ang = angleBetween(n, anchoredPlane.normal);
      const d = dist(c, anchoredPlane.center);
      if (ang > ANCHOR_REPLACE_ANGLE && d > ANCHOR_REPLACE_DIST) {
        anchoredReplaceCount = (anchoredReplaceCount || 0) + 1;
        if (anchoredReplaceCount >= ANCHOR_REPLACE_MIN) {
          anchoredPlane = { center: { ...c }, normal: { ...n }, age: 0 };
          anchoredReplaceCount = 0;
          console.log('Anchored plane replaced');
          showToast('Plane anchor updated');
        }
      } else {
        anchoredReplaceCount = 0;
      }
      return;
    }

    function countInlierOverlap(aPoints, bPoints, matchDist) {
      if (!aPoints || !bPoints || aPoints.length === 0 || bPoints.length === 0) return 0;
      const md2 = matchDist * matchDist;
      let matches = 0;
      for (let i = 0; i < aPoints.length; i++) {
        const a = aPoints[i];
        for (let j = 0; j < bPoints.length; j++) {
          const b = bPoints[j];
          const dx = a.X - b.X, dy = a.Y - b.Y, dz = a.Z - b.Z;
          const d2 = dx * dx + dy * dy + dz * dz;
          if (d2 <= md2) {
            matches++;
            break; // count each a at most once
          }
        }
      }
      return matches / Math.min(aPoints.length, bPoints.length);
    }

    // No anchor yet — compare to current candidate
    if (candidatePlane) {
      const ang = angleBetween(n, candidatePlane.normal);
      const d = dist(c, candidatePlane.center);

      // If both candidates have inlier points, compute overlap
      let overlap = 1.0;
      if (candidatePlane.inlierPoints && candidate.inlierPoints) {
        overlap = countInlierOverlap(candidatePlane.inlierPoints, candidate.inlierPoints, INLIER_MATCH_DIST);
      }

      console.log(`Candidate compare: ang=${ang.toFixed(3)}, d=${d.toFixed(3)}, overlap=${(overlap*100).toFixed(1)}%, cnt_old=${candidatePlane.inlierPoints ? candidatePlane.inlierPoints.length : 0}, cnt_new=${candidate.inlierPoints ? candidate.inlierPoints.length : 0}`);

      if (ang < ANCHOR_ANGLE_RAD && d < ANCHOR_DIST && overlap >= INLIER_OVERLAP_THRESH) {
        // Stable: increment and smooth
        candidateCount++;
        candidatePlane.center.x = candidatePlane.center.x * (1 - PLANE_SMOOTH_ALPHA) + c.x * PLANE_SMOOTH_ALPHA;
        candidatePlane.center.y = candidatePlane.center.y * (1 - PLANE_SMOOTH_ALPHA) + c.y * PLANE_SMOOTH_ALPHA;
        candidatePlane.center.z = candidatePlane.center.z * (1 - PLANE_SMOOTH_ALPHA) + c.z * PLANE_SMOOTH_ALPHA;
        candidatePlane.normal = normalize({
          x: candidatePlane.normal.x * (1 - PLANE_SMOOTH_ALPHA) + n.x * PLANE_SMOOTH_ALPHA,
          y: candidatePlane.normal.y * (1 - PLANE_SMOOTH_ALPHA) + n.y * PLANE_SMOOTH_ALPHA,
          z: candidatePlane.normal.z * (1 - PLANE_SMOOTH_ALPHA) + n.z * PLANE_SMOOTH_ALPHA
        });

        // Replace inlierPoints with the latest (could be smoothed/merged for more robustness)
        if (candidate.inlierPoints && candidate.inlierPoints.length > 0) {
          candidatePlane.inlierPoints = candidate.inlierPoints;
        }

        if (candidateCount >= ANCHOR_MIN_STABLE) {
          // Only anchor if we have sufficient inlier support
          const hasEnough = candidatePlane.inlierPoints && candidatePlane.inlierPoints.length >= MIN_INLIERS;
          if (hasEnough) {
            anchoredPlane = { center: { ...candidatePlane.center }, normal: { ...candidatePlane.normal }, inlierPoints: candidatePlane.inlierPoints, age: 0 };
            candidatePlane = null;
            candidateCount = 0;
            console.log('Plane anchored:', anchoredPlane, 'inliers:', anchoredPlane.inlierPoints.length);
            showToast('Plane anchored');
          } else {
            // Not enough inliers yet; keep accumulating
            console.log('Candidate stable but not enough inliers:', candidatePlane.inlierPoints ? candidatePlane.inlierPoints.length : 0);
          }
        }
      } else {
        // Reset candidate
        console.log('Reset candidate -> inliers:', candidate.inlierPoints ? candidate.inlierPoints.length : 0);
        candidatePlane = { center: { ...c }, normal: { ...n }, inlierPoints: candidate.inlierPoints || null };
        candidateCount = 1;
      }
    } else {
      candidatePlane = { center: { ...c }, normal: { ...n }, inlierPoints: candidate.inlierPoints || null };
      candidateCount = 1;
    }
  }

  function raycastToGroundPlane(pose) {
    // Intersect ray from camera through camera-forward with the world y=0 plane.
    // Use the pose's position (camera origin in world space) and yaw/pitch to build a forward vector.
    if (!pose || !pose.position || !pose.rotation) return null;

    const origin = pose.position;
    const yaw = pose.rotation.yaw || 0;
    const pitch = pose.rotation.pitch || 0;

    const dir = {
      x: Math.sin(yaw) * Math.cos(pitch),
      y: Math.sin(pitch),
      z: Math.cos(yaw) * Math.cos(pitch)
    };

    // Avoid divide by zero
    if (Math.abs(dir.y) < 1e-6) return null;

    const t = -origin.y / dir.y;
    if (t < 0) return null;

    return {
      x: origin.x + dir.x * t,
      y: 0,
      z: origin.z + dir.z * t
    };
  }

  function drawPlaneGrid(plane, opts = {}) {
    if (!plane || !debugCtx) return;

    const opacity = (opts.opacity !== undefined) ? opts.opacity : 0.5;
    const color = opts.color || '0,255,0';

    // Resolve a center and a normal from multiple possible plane shapes
    let center = null;
    if (plane.center) center = plane.center;
    else if (plane.point && plane.point.X !== undefined) center = { x: plane.point.X, y: plane.point.Y, z: plane.point.Z };
    else if (plane.x !== undefined) center = { x: plane.x, y: plane.y, z: plane.z };

    let normal = null;
    if (plane.normal && plane.normal.x !== undefined) normal = plane.normal;
    else normal = { x: 0, y: 1, z: 0 };

    if (!center || !normal) return;

    const size = 0.5; // meters
    const step = 0.1;

    function cross(a, b) {
      return { x: a.y * b.z - a.z * b.y, y: a.z * b.x - a.x * b.z, z: a.x * b.y - a.y * b.x };
    }
    function len(v) { return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }
    function normalize(v) { const L = len(v) || 1; return { x: v.x / L, y: v.y / L, z: v.z / L }; }

    const up = Math.abs(normal.y) > 0.9 ? { x: 1, y: 0, z: 0 } : { x: 0, y: 1, z: 0 };
    const tx = normalize(cross(up, normal));
    const tz = normalize(cross(normal, tx));

    debugCtx.save();
    debugCtx.strokeStyle = `rgba(${color},${opacity})`;
    debugCtx.lineWidth = 1;

    for (let s = -size; s <= size + 1e-9; s += step) {
      const start = {
        x: center.x + tx.x * s - tz.x * size,
        y: center.y + tx.y * s - tz.y * size,
        z: center.z + tx.z * s - tz.z * size
      };
      const end = {
        x: center.x + tx.x * s + tz.x * size,
        y: center.y + tx.y * s + tz.y * size,
        z: center.z + tx.z * s + tz.z * size
      };
      const a = projectWorldToScreen(start, latestPose);
      const b = projectWorldToScreen(end, latestPose);
      if (!a || !b) continue;
      debugCtx.beginPath();
      debugCtx.moveTo(a.x, a.y);
      debugCtx.lineTo(b.x, b.y);
      debugCtx.stroke();
    }

    for (let s = -size; s <= size + 1e-9; s += step) {
      const start = {
        x: center.x + tz.x * s - tx.x * size,
        y: center.y + tz.y * s - tx.y * size,
        z: center.z + tz.z * s - tx.z * size
      };
      const end = {
        x: center.x + tz.x * s + tx.x * size,
        y: center.y + tz.y * s + tx.y * size,
        z: center.z + tz.z * s + tx.z * size
      };
      const a2 = projectWorldToScreen(start, latestPose);
      const b2 = projectWorldToScreen(end, latestPose);
      if (!a2 || !b2) continue;
      debugCtx.beginPath();
      debugCtx.moveTo(a2.x, a2.y);
      debugCtx.lineTo(b2.x, b2.y);
      debugCtx.stroke();
    }

    debugCtx.restore();
  }

  function loop() {
    if (!running || !cvInstance) return;

    const cv = cvInstance;
    const frame = camera.grabFrame();
    if (!frame) {
      requestAnimationFrame(loop);
      return;
    }

    // Feed the raw frame to the SLAM core (it will triangulate matched features into 3D)
    try {
      if (slam) {
        const slamResult = slam.processFrame(frame);
        // If we have enough 3D points, ask the plane estimator to fit a plane
        if (slamResult && slamResult.mapPoints3D && slamResult.mapPoints3D.length >= 30 && planeEstimator) {
          const est = planeEstimator.estimatePlane(slamResult.mapPoints3D);
          if (est) {
            console.log('PlaneEstimator found a plane from triangulated points (inliers:', est.inlierCount + ')');
            latestPlaneEstimate = est;
          } else {
            latestPlaneEstimate = null;
          }
        }
      }
    } catch (err) {
      console.warn('SLAM processing error:', err);
    }

    const rgba = cv.matFromImageData(frame);
    const gray = new cv.Mat();
    cv.cvtColor(rgba, gray, cv.COLOR_RGBA2GRAY);

    if (!prevGray || !prevPoints) {
      // FIRST FRAME: detect features
      prevGray = gray.clone();
      prevPoints = new cv.Mat();
      cv.goodFeaturesToTrack(prevGray, prevPoints, 300, 0.01, 10);
    } else {
      // TRACK features
      const currPoints = new cv.Mat();
      const status = new cv.Mat();
      const err = new cv.Mat();

      cv.calcOpticalFlowPyrLK(
        prevGray,
        gray,
        prevPoints,
        currPoints,
        status,
        err
      );

      debugFeaturePoints = [];
      for (let i = 0; i < status.rows; i++) {
        if (status.data[i] === 1) {
          const x = currPoints.data32F[i * 2];
          const y = currPoints.data32F[i * 2 + 1];
          debugFeaturePoints.push({ x, y });
        }
      }

      prevGray.delete();
      prevPoints.delete();

      prevGray = gray.clone();
      prevPoints = currPoints.clone();

      currPoints.delete();
      status.delete();
      err.delete();
    }

    // Try pose estimation when we have a viable 4-point set
    try {
      const rawImagePts = getDetectedPoints(); // in processing canvas coords
      if (rawImagePts && cvInstance) {
        // scale to the video coordinate space used by initPose (video.videoWidth/height)
        const sx = videoEl.videoWidth / camera.cvCanvas.width;
        const sy = videoEl.videoHeight / camera.cvCanvas.height;
        const imagePtsScaled = rawImagePts.map(p => ({ x: p.x * sx, y: p.y * sy }));

        const objectPoints = [
          { x: -0.05, y: -0.05, z: 0 },
          { x:  0.05, y: -0.05, z: 0 },
          { x:  0.05, y:  0.05, z: 0 },
          { x: -0.05, y:  0.05, z: 0 }
        ];

        const pose = estimatePose(imagePtsScaled, objectPoints, cvInstance);
        if (pose) {
          // pose is now a smoothed poseState { position: {x,y,z}, rotation: {yaw,pitch,roll} }
          latestPose = pose;
          console.log('Position:', pose.position);
          console.log('Yaw:', pose.rotation.yaw);
        } else {
          latestPose = null;
        }

        // Prefer a plane found by a plane estimator (if implemented and has data)
        if (pose && planeEstimator && planeEstimator.raycast) {
          const hit = planeEstimator.raycast(pose.position, pose.rotation);
          if (hit) {
            lastPlaneHit = hit;
            console.log(
              'Plane hit:',
              hit.x.toFixed(3),
              hit.y.toFixed(3),
              hit.z.toFixed(3)
            );
          }
        }

        // Fallback: raycast from camera forward into world y=0 ground plane
        if (!lastPlaneHit && latestPose) {
          const gpHit = raycastToGroundPlane(latestPose);
          if (gpHit) {
            lastPlaneHit = gpHit;
            console.log('Ground-plane hit:', gpHit.x.toFixed(3), gpHit.y.toFixed(3), gpHit.z.toFixed(3));
          }
        }

        // Update our plane candidate (prefer estimator's smoothPlane if available)
        let currentCandidate = null;
        if (latestPlaneEstimate && latestPlaneEstimate.centroid && latestPlaneEstimate.inlierPoints && latestPlaneEstimate.inlierPoints.length > 0) {
          const p = latestPlaneEstimate.plane;
          const c = latestPlaneEstimate.centroid;
          currentCandidate = {
            center: { x: c.X, y: c.Y, z: c.Z },
            normal: { x: p.normal.x, y: p.normal.y, z: p.normal.z },
            inlierPoints: latestPlaneEstimate.inlierPoints
          };
          // also maintain backward-compatible lastPlaneHit
          lastPlaneHit = currentCandidate.center;
        } else if (planeEstimator && planeEstimator.smoothPlane) {
          const p = planeEstimator.smoothPlane;
          currentCandidate = {
            center: { x: p.point.X, y: p.point.Y, z: p.point.Z },
            normal: { x: p.normal.x, y: p.normal.y, z: p.normal.z }
          };
        } else if (lastPlaneHit) {
          currentCandidate = { center: lastPlaneHit, normal: { x: 0, y: 1, z: 0 } };
        }

        updatePlaneCandidate(currentCandidate);

        // Enable the Set World Zero button when we have either a last hit or an anchor
        if (setZeroBtn) setZeroBtn.disabled = !(lastPlaneHit || anchoredPlane);
      }
    } catch (e) {
      console.warn('Pose estimation error:', e);
    }

    rgba.delete();
    gray.delete();

    // DRAW
    const ctx = cvCanvas.getContext("2d");
    const drawRect = drawVideoPreserveAspect(ctx, videoEl, cvCanvas);

    if (showDebug) {
      // draw bounding rect for the video content (letterbox/pillarbox area)
      ctx.save();
      ctx.strokeStyle = 'red';
      ctx.lineWidth = 2;
      ctx.strokeRect(drawRect.offsetX + 0.5, drawRect.offsetY + 0.5, drawRect.drawWidth, drawRect.drawHeight);
      ctx.restore();

      // draw mapped feature points into screen-space
      ctx.fillStyle = "lime";

      const srcW = camera.cvCanvas.width;
      const srcH = camera.cvCanvas.height;

      for (const p of debugFeaturePoints) {
        const x = drawRect.offsetX + (p.x / srcW) * drawRect.drawWidth;
        const y = drawRect.offsetY + (p.y / srcH) * drawRect.drawHeight;

        ctx.beginPath();
        ctx.arc(x, y, 2, 0, Math.PI * 2);
        ctx.fill();
      }

      if (debugInfo) {
        debugInfo.hidden = false;
        debugInfo.setAttribute('aria-hidden', 'false');
        let anchorLine = 'anchor: none';
        if (anchoredPlane) {
          anchorLine = `anchor: yes (x:${anchoredPlane.center.x.toFixed(2)}, y:${anchoredPlane.center.y.toFixed(2)}, z:${anchoredPlane.center.z.toFixed(2)})`;
        } else {
          anchorLine = `candidateCount: ${candidateCount}`;
        }
        debugInfo.textContent = `drawRect:\noffsetX: ${drawRect.offsetX.toFixed(1)}\noffsetY: ${drawRect.offsetY.toFixed(1)}\ndrawW: ${drawRect.drawWidth.toFixed(1)}\ndrawH: ${drawRect.drawHeight.toFixed(1)}\nfeatures: ${debugFeaturePoints.length}\n${anchorLine}`;
      }
    } else {
      if (debugInfo) {
        debugInfo.hidden = true;
        debugInfo.setAttribute('aria-hidden', 'true');
      }
    }

    // Debug overlay: clear and optionally draw the hit marker
    if (debugCtx) {
      // clear using CSS pixel coordinates (context transform maps DPR)
      debugCtx.clearRect(0, 0, debugCanvas.clientWidth, debugCanvas.clientHeight);

      // Draw a marker at the last plane hit (if available)
      if (lastPlaneHit) {
        const screen = projectWorldToScreen(lastPlaneHit, latestPose);
        if (screen) {
          debugCtx.beginPath();
          debugCtx.arc(screen.x, screen.y, 6, 0, Math.PI * 2);
          debugCtx.fillStyle = "lime";
          debugCtx.fill();
        }
      }

      // Draw anchored plane (higher opacity) or candidate / fallback
      if (anchoredPlane) {
        drawPlaneGrid(anchoredPlane, { opacity: 0.9, color: '0,200,0' });
        // label the anchor
        const s = projectWorldToScreen(anchoredPlane.center, latestPose);
        if (s) {
          debugCtx.fillStyle = 'rgba(255,255,255,0.9)';
          debugCtx.font = '12px sans-serif';
          debugCtx.fillText('Anchored', s.x + 8, s.y - 8);
        }
      } else if (candidatePlane) {
        drawPlaneGrid(candidatePlane, { opacity: 0.45, color: '0,255,0' });
      } else if (planeEstimator && planeEstimator.smoothPlane) {
        drawPlaneGrid(planeEstimator.smoothPlane, { opacity: 0.45, color: '0,255,0' });
      } else if (lastPlaneHit) {
        drawPlaneGrid({ center: lastPlaneHit, normal: { x: 0, y: 1, z: 0 } }, { opacity: 0.35, color: '0,255,0' });
      }
    }

    requestAnimationFrame(loop); // KEEP GOING
  }

  window.addEventListener('resize', () => renderer.resize());
});
