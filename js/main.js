// main.js
import { CameraManager } from './camera.js';
import { ARRenderer } from './renderer.js';
import { SlamCore } from './slam_core.js';
import { initPose, estimatePose } from "./pose.js";

window.addEventListener('load', () => {
  const videoEl = document.getElementById('camera');
  const cvCanvas = document.getElementById('cvCanvas');
  const threeCanvas = document.getElementById('threeCanvas');
  const startBtn = document.getElementById('startBtn');
  const statusEl = document.getElementById('status');
  const debugToggle = document.getElementById('debugToggle');
  const debugInfo = document.getElementById('debugInfo');

  const camera = new CameraManager(videoEl, cvCanvas);
  const renderer = new ARRenderer(threeCanvas);

  let debugFeaturePoints = [];
  let showDebug = false;
  let latestPose = null;

  // Wire up debug toggle UI

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

  let running = false;
  let slam = null;

  let cvReady = false;
  let cvInstance = null;

  let prevGray = null;
  let prevPoints = null;

  // Feature tracking / reseeding (for debug dots + pose heuristics)
  const MAX_FEATURES = 300;
  const MIN_FEATURES = 80; // when tracked points drop below this, detect more
  const FEATURE_QUALITY = 0.01;
  const FEATURE_MIN_DIST = 10;
  const FEATURE_EXCLUSION_RADIUS = 12; // pixels; prevents re-detecting on top of existing points

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


  function loop() {
    if (!running || !cvInstance) return;

    const cv = cvInstance;
    const frame = camera.grabFrame();
    if (!frame) {
      requestAnimationFrame(loop);
      return;
    }

    // Feed the raw frame to the SLAM core
    try {
      if (slam) slam.processFrame(frame);
    } catch (err) {
      console.warn('SLAM processing error:', err);
    }

    const rgba = cv.matFromImageData(frame);
    const gray = new cv.Mat();
    cv.cvtColor(rgba, gray, cv.COLOR_RGBA2GRAY);

    if (!prevGray || !prevPoints || prevPoints.rows === 0) {
      // FIRST FRAME (or fully lost): detect features
      prevGray = gray.clone();
      prevPoints = new cv.Mat();
      cv.goodFeaturesToTrack(prevGray, prevPoints, MAX_FEATURES, FEATURE_QUALITY, FEATURE_MIN_DIST);

      debugFeaturePoints = [];
      for (let i = 0; i < prevPoints.rows; i++) {
        debugFeaturePoints.push({
          x: prevPoints.data32F[i * 2],
          y: prevPoints.data32F[i * 2 + 1]
        });
      }
    } else {
      // TRACK features
      const currPoints = new cv.Mat();
      const status = new cv.Mat();
      const err = new cv.Mat();

      cv.calcOpticalFlowPyrLK(prevGray, gray, prevPoints, currPoints, status, err);

      // Keep only successful tracks (status == 1)
      const goodCurrFloats = [];
      for (let i = 0; i < status.rows; i++) {
        if (status.data[i] === 1) {
          goodCurrFloats.push(
            currPoints.data32F[i * 2],
            currPoints.data32F[i * 2 + 1]
          );
        }
      }

      let mergedFloats = goodCurrFloats;
      const goodCount = goodCurrFloats.length / 2;

      // If we’re running low, detect more points on the current frame and merge them in.
      if (goodCount < MIN_FEATURES) {
        const want = Math.max(0, MAX_FEATURES - goodCount);
        if (want > 0) {
          const mask = new cv.Mat(gray.rows, gray.cols, cv.CV_8UC1, new cv.Scalar(255));
          // Exclude areas around existing points so we add *new* features.
          for (let i = 0; i < goodCount; i++) {
            const x = goodCurrFloats[i * 2];
            const y = goodCurrFloats[i * 2 + 1];
            cv.circle(mask, new cv.Point(x, y), FEATURE_EXCLUSION_RADIUS, new cv.Scalar(0), -1);
          }

          const extra = new cv.Mat();
          // mask parameter is optional; OpenCV.js supports it for goodFeaturesToTrack.
          cv.goodFeaturesToTrack(gray, extra, want, FEATURE_QUALITY, FEATURE_MIN_DIST, mask);

          if (extra.rows > 0) {
            mergedFloats = goodCurrFloats.slice();
            for (let i = 0; i < extra.rows; i++) {
              mergedFloats.push(extra.data32F[i * 2], extra.data32F[i * 2 + 1]);
            }
          }

          extra.delete();
          mask.delete();
        }
      }

      debugFeaturePoints = [];
      for (let i = 0; i < mergedFloats.length; i += 2) {
        debugFeaturePoints.push({ x: mergedFloats[i], y: mergedFloats[i + 1] });
      }

      // Update tracking state to only the surviving + newly detected points
      const nextPoints = mergedFloats.length
        ? cv.matFromArray(mergedFloats.length / 2, 1, cv.CV_32FC2, mergedFloats)
        : new cv.Mat(0, 1, cv.CV_32FC2);

      prevGray.delete();
      prevPoints.delete();
      prevGray = gray.clone();
      prevPoints = nextPoints;

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

        // pose estimate only
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
        debugInfo.textContent = `drawRect:\noffsetX: ${drawRect.offsetX.toFixed(1)}\noffsetY: ${drawRect.offsetY.toFixed(1)}\ndrawW: ${drawRect.drawWidth.toFixed(1)}\ndrawH: ${drawRect.drawHeight.toFixed(1)}\nfeatures: ${debugFeaturePoints.length}`;
      }
    } else {
      if (debugInfo) {
        debugInfo.hidden = true;
        debugInfo.setAttribute('aria-hidden', 'true');
      }
    }

    requestAnimationFrame(loop); // KEEP GOING
  }

  window.addEventListener('resize', () => renderer.resize());
});
