// slam_core.js
export class SlamCore {
    constructor(cv) {
        this.cv = cv;

        this.pose = {
            R: cv.Mat.eye(3, 3, cv.CV_64F),
            t: cv.Mat.zeros(3, 1, cv.CV_64F)
        };

        this.initialized = false;
        this.mapPoints = [];
        // 3D points triangulated from matched features (in camera coordinates)
        this.mapPoints3D = [];
        this.gray = null;
        this.currGray = null;
        this.prevGray = null;
        this.prevPoints = null;
    }

    _ensureMats(width, height) {
        if (!this.gray) {
            this.gray = new this.cv.Mat(height, width, this.cv.CV_8UC1);
            this.currGray = new this.cv.Mat(height, width, this.cv.CV_8UC1);
        }
    }

    processFrame(imageData) {
        const cv = this.cv;

        const width = imageData.width;
        const height = imageData.height;

        this._ensureMats(width, height);

        const rgbaMat = cv.matFromImageData(imageData);
        cv.cvtColor(rgbaMat, this.currGray, cv.COLOR_RGBA2GRAY);
        rgbaMat.delete();

        if (!this.initialized) {
            this.prevGray = this.currGray.clone();
            this.prevPoints = this._detectFeatures(this.prevGray);
            this.initialized = true;

            return { pose: this.pose, mapPoints: this.mapPoints };
        }

        const { currPoints, status } = this._trackFeatures(
            this.prevGray,
            this.currGray,
            this.prevPoints
        );

        const goodPrev = [];
        const goodCurr = [];

        for (let i = 0; i < status.rows; i++) {
            if (status.data[i] === 1) {
                goodPrev.push(
                    this.prevPoints.data32F[2 * i],
                    this.prevPoints.data32F[2 * i + 1]
                );
                goodCurr.push(
                    currPoints.data32F[2 * i],
                    currPoints.data32F[2 * i + 1]
                );
            }
        }

        if (goodPrev.length < 16) {
            this.prevGray = this.currGray.clone();
            this.prevPoints = this._detectFeatures(this.prevGray);
            return { pose: this.pose, mapPoints: this.mapPoints };
        }

        const prevMat = cv.matFromArray(
            goodPrev.length / 2,
            1,
            cv.CV_32FC2,
            goodPrev
        );

        const currMat = cv.matFromArray(
            goodCurr.length / 2,
            1,
            cv.CV_32FC2,
            goodCurr
        );

        const fx = 600, fy = 600;
        const cx = width / 2, cy = height / 2;

        const K = cv.matFromArray(3, 3, cv.CV_64F, [
            fx, 0, cx,
            0, fy, cy,
            0,  0,  1
        ]);

        const mask = new cv.Mat();
        let E = null;
        if (cv.findEssentialMat) {
            E = cv.findEssentialMat(currMat, prevMat, K, cv.RANSAC, 0.999, 1.0, mask);
        } else if (cv.findFundamentalMat) {
            // Fallback: compute Fundamental matrix then convert to Essential: E = K^T * F * K
            const F = cv.findFundamentalMat(currMat, prevMat, cv.FM_RANSAC, 3, 0.99, mask);
            try {
                const Kt = new cv.Mat();
                cv.transpose(K, Kt);
                const tmp = new cv.Mat();
                cv.gemm(Kt, F, 1, new cv.Mat(), 0, tmp);
                E = new cv.Mat();
                cv.gemm(tmp, K, 1, new cv.Mat(), 0, E);
                tmp.delete();
                Kt.delete();
            } catch (err) {
                console.warn('Failed to compute E from F fallback:', err);
                if (F) F.delete();
                throw err;
            }
            if (F) F.delete();
        } else {
            throw new Error('Neither cv.findEssentialMat nor cv.findFundamentalMat available in OpenCV.js build');
        }

        const R = new cv.Mat();
        const t = new cv.Mat();

        if (E) {
            try {
                cv.recoverPose(E, currMat, prevMat, K, R, t, mask);

                this.pose.R = R.mul(this.pose.R);
                this.pose.t = this._addTrans(this.pose.t, t);

                // Triangulate matched points into 3D (camera1 coordinate system)
                this.mapPoints3D = [];
                try {
                    if (goodPrev.length >= 16) {
                        const n = goodPrev.length / 2;

                        // build point arrays: xs then ys (2 x N mats)
                        const xs1 = [], ys1 = [], xs2 = [], ys2 = [];
                        for (let i = 0; i < goodPrev.length; i += 2) {
                            xs1.push(goodPrev[i]); ys1.push(goodPrev[i+1]);
                            xs2.push(goodCurr[i]); ys2.push(goodCurr[i+1]);
                        }

                        const pts1 = cv.matFromArray(2, n, cv.CV_64F, xs1.concat(ys1));
                        const pts2 = cv.matFromArray(2, n, cv.CV_64F, xs2.concat(ys2));

                        // Camera intrinsics (same heuristic used elsewhere)
                        const fx = 600, fy = 600;
                        const cx = width / 2, cy = height / 2;
                        const Kmat = cv.matFromArray(3, 3, cv.CV_64F, [
                            fx, 0, cx,
                            0, fy, cy,
                            0, 0, 1
                        ]);

                        // P1 = K * [I|0]
                        const P1 = new cv.Mat(3, 4, cv.CV_64F);
                        for (let r = 0; r < 3; r++) {
                            for (let c = 0; c < 3; c++) {
                                P1.doublePtr(r, c)[0] = Kmat.doublePtr(r, c)[0];
                            }
                            P1.doublePtr(r, 3)[0] = 0;
                        }

                        // Rt = [R|t]
                        const Rt = new cv.Mat(3, 4, cv.CV_64F);
                        for (let r = 0; r < 3; r++) {
                            for (let c = 0; c < 3; c++) {
                                Rt.doublePtr(r, c)[0] = R.doublePtr(r, c)[0];
                            }
                            Rt.doublePtr(r, 3)[0] = t.doublePtr(r, 0)[0];
                        }

                        const P2 = new cv.Mat();
                        cv.gemm(Kmat, Rt, 1, new cv.Mat(), 0, P2);

                        const points4D = new cv.Mat();
                        cv.triangulatePoints(P1, P2, pts1, pts2, points4D);

                        for (let i = 0; i < n; i++) {
                            const w = points4D.doublePtr(3, i)[0];
                            if (Math.abs(w) < 1e-8) continue;
                            const X = points4D.doublePtr(0, i)[0] / w;
                            const Y = points4D.doublePtr(1, i)[0] / w;
                            const Z = points4D.doublePtr(2, i)[0] / w;
                            // basic filtering of outliers
                            if (!isFinite(X) || !isFinite(Y) || !isFinite(Z)) continue;
                            if (Math.abs(X) > 100 || Math.abs(Y) > 100 || Math.abs(Z) > 100) continue;
                            this.mapPoints3D.push({ X, Y, Z });
                        }

                        points4D.delete();
                        P1.delete(); Rt.delete(); P2.delete(); pts1.delete(); pts2.delete(); Kmat.delete();
                    }
                } catch (err) {
                    console.warn('Triangulation failed:', err);
                }

                this.prevGray = this.currGray.clone();
                this.prevPoints = currMat.clone();

            } catch (err) {
                console.warn('recoverPose failed:', err);
                // Keep previous pose and skip triangulation for this frame
                this.mapPoints3D = [];
                this.prevGray = this.currGray.clone();
                this.prevPoints = currMat.clone();
            }

            if (E && E.delete) E.delete();
        } else {
            // No essential matrix computed; skip pose update and triangulation this frame
            console.warn('Skipping pose recovery: no essential/fundamental computation available for this frame');
            this.mapPoints3D = [];
            this.prevGray = this.currGray.clone();
            this.prevPoints = currMat.clone();
        }

        if (mask && mask.delete) mask.delete();
        if (prevMat && prevMat.delete) prevMat.delete();

        return { pose: this.pose, mapPoints: this.mapPoints, mapPoints3D: this.mapPoints3D };
    }

    _addTrans(t1, t2) {
        const cv = this.cv;
        const out = new cv.Mat(3, 1, cv.CV_64F);

        for (let i = 0; i < 3; i++) {
            out.doublePtr(i, 0)[0] =
                t1.doublePtr(i, 0)[0] + t2.doublePtr(i, 0)[0];
        }
        return out;
    }

    _detectFeatures(gray) {
      const cv = this.cv;
      const corners = new cv.Mat();
    
      cv.goodFeaturesToTrack(gray, corners, 300, 0.01, 10);
    
      // DEBUG: expose 2D points for rendering
      this.mapPoints = [];
      for (let i = 0; i < corners.rows; i++) {
        const x = corners.data32F[i * 2];
        const y = corners.data32F[i * 2 + 1];
        this.mapPoints.push({ x, y });
      }
    
      return corners;
    }

    _trackFeatures(prevGray, currGray, prevPoints) {
        const cv = this.cv;
        const currPoints = new cv.Mat();
        const status = new cv.Mat();
        const err = new cv.Mat();

        cv.calcOpticalFlowPyrLK(
            prevGray,
            currGray,
            prevPoints,
            currPoints,
            status,
            err
        );

        err.delete();
        return { currPoints, status };
    }
}
