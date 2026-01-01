// plane_estimator.js
export class PlaneEstimator {
  constructor() {
    this.smoothPlane = null;
    this.alpha = 0.1; // smoothing factor
  }

  estimatePlane(points) {
    if (!points || points.length < 30) return null;

    const maxIterations = 100;
    const threshold = 0.05; // meters
    let bestPlane = null;
    let bestInliers = 0;

    for (let it = 0; it < maxIterations; it++) {
      const sample = this._sampleThree(points);
      if (!sample) continue;
      const plane = this._planeFromPoints(sample[0], sample[1], sample[2]);
      if (!plane) continue;

      const inliers = this._countInliers(points, plane, threshold);
      if (inliers > bestInliers) {
        bestInliers = inliers;
        bestPlane = plane;
      }
    }

    if (!bestPlane) return null;

    // Smooth plane parameters over time
    this.smoothPlane = this._smoothPlane(this.smoothPlane, bestPlane);

    return this.smoothPlane;
  }

  _sampleThree(points) {
    if (points.length < 3) return null;
    const idx = () => Math.floor(Math.random() * points.length);
    let i1 = idx(), i2 = idx(), i3 = idx();
    if (i1 === i2 || i2 === i3 || i1 === i3) return null;
    return [points[i1], points[i2], points[i3]];
  }

  _planeFromPoints(p1, p2, p3) {
    const v1 = {
      x: p2.X - p1.X,
      y: p2.Y - p1.Y,
      z: p2.Z - p1.Z
    };
    const v2 = {
      x: p3.X - p1.X,
      y: p3.Y - p1.Y,
      z: p3.Z - p1.Z
    };

    const nx = v1.y * v2.z - v1.z * v2.y;
    const ny = v1.z * v2.x - v1.x * v2.z;
    const nz = v1.x * v2.y - v1.y * v2.x;

    const norm = Math.sqrt(nx * nx + ny * ny + nz * nz);
    if (norm < 1e-6) return null;

    const normal = { x: nx / norm, y: ny / norm, z: nz / norm };

    // Plane: n·X + d = 0, using p1
    const d = -(normal.x * p1.X + normal.y * p1.Y + normal.z * p1.Z);

    return {
      normal,
      d,
      point: { X: p1.X, Y: p1.Y, Z: p1.Z }
    };
  }

  _countInliers(points, plane, threshold) {
    const { normal, d } = plane;
    let count = 0;
    for (const p of points) {
      const dist = Math.abs(
        normal.x * p.X + normal.y * p.Y + normal.z * p.Z + d
      );
      if (dist < threshold) count++;
    }
    return count;
  }

  _smoothPlane(prev, curr) {
    if (!prev) return curr;
    const a = this.alpha;

    return {
      normal: {
        x: prev.normal.x * (1 - a) + curr.normal.x * a,
        y: prev.normal.y * (1 - a) + curr.normal.y * a,
        z: prev.normal.z * (1 - a) + curr.normal.z * a
      },
      d: prev.d * (1 - a) + curr.d * a,
      point: {
        X: prev.point.X * (1 - a) + curr.point.X * a,
        Y: prev.point.Y * (1 - a) + curr.point.Y * a,
        Z: prev.point.Z * (1 - a) + curr.point.Z * a
      }
    };
  }

  /**
   * Convert plane to renderer pose
   * Returns: { position: [x,y,z], normal: [nx,ny,nz] }
   */
  planeToPose(plane) {
    if (!plane) return null;
    return {
      position: [plane.point.X, plane.point.Y, plane.point.Z],
      normal: [plane.normal.x, plane.normal.y, plane.normal.z]
    };
  }
}
