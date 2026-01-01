// renderer.js
export class ARRenderer {
  constructor(canvasEl) {
    this.canvas = canvasEl;

    this.scene = new THREE.Scene();

    const fov = 60;
    const aspect = canvasEl.clientWidth / canvasEl.clientHeight;
    this.camera = new THREE.PerspectiveCamera(fov, aspect, 0.01, 100);
    this.camera.position.set(0, 0, 0);

    this.renderer = new THREE.WebGLRenderer({
      canvas: this.canvas,
      antialias: true,
      alpha: true
    });
    this.renderer.setSize(canvasEl.clientWidth, canvasEl.clientHeight, false);

    // Ambient light
    this.scene.add(new THREE.AmbientLight(0xffffff, 1.0));

    // Video background quad
    this.videoTexture = null;
    this.bgMesh = null;

    // Grid plane (initially disabled)
    this.gridPlane = null;
    this._initGridPlane();
  }

  _initGridPlane() {
    const size = 1.5;    // 1.5 meters approx
    const divisions = 15;

    const gridHelper = new THREE.GridHelper(size, divisions);
    gridHelper.rotation.x = -Math.PI / 2; // lie flat on XZ

    const plane = new THREE.Group();
    plane.add(gridHelper);

    plane.visible = false; // hidden until we have a plane
    this.gridPlane = plane;
    this.scene.add(this.gridPlane);
  }

  setVideoTexture(video) {
    const texture = new THREE.VideoTexture(video);
    texture.minFilter = THREE.LinearFilter;
    texture.magFilter = THREE.LinearFilter;
    texture.format = THREE.RGBFormat;
  
    const geometry = new THREE.PlaneGeometry(2, 2);
    const material = new THREE.MeshBasicMaterial({ map: texture });
    const mesh = new THREE.Mesh(geometry, material);
  
    this.scene.add(mesh);
  }

  /**
   * Update grid plane transform from plane pose:
   * planePose = { position: [x,y,z], normal: [nx,ny,nz] }
   */
  updatePlanePose(planePose) {
    if (!planePose) {
      this.gridPlane.visible = false;
      return;
    }

    this.gridPlane.visible = true;

    const [px, py, pz] = planePose.position;
    const [nx, ny, nz] = planePose.normal;

    // position
    this.gridPlane.position.set(px, py, pz);

    // orient Y-up of gridPlane to align with plane normal
    const normal = new THREE.Vector3(nx, ny, nz).normalize();
    const up = new THREE.Vector3(0, 1, 0);
    const quat = new THREE.Quaternion().setFromUnitVectors(up, normal);
    this.gridPlane.quaternion.copy(quat);
  }

  render() {
    this.renderer.render(this.scene, this.camera);
  }

  resize() {
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }
}
