import * as THREE from '../vendor/build/three.module.min.js';
import { OrbitControls } from '../vendor/examples/jsm/controls/OrbitControls.js';

const cornerView = new THREE.Vector3(8.1, 6.5, 9).normalize();
const target = new THREE.Vector3(0, 0.77, -0.22);

export class MapViewer {
  constructor(canvas) {
    this.canvas = canvas;
    this.frames = [];
    this.replayMaps = [];
    this.scene = new THREE.Scene();
    const webgl = canvas.getContext('webgl2', { alpha: true, antialias: true });
    if (webgl) {
      this.renderer = new THREE.WebGLRenderer({ canvas, context: webgl, alpha: true, antialias: true });
      this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    } else {
      this.context = canvas.getContext('2d');
      if (!this.context) throw new Error('Canvas rendering is unavailable');
    }
    this.camera = new THREE.PerspectiveCamera(40, 1, 0.04, 100);
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enablePan = true;
    this.controls.minDistance = 0.65;
    this.controls.maxDistance = 35;
    this.controls.maxPolarAngle = Math.PI * 0.88;
    this.controls.addEventListener('change', () => this.render());

    this.points = new THREE.Group();
    this.scene.add(this.points);
    const dot = document.createElement('canvas');
    dot.width = dot.height = 32;
    const brush = dot.getContext('2d');
    const gradient = brush.createRadialGradient(16, 16, 1, 16, 16, 16);
    gradient.addColorStop(0, 'white');
    gradient.addColorStop(0.75, 'white');
    gradient.addColorStop(1, 'transparent');
    brush.fillStyle = gradient;
    brush.fillRect(0, 0, 32, 32);
    this.material = new THREE.PointsMaterial({
      size: 0.078, vertexColors: true, map: new THREE.CanvasTexture(dot),
      transparent: true, alphaTest: 0.21,
    });
    this.path = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({
      color: 0xffc485, opacity: 0.84, transparent: true, depthTest: false,
    }));
    this.path.renderOrder = 2;
    this.scene.add(this.path);
    this.marker = new THREE.Mesh(new THREE.SphereGeometry(0.087, 12, 8),
      new THREE.MeshBasicMaterial({ color: 0xffd196, depthTest: false }));
    this.marker.renderOrder = 3;
    this.marker.visible = false;
    this.scene.add(this.marker);

    const grid = new THREE.GridHelper(15, 30, 0x253a52, 0x28394f);
    grid.position.y = -0.06;
    grid.material.transparent = true;
    grid.material.opacity = 0.55;
    this.scene.add(grid);
    this.grid = grid;

    let sized = false;
    const resize = new ResizeObserver(() => {
      const { width, height } = canvas.getBoundingClientRect();
      if (!width || !height) return;
      if (this.renderer) {
        this.renderer.setSize(width, height, false);
      } else {
        canvas.width = Math.round(width);
        canvas.height = Math.round(height);
        this.pixels = this.context.createImageData(canvas.width, canvas.height);
        this.depths = new Float32Array(canvas.width * canvas.height);
      }
      this.camera.aspect = width / height;
      this.camera.updateProjectionMatrix();
      if (!sized) {
        sized = true;
        this.reset();
      }
      this.render();
    });
    resize.observe(canvas);
    canvas.addEventListener('keydown', (event) => this.keyOrbit(event));
    this.reset();
  }

  render() {
    if (this.renderer) this.renderer.render(this.scene, this.camera);
    else if (this.pixels) this.renderCanvas();
  }

  // Keep orbiting and scrubbing available when the browser cannot open WebGL 2.
  renderCanvas() {
    const { width, height, data } = this.pixels;
    data.fill(0);
    this.depths.fill(Infinity);
    this.camera.updateMatrixWorld();
    this.scene.updateMatrixWorld();
    const projection = new THREE.Matrix4().multiplyMatrices(this.camera.projectionMatrix, this.camera.matrixWorldInverse);
    const stride = Math.max(1, Math.ceil(this.visiblePoints / 45_000));
    const point = new THREE.Vector3();
    for (const frame of this.points.children) {
      if (!frame.visible) continue;
      const matrix = new THREE.Matrix4().multiplyMatrices(projection, frame.matrixWorld);
      const points = frame.geometry.getAttribute('position').array;
      const colors = frame.userData.colors;
      for (let i = 0; i < points.length; i += 3 * stride) {
        point.fromArray(points, i).applyMatrix4(matrix);
        if (point.z < -1 || point.z > 1) continue;
        const x = Math.floor((point.x + 1) * width / 2);
        const y = Math.floor((1 - point.y) * height / 2);
        if (x < 0 || x >= width || y < 0 || y >= height) continue;
        const pixel = y * width + x;
        if (point.z >= this.depths[pixel]) continue;
        this.depths[pixel] = point.z;
        data[4 * pixel] = colors[i];
        data[4 * pixel + 1] = colors[i + 1];
        data[4 * pixel + 2] = colors[i + 2];
        data[4 * pixel + 3] = 205;
      }
    }
    this.context.putImageData(this.pixels, 0, 0);
    const trajectory = this.path.geometry.getAttribute('position');
    if (!trajectory?.count) return;
    this.context.beginPath();
    for (let i = 0; i < trajectory.count; i++) {
      point.fromBufferAttribute(trajectory, i).applyMatrix4(projection);
      const x = (point.x + 1) * width / 2;
      const y = (1 - point.y) * height / 2;
      if (i) this.context.lineTo(x, y);
      else this.context.moveTo(x, y);
    }
    this.context.strokeStyle = this.path.material.color.getStyle();
    this.context.stroke();
    this.context.fillStyle = '#ffd196';
    this.context.beginPath();
    this.context.arc((point.x + 1) * width / 2, (1 - point.y) * height / 2, 3, 0, 2 * Math.PI);
    this.context.fill();
  }

  keyOrbit(event) {
    const keys = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', '+', '=', '-'];
    if (!keys.includes(event.key)) return;
    event.preventDefault();
    const offset = this.camera.position.clone().sub(this.controls.target);
    const sphere = new THREE.Spherical().setFromVector3(offset);
    if (event.key === 'ArrowLeft') sphere.theta -= 0.13;
    if (event.key === 'ArrowRight') sphere.theta += 0.13;
    if (event.key === 'ArrowUp') sphere.phi -= 0.13;
    if (event.key === 'ArrowDown') sphere.phi += 0.13;
    if (event.key === '+' || event.key === '=') sphere.radius /= 1.13;
    if (event.key === '-') sphere.radius *= 1.13;
    sphere.makeSafe();
    this.camera.position.copy(offset.setFromSpherical(sphere).add(this.controls.target));
    this.controls.update();
  }

  reset() {
    const distance = this.liveBounds
      ? Math.max(2.8, this.liveBounds.radius * 2.55) * Math.max(1, 1.27 / this.camera.aspect)
      : 10.8 * Math.max(1, 1.35 / this.camera.aspect);
    this.controls.target.copy(this.liveBounds?.center ?? target);
    this.camera.position.copy(cornerView).multiplyScalar(distance).add(this.controls.target);
    this.controls.update();
    this.render();
  }

  clear(live = false) {
    for (const frame of this.points.children) frame.geometry.dispose();
    this.frames = [];
    this.replayMaps = [];
    this.points.clear();
    this.material.size = live ? 0.067 : 0.078;
    this.live = live;
    this.liveBounds = null;
    this.grid.visible = !live;
    this.setVisible(0, []);
    this.reset();
  }

  pointCloud({ points, colors }) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(points, 3));
    if (this.renderer) {
      const linear = new Float32Array(colors.length);
      for (let i = 0; i < colors.length; i++) {
        const s = colors[i] / 255;
        linear[i] = s <= 0.04045 ? s / 12.92 : ((s + 0.055) / 1.055) ** 2.4;
      }
      geometry.setAttribute('color', new THREE.BufferAttribute(linear, 3));
    }
    const frame = new THREE.Points(geometry, this.material);
    if (this.context) frame.userData.colors = colors;
    frame.visible = false;
    this.points.add(frame);
    return frame;
  }

  addFrame(sample) {
    const frame = this.pointCloud(sample);
    this.frames.push(frame);
    if (this.live && this.frames.length === 1) {
      frame.geometry.computeBoundingSphere();
      this.liveBounds = frame.geometry.boundingSphere.clone();
      this.reset();
    }
  }

  setReplayMaps(samples) {
    for (const map of this.replayMaps) {
      this.points.remove(map);
      map.geometry.dispose();
    }
    this.replayMaps = samples.map((sample) => this.pointCloud(sample));
  }

  setVisible(count, poses, adjustment) {
    const path = [];
    let points = 0;
    const merged = adjustment?.amount === 0 ? this.replayMaps[0]
      : adjustment?.amount === 1 ? this.replayMaps[1] : null;
    for (const map of this.replayMaps) map.visible = map === merged;
    const nextPosition = new THREE.Vector3();
    const nextRotation = new THREE.Quaternion();
    this.frames.forEach((frame, index) => {
      const included = index < count && !!poses[index];
      frame.visible = included && !merged;
      if (!included) return;
      const { translation, rotation } = poses[index];
      frame.position.fromArray(translation);
      frame.quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
      if (adjustment) {
        const corrected = adjustment.poses[index];
        const q = corrected.rotation;
        frame.position.lerp(nextPosition.fromArray(corrected.translation), adjustment.amount);
        frame.quaternion.slerp(nextRotation.set(q.x, q.y, q.z, q.w), adjustment.amount);
      }
      path.push(frame.position);
      points += frame.geometry.getAttribute('position').count;
    });
    this.path.geometry.dispose();
    this.path.geometry = new THREE.BufferGeometry().setFromPoints(path);
    this.path.material.color.set(adjustment?.amount > 0.5 ? 0x85ebcf : 0xffc485);
    if (merged) points = merged.geometry.getAttribute('position').count;
    this.visiblePoints = points;
    this.marker.visible = path.length > 0;
    if (this.marker.visible) {
      this.marker.position.copy(path.at(-1));
      this.marker.material.color.set(adjustment?.amount === 1 ? 0x85ebcf : 0xffd196);
    }
    this.render();
    return points;
  }
}
