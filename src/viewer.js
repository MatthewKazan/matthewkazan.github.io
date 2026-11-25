import * as THREE from '../vendor/build/three.module.min.js';
import { OrbitControls } from '../vendor/examples/jsm/controls/OrbitControls.js';

const cornerView = new THREE.Vector3(8.1, 6.5, 9).normalize();
const target = new THREE.Vector3(0, 0.77, -0.22);

export class MapViewer {
  constructor(canvas) {
    this.canvas = canvas;
    this.frames = [];
    this.scene = new THREE.Scene();
    this.renderer = new THREE.WebGLRenderer({ canvas, alpha: true, antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
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

    const resize = new ResizeObserver(() => {
      const { width, height } = canvas.getBoundingClientRect();
      if (!width || !height) return;
      if (this.camera.aspect === 1 && this.frames.length === 0) {
        this.camera.aspect = width / height;
        this.reset();
      }
      this.renderer.setSize(width, height, false);
      this.camera.aspect = width / height;
      this.camera.updateProjectionMatrix();
      this.render();
    });
    resize.observe(canvas);
    canvas.addEventListener('keydown', (event) => this.keyOrbit(event));
    this.reset();
  }

  render() {
    this.renderer.render(this.scene, this.camera);
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
    for (const frame of this.frames) frame.geometry.dispose();
    this.frames = [];
    this.points.clear();
    this.material.size = live ? 0.067 : 0.078;
    this.live = live;
    this.liveBounds = null;
    this.grid.visible = !live;
    this.setVisible(0, []);
    this.reset();
  }

  addFrame({ points, colors }) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(points, 3));
    const linear = new Float32Array(colors.length);
    for (let i = 0; i < colors.length; i++) {
      const s = colors[i] / 255;
      linear[i] = s <= 0.04045 ? s / 12.92 : ((s + 0.055) / 1.055) ** 2.4;
    }
    geometry.setAttribute('color', new THREE.BufferAttribute(linear, 3));
    const frame = new THREE.Points(geometry, this.material);
    frame.visible = false;
    this.points.add(frame);
    this.frames.push(frame);
    if (this.live && this.frames.length === 1) {
      geometry.computeBoundingSphere();
      this.liveBounds = geometry.boundingSphere.clone();
      this.reset();
    }
  }

  setVisible(count, poses) {
    const path = [];
    let points = 0;
    this.frames.forEach((frame, index) => {
      frame.visible = index < count && !!poses[index];
      if (!frame.visible) return;
      const { translation, rotation } = poses[index];
      frame.position.fromArray(translation);
      frame.quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
      path.push(frame.position);
      points += frame.geometry.getAttribute('position').count;
    });
    this.path.geometry.dispose();
    this.path.geometry = new THREE.BufferGeometry().setFromPoints(path);
    this.marker.visible = path.length > 0;
    if (this.marker.visible) this.marker.position.copy(path.at(-1));
    this.render();
    return points;
  }
}
