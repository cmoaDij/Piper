import * as THREE from 'three';
import {OrbitControls} from '/static/vendor/OrbitControls.js';
import {STLLoader} from '/static/vendor/STLLoader.js';

function setOrigin(group, xyz, rpy) {
  group.position.set(...xyz);
  group.quaternion.setFromEuler(new THREE.Euler(...rpy, 'ZYX'));
}
function makeRig(model) {
  const links = new Map(model.links.map(link => [link.name, new THREE.Group()]));
  const joints = new Map();
  const children = new Set();
  for (const joint of model.joints) {
    const origin = new THREE.Group(), pivot = new THREE.Group();
    setOrigin(origin, joint.xyz, joint.rpy);
    links.get(joint.parent).add(origin); origin.add(pivot); pivot.add(links.get(joint.child));
    children.add(joint.child); joints.set(joint.name, {pivot, joint, axis: new THREE.Vector3(...joint.axis).normalize()});
  }
  const root = new THREE.Group();
  for (const [name, group] of links) if (!children.has(name)) root.add(group);
  const tcp = new THREE.Group();
  setOrigin(tcp, model.tcp_offset.slice(0,3), model.tcp_offset.slice(3));
  links.get(model.tip_link).add(tcp);
  function pose(q, gripper) {
    const values = Object.fromEntries(q.map((v,i) => [`joint${i+1}`,v])); values.gripper = gripper;
    for (const {pivot,joint,axis} of joints.values()) {
      let value = values[joint.name] ?? 0;
      if (joint.mimic) value = (values[joint.mimic.joint] ?? 0) * joint.mimic.multiplier + joint.mimic.offset;
      if (joint.type === 'revolute' || joint.type === 'continuous') pivot.quaternion.setFromAxisAngle(axis,value);
      if (joint.type === 'prismatic') pivot.position.copy(axis).multiplyScalar(value);
    }
    root.updateMatrixWorld(true);
  }
  return {root,links,joints,tcp,pose};
}

export async function createRobotView(container, model) {
  const renderer = new THREE.WebGLRenderer({antialias:true, alpha:false, powerPreference:'low-power'});
  renderer.setPixelRatio(Math.min(devicePixelRatio,2));
  renderer.setClearColor(0x0d1723); renderer.outputColorSpace = THREE.SRGBColorSpace;
  container.prepend(renderer.domElement);
  const scene = new THREE.Scene(); scene.fog = new THREE.Fog(0x0d1723,3.5,7);
  const camera = new THREE.PerspectiveCamera(38,1,.005,12); camera.up.set(0,0,1);
  const controls = new OrbitControls(camera,renderer.domElement);
  controls.enableDamping = true; controls.dampingFactor = .09;
  controls.minDistance = .25; controls.maxDistance = 4; controls.target.set(.15,0,.25);
  const ambient = new THREE.HemisphereLight(0xcfe5ff,0x283447,2.2); ambient.position.set(0,0,2); scene.add(ambient);
  const light = new THREE.DirectionalLight(0xffffff,3); light.position.set(.5,.8,2); scene.add(light);
  const rim = new THREE.DirectionalLight(0x69b8e4,1.6); rim.position.set(-.8,-.8,.5); scene.add(rim);
  const grid = new THREE.GridHelper(2.4,24,0x426078,0x263d51); grid.rotation.x = Math.PI/2; grid.material.transparent = true; grid.material.opacity = .55; scene.add(grid);
  scene.add(new THREE.AxesHelper(.16));
  const actual = makeRig(model), target = makeRig(model); scene.add(actual.root,target.root);
  const materials = [];
  const loader = new STLLoader();
  const cache = new Map();
  const jobs = [];
  for (const link of model.links) for (const mesh of link.meshes) {
    if (!cache.has(mesh.url)) cache.set(mesh.url,loader.loadAsync(mesh.url));
    jobs.push(cache.get(mesh.url).then(geometry => {
      const dark = /base|gripper|flange/.test(link.name), color = dark ? 0x45566b : 0xd9e1e9;
      const material = new THREE.MeshStandardMaterial({color,roughness:.42,metalness:.28}); materials.push({material,color});
      const surface = new THREE.Mesh(geometry,material); setOrigin(surface,mesh.xyz,mesh.rpy); surface.scale.set(...mesh.scale);
      actual.links.get(link.name).add(surface);
    }));
  }
  const tipAxes = new THREE.AxesHelper(.075); actual.tcp.add(tipAxes);
  const marker = new THREE.Mesh(new THREE.SphereGeometry(.006,12,8),new THREE.MeshBasicMaterial({color:0xf5c476})); actual.tcp.add(marker);
  const targetGeometry = new THREE.BufferGeometry(), targetMaterial = new THREE.LineBasicMaterial({color:0x40d5bd,transparent:true,opacity:.8,depthTest:false});
  targetGeometry.setAttribute('position',new THREE.Float32BufferAttribute(new Float32Array(8*3),3));
  const targetLine = new THREE.Line(targetGeometry,targetMaterial); targetLine.renderOrder=3; scene.add(targetLine);
  const pathGeometry = new THREE.BufferGeometry(); pathGeometry.setAttribute('position',new THREE.Float32BufferAttribute(new Float32Array(250*3),3));pathGeometry.setDrawRange(0,0);
  const path = new THREE.Line(pathGeometry,new THREE.LineBasicMaterial({color:0xf5c476,transparent:true,opacity:.65})); scene.add(path);
  let pathPoints=[],lastStamp=0,live=false,targetPresent=false,targetVisible=true,pathVisible=true;
  const vector = new THREE.Vector3();
  function update(q, goal, width=.04, fresh=false, stamp=0) {
    live=fresh; actual.pose(q,width); target.pose(goal ?? q,width); targetPresent=Boolean(goal)&&fresh;
    const order = [model.base_link,...Array.from({length:6},(_,i)=>`link${i+1}`)];
    const positions = targetGeometry.attributes.position.array;
    order.forEach((name,i)=>{target.links.get(name).getWorldPosition(vector);vector.toArray(positions,i*3)});
    target.tcp.getWorldPosition(vector);vector.toArray(positions,21);targetGeometry.attributes.position.needsUpdate=true;targetGeometry.computeBoundingSphere();
    if (fresh && stamp !== lastStamp) {
      actual.tcp.getWorldPosition(vector);
      if (!pathPoints.length || vector.distanceTo(pathPoints[pathPoints.length-1])>.001) {
        pathPoints.push(vector.clone()); if(pathPoints.length>250)pathPoints.shift();
        pathPoints.forEach((p,i)=>p.toArray(pathGeometry.attributes.position.array,i*3));
        pathGeometry.attributes.position.needsUpdate=true;pathGeometry.setDrawRange(0,pathPoints.length);pathGeometry.computeBoundingSphere();
      }
      lastStamp=stamp;
    }
    for(const {material,color} of materials) { material.color.setHex(fresh?color:0x617080);material.transparent=!fresh;material.opacity=fresh?1:.48; }
    targetLine.visible=targetVisible&&targetPresent; path.visible=pathVisible&&fresh;
  }
  function view(name) {
    const box=new THREE.Box3().setFromObject(actual.root);
    const size=box.isEmpty()?new THREE.Vector3(.4,.4,.35):box.getSize(new THREE.Vector3());
    const center=box.isEmpty()?new THREE.Vector3(.12,0,.15):box.getCenter(new THREE.Vector3());
    const distance=Math.max(.65,Math.max(size.x,size.y,size.z)/Math.min(1,camera.aspect)/(2*Math.tan(THREE.MathUtils.degToRad(camera.fov/2)))*1.6);
    controls.target.copy(center);
    const offset = name==='front'?new THREE.Vector3(0,1,.07):name==='top'?new THREE.Vector3(.001,0,1):new THREE.Vector3(.75,.9,.58);
    camera.position.copy(center).add(offset.normalize().multiplyScalar(distance)); camera.lookAt(center);controls.update();
  }
  const observer = new ResizeObserver(()=>{const w=container.clientWidth,h=container.clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix()});observer.observe(container);
  view('iso');update([0,0,0,0,0,0],null,.04,false);
  let lastRender=0;
  function draw(time) {requestAnimationFrame(draw);if(document.hidden||time-lastRender<30)return;lastRender=time;controls.update();renderer.render(scene,camera)}
  requestAnimationFrame(draw);
  await Promise.all(jobs);
  view('iso');
  const message=document.getElementById('model-status');message.textContent='本地 URDF / STL 模型已加载';setTimeout(()=>message.hidden=true,2200);
  return {update,view,showTarget:value=>{targetVisible=value;targetLine.visible=value&&targetPresent},showPath:value=>{pathVisible=value;path.visible=value&&live},clearPath:()=>{pathPoints=[];pathGeometry.setDrawRange(0,0)},tcpPosition:()=>actual.tcp.getWorldPosition(new THREE.Vector3()).toArray()};
}
