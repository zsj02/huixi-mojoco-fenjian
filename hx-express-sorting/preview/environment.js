import * as T from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';
try {
  const response=await fetch('../scenes/g2_environment.json');
  if(!response.ok) throw Error(`HTTP ${response.status}`);
  const data=await response.json();
  const scene=new T.Scene();scene.background=new T.Color('#131d24');
  const camera=new T.PerspectiveCamera(43,innerWidth/innerHeight,.01,100);
  camera.up.set(0,0,1);camera.position.set(4,-6,4);
  const renderer=new T.WebGLRenderer({antialias:true});renderer.setSize(innerWidth,innerHeight);renderer.setPixelRatio(Math.min(devicePixelRatio,2));document.body.appendChild(renderer.domElement);
  const controls=new OrbitControls(camera,renderer.domElement);controls.target.set(.6,0,.8);controls.update();
  scene.add(new T.HemisphereLight(0xe2efff,0x44545e,2));
  const light=new T.DirectionalLight(0xffedcf,2.5);light.position.set(-1,-4,6);scene.add(light);
  const groups=data.initial.pos.map((p,i)=>{const g=new T.Group();g.position.set(...p);const q=data.initial.quat[i];g.quaternion.set(q[1],q[2],q[3],q[0]);scene.add(g);return g;});
  const loader=new T.TextureLoader();
  for(const o of data.objects){
    let g;
    if(o.kind==='box')g=new T.BoxGeometry(...o.size.map(x=>x*2));
    else if(o.kind==='cylinder'){g=new T.CylinderGeometry(o.size[0],o.size[0],o.size[1]*2,24);g.rotateX(Math.PI/2);}
    else if(o.kind==='sphere')g=new T.SphereGeometry(o.size[0],20,14);
    else{g=new T.BufferGeometry();g.setAttribute('position',new T.Float32BufferAttribute(o.vertices,3));g.setIndex(o.faces);g.computeVertexNormals();}
    const material=new T.MeshStandardMaterial({color:new T.Color(...o.color.slice(0,3)),roughness:.7});
    if(o.texture){material.map=loader.load('../scenes/'+o.texture);material.map.colorSpace=T.SRGBColorSpace;material.color.set(0xffffff);}
    const mesh=new T.Mesh(g,material);mesh.position.set(...o.pos);mesh.quaternion.set(o.quat[1],o.quat[2],o.quat[3],o.quat[0]);groups[o.body].add(mesh);
  }
  document.querySelector('#status').textContent='主模型 SHA-256: '+data.model_sha256;
  addEventListener('resize',()=>{camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix();renderer.setSize(innerWidth,innerHeight);});
  renderer.setAnimationLoop(()=>{controls.update();renderer.render(scene,camera);});
}catch(error){document.querySelector('#status').textContent='加载失败：'+error.message;console.error(error);}
