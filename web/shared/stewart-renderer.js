import * as THREE from 'three';

export const DEG = Math.PI / 180;

export const StewartStyle = {
  scene: {
    lightBg: 0xF8F7F4,
    darkBg: 0x050707,
    gridMajor: 0xDBD6CD,
    gridMinor: 0xE8E4DB,
    darkGridMajor: 0x263034,
    darkGridMinor: 0x151D20,
  },
  colors: {
    baseJoint: 0xC0392B,
    actuatorJoint: 0xE6B300,
    platformJoint: 0x4A6B8C,
    lowerLeg: 0x4A6B8C,
    upperLeg: 0xD35400,
    basePlate: 0xDBD6CD,
    platformPlate: 0xC0392B,
    ghost: 0x27AE60,
    trace: 0x5CC7F2,
  },
  radii: {
    baseJoint: 5,
    actuatorJoint: 4,
    platformJoint: 4,
    trace: 6,
  },
  opacity: {
    basePlate: 0.5,
    platformPlate: 0.35,
    ghostLine: 0.5,
    ghostPlate: 0.15,
  },
};

export function createStewartModel(kin = window.Kin || {}) {
  const baseRadius = kin.BASE_RADIUS || 152;
  const platformRadius = kin.PLATFORM_RADIUS || 103;
  const baseAngles = kin.BASE_ANGLES || new Array(6).fill(0);
  const platformAngles = kin.PLATFORM_ANGLES || new Array(6).fill(0);
  const B = [];
  const P = [];
  for (let i = 0; i < 6; i++) {
    const ba = Number(baseAngles[i] || 0) * DEG;
    const pa = Number(platformAngles[i] || 0) * DEG;
    B.push(new THREE.Vector3(baseRadius * Math.cos(ba), baseRadius * Math.sin(ba), 0));
    P.push(new THREE.Vector3(platformRadius * Math.cos(pa), platformRadius * Math.sin(pa), 0));
  }
  return {
    kin,
    B,
    P,
    lowerLeg: kin.LOWER_LEG || 65,
    upperLeg: kin.UPPER_LEG || 130,
    neutralZ: kin.NEUTRAL_Z || 105,
    motorPlaneAngle: kin.MOTOR_PLANE_ANGLE || new Array(6).fill(0),
  };
}

export function computeActuatorPoints(angles, model) {
  const source = Array.isArray(angles) ? angles : new Array(6).fill(90);
  return source.map((ang, i) => {
    const a = Number(ang || 0) * DEG;
    const th = Number(model.motorPlaneAngle[i] || 0) * DEG;
    return new THREE.Vector3(
      model.lowerLeg * Math.cos(a) * Math.cos(th) + model.B[i].x,
      model.lowerLeg * Math.cos(a) * Math.sin(th) + model.B[i].y,
      model.lowerLeg * Math.sin(a) + model.B[i].z
    );
  });
}

export function platformPointsFromPose(pose, model) {
  const roll = Number(pose?.[3] || 0);
  const pitch = Number(pose?.[4] || 0);
  const yaw = Number(pose?.[5] || 0);
  const x = Number(pose?.[0] || 0);
  const y = Number(pose?.[1] || 0);
  const z = Number(pose?.[2] ?? model.neutralZ);
  const cr = Math.cos(roll * DEG), sr = Math.sin(roll * DEG);
  const cp = Math.cos(pitch * DEG), sp = Math.sin(pitch * DEG);
  const cy = Math.cos(yaw * DEG), sy = Math.sin(yaw * DEG);
  const R = [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
  return model.P.map(p => new THREE.Vector3(
    R[0][0] * p.x + R[0][1] * p.y + R[0][2] * p.z + x,
    R[1][0] * p.x + R[1][1] * p.y + R[1][2] * p.z + y,
    R[2][0] * p.x + R[2][1] * p.y + R[2][2] * p.z + z
  ));
}

export function updateLineObject(line, a, b) {
  const p = line.geometry.attributes.position.array;
  p[0] = a.x; p[1] = a.y; p[2] = a.z;
  p[3] = b.x; p[4] = b.y; p[5] = b.z;
  line.geometry.attributes.position.needsUpdate = true;
}

export function updatePlateGeometry(geo, points) {
  const p = geo.attributes.position.array;
  let cx = 0, cy = 0, cz = 0;
  for (const point of points) {
    cx += point.x;
    cy += point.y;
    cz += point.z;
  }
  cx /= points.length;
  cy /= points.length;
  cz /= points.length;
  for (let i = 0; i < 6; i++) {
    const j = (i + 1) % 6;
    const k = i * 9;
    p[k] = cx; p[k + 1] = cy; p[k + 2] = cz;
    p[k + 3] = points[i].x; p[k + 4] = points[i].y; p[k + 5] = points[i].z;
    p[k + 6] = points[j].x; p[k + 7] = points[j].y; p[k + 8] = points[j].z;
  }
  geo.attributes.position.needsUpdate = true;
  geo.computeVertexNormals();
}

export function createSphereObject(scene, color, radius) {
  const mesh = new THREE.Mesh(
    new THREE.SphereGeometry(radius, 14, 10),
    new THREE.MeshPhongMaterial({ color })
  );
  scene.add(mesh);
  return mesh;
}

export function createLineObject(scene, color, opacity = 1) {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute([0, 0, 0, 0, 0, 0], 3));
  const line = new THREE.Line(
    geo,
    new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity })
  );
  scene.add(line);
  return line;
}

export function createBasePlate(scene, model, {
  color = StewartStyle.colors.basePlate,
  opacity = StewartStyle.opacity.basePlate,
} = {}) {
  const shape = new THREE.Shape();
  shape.moveTo(model.B[0].x, model.B[0].y);
  for (let i = 1; i < 6; i++) shape.lineTo(model.B[i].x, model.B[i].y);
  shape.closePath();
  const mesh = new THREE.Mesh(
    new THREE.ShapeGeometry(shape),
    new THREE.MeshPhongMaterial({ color, transparent: opacity < 1, opacity, side: THREE.DoubleSide })
  );
  scene.add(mesh);
  return mesh;
}

export function createPlatformPlate(scene, {
  color = StewartStyle.colors.platformPlate,
  opacity = StewartStyle.opacity.platformPlate,
  visible = true,
} = {}) {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(54), 3));
  const mesh = new THREE.Mesh(
    geo,
    new THREE.MeshPhongMaterial({ color, transparent: opacity < 1, opacity, side: THREE.DoubleSide })
  );
  mesh.visible = visible;
  scene.add(mesh);
  return { geo, mesh };
}

export function createStewartRig(scene, model, {
  CSS2DObject = null,
  labels = false,
  labelClassName = 'motor-label',
} = {}) {
  const baseMesh = createBasePlate(scene, model);
  const platform = createPlatformPlate(scene);
  const bSpheres = [], aSpheres = [], qSpheres = [];
  const lowerLines = [], upperLines = [], motorLabels = [];

  for (let i = 0; i < 6; i++) {
    bSpheres.push(createSphereObject(scene, StewartStyle.colors.baseJoint, StewartStyle.radii.baseJoint));
    aSpheres.push(createSphereObject(scene, StewartStyle.colors.actuatorJoint, StewartStyle.radii.actuatorJoint));
    qSpheres.push(createSphereObject(scene, StewartStyle.colors.platformJoint, StewartStyle.radii.platformJoint));
    lowerLines.push(createLineObject(scene, StewartStyle.colors.lowerLeg));
    upperLines.push(createLineObject(scene, StewartStyle.colors.upperLeg));
    if (labels && CSS2DObject) {
      const div = document.createElement('div');
      div.className = labelClassName;
      div.textContent = `M${i + 1}`;
      const label = new CSS2DObject(div);
      scene.add(label);
      motorLabels.push(label);
    }
  }

  const updateFromPoints = (A, Q) => {
    updatePlateGeometry(platform.geo, Q);
    for (let i = 0; i < 6; i++) {
      bSpheres[i].position.copy(model.B[i]);
      aSpheres[i].position.copy(A[i]);
      qSpheres[i].position.copy(Q[i]);
      updateLineObject(lowerLines[i], model.B[i], A[i]);
      updateLineObject(upperLines[i], A[i], Q[i]);
      if (motorLabels[i]) motorLabels[i].position.copy(Q[i]).add(new THREE.Vector3(0, 0, 12));
    }
  };

  const update = (angles, pose) => {
    updateFromPoints(computeActuatorPoints(angles, model), platformPointsFromPose(pose, model));
  };

  return {
    baseMesh,
    platformGeo: platform.geo,
    platformMesh: platform.mesh,
    bSpheres,
    aSpheres,
    qSpheres,
    lowerLines,
    upperLines,
    motorLabels,
    update,
    updateFromPoints,
  };
}

export function createGhostRig(scene, model) {
  const lines = [];
  for (let i = 0; i < 12; i++) {
    const line = createLineObject(scene, StewartStyle.colors.ghost, StewartStyle.opacity.ghostLine);
    line.visible = false;
    lines.push(line);
  }
  const plate = createPlatformPlate(scene, {
    color: StewartStyle.colors.ghost,
    opacity: StewartStyle.opacity.ghostPlate,
    visible: false,
  });

  const hide = () => {
    plate.mesh.visible = false;
    lines.forEach(line => { line.visible = false; });
  };

  const updateFromPoints = (A, Q) => {
    for (let i = 0; i < 6; i++) {
      updateLineObject(lines[i], model.B[i], A[i]);
      updateLineObject(lines[i + 6], A[i], Q[i]);
      lines[i].visible = true;
      lines[i + 6].visible = true;
    }
    updatePlateGeometry(plate.geo, Q);
    plate.mesh.visible = true;
  };

  return {
    lines,
    lineMaterial: lines[0]?.material,
    plateGeo: plate.geo,
    plateMesh: plate.mesh,
    hide,
    updateFromPoints,
    update: (angles, pose) => updateFromPoints(
      computeActuatorPoints(angles, model),
      platformPointsFromPose(pose, model)
    ),
  };
}
