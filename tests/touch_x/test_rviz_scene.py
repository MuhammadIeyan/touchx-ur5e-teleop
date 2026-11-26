from pathlib import Path
import sys
import numpy as np
import mujoco
from visualization_msgs.msg import Marker
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'scripts'))
from sim_rviz_scene import MODEL, scene_markers


def test_scene_matches_mujoco_geometry_and_follows_joints():
    model=mujoco.MjModel.from_xml_path(str(MODEL));data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,0);mujoco.mj_forward(model,data)
    markers={m.id:m for m in scene_markers(model,data).markers}
    for name in ('surgical_table','surgical_organ','trocar_ring','instrument_tip_collision'):
        gid=model.geom(name).id;m=markers[gid]
        np.testing.assert_allclose([m.pose.position.x,m.pose.position.y,m.pose.position.z],data.geom_xpos[gid])
    table=markers[model.geom('surgical_table').id]
    np.testing.assert_allclose([table.scale.x,table.scale.y,table.scale.z],[.5,.6,.03])
    tool=markers[model.geom('bowel_grasper_visual').id]
    assert tool.type==Marker.MESH_RESOURCE
    assert Path(tool.mesh_resource.removeprefix('file://')).exists()
    tip_id=model.geom('instrument_tip_collision').id
    before=data.geom_xpos[tip_id].copy()
    data.qpos[0]+=.1;mujoco.mj_forward(model,data)
    after={m.id:m for m in scene_markers(model,data).markers}[tip_id]
    xyz=[after.pose.position.x,after.pose.position.y,after.pose.position.z]
    assert np.linalg.norm(np.array(xyz)-before)>.001
    np.testing.assert_allclose(xyz,data.geom_xpos[tip_id])


def test_urdf_flange_aligns_with_mujoco_world():
    import xacro
    from xml.etree import ElementTree
    from scipy.spatial.transform import Rotation
    root=Path(__file__).resolve().parents[2]
    xml=ElementTree.fromstring(xacro.process_file(str(root/'description/ur5e_mujoco.urdf.xacro')).toxml())
    model=mujoco.MjModel.from_xml_path(str(MODEL));data=mujoco.MjData(model)
    for q in ([-1.5708,-1.5708,1.5708,-1.5708,-1.5708,0],[-1.4,-1.3,1.2,-1.1,-1.5,.2]):
        data.qpos[:]=q;mujoco.mj_forward(model,data)
        values={model.joint(i).name:q[i] for i in range(model.njnt)}
        transforms={'world':np.eye(4)}
        pending=list(xml.findall('joint'))
        while pending:
            progress=False
            for joint in pending[:]:
                parent=joint.find('parent').get('link')
                if parent not in transforms:continue
                origin=joint.find('origin');t=np.eye(4)
                if origin is not None:
                    t[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                    t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
                if joint.get('type')=='revolute':
                    axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
                    t[:3,:3]=t[:3,:3]@Rotation.from_rotvec(axis*values[joint.get('name')]).as_matrix()
                transforms[joint.find('child').get('link')]=transforms[parent]@t
                pending.remove(joint);progress=True
            assert progress
        tool=transforms['tool0'];body=model.body('bowel_grasper_body').id
        # Menagerie rounds nominal link lengths; display agreement is ~1 mm,
        # not calibration for the physical robot.
        assert np.linalg.norm(tool[:3,3]-data.xpos[body])<.002
        np.testing.assert_allclose(tool[:3,:3],data.xmat[body].reshape(3,3),atol=1e-6)
