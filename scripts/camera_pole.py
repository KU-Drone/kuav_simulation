from pcg_gazebo.parsers.sdf import create_sdf_element, Sensor, Model, Joint, Link
import math
from rospkg import RosPack
import os

def add_360_camera(model:Model, name: str, width: int, height: int, horizontal_fov: float, angle: float) -> Sensor:
    assert len(name)>0, "Name cant be empty"
    link_name = name+"_link"
    joint_name = name+"_joint"
    link = create_sdf_element("link")
    link.pose=[math.cos(angle)*0.02, math.sin(angle)*0.02, 0.4, 0, 0, angle]
    link.mass = 1e-10
    link.inertia.ixx = 0.1
    link.inertia.iyy = 0.1
    link.inertia.izz = 0.1
    sensor = create_sdf_element("sensor")
    sensor.name=name
    sensor.type="camera"
    sensor.camera = create_sdf_element("camera")
    # sensor.reset(mode="camera", with_optional_elements=True)
    sensor.camera.name = name
    sensor.camera.horizontal_fov = horizontal_fov
    sensor.camera.image.width = width
    sensor.camera.image.height = height
    sensor.camera.clip.near = 0.1
    sensor.camera.clip.far = 1000
    sensor.always_on=True
    sensor.update_rate=30
    sensor.visualize=True
    pluginArgs = dict(
        always_on=True,
        update_rate=30,
        cameraName=name,
        imageTopicName="raw",
        cameraInfoTopicName="info",
        frameName=link_name
    )
    plugin = create_sdf_element("plugin", pluginArgs)
    plugin.value = pluginArgs
    plugin.name = "camera_controller"
    plugin.filename = "libgazebo_ros_camera.so"
    sensor.add_plugin("camera_controller", plugin)
    link.add_sensor(name, sensor)
    model.add_link(link_name, link)
    joint = create_sdf_element("joint")
    joint.type = "fixed"
    joint.parent="base_link"
    joint.child=link_name
    model.add_joint(joint_name,joint)

if __name__ == "__main__":
    RAD_TO_DEG = 180/math.pi
    DEG_TO_RAD = 1/RAD_TO_DEG
    NUMBERS = ["zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine"]

    CAM_360_WIDTH = 1280
    CAM_360_HEIGHT = 720
    CAM_360_HORIZONTAL_FOV = 120*DEG_TO_RAD
    CAM_360_COUNT = 4

    file = create_sdf_element("sdf")
    file.add_model("camera_pole")
    model = file.models[0]
    model.add_link(name="base_link")
    link = model.links[0]
    link.mass = 1e-10
    link.inertia.ixx = 0.0083137104
    link.inertia.iyy = 0.0387382402
    link.inertia.izz = 0.0469845106

    
    collision = create_sdf_element("collision")
    collision.pose = [0, 0, 0.2, 0, 0, 0]
    collision.geometry.cylinder = create_sdf_element("cylinder")
    collision.geometry.cylinder.radius = 0.02
    collision.geometry.cylinder.length = 0.4
    link.add_collision("collision", collision)

    link.add_visual("visual_cylinder")
    visual = link.visuals[-1]
    visual.pose = [0, 0, 0.2, 0, 0, 0]
    visual.geometry.cylinder = create_sdf_element("cylinder")
    visual.geometry.cylinder.radius = 0.02
    visual.geometry.cylinder.length = 0.4
    
    link.add_visual("visual_sphere")
    visual = link.visuals[-1]
    visual.pose = [0, 0, 0.4, 0, 0, 0]
    visual.geometry.sphere = create_sdf_element("sphere")
    visual.geometry.sphere.radius = 0.05

    link = create_sdf_element("link")
    link.pose=[0, 0, 0.42, 0, -1.57079633, 0]
    link.mass = 1e-10
    link.inertia.ixx = 0.1
    link.inertia.iyy = 0.1
    link.inertia.izz = 0.1
    sensor = create_sdf_element("sensor")
    sensor.name="fwd_cam"
    sensor.type="camera"
    sensor.camera = create_sdf_element("camera")
    # sensor.reset(mode="camera", with_optional_elements=True)
    sensor.camera.name = "fwd_cam"
    sensor.camera.horizontal_fov = 2.0943951
    sensor.camera.image.width = 1280
    sensor.camera.image.height = 720
    sensor.camera.clip.near = 0.1
    sensor.camera.clip.far = 1000
    sensor.always_on=True
    sensor.update_rate=30
    sensor.visualize=True
    pluginArgs = dict(
        always_on=True,
        update_rate=30,
        cameraName="fwd_cam",
        imageTopicName="raw",
        cameraInfoTopicName="info",
        frameName="fwd_cam_link"
    )
    plugin = create_sdf_element("plugin", pluginArgs)
    plugin.value = pluginArgs
    plugin.name = "camera_controller"
    plugin.filename = "libgazebo_ros_camera.so"
    sensor.add_plugin("camera_controller", plugin)
    link.add_sensor("fwd_cam", sensor)
    model.add_link("fwd_cam_link", link)
    joint = create_sdf_element("joint")
    joint.type="fixed"
    joint.parent="base_link"
    joint.child="fwd_cam_link"
    model.add_joint("fwd_cam_joint",joint)

    for i in range(min(CAM_360_COUNT, len(NUMBERS))):
        name = "cam_" + NUMBERS[i]

        add_360_camera(model, name, CAM_360_WIDTH, CAM_360_HEIGHT, CAM_360_HORIZONTAL_FOV, (i/CAM_360_COUNT)*(2*math.pi))
    
    # print(file)
    file.export_xml(os.path.join(RosPack().get_path("kuav_simulation"), "models/camera_pole/camera_pole.sdf"))
