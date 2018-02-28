#!/usr/bin/env python
import rospy
import time,os
import re
import yaml
from collections import OrderedDict
import perception
import numpy as np
from skimage import img_as_float
from autolab_core import YamlConfig
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from moveit_gpd_pick_object.srv import depth_GQCNNPredict
from gqcnn import GQCNN
from gqcnn.msg import GQCNNGrasp, BoundingBox

def read_cfgfile(filename):
    fh = open(filename, 'r')
    file_contents = fh.read()
    # Replace !include directives with content
    config_dir = os.path.split(filename)[0]
    include_re = re.compile('^!include\s+(.*)$', re.MULTILINE)
    def include_repl(matchobj):
        fname = os.path.join(config_dir, matchobj.group(1))
        with open(fname) as f:
            return f.read()
    while re.search(include_re, file_contents): # for recursive !include
        file_contents = re.sub(include_re, include_repl, file_contents)

    # Read in dictionary
    class OrderedLoader(yaml.Loader):
        pass
    OrderedLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
        lambda loader, node: OrderedDict(loader.construct_pairs(node)))
    config=yaml.load(file_contents, OrderedLoader)
    # Convert functions of other params to true expressions
    for k in config.keys():
        if type(config[k]) is str and len(config[k]) > 2 and config[k][1] == '!':
            config[k] = eval(config[k][2:-1])
    fh.close()
    return config

class GPD_depth_GQCNN(object):
    def __init__(self, config):
        self.bridge = CvBridge()
        sensor = "realsense_r200"
        # Connect image topic, default use realsense
        img_topic = "/camera/rgb/image_rect_color"
        depth_topic = "/camera/depth_registered/sw_registered/image_rect"
        camera_info = "/camera/depth_registered/sw_registered/camera_info"
        if sensor == "kinect":
            img_topic = "/kinect2/qhd/image_color"
            depth_topic = "/kinect2/qhd/image_depth_rect"
            camera_info = "/kinect2/qhd/camera_info"

        self.image_sub = rospy.Subscriber(img_topic, Image, self.image_callback)
        self.depth_sub = rospy.Subscriber(depth_topic, Image, self.depth_callback)
        self.color_image = None
        self.depth_image = None
        self.camera_info = rospy.wait_for_message(camera_info, CameraInfo, timeout=2)
        self.bounding_box = BoundingBox()
        self.bounding_box.maxX = 4400
        self.bounding_box.maxY = 3000
        self.bounding_box.minX = -1
        self.bounding_box.minY = -1
        # store parameters
        self._config = config
        self._gripper_width = config['policy']['gripper_width']
        self._crop_height = config['policy']['crop_height']
        self._crop_width = config['policy']['crop_width']
        self._sampling_config = config['policy']['sampling']
        self._gqcnn_model_dir = config['policy']['gqcnn_model']
        self._logging_dir = None
        if 'logging_dir' in config.keys():
            self._logging_dir = config['logging_dir']
            self._policy_dir = self._logging_dir
            if not os.path.exists(self._logging_dir):
                os.mkdir(self._logging_dir)
        sampler_type = self._sampling_config['type']
        # init GQ-CNN
        self.gqcnn = GQCNN.load(self._gqcnn_model_dir)
        self.gqcnn_im_height = self.gqcnn.im_height
        self.gqcnn_im_width = self.gqcnn.im_width
        self.gqcnn_num_channels = self.gqcnn.num_channels
        self.gqcnn_pose_dim = self.gqcnn.pose_dim
        # open tensorflow session for gqcnn
        self.gqcnn.open_session()

    def __del__(self):
        try:
            self.gqcnn.close_session()
        except:
            pass
        del self

    def image_callback(self, data):
        self.color_image = data

    def depth_callback(self, data):
        self.depth_image = data

    def service_predict(self,req):
        # The code not support uint depth, need to convert to float
        converted_depth = self.cv_bridge.imgmsg_to_cv2(self.depth_image, desired_encoding="passthrough")
        converted_depth = img_as_float(converted_depth)

        converted_image = self.cv_bridge.imgmsg_to_cv2(self.color_image, "rgb8")

        # wrap the camera info in a perception CameraIntrinsics object
        camera_intrinsics = perception.CameraIntrinsics(raw_camera_info.header.frame_id,
                                                        raw_camera_info.K[0],
                                                        raw_camera_info.K[4], raw_camera_info.K[2],
                                                        raw_camera_info.K[5], raw_camera_info.K[1],
                                                        raw_camera_info.height, raw_camera_info.width)

        # Create wrapped Perception RGB and Depth Images by unpacking the ROS Images using CVBridge ###
        color_image = perception.ColorImage(converted_image, frame=camera_intrinsics.frame)
        depth_image = perception.DepthImage(converted_depth, frame=camera_intrinsics.frame)
        # Hongzhuo:
        color_image = color_image.inpaint(rescale_factor=self.cfg['inpaint_rescale_factor'])
        depth_image = depth_image.inpaint(rescale_factor=self.cfg['inpaint_rescale_factor'])

        min_x = bounding_box.minX
        min_y = bounding_box.minY
        max_x = bounding_box.maxX
        max_y = bounding_box.maxY

        # contain box to image->don't let it exceed image height/width bounds
        no_pad = False
        if min_x < 0:
            min_x = 0
            no_pad = True
        if min_y < 0:
            min_y = 0
            no_pad = True
        if max_x > rgbd_image.width:
            max_x = rgbd_image.width
            no_pad = True
        if max_y > rgbd_image.height:
            max_y = rgbd_image.height
            no_pad = True

        centroid_x = (max_x + min_x) / 2
        centroid_y = (max_y + min_y) / 2

        # add some padding to bounding box to prevent empty pixel regions when the image is
        # rotated during grasp planning
        if not no_pad:
            width = (max_x - min_x) + self.cfg['width_pad']
            height = (max_y - min_y) + self.cfg['height_pad']
        else:
            width = (max_x - min_x)
            height = (max_y - min_y)
        cropped_camera_intrinsics = camera_intrinsics.crop(height, width, centroid_y, centroid_x)
        #cropped_rgbd_image = rgbd_image.crop(height, width, centroid_y, centroid_x)
        depth_im = depth_image.crop(height, width,centroid_y,center_j=centroid_x)

        # create an RGBDImageState with the cropped RGBDImage and CameraIntrinsics
        #image_state = RgbdImageState(cropped_rgbd_image, cropped_camera_intrinsics)
        #depth_im=image_state.rgbd_im.depth;
        grasp_position=np.array([req.gripper_position.x,req.gripper_position.y,req.gripper_position.z])
        grasp_depth=np.sqrt(np.sum(np.square(grasp_position)))
        pose_tensor = np.zeros([1, self.gqcnn_pose_dim])
        image_tensor = np.zeros([1, self.gqcnn_im_height, self.gqcnn_im_width, self.gqcnn_num_channels])
        pose_tensor = grasp_depth

        scale = float(self.gqcnn_im_height) / self._crop_height
        depth_im_scaled = depth_im.resize(scale)
        im_tf = im_tf.crop(self.gqcnn_im_height, self.gqcnn_im_width)
        image_tensor=im_tf.raw_data
        grasp=Grasp2D(0.0,0.0,pose_tensor,state.camera_intr)
        output_arr = self.gqcnn.predict(image_tensor, pose_tensor)
        q_values = output_arr[:,-1]

        if self.config['vis']['grasp_candidates']:
            # display each grasp on the original image, colored by predicted success
            vis.figure(size=(FIGSIZE,FIGSIZE))
            vis.imshow(depth_im)
            vis.grasp(grasp, scale=1.5, show_center=False, show_axis=True,
                          color=plt.cm.RdYlBu(q_values))
            vis.title('Sampled grasps')
            self.show('grasp_candidates.png')

        return q_values>0.9

if __name__ == '__main__':
    rospy.init_node('GPD_depth_GQCNN', anonymous=True)
    filename=rospy.get_param("cfg_file","/homeL/demo/ws_grasp/src/upstream/gqcnn/cfg/ros_nodes/grasp_planner_node.yaml")
    cfg_config=read_cfgfile(filename)
    depth_gqcnn=GPD_depth_GQCNN(cfg_config)
    service=rospy.Service("depth_gqcnn_predict",depth_GQCNNPredict,depth_gqcnn.service_predict)
    rospy.spin()
