## Installation DIR; TODO: Configure based on your needs
INSTALL_DIR=ros2_openvino

## Create working dir
mkdir -p ${INSTALL_DIR}/src && cd ${INSTALL_DIR}/src

## Clone `ros2_openvino_toolkit` on ROS_DISTRO brach
git clone https://github.com/intel/ros2_openvino_toolkit -b ${ROS_DISTRO}

## Configure environment setup; TODO: Configure based on your needs
cat > ros2_openvino_toolkit/script/modules.conf << EOF
clean=0 - please do NOT use '1'
ros2_src=0
opencv=0
opencl=1
dldt=1
model_zoo=1
librealsense=0
openvino=? - Choice does not seem to matter in this version
other_dependency=1
EOF

## Use current dir to install env instad of ~/code
sed -i 's|~/code|'"$PWD"'|g' ros2_openvino_toolkit/script/environment_setup.sh

## Setup environment
cd ros2_openvino_toolkit/script/
./environment_setup.sh
cd ../..

## Install dependencies
sudo apt-get install -y ros-${ROS_DISTRO}-vision-opencv ros-${ROS_DISTRO}-message-filters ros-${ROS_DISTRO}-image-common

## Add source prototype to '~/.bashrc'
echo -e "\n### ROS2 (${ROS_DISTRO}) OpenVINO" >> ~/.bashrc
echo -e "# source ${PWD}/install/local_setup.bash" >> ~/.bashrc
echo -e "# export InferenceEngine_DIR=${InferenceEngine_DIR}:/opt/openvino_toolkit/dldt/inference-engine/build" >> ~/.bashrc
echo -e "# export CPU_EXTENSION_LIB=${CPU_EXTENSION_LIB}:/opt/openvino_toolkit/dldt/inference-engine/bin/intel64/Release/lib/libcpu_extension.so" >> ~/.bashrc
echo -e "# export GFLAGS_LIB=${GFLAGS_LIB}:/opt/openvino_toolkit/dldt/inference-engine/bin/intel64/Release/lib/libgflags_nothreads.a" >> ~/.bashrc
echo -e "# export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/openvino_toolkit/dldt/inference-engine/bin/intel64/Release/lib" >> ~/.bashrc

## Export ENV
export InferenceEngine_DIR=${InferenceEngine_DIR}:/opt/openvino_toolkit/dldt/inference-engine/build
export CPU_EXTENSION_LIB=${CPU_EXTENSION_LIB}:/opt/openvino_toolkit/dldt/inference-engine/bin/intel64/Release/lib/libcpu_extension.so
export GFLAGS_LIB=${GFLAGS_LIB}:/opt/openvino_toolkit/dldt/inference-engine/bin/intel64/Release/lib/libgflags_nothreads.a
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/openvino_toolkit/dldt/inference-engine/bin/intel64/Release/lib

## Clone `ros2_object_msgs`
git clone https://github.com/intel/ros2_object_msgs

## Ignore `dldt` and `open_model_zoo`
touch dldt/COLCON_IGNORE
touch open_model_zoo/COLCON_IGNORE


## Return to ${INSTALL_DIR}
cd ..

## Compile
colcon build --symlink-install

## Create a symbolic link to the ros2 repo
sudo mkdir -p /opt/openvino_toolkit
sudo ln -sf ${PWD}/src/ros2_openvino_toolkit /opt/openvino_toolkit/


#object segmentation model
cd /opt/openvino_toolkit/dldt/model-optimizer/install_prerequisites
sudo ./install_prerequisites.sh
cd ~/Downloads
wget http://download.tensorflow.org/models/object_detection/mask_rcnn_inception_v2_coco_2018_01_28.tar.gz
tar -zxvf mask_rcnn_inception_v2_coco_2018_01_28.tar.gz
cd mask_rcnn_inception_v2_coco_2018_01_28
#FP32
sudo python3 /opt/openvino_toolkit/dldt/model-optimizer/mo_tf.py --input_model frozen_inference_graph.pb --tensorflow_use_custom_operations_config /opt/openvino_toolkit/dldt/model-optimizer/extensions/front/tf/mask_rcnn_support.json --tensorflow_object_detection_api_pipeline_config pipeline.config --reverse_input_channels --output_dir /opt/openvino_toolkit/models/segmentation/output/FP32
#FP16
sudo python3 /opt/openvino_toolkit/dldt/model-optimizer/mo_tf.py --input_model frozen_inference_graph.pb --tensorflow_use_custom_operations_config /opt/openvino_toolkit/dldt/model-optimizer/extensions/front/tf/mask_rcnn_support.json --tensorflow_object_detection_api_pipeline_config pipeline.config --reverse_input_channels --data_type=FP16 --output_dir /opt/openvino_toolkit/models/segmentation/output/FP16


## Download the optimized Intermediate Representation
cd /opt/openvino_toolkit/open_model_zoo/tools/downloader
sudo python3 downloader.py --name face-detection-adas-0001 --output_dir /opt/openvino_toolkit/models/face_detection/output
sudo python3 downloader.py --name age-gender-recognition-retail-0013 --output_dir /opt/openvino_toolkit/models/age-gender-recognition/output
sudo python3 downloader.py --name emotions-recognition-retail-0003 --output_dir /opt/openvino_toolkit/models/emotions-recognition/output
sudo python3 downloader.py --name head-pose-estimation-adas-0001 --output_dir /opt/openvino_toolkit/models/head-pose-estimation/output
sudo python3 downloader.py --name person-detection-retail-0013 --output_dir /opt/openvino_toolkit/models/person-detection/output
sudo python3 downloader.py --name person-reidentification-retail-0076 --output_dir /opt/openvino_toolkit/models/person-reidentification/output
sudo python3 downloader.py --name vehicle-license-plate-detection-barrier-0106 --output_dir /opt/openvino_toolkit/models/vehicle-license-plate-detection/output
sudo python3 downloader.py --name vehicle-attributes-recognition-barrier-0039 --output_dir /opt/openvino_toolkit/models/vehicle-attributes-recongnition/output
sudo python3 downloader.py --name license-plate-recognition-barrier-0001 --output_dir /opt/openvino_toolkit/models/license-plate-recognition/output
sudo python3 downloader.py --name landmarks-regression-retail-0009 --output_dir /opt/openvino_toolkit/models/landmarks-regression/output
sudo python3 downloader.py --name face-reidentification-retail-0095 --output_dir /opt/openvino_toolkit/models/face-reidentification/output

## Copy label files
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/models/emotions-recognition/output/intel/emotions-recognition-retail-0003/FP32/
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP32/
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP16/
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/segmentation/output/FP32/
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/segmentation/output/FP16/
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/openvino_toolkit/models/vehicle-license-plate-detection/output/intel/vehicle-license-plate-detection-barrier-0106/FP32
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP32/
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP16/

## Cleanup
cd ~/Downloads
rm intel-gmmlib_18.4.1_*
rm intel-igc-core_18.50.1270_*
rm intel-igc-opencl_18.50.1270_*
rm intel-ocloc_19.04.12237_*
rm intel-opencl_19.04.12237_*
rm -r mask_rcnn_inception_v2_coco_2018_01_28
rm mask_rcnn_inception_v2_coco_2018_01_28.tar.gz
