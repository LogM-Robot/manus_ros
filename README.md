# manus_ros1
ROS 1 Node for Publishing Manus VR Glove Messages to ROS 1
## Dependency on the MANUS SDK
Note that this project does not include the MANUS SDK itself - you must download this yourself with a developer account from https://www.manus-meta.com/resources/downloads. We are not affiliated with MANUS directly, nor do we publish or maintain the official SDK. 

Extract the ZIP into the /ext folder of this source tree. 

Properly installed, the MANUS SDK should be in:

/ext/MANUS_Core_2.4.X.X_SDK

## ROS 1 Messages and Node Functions
This node is super simple. It does two things:

1) Launches a minimal client to connect to Manus Core (running on a Windows PC on the same network) and begins receiving hand poses for the left and right gloves via the Manus SDK.
2) Re-broadcasts these poses as ROS1 PoseArray messages containing the positions and orientations reported by the Manus client.

These messages are as follows:

- `manus_left`: A ROS 1 PoseArray message containing positions and rotations for the left hand
- `manus_right`: A ROS 1 PoseArray message containing positions and rotations for the right hand

This data is provided verbatim in exactly the same order and format as received from the Manus client with no additional transforms or logic. If needed, you can modify your own fork of this node to do that, but we'd actually recommend just doing it in the subscriber to these messages so that you are not convoluting the data being reported from the Manus SDK. 

Currently, the node is on a 50hz timer, but you can experiment with increasing or decreasing this as needed.

