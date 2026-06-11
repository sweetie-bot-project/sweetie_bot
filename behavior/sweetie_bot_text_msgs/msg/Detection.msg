# Represents object detected by spartial visual system.
#
#  * id --- unique id assotiate with each new detected object. Do not change until object is visible.
#  * label --- long time object identificator. Identifies specific object in given class. Can be left empty if object is not recognized. Examples: human name, AprilTag id and so on.
#  * type --- object (detector) type.
#  * attributes, values --- atrribute and value pair to identify object properties: (color, red), (smiling, no) and so on.
#  * score --- probability. 
#  * pose, box --- object pose and bounding box in header.frame_id
#
#  Possible future changes:
#  1. Merge (id, label, type) in separate messge.
#  2. Add parent field (id, label, type) to represent object relations.
#  3. Add image rect.
#
std_msgs/Header header

int32 id
string label
string type
string[] attribute
string[] value
float32 score

geometry_msgs/Pose pose
geometry_msgs/Vector3 box

#  * keypoints_3d --- per-keypoint metric 3D positions in header.frame_id (e.g. a COCO-17 body skeleton);
#    a non-finite (NaN) point marks an absent/low-confidence joint. Empty if the object has no skeleton.
#  * keypoint_layout --- names the keypoint ordering, e.g. "coco17".
geometry_msgs/Point[] keypoints_3d
string keypoint_layout
