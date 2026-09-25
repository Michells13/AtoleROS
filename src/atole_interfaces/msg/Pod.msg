# Un pod con su pose estimada (completion -> superquadric -> PCA).
int32 id
int32 detection_id
string status                      # ok | out_of_workspace | not_selected | error: <motivo>
string class_name                  # "pod" hoy; clases de madurez con el modelo de ripeness
float32 score                      # confianza de la detección
float32 pose_confidence
bool in_workspace                  # pasó el filtro de área de trabajo
bool reachable                     # el pre-pick tiene IK de frente (lo rellena pod_selector)
# Todo en base_link:
geometry_msgs/Point grasp_bottom
geometry_msgs/Point grasp_midpoint
geometry_msgs/Point tip
geometry_msgs/Point centroid
geometry_msgs/Vector3 axis         # de la base a la punta
geometry_msgs/Quaternion orientation
float32 length_m
float32 width_m
uint32 n_partial
uint32 n_completed
