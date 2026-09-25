# Estado de la misión (lo publica mission_manager).
std_msgs/Header header
string state                       # IDLE | HOME2 | DETECT | ESTIMATE | SELECT | EIH_REFINE | PREPICK | PICK | CLOSE | ROTATE | RETREAT | RELEASE | ERROR
string mode                        # all | next | ""
bool step_mode                     # paso a paso con confirmación
bool awaiting_confirmation
string confirmation_prompt         # qué movimiento espera confirmación
int32 current_pod_id
int32 pods_done
int32 pods_remaining
string eih_strategy                # single | fused_2view | ppp
string message
