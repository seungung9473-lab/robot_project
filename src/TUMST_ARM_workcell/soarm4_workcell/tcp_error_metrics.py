"""Model-based TCP error propagation. Joint contributions are linear estimates."""
import numpy as np

def tcp_errors(fk,goal,actual,target):
    goal=np.asarray(goal,float);actual=np.asarray(actual,float);target=np.asarray(target,float)
    if goal.shape!=(4,) or actual.shape!=(4,) or target.shape!=(3,) or not np.all(np.isfinite(np.r_[goal,actual,target])):raise ValueError('Expected finite 4-joint angles and 3D target')
    pg=np.asarray(fk(goal),float);pa=np.asarray(fk(actual),float);total=(pa-target)*1000;tracking=(pa-pg)*1000;residual=(pg-target)*1000
    step=1e-6;J=np.column_stack([(np.asarray(fk(goal+np.eye(4)[i]*step))-np.asarray(fk(goal-np.eye(4)[i]*step)))/(2*step) for i in range(4)])
    contributions=(J*(actual-goal)[None,:]).T*1000
    return {'goal_fk_table_m':pg.tolist(),'actual_fk_table_m':pa.tolist(),'model_tcp_error_mm':total.tolist(),'model_tcp_error_norm_mm':float(np.linalg.norm(total)),'joint_tracking_tcp_error_mm':tracking.tolist(),'joint_tracking_tcp_error_norm_mm':float(np.linalg.norm(tracking)),'ik_position_residual_mm':residual.tolist(),'per_joint_linear_tcp_error_mm':contributions.tolist(),'per_joint_linear_tcp_error_norm_mm':np.linalg.norm(contributions,axis=1).tolist(),'linearization_remainder_mm':(tracking-contributions.sum(0)).tolist()}
