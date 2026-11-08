"""Search complete pick sequences before committing any movement."""
import time

class SearchBudgetExceeded(RuntimeError):
    def __init__(self, trials):
        super().__init__('Batch search limit reached; no execution permitted')
        self.trials = list(trials)

def search_all(pending, completed, placements, evaluate, limit=100, time_limit=None):
    trials=[];visits=0;started=time.monotonic()
    def visit(todo,done,placed,path):
        nonlocal visits
        if not todo:return path
        for key in todo:
            for stage in evaluate(key,tuple(done),tuple(placed)):
                visits+=1;trials.append(stage.get('review',{}))
                if visits>limit or (time_limit is not None and time.monotonic()-started>time_limit):raise SearchBudgetExceeded(trials)
                if not stage.get('success'):continue
                result=visit([k for k in todo if k!=key],done+[key],placed+([] if (isinstance(stage['placement'],dict) and stage['placement'].get('removed_from_workcell')) else [stage['placement']]),path+[stage])
                if result is not None:return result
        return None
    return visit(list(pending),list(completed),list(placements),[]),trials

def require_complete(plan):
    if not plan.get('all_stages_ready') or len(plan.get('stages',[]))!=len(plan.get('requested_objects',[])):
        raise RuntimeError('Every requested stage must pass before any execution')
    if {s['object'] for s in plan['stages']}!=set(plan['requested_objects']):
        raise RuntimeError('Batch stages do not cover the requested objects exactly')
    if plan.get('finish_above'):
        import math
        if plan.get('return_home_between_stages') is not False:
            raise RuntimeError('Chained batch cannot return HOME between stages')
        previous=None
        for stage in plan['stages']:
            start=stage.get('start_q_rad',[]);end=stage.get('exit_q_rad',[])
            if any(len(q)!=4 or any(not math.isfinite(v) for v in q) for q in (start,end)):
                raise RuntimeError('Chained batch requires finite reviewed start/exit poses')
            if previous is not None and max(abs(a-b) for a,b in zip(start,previous))>1e-6:
                raise RuntimeError('Chained batch starts do not match previous exits')
            previous=end


def invariant_source_overlap(error):
    """Rigid co-rotation cannot resolve source object/object overlap at lift=0.

    Do not prune robot-link collisions, later lift collisions, or uncertain errors.
    A different object order can still resolve the same source overlap.
    """
    import ast
    prefix='Attached lift collision at 0.0: '
    if not isinstance(error,str) or not error.startswith(prefix):return False
    try:pairs=ast.literal_eval(error[len(prefix):])
    except (ValueError,SyntaxError):return False
    return bool(pairs) and isinstance(pairs,list) and all(isinstance(p,(list,tuple)) and len(p)==2 and 'scanned_block' in p and any(isinstance(k,str) and k.startswith('inclined_obstacle_object_') for k in p) for p in pairs)
