"""Search complete pick sequences before committing any movement."""
def search_all(pending, completed, placements, evaluate, limit=100):
    trials=[];visits=0
    def visit(todo,done,placed,path):
        nonlocal visits
        if not todo:return path
        for key in todo:
            for stage in evaluate(key,tuple(done),tuple(placed)):
                visits+=1;trials.append(stage.get('review',{}))
                if visits>limit:raise RuntimeError('Batch search limit reached; no execution permitted')
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
