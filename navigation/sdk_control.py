"""Optional SDK compare-and-control. Existing APP requests remain unchanged."""
def version():
    return {'version':'1.0','conditional_task_control':True}

def verify(node, expected):
    if not isinstance(expected,dict) or set(expected)!={'generation','run_id','epoch','map_id'}:
        raise ValueError('SDK task reference requires generation, run_id, epoch and map_id')
    current={'generation':node.session.generation,'run_id':node.reception.engine.run_id,'epoch':node.reception.engine.epoch,'map_id':node.reception.map_id()}
    if expected!=current:raise ValueError('SDK task reference changed; no newer task was stopped')

def stop(node, data):
    with node.lock:
        if '_sdk_expected' in data:
            if set(data)!={'_sdk_expected'}:raise ValueError('Invalid SDK stop fields')
            verify(node,data['_sdk_expected'])
        # A paused mission still owns resumable intent; cancel must end it too.
        if getattr(node.reception.engine,'phase',None)=='paused':node.reception.engine.cancel()
        node.stop()
        return {'stopped':True}

def mission(node, data):
    if set(data)-{'reception_command','_sdk_expected'}:raise ValueError('Invalid SDK mission fields')
    with node.lock:
        if '_sdk_expected' in data:verify(node,data['_sdk_expected'])
        return node.reception.command(data['reception_command'])
