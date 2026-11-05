"""Separate semantic identity from the optional legacy colour-routing task."""

def identity_routing(identity,policy,*,identity_only=False,color_parts=()):
    if identity_only:
        return {'label':identity['label'],'color':'not_evaluated','color_support_fraction':None,
                'destination_pad_id':None,'route_status':'awaiting_expiry_date',
                'color_routing_evaluated':False}
    import cv2,numpy as np
    colors=np.vstack(color_parts)
    hsv=cv2.cvtColor(colors.astype('uint8').reshape(-1,1,3),cv2.COLOR_RGB2HSV).reshape(-1,3)
    h=hsv[hsv[:,1]>55,0];red=float(np.mean((h<12)|(h>170))) if len(h) else 0.
    green=float(np.mean((h>30)&(h<95))) if len(h) else 0.;gray=float(np.mean(hsv[:,1]<45))
    color='gray' if gray>.7 else ('red' if red>.7 else ('green' if green>.7 else 'unknown'))
    label=identity['label'];expected='red' if '빨' in label else ('green' if '초록' in label or '녹색' in label else ('gray' if '회색' in label or '회' in label else 'unknown'))
    if color=='unknown' or color!=expected:raise RuntimeError('Gemini label and measured segmented RGB colour disagree')
    return {'label':label,'color':color,'color_support_fraction':max(red,green,gray),
            'destination_pad_id':policy['routes'].get(color,{}).get('pad_id'),
            'route_status':'assigned' if color in policy['routes'] else 'unassigned_keep_as_obstacle',
            'color_routing_evaluated':True}
