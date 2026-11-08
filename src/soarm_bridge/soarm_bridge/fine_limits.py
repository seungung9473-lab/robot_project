"""Integer correction command bounds inside both URDF and encoder limits."""
import math
import xml.etree.ElementTree as ET

def correction_limits(urdf,names,centers,signs,gains,lower,upper):
    root=ET.parse(str(urdf)).getroot();joints={j.get('name'):j for j in root.findall('joint')};lo=[];hi=[]
    for name,c,s,g,low,high in zip(names,centers,signs,gains,lower,upper):
        if s not in (-1,1) or g<=-1:raise ValueError('Invalid encoder conversion')
        lim=joints[name].find('limit');a=float(lim.get('lower'));b=float(lim.get('upper'))
        if not math.isfinite(a+b) or a>b:raise ValueError('Invalid URDF limits')
        counts=[c+s*(q*(1+g) if q>0 else q)*4096/(2*math.pi) for q in (a,b)]
        low=max(int(low),math.ceil(min(counts)-1e-8));high=min(int(high),math.floor(max(counts)+1e-8))
        if low>high:raise ValueError('No non-clamping correction interval')
        lo.append(low);hi.append(high)
    if len(lo)!=len(names):raise ValueError('Joint parameter lengths differ')
    return lo,hi
