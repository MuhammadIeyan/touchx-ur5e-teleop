#!/usr/bin/env python3
"""Output-only RTDE configuration snapshot. No input recipe or robot commands.
Protocol reference: https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html
"""
import argparse,ipaddress,json,math,socket,struct,time
FIELDS={'timestamp':'DOUBLE','actual_TCP_pose':'VECTOR6D','tcp_offset':'VECTOR6D',
        'payload':'DOUBLE','payload_cog':'VECTOR3D','actual_TCP_force':'VECTOR6D',
        'robot_mode':'INT32','safety_mode':'INT32','runtime_state':'UINT32'}
FORMATS={'DOUBLE':'d','VECTOR6D':'6d','VECTOR3D':'3d','INT32':'i','UINT32':'I'}

def receive_exact(sock,count):
    data=b''
    while len(data)<count:
        part=sock.recv(count-len(data))
        if not part:raise ConnectionError('RTDE closed')
        data+=part
    return data

def receive(sock,expected):
    deadline=time.monotonic()+3
    while time.monotonic()<deadline:
        size,kind=struct.unpack('>HB',receive_exact(sock,3))
        if not 3<=size<=4096:raise ValueError('Invalid RTDE packet size')
        data=receive_exact(sock,size-3)
        if kind==expected:return data
        if kind!=77:raise ValueError(f'Unexpected RTDE packet type {kind}')
    raise TimeoutError('RTDE response timeout')

def send_read_request(sock,kind,payload=b''):
    # Allow only version negotiation, output setup and output streaming start.
    # START starts telemetry delivery, not a robot program.
    if kind not in (86,79,83):raise ValueError('Non-read-only RTDE request prohibited')
    sock.sendall(struct.pack('>HB',len(payload)+3,kind)+payload)

def decode(data,recipe):
    if not data or data[0]!=recipe:raise ValueError('Unexpected output recipe')
    result={};offset=1
    for name,kind in FIELDS.items():
        fmt='>'+FORMATS[kind];size=struct.calcsize(fmt)
        if offset+size>len(data):raise ValueError('Truncated output data')
        values=struct.unpack_from(fmt,data,offset);offset+=size
        if not all(math.isfinite(v) for v in values):raise ValueError('Nonfinite telemetry')
        result[name]=list(values) if len(values)>1 else values[0]
    if offset!=len(data):raise ValueError('Extra output data')
    return result

def snapshot(ip):
    address=ipaddress.IPv4Address(ip)
    if address.is_unspecified or address.is_multicast or address.is_loopback:raise ValueError('Robot LAN IPv4 required')
    with socket.create_connection((str(address),30004),timeout=3) as sock:
        sock.settimeout(3)
        send_read_request(sock,86,struct.pack('>H',2))
        if receive(sock,86)!=b'\x01':raise RuntimeError('RTDE v2 rejected')
        send_read_request(sock,79,struct.pack('>d',10.)+','.join(FIELDS).encode())
        recipe=receive(sock,79)
        if not recipe or recipe[0]==0 or recipe[1:].decode().split(',')!=list(FIELDS.values()):
            raise RuntimeError('RTDE output recipe unavailable or mismatched')
        send_read_request(sock,83)
        if receive(sock,83)!=b'\x01':raise RuntimeError('Output stream rejected')
        result=decode(receive(sock,85),recipe[0])
        result['source']='output-only RTDE';result['robot_ipv4']=str(address);result['wall_time_ns']=time.time_ns()
        return result

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('robot_ipv4');a=p.parse_args()
    print(json.dumps(snapshot(a.robot_ipv4),indent=2,allow_nan=False))
if __name__=='__main__':main()
