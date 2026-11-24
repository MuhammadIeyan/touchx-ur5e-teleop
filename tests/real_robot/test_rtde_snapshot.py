import sys,struct,unittest
from pathlib import Path
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from real_robot.rtde_snapshot import FIELDS,FORMATS,decode,send_read_request,receive_exact
class SnapshotTests(unittest.TestCase):
    def test_no_command_packets(self):
        sock=Mock()
        for kind in (73,85,77,80,0):
            with self.assertRaises(ValueError):send_read_request(sock,kind)
        sock.sendall.assert_not_called()
    def test_recipe_decode_and_corruption(self):
        data=b'\x01'
        for kind in FIELDS.values():
            fmt=FORMATS[kind];n=int(fmt[:-1]) if len(fmt)>1 else 1
            data+=struct.pack('>'+fmt,*([0]*n))
        self.assertEqual(decode(data,1)['tcp_offset'],[0.]*6)
        for bad in (data[:-1],data+b'\0',b'\x02'+data[1:]):
            with self.assertRaises(ValueError):decode(bad,1)
    def test_partial_tcp_reads(self):
        sock=Mock();sock.recv.side_effect=[b'ab',b'c']
        self.assertEqual(receive_exact(sock,3),b'abc')
