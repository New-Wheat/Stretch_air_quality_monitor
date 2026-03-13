import socket
import json
from stretch_body.wacc import Wacc
from stretch_body.transport import unpack_float_t,unpack_uint16_t,unpack_uint8_t

class WaccSensor(Wacc):
    """
    This class extends the Wacc class with custom data.
    """
    def __init__(self,verbose=False):
        Wacc.__init__(self, ext_status_cb=self.ext_unpack_status)
        self.status['sensor'] = {'temperature':0, 'humidity':0, 'TVOC':0, 'eCO2':0, 'state':0}
        self.valid_firmware_protocol = 'p3'

        self.udp_ip = "127.0.0.1"
        self.udp_port = 9222    # wacc
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    def ext_unpack_status(self,s):
        """
        s: byte array to unpack
        return: number of bytes unpacked
        """
        sidx = 0
        try:
            self.status['sensor']['temperature'] = unpack_float_t(s[sidx:])
            sidx += 4
            self.status['sensor']['humidity'] = unpack_float_t(s[sidx:])
            sidx += 4
            self.status['sensor']['TVOC'] = unpack_uint16_t(s[sidx:])
            sidx += 2
            self.status['sensor']['eCO2'] = unpack_uint16_t(s[sidx:])
            sidx += 2
            self.status['sensor']['state'] = unpack_uint8_t(s[sidx:])
            sidx += 1
            
            json_data = json.dumps(self.status['sensor'])
            self.sock.sendto(json_data.encode('utf-8'), (self.udp_ip, self.udp_port))
            
        except Exception:
            pass 
            
        return sidx
