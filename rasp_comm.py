from email.policy import default
from encodings import utf_8
import serial
import time

class robo_brain() :
    def __init__(self) :
        self.ser = serial.Serial(port='/dev/cu.usbserial-0001', baudrate=115200, timeout=1)
        self.ser.reset_input_buffer()

        self.imu_a = [0., 0., 0.]
        self.imu_g = [0., 0., 0.]
        self.imu_q = [0., 0., 0., 0.]
        self.gps_c = [0., 0.]
        self.batt = 0

        time.sleep(1)
        while self.ser.in_waiting:
            self.decode_input()

    def decode_input(self):
        self.ser.in_waiting
        line = self.ser.readline().rstrip()
        words = str(line).split(',')
        
        match words[0]:
            case "b'GPS_G":
                self.gps_c[0] = words[1]
                self.gps_c[1] = words[2]
                print(self.gps_c)
            case "b'IMU_A":
                self.imu_a[0] = words[1]
                self.imu_a[1] = words[2]
                self.imu_a[2] = words[3]
                print(self.imu_a)
            case "b'IMU_G":
                self.imu_g[0] = words[1]
                self.imu_g[1] = words[2]
                self.imu_g[2] = words[3]
                print(self.imu_g)
            case "b'IMU_Q":
                self.imu_q[0] = words[1]
                self.imu_q[1] = words[2]
                self.imu_q[2] = words[3]
                self.imu_q[3] = words[4]
                print(self.imu_q)
            case "b'BATT":
                self.batt = words[1].strip('\'')
                print(self.batt)
            case _:
                test = str(line).split()
                match test[0]:
                    case "b'Error":
                        print(line.decode('utf-8'))
                    #case "b'BUMP":
                     #   print(line.decode('utf-8'))
                    case "b'Lid":
                        print(line.decode('utf-8'))
                    #case _:
                        #print(line)
                
    def get_gps(self) :
        self.ser.write(b"0\n")
        self.decode_input()
        
    def get_imu_accel(self) :
        self.ser.write(b"1\n")
        self.decode_input()

    def get_imu_gyro(self) :
        self.ser.write(b"2\n")
        self.decode_input()

    def get_imu_quat(self) :
        self.ser.write(b"3\n")
        self.decode_input()

    def unlock_lockbox(self) :
        self.ser.write(b"4\n")

    def poll_box(self) :
        self.ser.write(b"6\n")
        self.decode_input()

    def get_batt(self) :
        self.ser.write(b"5\n")
        self.decode_input()

def main(args=0) :
    apdr = robo_brain()

    for x in range(6):
        apdr.get_batt()
    
    
if __name__ == '__main__' :
    main()