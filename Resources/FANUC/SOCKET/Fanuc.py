import socket
import sys
import subprocess


class Fanuc:
    def __init__(self, host, port):
        self.port = port
        self.host = host
        #initializing the xml msg
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.host,self.port))
        #196 = acc
        #195 = speed
        #194 = num of pose

    def convert_frame_to_fanuc_string(self,frame):
        assert all(isinstance(item, (float, int)) for item in
                   frame), 'Argument of wrong type! Please use list of Float/Int.'
        assert len(frame) == 6, 'Please add all the values of xyzabc in the argument'
        formatted_numbers = [f"{n:+09.3f}" for n in frame]

        # Joining all formatted numbers into a single string separated by commas
        result = ",".join(formatted_numbers)
        return result

    def set_base(self,base_frame):
        assert all(isinstance(item, (float, int)) for item in base_frame), 'Argument of wrong type! Please use list of Float/Int.'
        assert len(base_frame) == 6, 'Please add all the values of xyzabc in the argument'
        req_number = 1
        req_number = f"{req_number:03d}"
        self.client_socket.send(req_number.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:',data)

        string = self.convert_frame_to_fanuc_string(base_frame)
        self.client_socket.send(string.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:',data)

    def set_tcp(self,tcp_frame):
        assert all(isinstance(item, (float, int)) for item in tcp_frame), 'Argument of wrong type! Please use list of Float/Int.'
        assert len(tcp_frame) == 6, 'Please add all the values of xyzabc in the argument'
        req_number = 2
        req_number = f"{req_number:03d}"
        self.client_socket.send(req_number.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:',data)

        string = self.convert_frame_to_fanuc_string(tcp_frame)
        self.client_socket.send(string.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:',data)

    def set_acc_speed(self, acceleration, speed):
        assert isinstance(acceleration, (float, int)), 'Argument of wrong type! Please use Float/Int.'
        assert isinstance(speed, (float, int)), 'Argument of wrong type! Please use Float/Int.'

        req_number = 3
        req_number = f"{req_number:03d}"
        self.client_socket.send(req_number.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)

        string = f"{acceleration:+09.3f}"
        self.client_socket.send(string.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)

        string = f"{speed:+09.3f}"
        self.client_socket.send(string.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)

    def move_j(self,poses):

        if len(poses) == 6 and all(isinstance(item, (float, int)) for item in poses):
            # Adding an extra axis
            poses = [poses]
            print("List with an added axis:", poses)
        else:
            print("List does not meet the conditions")

        for pose in poses:
            assert all(isinstance(item, (float, int)) for item in pose), 'Argument of wrong type! Please use list of Float/Int.'
            assert len(pose) == 6, 'Please add all the values of xyzabc in each pose in the argument'
        req_number = 4
        req_number = f"{req_number:03d}"
        self.client_socket.send(req_number.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)

        length = len(poses)
        three_digit_string = f"{length:03d}"
        print(three_digit_string)
        self.client_socket.send(three_digit_string.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)


        for count,pose in enumerate(poses):
            string = self.convert_frame_to_fanuc_string(pose)
            # string = f"{pose:+09.3f}"
            self.client_socket.send(string.encode('utf-8'))
            data = self.client_socket.recv(1024)
            print('data:', data)

    def move_l(self,poses):
        if len(poses) == 6 and all(isinstance(item, (float, int)) for item in poses):
            # Adding an extra axis
            poses = [poses]
            print("List with an added axis:", poses)
        else:
            print("List does not meet the conditions")
        for pose in poses:
            assert all(isinstance(item, (float, int)) for item in pose), 'Argument of wrong type! Please use list of Float/Int.'
            assert len(pose) == 6, 'Please add all the values of xyzabc in each pose in the argument'
        req_number = 5
        req_number = f"{req_number:03d}"
        self.client_socket.send(req_number.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)

        length = len(poses)
        three_digit_string = f"{length:03d}"
        print(three_digit_string)
        self.client_socket.send(three_digit_string.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)


        for count,pose in enumerate(poses):
            string = self.convert_frame_to_fanuc_string(pose)
            # string = f"{pose:+09.3f}"
            self.client_socket.send(string.encode('utf-8'))
            data = self.client_socket.recv(1024)
            print('data:', data)

    def move_lin(self,poses):
        self.move_l(poses)

    def get_current_state(self):
        tcp_pose = []
        joints_real = []
        joints_fanuc = []

        req_number = 6
        req_number = f"{req_number:03d}"
        self.client_socket.send(req_number.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)


        string = "0"
        self.client_socket.send(string.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('tcp data:', data)

        tcp_pose = [float(item.strip()) for item in data.decode('utf-8').split(',')]
        tcp_pose_temp = tcp_pose.copy()
        tcp_pose[3] = tcp_pose_temp[5]
        tcp_pose[5] = tcp_pose_temp[3]

        string = "0"
        self.client_socket.send(string.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('joint fanuc data:', data)

        joints_fanuc = [float(item.strip()) for item in data.decode('utf-8').split(',')]

        joints_real = joints_fanuc.copy()
        joints_real[2] = joints_real[2]+joints_real[1]



        return tcp_pose,joints_real,joints_fanuc

    def set_digout(self,number,value):
        #TODO: Have to implement
        assert isinstance(number, int), 'Argument of wrong type! Please use Int for number.'
        assert isinstance(value, bool), 'Argument of wrong type! Please use bool for value.'

        req_number = 7
        req_number = f"{req_number:03d}"
        self.client_socket.send(req_number.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)


        three_digit_string = f"{number:03d}"
        self.client_socket.send(three_digit_string.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)

        if value == False:
            self.client_socket.send('0'.encode('utf-8'))
            data = self.client_socket.recv(1024)
            print('data:', data)
        else:
            self.client_socket.send('1'.encode('utf-8'))
            data = self.client_socket.recv(1024)
            print('data:', data)








    def call_subroutine(self, name):
        # TODO: Have to implement
        req_number = 8
        req_number = f"{req_number:03d}"
        self.client_socket.send(req_number.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)

        three_digit_string = f"{len(name):03d}"
        self.client_socket.send(three_digit_string.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)

        self.client_socket.send(name.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)

    def set_weld_proc_sched(self,procedure,schedule):
        #TODO: Have to implement
        assert isinstance(procedure, int), 'Argument of wrong type! Please use Int for number.'
        assert procedure > 0, 'Number must be positive.'

        assert isinstance(schedule, int), 'Argument of wrong type! Please use Int for value.'
        assert schedule > 0, 'Value must be positive.'

        req_number = 9
        req_number = f"{req_number:03d}"
        self.client_socket.send(req_number.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)



        self.client_socket.send(str(procedure).encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)


        self.client_socket.send(str(schedule).encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)

    def move_l_weld(self,poses):
        if len(poses) == 6 and all(isinstance(item, (float, int)) for item in poses):
            # Adding an extra axis
            poses = [poses]
            print("List with an added axis:", poses)
        else:
            print("List does not meet the conditions")
        for pose in poses:
            assert all(isinstance(item, (float, int)) for item in
                       pose), 'Argument of wrong type! Please use list of Float/Int.'
            assert len(pose) == 6, 'Please add all the values of xyzabc in each pose in the argument'
        req_number = 10
        req_number = f"{req_number:03d}"
        self.client_socket.send(req_number.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)

        length = len(poses)
        three_digit_string = f"{length:03d}"
        print(three_digit_string)
        self.client_socket.send(three_digit_string.encode('utf-8'))
        data = self.client_socket.recv(1024)
        print('data:', data)

        for count, pose in enumerate(poses):
            string = self.convert_frame_to_fanuc_string(pose)
            # string = f"{pose:+09.3f}"
            self.client_socket.send(string.encode('utf-8'))
            data = self.client_socket.recv(1024)
            print('data:', data)

