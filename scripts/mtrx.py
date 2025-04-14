import socket

# mtrx_socket.settimeout(1)

class MTRX:

    def send(self, command):
        command = command + "\n"
        self.mtrx_socket.send(command.encode('utf-8'))

    def recv(self):
        receiving = True
        response = ""
        while receiving:
            try:
                received = self.mtrx_socket.recv(1024)
            except socket.timeout:
                receiving = False
            response += received.decode('utf-8')
            if not received or len(received) < 1024:
                receiving = False

        response = response.strip()
        # print(response)
        return response

    def __del__(self):
        self.mtrx_socket.close()

    def __init__(self):
        port = 3334
        address = '172.27.2.197'

        self.mtrx_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.mtrx_socket.connect((address, port))

        # Test connection
        self.send("*idn?")
        self.recv()

        # Generator state parameters
        # INSTrument:STAte:ADD 2,<name>,<playback source count>,<output count>,<Bandwidth [in MHz, optional]>,<rf connector type [optional]>
        # Bandwidth: 100 = 100MHz, 200 = 200 MHz (default)
        # Rf Connector type: 0 = Rf Out (non-duplex connectors), 1 = Rf In/Out (duplex connectors, default)
        # self.send("INSTrument:STAte:ADD 2,WGN_50Mhz,1,1")

        # Get list of states
        # Syntax:
        # INSTrument:STAte:LIST?
        # Description:
        # Get list of available states. Return value specifies each state’s id, mode, and name. The mode is specified as follows:
        # 1 Analyzer
        # 2 Generator
        # 3 Pass Through
        # 4 Transceiver (Generator and Analyzer)
        # Example:
        # // List states
        # inst:state:list?
        # 1,1,MyAnalyzerState
        # self.send("INSTrument:STAte:LIST?")
        # self.recv()

        # Syntax:
        # INSTrument:STAte:SELect <id>
        # Description:
        # Select a state specified with an id. The selected state is loaded into hardware.
        self.state_id = 50
        self.send(f"INSTrument:STAte:SELect {self.state_id}")

        self.send("INSTrument:STAte:OUTPut:IDS?")
        self.output_id = self.recv()
        print(f"Output ID: {self.output_id}")
        

    def get_output_level(self):
        # Get the average output level
        # This command is for the generator, transceiver, and pass-through mode.
        # Syntax:
        # INSTrument:OUTPut:LEVel:GET? <output id>
        # Description:
        # Get the average output level of the specified output port in dBm.
        # Example:
        # // get output 1 level
        # inst:output:level:get? 1
        # -20
        self.send(f"INSTrument:OUTPut:LEVel:GET? {self.output_id}")
        print(f"Output Level: {self.recv()}")

    def set_output_level(self, level):
        if level > -1.2:
            print("WARN: Level too high, setting to -1.2")
        elif level < -116.2:
            print("WARN: Level too low, setting to -116.2")
        level = max(-116.2, min(-1.2, level))

        # Set the output level
        # Syntax:
        # INSTrument:OUTPut:LEVel:SET <output id>,<set value>
        # Description:
        # Set the output level of the specified output port in dBm. The set value is specified in dBm.
        # Example:
        # // set output 1 level to -20dBm
        # inst:output:level:set 1,-20    
        self.send(f"INSTrument:OUTPut:LEVel:SET {self.output_id},{level}")
        self.get_output_level()

    def get_output_state(self):
        # Get the enable state of the output
        # Syntax:
        # INSTrument:OUTPut:ENable:GET? <output id>
        # Description:
        # Get the enable state of the specified output port. Return value is specified as follows:
        # 0 Output disabled
        # 1 Output enabled
        # Example:
        # // get output 1 enable state
        # inst:output:enable:get? 1
        # 1
        self.send(f"INSTrument:OUTPut:ENable:GET? {self.output_id}")
        print(f"Output State: {self.recv()}")
    
    def set_output_state(self, state):
        # Set the enable state of the output
        # Syntax:
        # INSTrument:OUTPut:ENable:SET <output id>,<set value>
        # Description:
        # Set the enable state of the specified output. Possible set values are as follows:
        # 0 Output disabled
        # 1 Output enabled
        # Example:
        # // set output 1 enabled
        # inst:output:enable:set 1,1
        self.send(f"INSTrument:OUTPut:ENable:SET {self.output_id},{state}")
        self.get_output_state()


    def get_output_frequency(self):
        # Get the output frequency
        # Syntax:
        # INSTrument:OUTPut:FREQuency:GET? <output id>
        # Description:
        # Get the frequency of the specified output port in MHz.
        # Example:
        # // get output 1 frequency
        # inst:output:frequency:get? 1
        # 3500
        self.send(f"INSTrument:OUTPut:FREQuency:GET? {self.output_id}")
        print(f"Output Frequency: {self.recv()}")

    def set_output_frequency(self, frequency):
        if frequency > 3500:
            print("WARN: Frequency too high, setting to 3500")
        elif frequency < 1:
            print("WARN: Frequency too low, setting to 1")
        frequency = max(1, min(3500, frequency))
        
        # Set the output frequency
        # Syntax:
        # INSTrument:OUTPut:FREQuency:SET <output id>,<frequency>
        # Description:
        # Set the frequency of the specified output port.
        # Example:
        # // set output 1 frequency
        # inst:output:frequency:set 1,3500
        self.send(f"INSTrument:OUTPut:FREQuency:SET {self.output_id},{frequency}")
        self.get_output_frequency()

# mtrx = MTRX()
# mtrx.get_output_level()
# mtrx.get_output_state()

# mtrx.set_output_level(-4.5)
# mtrx.set_output_state(1)

# mtrx.get_output_frequency()
# mtrx.set_output_frequency(3450)
# mtrx.set_output_state(1)
