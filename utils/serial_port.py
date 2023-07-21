#!/usr/bin/env python3
import math
import threading
from threading import Thread
from time import sleep
import serial

class SerialPort():
    """Class responsible for serial communications.

    Parameters
    ----------
    com : str
        String that represents COM port to be opened, for example "COM1".
    baud_rate : int
        Baud-rate to use when opening the port.
    """
    #pylint: disable=too-many-instance-attributes
    _THREAD_DELAY = 0.01

    def __init__(self, com="/dev/ttyUSB0", baud_rate=115200):
        self.com = com
        """String that represents COM port."""
        self.baudrate = baud_rate
        """Serial port baud rate."""
        self.serial_port = None
        """Serial port object."""
        self._port = None
        self._receive_callback = []
        self._run = False
        self._thread = None
        self._lock = threading.Lock()

    def __enter__(self):
        """Opens port on enter.
        """
        self.close()
        #try:
        com = self.com
        self._port = serial.Serial(com, self.baudrate)
        #self._port.setDTR(True)
        #self._port.setRTS(True)
        self._port.writeTimeout = 1
        print ("Opening serial port", com, self.baudrate)
        #except:
        #    print ("Serial port is none")
        #    self._port = None

        if (self._port is not None) and self._port.isOpen() and not self._run:
            print ("Starting serial port")
            self._run = True
            self._thread = Thread(target=self._receive_listener, name="Waterlink serial thread")
            self._thread.daemon = True
            self._thread.start()
        else:
            self._port = None

    def __exit__(self, exc_type, exc_value, traceback):
        """Closes port on exit.
        """
        self.close()

    def open(self, com: str, baud_rate: int) -> bool:
        """Opens serial port.

        Parameters
        ----------
        com : str
            String that represents COM port to be opened, for example "COM1".
        baud_rate : int
            Baud-rate to use when opening the port.

        Returns
        -------
        bool
            True if port is opened, False otherwise.
        """
        self.com = com
        self.baudrate = baud_rate
        self.__enter__()
        return self._run

    def is_open(self) -> bool:
        """Checks if port is open.

        Returns
        -------
        bool
            True if port is opened, False otherwise.
        """
        return self._run

    def close(self):
        """Closes serial port.
        """
        if self._run:
            self._run = False
            self._thread.join(2)
        if self._port is not None and self._port.isOpen():
            self._port.close()
            self.serial_port = None

    def write(self, array: bytearray):
        """Writes byte array to port.

        Parameters
        ----------
        array : byte array
            The byte array to be written to serial port.
        """
        if self._port is not None and self._port.isOpen():
            with self._lock:
                try:
                    self._port.write(array)
                except serial.SerialException:
                    pass

    def register_receive_callback(self, function):
        """Registers receive callback function.

        Parameters
        ----------
        function
            Callback function.
        """
        self._receive_callback.append(function)

    def unregister_all_callbacks(self):
        """Unregisters all callback functions.
        """
        self._receive_callback.clear()

    def _receive_listener(self):
        """Thread function that reads bytes from port.
        """
        while self._run:
            if self._port.isOpen():
                bytes_to_read = self._port.inWaiting()
                if bytes_to_read > 0:
                    with self._lock:
                        arr = self._port.read_all()
                    if self._run and len(arr) > 0:
                        for func in self._receive_callback:
                            func(arr)
            sleep(SerialPort._THREAD_DELAY)
        #print("Finished serial thread")

    def set_baudrate(self, baud_rate: int) -> bool:
        """Changes baud-rate of the open port.

        Parameters
        ----------
        baud_rate : int
            Changes the baud rate and re-opens the port.

        Returns
        -------
        bool
            True if port is opened, False otherwise.
        """
        self.baudrate = baud_rate
        if self._port is not None:
            self.close()
            sleep(0.1)
        return self.open(self.com, self.baudrate)