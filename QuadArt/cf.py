import root as r
from pyqtgraph.Qt import QtCore
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cfclient.utils.logconfigreader import LogConfig
from threading import Thread
import time

class crazyflie_thread(QtCore.QThread):
    def __init__(self, parent=None):
        super(crazyflie_thread, self).__init__(parent)
        # Initialize the low-level drivers for crazyflie
        cflib.crtp.init_drivers(enable_debug_driver=False)
        # Create a Crazyflie object
        self._cf = Crazyflie()
        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self.roll = self.pitch = self.yaw = 0
        self.thrust = 0
        self.Sig_cf = QtCore.pyqtSignal(float, float, float, int)
        
        
    def run(self):
        # Try to connect to the Crazyflie
        self._cf.open_link(r.link_uri)
        while True:
            time.sleep(r.dt)
            self.emit(QtCore.SIGNAL("Sig_cf(PyQt_PyObject, PyQt_PyObject, PyQt_PyObject, PyQt_PyObject)"), self.roll, self.pitch, self.yaw, self.thrust)
            
    def _do_control(self):
        # Thrust control
        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        thrust_min = 35000
        thrust_max = 50000
        
        thrust = 0
        pitch = 0
        roll = 0
        yawrate = 0
        while r.t < 20:
            if r.marker_found:
                thrust = r.thrust_set
                thrust += r.thrust_eq
                if thrust > thrust_max:
                    thrust = thrust_max
                if thrust < thrust_min:
                    thrust = thrust_min
                    
                pitch = r.pitch_set
                roll = r.roll_set
                
            elif thrust > 0:
                thrust -= 100
                pitch = 0
            self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(1/30.0)
            
        while thrust > 0:
            thrust -= 100
            time.sleep(1/30.0)
        self._cf.commander.send_setpoint(0, 0, 0, 0)
            
            
    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print "Connected to %s" % link_uri

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name="Stabilizer", period_in_ms=1000 * r.dt)
        self._lg_stab.add_variable("stabilizer.roll", "float")
        self._lg_stab.add_variable("stabilizer.pitch", "float")
        self._lg_stab.add_variable("stabilizer.yaw", "float")
        self._lg_stab.add_variable("stabilizer.thrust", "uint16_t")
        # Start control loop thread
        Thread(target=self._do_control).start()
        
        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print "Could not start log configuration," \
                  "{} not found in TOC".format(str(e))
        except AttributeError:
            print "Could not add Stabilizer log config, bad configuration."
            
    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print "Error when logging %s: %s" % (logconf.name, msg)

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        self.roll = data['stabilizer.roll']
        self.pitch = data['stabilizer.pitch']
        self.yaw = data['stabilizer.yaw']
        self.thrust = data['stabilizer.thrust']

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print "Connection to %s failed: %s" % (link_uri, msg)
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print "Connection to %s lost: %s" % (link_uri, msg)

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print "Disconnected from %s" % link_uri
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        self.is_connected = False
