#from w1thermsensor import W1ThermSensor
import time

sensor = None
#try:
#    sensor = W1ThermSensor()
#except Exception:
    #pass  # no sensor connected

def get_temperature():
    """Return temperature in °C from DS18B20 sensor or None if not available."""
    #if sensor:
        #return sensor.get_temperature()
    return None

def calculate_power(v, i):
    """Calculate instantaneous power in Watts."""
    return abs(v * i)

def check_limits(v, i_limit, component_params):
    """Check voltage/current/power/temperature limits from YAML."""
    p = calculate_power(v, i_limit)
    pd_max = component_params.get('pd_max', 0.5)
    if p > pd_max:
        return False, f"Power {p*1000:.1f} mW > Pd_max {pd_max*1000:.1f} mW!"
    if v > component_params.get('max_v', 10):
        return False, "Voltage too high!"
    t = get_temperature()
    if t and t > component_params.get('t_max', 80):
        return False, f"Temperature {t:.1f}°C > limit!"
    return True, "OK"