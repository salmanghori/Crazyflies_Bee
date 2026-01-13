# soft_reboot_cf.py
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')  # <-- your URI

def soft_reboot_cf(uri: str, settle_s: float = 2.0):
    """
    Preflight 'soft reboot':
      1) Connect
      2) Stop any high-level activity
      3) Reset the EKF (onboard estimator)
      4) Cleanly close the link
      5) Wait a moment so tasks settle
    """
    print(f"[PRE] Connecting to {uri} ...")
    cflib.crtp.init_drivers()

    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf

            # Make sure HL commander is off and the stack is idle
            try:
                cf.param.set_value('commander.enHighLevel', '0')
            except Exception:
                pass  # not critical if missing

            # Ensure the vehicle is not in any running trajectory
            try:
                cf.high_level_commander.stop()
            except Exception:
                pass

            # Reset the onboard estimator (this is the important part)
            print("[PRE] Resetting onboard estimator (EKF) ...")
            reset_estimator(cf)

            # Optionally re-enable HL for your flight script
            try:
                cf.param.set_value('commander.enHighLevel', '1')
            except Exception:
                pass

            print("[PRE] Soft reset done. Closing link...")
        # Exiting the 'with' closes the link
    except Exception as e:
        print(f"[PRE] Soft reboot failed: {e}")

    print(f"[PRE] Waiting {settle_s:.1f}s for tasks to settle...")
    time.sleep(settle_s)
    print("[PRE] Ready for main script.")

if __name__ == "__main__":
    soft_reboot_cf(URI)
