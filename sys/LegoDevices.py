# Shim — real implementation is in PiStorms/devices/LegoDevices.py
import importlib.util as _ilu, os as _os, sys as _sys
_path = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'devices', 'LegoDevices.py')
_spec = _ilu.spec_from_file_location('LegoDevices', _path)
_mod  = _ilu.module_from_spec(_spec)
_sys.modules['LegoDevices'] = _mod
_spec.loader.exec_module(_mod)
# re-export everything
globals().update({k: v for k, v in vars(_mod).items() if not k.startswith('_')})
