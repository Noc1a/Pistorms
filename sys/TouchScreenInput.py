# Shim — real implementation is in PiStorms/ui/TouchScreenInput.py
import importlib.util as _ilu, os as _os, sys as _sys
_path = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'ui', 'TouchScreenInput.py')
_spec = _ilu.spec_from_file_location('TouchScreenInput', _path)
_mod  = _ilu.module_from_spec(_spec)
_sys.modules['TouchScreenInput'] = _mod
_spec.loader.exec_module(_mod)
TouchScreenInput = _mod.TouchScreenInput
_N_KEYS = _mod._N_KEYS
