# Shim — real implementation is in PiStorms/ui/mindsensorsUI.py
import importlib.util as _ilu, os as _os, sys as _sys
_path = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'ui', 'mindsensorsUI.py')
_spec = _ilu.spec_from_file_location('mindsensorsUI', _path)
_mod  = _ilu.module_from_spec(_spec)
_sys.modules['mindsensorsUI'] = _mod
_spec.loader.exec_module(_mod)
mindsensorsUI = _mod.mindsensorsUI
