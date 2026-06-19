# -*- coding: utf-8 -*-
#
# ,---------,       ____  _ __
# |  ,-^-,  |      / __ )(_) /_______________ _____  ___
# | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
# | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
# Copyright (C) 2022 Bitcraze AB
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

"""
Abstract interface definitions for Lighthouse geometry and calibration data types.

========================================
NEEDED FROM cflib - ASK FOR THESE FUNCTIONS
========================================

To integrate actual Crazyflie library code, provide the following from cflib:

1. cflib.crazyflie.mem.LighthouseBsGeometry
   Repository: https://github.com/bitcraze/crazyflie-lib-python/blob/master/cflib/crazyflie/mem.py
   Needed methods/properties:
   - __init__()
   - valid (property) -> bool
   - position (property) -> array [x, y, z]
   - rotation_quat (property) -> array [x, y, z, w]
   - as_file_object() -> dict
   - from_file_object(dict) -> LighthouseBsGeometry (classmethod)

2. cflib.crazyflie.mem.LighthouseBsCalibration
   Needed methods/properties:
   - __init__()
   - valid (property) -> bool
   - as_file_object() -> dict
   - from_file_object(dict) -> LighthouseBsCalibration (classmethod)

3. cflib.crazyflie.mem.LighthouseMemHelper
   Needed methods:
   - __init__(cf)
   - write_geos(geos: dict, callback)
   - write_calibs(calibs: dict, callback)

4. cflib.crazyflie.param.PersistentParamState
   Needed:
   - __init__(is_stored: bool, default_value, stored_value)
   - Attributes: is_stored, default_value, stored_value

INTEGRATION INSTRUCTIONS:
========================
Option 1 - Copy cflib implementations:
  Just paste the cflib code inside the stub classes below to replace them

Option 2 - Use direct imports (when cflib is installed):
  Replace this entire file's stub classes with:
    from cflib.crazyflie.mem import LighthouseBsGeometry, LighthouseBsCalibration, LighthouseMemHelper
    from cflib.crazyflie.param import PersistentParamState

Current state: Using STUB implementations (fully functional for standalone calibration)
"""

from __future__ import annotations

import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation
from abc import ABC, abstractmethod


class LighthouseBsGeometryInterface(ABC):
    """
    Interface defining required methods/properties for Lighthouse base station geometry.
    
    Required from cflib.crazyflie.mem.LighthouseBsGeometry:
    - position property (3D position vector)
    - rotation_quat property (quaternion as [x, y, z, w])
    - valid property (bool indicating if geometry is valid)
    - as_file_object() method (serialize to dict for YAML)
    - from_file_object(dict) classmethod (deserialize from dict)
    """
    
    @property
    @abstractmethod
    def valid(self) -> bool:
        """Whether this geometry is valid"""
        pass
    
    @property
    @abstractmethod
    def position(self) -> npt.NDArray:
        """Get position vector [x, y, z]"""
        pass
    
    @property
    @abstractmethod
    def rotation_quat(self) -> npt.NDArray:
        """Get rotation as quaternion [x, y, z, w]"""
        pass
    
    @abstractmethod
    def as_file_object(self) -> dict:
        """Serialize to dictionary for YAML storage"""
        pass
    
    @classmethod
    @abstractmethod
    def from_file_object(cls, data: dict) -> 'LighthouseBsGeometryInterface':
        """Deserialize from dictionary (from YAML)"""
        pass


class LighthouseBsCalibrationInterface(ABC):
    """
    Interface defining required methods/properties for Lighthouse calibration data.
    
    Required from cflib.crazyflie.mem.LighthouseBsCalibration:
    - valid property (bool indicating if calibration is valid)
    - as_file_object() method (serialize to dict for YAML)
    - from_file_object(dict) classmethod (deserialize from dict)
    """
    
    @property
    @abstractmethod
    def valid(self) -> bool:
        """Whether this calibration is valid"""
        pass
    
    @abstractmethod
    def as_file_object(self) -> dict:
        """Serialize to dictionary for YAML storage"""
        pass
    
    @classmethod
    @abstractmethod
    def from_file_object(cls, data: dict) -> 'LighthouseBsCalibrationInterface':
        """Deserialize from dictionary (from YAML)"""
        pass


class LighthouseMemHelperInterface(ABC):
    """
    Interface defining required methods for Lighthouse memory operations.
    
    Required from cflib.crazyflie.mem.LighthouseMemHelper:
    - write_geos(geos: dict, callback) method
    - write_calibs(calibs: dict, callback) method
    
    These are only used when writing to actual Crazyflie hardware.
    For standalone calibration, these can be no-ops.
    """
    
    @abstractmethod
    def write_geos(self, geos: dict, callback) -> None:
        """Write geometry data to device memory"""
        pass
    
    @abstractmethod
    def write_calibs(self, calibs: dict, callback) -> None:
        """Write calibration data to device memory"""
        pass


# ============================================================================
# STUB IMPLEMENTATIONS - Replace with cflib imports when available
# ============================================================================


class LighthouseBsGeometry(LighthouseBsGeometryInterface):
    """
    STUB IMPLEMENTATION of Lighthouse base station geometry.
    
    This standalone implementation stores position and rotation.
    When cflib is available, replace this import with:
        from cflib.crazyflie.mem import LighthouseBsGeometry
    """

    def __init__(self, position: npt.ArrayLike = None, rotation_quat: npt.ArrayLike = None) -> None:
        """
        Initialize geometry with position and rotation.
        
        :param position: Position vector [x, y, z] in meters (default: origin)
        :param rotation_quat: Quaternion [x, y, z, w] (default: identity rotation)
        """
        self._valid = False
        
        if position is None:
            self._position = np.array([0.0, 0.0, 0.0])
        else:
            self._position = np.array(position, dtype=float)
        
        if rotation_quat is None:
            # Identity quaternion
            self._rotation_quat = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            self._rotation_quat = np.array(rotation_quat, dtype=float)
    
    @property
    def valid(self) -> bool:
        """Whether this geometry has been initialized with valid data."""
        return self._valid
    
    @property
    def position(self) -> npt.NDArray:
        """Get the position vector [x, y, z]."""
        return self._position
    
    @property
    def rotation_quat(self) -> npt.NDArray:
        """Get the rotation as quaternion [x, y, z, w]."""
        return self._rotation_quat
    
    @property
    def rotation_matrix(self) -> npt.NDArray:
        """Get the rotation as a 3x3 matrix."""
        rot = Rotation.from_quat(self._rotation_quat)
        return rot.as_matrix()
    
    def set_from_pose(self, pose_matrix_vec: tuple[npt.NDArray, npt.NDArray]) -> None:
        """
        Set geometry from a rotation matrix and translation vector.
        
        :param pose_matrix_vec: Tuple of (rotation_matrix, translation_vector)
        """
        rot_matrix, translation = pose_matrix_vec
        rot = Rotation.from_matrix(rot_matrix)
        self._rotation_quat = rot.as_quat()
        self._position = np.array(translation, dtype=float)
        self._valid = True
    
    def as_file_object(self) -> dict:
        """
        Convert to a dictionary that can be serialized to YAML.
        
        :return: Dictionary with geometry data
        """
        return {
            'position': self._position.tolist(),
            'rotation_quat': self._rotation_quat.tolist(),
        }
    
    @classmethod
    def from_file_object(cls, data: dict) -> 'LighthouseBsGeometry':
        """
        Create a LighthouseBsGeometry from deserialized YAML data.
        
        :param data: Dictionary from YAML with position and rotation_quat keys
        :return: LighthouseBsGeometry instance
        """
        obj = cls()
        
        if isinstance(data, dict):
            if 'position' in data:
                obj._position = np.array(data['position'], dtype=float)
            
            if 'rotation_quat' in data:
                obj._rotation_quat = np.array(data['rotation_quat'], dtype=float)
            
            # Mark as valid if we have position data
            if 'position' in data:
                obj._valid = True
        
        return obj


class LighthouseBsCalibration(LighthouseBsCalibrationInterface):
    """
    STUB IMPLEMENTATION of Lighthouse calibration data.
    
    When cflib is available, replace this import with:
        from cflib.crazyflie.mem import LighthouseBsCalibration
    """
    
    def __init__(self) -> None:
        """Initialize an empty calibration object."""
        self._valid = False
        self._calibration_data = {}
    
    @property
    def valid(self) -> bool:
        """Whether this calibration has been initialized with valid data."""
        return self._valid
    
    def as_file_object(self) -> dict:
        """
        Convert to a dictionary that can be serialized to YAML.
        
        :return: Dictionary with calibration data
        """
        return self._calibration_data
    
    @classmethod
    def from_file_object(cls, data: dict) -> 'LighthouseBsCalibration':
        """
        Create a LighthouseBsCalibration from deserialized YAML data.
        
        :param data: Dictionary from YAML with calibration data
        :return: LighthouseBsCalibration instance
        """
        obj = cls()
        
        if isinstance(data, dict):
            obj._calibration_data = dict(data)
            # Mark as valid if we have any calibration data
            if data:
                obj._valid = True
        
        return obj


class LighthouseMemHelper(LighthouseMemHelperInterface):
    """
    STUB IMPLEMENTATION of Lighthouse memory helper.
    
    When cflib is available, replace this import with:
        from cflib.crazyflie.mem import LighthouseMemHelper
    """
    
    def __init__(self, cf=None) -> None:
        """Initialize with optional Crazyflie object (usually None in standalone mode)."""
        self._cf = cf
    
    def write_geos(self, geos: dict, callback) -> None:
        """Placeholder for writing geometries to device memory."""
        if callback:
            callback(True)
    
    def write_calibs(self, calibs: dict, callback) -> None:
        """Placeholder for writing calibrations to device memory."""
        if callback:
            callback(True)


class PersistentParamState:
    """
    STUB IMPLEMENTATION of persistent parameter state.
    
    When cflib is available, replace this import with:
        from cflib.crazyflie.param import PersistentParamState
    """
    
    def __init__(self, is_stored: bool = False, default_value=None, stored_value=None) -> None:
        """
        Initialize a persistent parameter state.
        
        :param is_stored: Whether the parameter is stored on device
        :param default_value: Default value for the parameter
        :param stored_value: Currently stored value on device
        """
        self.is_stored = is_stored
        self.default_value = default_value
        self.stored_value = stored_value



