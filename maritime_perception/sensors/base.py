"""
sensors/base.py
===============
AbstractSensorPipeline — the contract every sensor module must satisfy.

The fusion engine calls these three methods and nothing else.
This is the entire interface between sensors and fusion.

Adding a new sensor:
    1. Create sensors/<name>/pipeline.py
    2. Implement AbstractSensorPipeline
    3. Add to the sensors list in main.py
    4. Zero changes to fusion, world model, safety, or navigation.

Removing a sensor:
    1. Remove from the sensors list in main.py
    2. Zero changes anywhere else.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from maritime_perception.models.observation import DetectionObservation


class AbstractSensorPipeline(ABC):
    """
    Interface that every sensor pipeline must implement.
    The fusion engine depends only on this interface.
    """

    @abstractmethod
    def latest(self) -> list[DetectionObservation]:
        """
        Return the most recent list of DetectionObservations.
        Must never block. Must never raise.
        Returns empty list if no data is available yet.
        """
        ...

    @abstractmethod
    def is_healthy(self) -> bool:
        """
        Return True if the sensor is operating normally.
        Return False if: driver disconnected, data stale, hardware fault.
        The health watchdog polls this every cycle.
        """
        ...

    @abstractmethod
    def start(self) -> None:
        """Start the sensor thread and begin reading data."""
        ...

    @abstractmethod
    def stop(self) -> None:
        """
        Gracefully stop the sensor thread.
        Must be safe to call multiple times.
        Must return within 2 seconds.
        """
        ...