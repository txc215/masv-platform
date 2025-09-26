# plugins/base_node.py

from abc import ABC, abstractmethod
from typing import Optional, Dict, Any

class BaseNode(ABC):
    @abstractmethod
    def reset(self) -> None: ...
    @abstractmethod
    def on_event(self, ev: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        example：
          {"_topic":"imu","timestamp":12.34,"ax":...,"gz":...}
          {"_topic":"gnss","timestamp":12.50,"lat":...,"lon":...}
          {"_topic":"mag","timestamp":12.52,"mx":...,"my":...}
        return (if has output)
          {"timestamp": t, "x":..., "y":..., "yaw":...}
        """
